"""
server.py
Flask server that serves the web UI and point cloud data.
Also receives the selected goal from the browser and forwards it to ros_bridge,
plus relays initial-pose hints + live robot poses between the GUI and one or
more ros_pose_bridge.py instances for RTAB-Map localization.

Multi-robot model
-----------------
Each `ros_pose_bridge.py` instance is launched with `--ros-args -p robot_id:=<id>`
(e.g. `tb3_0`, `tb3_1`) and POSTs its TF lookup to /pose with `robot_id`
embedded in the JSON body. The server demuxes by `robot_id` and stores the
latest pose per robot. The GUI then polls /pose/<robot_id> for each robot it
wants to render. Same pattern for initial-pose hints: the GUI POSTs to
/set_initial_pose with a `robot_id` (or to /set_initial_pose/<robot_id>) and
each bridge polls /set_initial_pose/<its_own_id>, only acting when its
per-robot `seq` increments.

Backward compatibility
----------------------
A bridge that POSTs without `robot_id` is bucketed under the literal id
"default", and the legacy GET /pose returns the most recently received pose
across all robots — so a single-robot deployment still Just Works.

Endpoints:
    GET  /                         web UI
    GET  /map                      primary map JSON
    GET  /map_overlay              optional A/B map JSON  (204 if not loaded)
    POST /goal                     browser -> server: new goal
    GET  /goal                     anything -> server: read latest goal

    POST /pose                     ros_pose_bridge -> server: live pose
                                     (demuxed by `robot_id` field in body)
    GET  /pose                     legacy: most-recent pose across all robots
    GET  /pose/<robot_id>          per-robot live pose feed for the GUI

    POST /set_initial_pose         browser -> server: queue a hint
                                     (uses `robot_id` field in body, or "default")
    POST /set_initial_pose/<id>    explicit per-robot variant
    GET  /set_initial_pose         legacy: poll the "default" queue
    GET  /set_initial_pose/<id>    per-robot poll for ros_pose_bridge

Usage:
    python3 server.py --map map.json --port 5002
"""

import argparse
import json
import math
import threading
import time
from datetime import datetime, timezone
from flask import Flask, jsonify, request, send_from_directory
from flask_cors import CORS

app = Flask(__name__, static_folder=".")
CORS(app)

# ── Map / goal state ────────────────────────────────────────────────────
map_data       = None
overlay_data   = None       # optional second map for visual A/B
latest_goal    = None
goal_callbacks = []         # functions to call when a new goal is received

# ── Live-pose relay (ros_pose_bridge → GUI) ─────────────────────────────
# One entry per robot, keyed by the `robot_id` the bridge sent in its POST
# body. Each entry is { "data": <last pose JSON>, "recv_t": <wall clock> }.
# Multi-threaded Flask dev server means we guard the read-modify-write
# behind a lock.
latest_poses     = {}        # robot_id -> {"data": dict, "recv_t": float}
_poses_lock      = threading.Lock()

# ── Initial-pose hints (GUI → ros_pose_bridge → /initialpose for RTAB-Map)
# RViz "2D Pose Estimate" equivalent. Per-robot queue: each bridge tracks
# its own last_seen_seq for its own robot_id and only acts when its
# entry's seq increments past it. POSTs without an explicit robot_id land
# in the "default" bucket — single-robot deployments stay one-line.
pending_initial_poses = {}    # robot_id -> {seq, x, y, yaw_rad, frame, wall_clock_iso}
_initial_pose_lock    = threading.Lock()

DEFAULT_ROBOT_ID = "default"


# ── Helpers ─────────────────────────────────────────────────────────────

def _resolve_robot_id(data, override=None):
    """Pick a robot_id from an explicit URL override, then a body field,
    then fall back to DEFAULT_ROBOT_ID. Always returned as a string."""
    if override is not None and override != "":
        return str(override)
    if isinstance(data, dict):
        rid = data.get("robot_id")
        if rid is not None and rid != "":
            return str(rid)
    return DEFAULT_ROBOT_ID


# ── Routes ──────────────────────────────────────────────────────────────

@app.route("/")
def index():
    return send_from_directory(".", "index.html")


@app.route("/map")
def get_map():
    if map_data is None:
        return jsonify({"error": "Map not loaded"}), 500
    return jsonify(map_data)


@app.route("/map_overlay")
def get_map_overlay():
    if overlay_data is None:
        return ("", 204)
    return jsonify(overlay_data)


@app.route("/goal", methods=["POST"])
def receive_goal():
    """Receive selected destination from the browser.

    Z-up convention: x and y are floor-plane, z is height (= floor_z).
    Body: { "x": float, "y": float, "z": float, "orientation": float (deg) }
    """
    global latest_goal
    data = request.get_json()

    if not data or "x" not in data or "y" not in data:
        return jsonify({"error": "Invalid goal data"}), 400

    latest_goal = {
        "x"           : float(data["x"]),
        "y"           : float(data["y"]),
        "z"           : float(data.get("z", 0.0)),
        "orientation" : float(data.get("orientation", 0.0))
    }

    print(f"\n--- New Goal Received ---")
    print(f"X: {latest_goal['x']:.3f}m")
    print(f"Y: {latest_goal['y']:.3f}m")
    print(f"Z: {latest_goal['z']:.3f}m")
    print(f"Orientation: {latest_goal['orientation']:.1f}°")
    print("-------------------------\n")

    for cb in goal_callbacks:
        threading.Thread(target=cb, args=(latest_goal,), daemon=True).start()

    return jsonify({"status": "ok", "goal": latest_goal})


@app.route("/goal", methods=["GET"])
def get_latest_goal():
    if latest_goal is None:
        return jsonify({"status": "no goal set"}), 204
    return jsonify(latest_goal)


@app.route("/pose", methods=["POST"])
def receive_pose():
    """Receive a live robot pose from a ros_pose_bridge.py instance.

    The bridge embeds its own `robot_id` in the body so this server can
    demux multiple bridges into per-robot slots.

    Body schema (see ros_pose_bridge.py docstring):
        {
          "robot_id":           "tb3_0" | "tb3_1" | ...,
          "localized":          true | false,
          "x": ..., "y": ..., "yaw_rad": ..., "yaw_deg": ...,
          "frame":              "map",
          "last_match_age_sec": float | null,
          "wall_clock_iso":     "..."
        }
    """
    data = request.get_json(silent=True)
    if data is None:
        return jsonify({"error": "no JSON body"}), 400
    robot_id = _resolve_robot_id(data)
    with _poses_lock:
        latest_poses[robot_id] = {"data": data, "recv_t": time.time()}
    return jsonify({"status": "ok", "robot_id": robot_id})


@app.route("/pose", methods=["GET"])
def get_pose_legacy():
    """Legacy single-robot read: return the most-recently received pose
    across ALL robots. Single-robot deployments keep working unchanged.
    Multi-robot GUIs should hit /pose/<robot_id> instead."""
    with _poses_lock:
        if not latest_poses:
            return jsonify({"available": False, "reason": "no_bridge_yet"}), 200
        # Pick the freshest entry across robots.
        rid, entry = max(latest_poses.items(), key=lambda kv: kv[1]["recv_t"])
        age_sec = time.time() - entry["recv_t"]
        body    = entry["data"]
    return jsonify({
        "available": True,
        "age_sec":   age_sec,
        "robot_id":  rid,
        **body,
    }), 200


@app.route("/pose/<robot_id>", methods=["GET"])
def get_pose_for_robot(robot_id):
    """Per-robot live-pose feed. The browser polls one of these per robot
    it wants to render. Returns `available: false` if that robot's bridge
    has never posted, so the GUI can show a clear NO BRIDGE pill."""
    with _poses_lock:
        entry = latest_poses.get(robot_id)
        if entry is None:
            return jsonify({
                "available": False,
                "reason":    "no_bridge_yet",
                "robot_id":  robot_id,
            }), 200
        age_sec = time.time() - entry["recv_t"]
        body    = entry["data"]
    return jsonify({
        "available": True,
        "age_sec":   age_sec,
        "robot_id":  robot_id,
        **body,
    }), 200


def _queue_initial_pose(robot_id, data):
    """Common: stamp + bump per-robot seq + store. Returns the snapshot."""
    with _initial_pose_lock:
        prev = pending_initial_poses.get(robot_id)
        next_seq = (prev["seq"] + 1) if prev is not None else 1
        snapshot = {
            "seq":            next_seq,
            "robot_id":       robot_id,
            "x":              float(data["x"]),
            "y":              float(data["y"]),
            "yaw_rad":        float(data.get("yaw_rad", 0.0)),
            "frame":          "map",
            "wall_clock_iso": datetime.now(timezone.utc).isoformat(),
        }
        pending_initial_poses[robot_id] = snapshot
    return snapshot


@app.route("/set_initial_pose", methods=["POST"])
def set_initial_pose():
    """Receive an initial-pose hint from the browser GUI.

    Body: {"x": float, "y": float, "yaw_rad": float, "robot_id"?: str}

    Z-up convention: x and y are ROS map-frame coordinates on the floor
    plane (the same frame /goal uses). Each robot's bridge polls
    /set_initial_pose/<its_own_id> at its tick rate; when seq increments
    for that robot, the bridge publishes one PoseWithCovarianceStamped to
    /initialpose for its RTAB-Map. This is the same topic AMCL / RViz "2D
    Pose Estimate" use, so RTAB-Map seeds its pose at the hint and
    converges in 1-2 sec instead of doing a slow global re-localization
    search.

    Posts that don't carry a `robot_id` land in the "default" bucket. A
    single-robot deployment that always polls /set_initial_pose (no path
    suffix) keeps working unchanged.
    """
    data = request.get_json(silent=True)
    if data is None or "x" not in data or "y" not in data:
        return jsonify({"error": "missing x/y in JSON body"}), 400
    robot_id = _resolve_robot_id(data)
    snap = _queue_initial_pose(robot_id, data)
    print(f"[server] initial pose hint #{snap['seq']} for {robot_id}: "
          f"x={snap['x']:.2f}, y={snap['y']:.2f}, "
          f"yaw={math.degrees(snap['yaw_rad']):.0f}deg")
    return jsonify({"status": "queued",
                    "seq":      snap["seq"],
                    "robot_id": robot_id})


@app.route("/set_initial_pose/<robot_id>", methods=["POST"])
def set_initial_pose_for(robot_id):
    """Explicit per-robot POST. URL path overrides body's robot_id."""
    data = request.get_json(silent=True)
    if data is None or "x" not in data or "y" not in data:
        return jsonify({"error": "missing x/y in JSON body"}), 400
    snap = _queue_initial_pose(robot_id, data)
    print(f"[server] initial pose hint #{snap['seq']} for {robot_id}: "
          f"x={snap['x']:.2f}, y={snap['y']:.2f}, "
          f"yaw={math.degrees(snap['yaw_rad']):.0f}deg")
    return jsonify({"status": "queued",
                    "seq":      snap["seq"],
                    "robot_id": robot_id})


@app.route("/set_initial_pose", methods=["GET"])
def get_initial_pose_legacy():
    """Legacy single-queue poll for the bridge. Returns the "default"
    bucket so single-robot setups keep working without changes."""
    with _initial_pose_lock:
        snap = pending_initial_poses.get(DEFAULT_ROBOT_ID)
    if snap is None:
        return jsonify({"available": False, "seq": 0}), 200
    return jsonify({"available": True, **snap}), 200


@app.route("/set_initial_pose/<robot_id>", methods=["GET"])
def get_initial_pose_for(robot_id):
    """Per-robot poll for ros_pose_bridge.py.

    Each bridge tracks its own last_seen_seq; when the seq we return here
    is greater than what the bridge last saw, it publishes one
    PoseWithCovarianceStamped on /initialpose and updates its
    last_seen_seq. Bridges only ever read the queue for their own
    robot_id, so a hint for tb3_0 never accidentally seeds tb3_1."""
    with _initial_pose_lock:
        snap = pending_initial_poses.get(robot_id)
    if snap is None:
        return jsonify({"available": False, "seq": 0,
                        "robot_id": robot_id}), 200
    return jsonify({"available": True, **snap}), 200


def register_goal_callback(fn):
    """Register a function to be called when a new goal is received."""
    goal_callbacks.append(fn)


# ── A* path planning ────────────────────────────────────────────────────
# Browser POSTs `{start: {x,y}, goal: {x,y}}` to /plan. We call
# compute_plan() from astar_planner.py, write the resulting waypoints to
# path_plan.json with a bumped seq, and return OK. The GUI polls
# /path_plan.json (served by the catch-all static route below) and
# renders the waypoints as dots on the floor.
#
# Defaults are tuned for two TurtleBot3 Conveyors in formation: each
# robot is treated as a 0.20 m disc, separation 0.50 m → formation
# bounding radius ≈ 0.45 m. Walls inflate by that much so wherever the
# virtual centre can go, the whole pair fits.

_plan_seq      = 0
_plan_seq_lock = threading.Lock()
PLAN_PATH      = "path_plan.json"

PLAN_ROBOT_RADIUS       = 0.20    # meters
PLAN_FORMATION_DISTANCE = 0.50    # meters


@app.route("/plan", methods=["POST"])
def plan_explicit():
    """Compute a path from `start` to `goal` (both in world coords) and
    write it to path_plan.json. Body:
        { "start": {"x": float, "y": float},
          "goal":  {"x": float, "y": float} }
    """
    global _plan_seq
    if map_data is None:
        return jsonify({"error": "no map loaded"}), 500

    body = request.get_json(silent=True) or {}
    s = body.get("start") or {}
    g = body.get("goal")  or {}
    if "x" not in s or "y" not in s or "x" not in g or "y" not in g:
        return jsonify({"error": "need start.x, start.y, goal.x, goal.y"}), 400

    start_xy = (float(s["x"]), float(s["y"]))
    goal_xy  = (float(g["x"]), float(g["y"]))
    print(f"[plan] start={start_xy} goal={goal_xy}")

    # Live formation geometry from the GUI (when both robots are
    # localized). Each entry: {name, offset_local: [x, y], color?}.
    # offset_local is in the SYSTEM body frame, exactly what compute_plan
    # expects. Falls back to a sensible 50 cm leader/follower default
    # when the GUI hasn't sent anything (e.g. CLI debug callers).
    robots_in = body.get("robots")
    if robots_in:
        try:
            robots = [
                {
                    "name":         str(r.get("name", f"robot{i}")),
                    "offset_local": [float(r["offset_local"][0]),
                                     float(r["offset_local"][1])],
                    "color":        str(r.get("color", "#aaaaaa")),
                }
                for i, r in enumerate(robots_in)
            ]
        except (KeyError, TypeError, ValueError, IndexError) as e:
            return jsonify({"error": f"bad robots[]: {e}"}), 400
        # Bounding circle of the formation = furthest robot from centre
        # + per-robot footprint radius. With live offsets this tracks the
        # actual inter-robot distance instead of a hardcoded constant.
        max_off = max(math.hypot(r["offset_local"][0], r["offset_local"][1])
                      for r in robots) if robots else 0.0
        formation_radius = max_off + PLAN_ROBOT_RADIUS
    else:
        half = PLAN_FORMATION_DISTANCE / 2
        robots = [
            {"name": "leader",   "offset_local": [+half, 0.0], "color": "#00e5ff"},
            {"name": "follower", "offset_local": [-half, 0.0], "color": "#ff9933"},
        ]
        formation_radius = PLAN_FORMATION_DISTANCE / 2 + PLAN_ROBOT_RADIUS
    print(f"[plan] formation_radius={formation_radius:.3f} m  "
          f"({len(robots)} robots: "
          f"{', '.join(r['name'] for r in robots)})")

    try:
        from astar_planner import compute_plan
        plan = compute_plan(map_data, start_xy, goal_xy,
                            formation_radius=formation_radius,
                            robots=robots)
    except Exception as e:
        print(f"[plan] A* failed: {e}")
        return jsonify({"error": str(e)}), 500

    with _plan_seq_lock:
        _plan_seq += 1
        plan["seq"] = _plan_seq
        plan["wall_clock_iso"] = datetime.now(timezone.utc).isoformat()

    with open(PLAN_PATH, "w") as f:
        json.dump(plan, f, separators=(",", ":"))
    print(f"[plan] wrote {PLAN_PATH} seq={_plan_seq} "
          f"({len(plan['virtual_center']['waypoints'])} VC waypoints, "
          f"{plan['metadata']['vc_distance']:.2f} m)")

    return jsonify({"status":      "ok",
                    "seq":         plan["seq"],
                    "waypoints":   plan['virtual_center']['waypoints'],
                    "headings":    plan['virtual_center']['headings'],
                    "vc_distance": plan['metadata']['vc_distance']})


# Catch-all static-file route — lets the browser fetch additional files
# from this folder directly (e.g. mesh .obj / .mtl / .jpg if you add a
# textured overlay). Defined LAST so it doesn't shadow the specific
# routes above.
@app.route("/<path:filename>")
def static_files(filename):
    return send_from_directory(".", filename)


def load_map(path):
    global map_data
    print(f"Loading map: {path}")
    with open(path, "r") as f:
        map_data = json.load(f)
    print(f"Map loaded: {map_data['metadata']['point_count']} points")


def load_overlay(path):
    """Optional second map served on /map_overlay for visual A/B."""
    global overlay_data
    print(f"Loading overlay map: {path}")
    with open(path, "r") as f:
        overlay_data = json.load(f)
    print(f"Overlay loaded: {overlay_data['metadata']['point_count']} points")


def run(host="0.0.0.0", port=5002):
    print(f"Server running at http://localhost:{port}")
    print("Open this URL in your browser.")
    app.run(host=host, port=port, debug=False, use_reloader=False)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Map visualization server")
    parser.add_argument("--map",     default="map.json",
                        help="Primary map JSON")
    parser.add_argument("--overlay", default=None,
                        help="Optional second map JSON to overlay (for A/B accuracy check)")
    parser.add_argument("--port",    type=int, default=5002, help="Port to serve on")
    args = parser.parse_args()

    load_map(args.map)
    if args.overlay:
        load_overlay(args.overlay)
    run(port=args.port)
