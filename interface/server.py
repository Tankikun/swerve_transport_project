"""
server.py
Flask server that serves the web UI and point cloud data.
Also receives the selected goal from the browser and forwards it to ros_bridge,
plus relays initial-pose hints + live robot pose between the GUI and
ros_pose_bridge.py for RTAB-Map localization.

Endpoints:
    GET  /                    web UI
    GET  /map                 primary map JSON
    GET  /map_overlay         optional A/B map JSON  (204 if not loaded)
    POST /goal                browser -> server: new goal
    GET  /goal                anything -> server: read latest goal
    POST /pose                ros_pose_bridge -> server: live robot pose
    GET  /pose                browser -> server: read latest pose + age
    POST /set_initial_pose    browser -> server: queue an initial-pose hint
    GET  /set_initial_pose    ros_pose_bridge -> server: poll for new hint

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
goal_callbacks = []

# ── Live-pose relay (ros_pose_bridge → GUI) ─────────────────────────────
latest_pose         = None  # most recent pose dict from ros_pose_bridge
latest_pose_recv_t  = 0.0   # server-side wall clock when pose was received

# ── Initial-pose hint (GUI → ros_pose_bridge → /initialpose for RTAB-Map)
# RViz "2D Pose Estimate" equivalent. The bridge polls /set_initial_pose
# at its tick rate and only acts when `seq` increments past its
# last_seen_seq. Flask's dev server is multi-threaded, so the
# read-modify-write on initial_pose_seq + pending_initial_pose is
# protected against simultaneous POSTs racing.
pending_initial_pose = None
initial_pose_seq     = 0
_initial_pose_lock   = threading.Lock()


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
    """Receive live robot pose from ros_pose_bridge.py.

    Body schema (see ros_pose_bridge.py docstring):
        {
          "robot_id":           "tb3_1",
          "localized":          true | false,
          "x": ..., "y": ..., "yaw_rad": ..., "yaw_deg": ...,
          "frame":              "map",
          "last_match_age_sec": float | null,
          "wall_clock_iso":     "..."
        }
    """
    global latest_pose, latest_pose_recv_t
    data = request.get_json(silent=True)
    if data is None:
        return jsonify({"error": "no JSON body"}), 400
    latest_pose        = data
    latest_pose_recv_t = time.time()
    return jsonify({"status": "ok"})


@app.route("/pose", methods=["GET"])
def get_pose():
    """Serve the latest robot pose to the GUI.

    Adds a server-side `age_sec` so the browser can decide if the bridge
    has gone silent (e.g. ros_pose_bridge crashed). When no bridge has
    ever posted a pose, returns `available: false` so the GUI can show
    a clear "NO BRIDGE" badge instead of stale data.
    """
    if latest_pose is None:
        return jsonify({"available": False, "reason": "no_bridge_yet"}), 200
    age_sec = time.time() - latest_pose_recv_t
    return jsonify({
        "available": True,
        "age_sec":   age_sec,
        **latest_pose,
    }), 200


@app.route("/set_initial_pose", methods=["POST"])
def set_initial_pose():
    """Receive an initial-pose hint from the browser GUI.

    Body: {"x": float, "y": float, "yaw_rad": float}

    Z-up convention: x and y are ROS map-frame coordinates on the floor
    plane (the same frame /goal uses). The bridge polls
    /set_initial_pose (GET) at its tick rate; when seq increments, it
    publishes one PoseWithCovarianceStamped to /initialpose for
    RTAB-Map. This is the same topic AMCL / RViz "2D Pose Estimate"
    use, so RTAB-Map seeds its pose at the hint and converges in
    1-2 sec instead of doing a slow global re-localization search.
    """
    global pending_initial_pose, initial_pose_seq
    data = request.get_json(silent=True)
    if data is None or "x" not in data or "y" not in data:
        return jsonify({"error": "missing x/y in JSON body"}), 400
    with _initial_pose_lock:
        initial_pose_seq += 1
        pending_initial_pose = {
            "seq":            initial_pose_seq,
            "x":              float(data["x"]),
            "y":              float(data["y"]),
            "yaw_rad":        float(data.get("yaw_rad", 0.0)),
            "frame":          "map",
            "wall_clock_iso": datetime.now(timezone.utc).isoformat(),
        }
        seq_for_log = initial_pose_seq
        snapshot    = pending_initial_pose
    print(f"[server] initial pose hint #{seq_for_log}: "
          f"x={snapshot['x']:.2f}, "
          f"y={snapshot['y']:.2f}, "
          f"yaw={math.degrees(snapshot['yaw_rad']):.0f}deg")
    return jsonify({"status": "queued", "seq": seq_for_log})


@app.route("/set_initial_pose", methods=["GET"])
def get_initial_pose():
    """Polled by ros_pose_bridge.py. Returns current pending pose + seq.

    The bridge tracks its own last_seen_seq; whenever the seq we return
    here is greater, the bridge publishes one PoseWithCovarianceStamped
    on /initialpose and updates its last_seen_seq.
    """
    if pending_initial_pose is None:
        return jsonify({"available": False, "seq": 0}), 200
    return jsonify({"available": True, **pending_initial_pose}), 200


def register_goal_callback(fn):
    """Register a function to be called when a new goal is received."""
    goal_callbacks.append(fn)


# ── A* auto-replan on /goal ──────────────────────────────────────────────
# When the browser POSTs to /goal, run the global A* planner against the
# current map.json and write path_plan.json + bump its seq counter.
# path_viewer.html polls /path_plan.json and re-renders when seq changes.

_plan_seq      = 0
_plan_seq_lock = threading.Lock()
PLAN_PATH      = "path_plan.json"

# Defaults — match astar_planner.py CLI defaults
PLAN_ROBOT_RADIUS       = 0.20    # meters
PLAN_FORMATION_DISTANCE = 0.50    # meters


def _default_leader_start():
    """Pick a sensible default leader start: lower-left of the navigable area."""
    if map_data is None:
        return None
    mt = map_data["metadata"]
    res = mt["resolution"]
    inset = 4 * res + PLAN_ROBOT_RADIUS
    # Find ground cells to bound the start to actual mapped area
    import numpy as np
    g = np.asarray(map_data["grid"], dtype=np.int32)
    rows, cols = np.where(g == 1)
    if len(rows) == 0:
        return (mt["min_x"] + inset, mt["min_y"] + inset)
    gx_min = mt["min_x"] + cols.min() * res
    gy_min = mt["min_y"] + rows.min() * res
    return (gx_min + inset, gy_min + inset)


def _replan_on_goal(goal: dict):
    """Goal-callback: compute A* and write path_plan.json with bumped seq."""
    global _plan_seq
    if map_data is None:
        print("[plan] no map loaded — skipping A*")
        return

    # Use latest robot pose if available; otherwise fall back to default start.
    if latest_pose is not None and latest_pose.get("localized") and \
            "x" in latest_pose and "y" in latest_pose:
        leader_start = (float(latest_pose["x"]), float(latest_pose["y"]))
        start_source = "live pose"
    else:
        leader_start = _default_leader_start()
        start_source = "default (no live pose)"
    if leader_start is None:
        print("[plan] cannot determine leader start")
        return

    goal_xy = (goal["x"], goal["y"])
    print(f"[plan] start={leader_start} ({start_source}) -> goal={goal_xy}")

    try:
        from astar_planner import compute_plan
        # Formation radius = formation_distance/2 + per-robot radius.
        # Two robots, each with PLAN_ROBOT_RADIUS half-width, separated by
        # PLAN_FORMATION_DISTANCE → bounding circle = distance/2 + radius.
        formation_radius = PLAN_FORMATION_DISTANCE / 2 + PLAN_ROBOT_RADIUS
        half = PLAN_FORMATION_DISTANCE / 2
        robots = [
            {"name": "leader",   "offset_local": [+half, 0.0], "color": "#00e5ff"},
            {"name": "follower", "offset_local": [-half, 0.0], "color": "#ff9933"},
        ]
        plan = compute_plan(map_data, leader_start, goal_xy,
                            formation_radius=formation_radius,
                            robots=robots)
    except Exception as e:
        print(f"[plan] A* failed: {e}")
        return

    with _plan_seq_lock:
        _plan_seq += 1
        plan["seq"] = _plan_seq
        plan["wall_clock_iso"] = datetime.now(timezone.utc).isoformat()

    with open(PLAN_PATH, "w") as f:
        json.dump(plan, f, separators=(",", ":"))
    print(f"[plan] wrote {PLAN_PATH} seq={_plan_seq} "
          f"({len(plan['virtual_center']['waypoints'])} VC waypoints, "
          f"{plan['metadata']['vc_distance']:.2f} m)")


goal_callbacks.append(_replan_on_goal)


# ── /plan: explicit (VC_start, VC_goal) — used by the new UI flow ────────
# Browser POSTs both poses at once when the user presses "Plan Path".
# Body: { "start": {"x": float, "y": float, "yaw": float? },
#         "goal":  {"x": float, "y": float, "yaw": float? } }
# Bumps _plan_seq + writes path_plan.json (same shape as _replan_on_goal).
# Independent of robot localization — no /pose dependency.

@app.route("/plan", methods=["POST"])
def plan_explicit():
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
    print(f"[plan] explicit start={start_xy} goal={goal_xy}")

    # Optional knobs the GUI can pass in the request body
    start_yaw = float(s.get("yaw", 0.0))
    goal_yaw  = float(g.get("yaw", 0.0))
    yaw_policy = str(body.get("yaw_policy", "tangent"))   # "tangent" | "free" | "smooth_to_goal"
    target_spacing = float(body.get("target_spacing", 0.15))   # 0 disables resampling

    try:
        from astar_planner import compute_plan
        formation_radius = PLAN_FORMATION_DISTANCE / 2 + PLAN_ROBOT_RADIUS
        half = PLAN_FORMATION_DISTANCE / 2
        robots = [
            {"name": "leader",   "offset_local": [+half, 0.0], "color": "#00e5ff"},
            {"name": "follower", "offset_local": [-half, 0.0], "color": "#ff9933"},
        ]
        plan = compute_plan(map_data, start_xy, goal_xy,
                            formation_radius=formation_radius,
                            robots=robots,
                            yaw_policy=yaw_policy,
                            start_yaw=start_yaw,
                            goal_yaw=goal_yaw,
                            target_spacing=target_spacing)
    except Exception as e:
        print(f"[plan] explicit A* failed: {e}")
        return jsonify({"error": str(e)}), 500

    with _plan_seq_lock:
        _plan_seq += 1
        plan["seq"] = _plan_seq
        plan["wall_clock_iso"] = datetime.now(timezone.utc).isoformat()
        plan["start_source"] = "explicit (user click)"
        # compute_plan already wrote yaw_policy / goal_yaw / start_yaw into
        # metadata, but make sure they reflect what the request asked for
        # (in case compute_plan defaulted them).
        plan["metadata"]["goal_yaw"]   = goal_yaw
        plan["metadata"]["start_yaw"]  = start_yaw
        plan["metadata"]["yaw_policy"] = yaw_policy

    with open(PLAN_PATH, "w") as f:
        json.dump(plan, f, separators=(",", ":"))
    print(f"[plan] wrote {PLAN_PATH} seq={_plan_seq} "
          f"({len(plan['virtual_center']['waypoints'])} VC waypoints, "
          f"{plan['metadata']['vc_distance']:.2f} m)")
    return jsonify({"status": "ok", "seq": plan["seq"],
                    "vc_distance": plan['metadata']['vc_distance'],
                    "waypoints": len(plan['virtual_center']['waypoints'])})


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
    parser.add_argument("--map",     default="map.json", help="Primary map JSON")
    parser.add_argument("--overlay", default=None,
                        help="Optional second map JSON to overlay (for A/B accuracy check)")
    parser.add_argument("--port",    type=int, default=5002, help="Port to serve on")
    args = parser.parse_args()

    load_map(args.map)
    if args.overlay:
        load_overlay(args.overlay)
    run(port=args.port)
