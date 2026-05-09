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

    POST /set_initial_pose/<id>/ack
                                   bridge -> server: confirm `seq` was published
                                     to /initialpose. Lets the GUI distinguish
                                     "hint queued, no bridge consumed it yet"
                                     from "bridge confirmed publish to RTAB-Map".
    GET  /pose_hint_status/<id>    GUI -> server: read latest hint seq + ack
                                     state. Used by the 📍 Set Initial Pose
                                     button to update its label after click.

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
    """Common: stamp + bump per-robot seq + store. Returns the snapshot.

    Each snapshot also carries `acked_seq` (last seq the bridge confirmed
    via POST /set_initial_pose/<id>/ack) and `acked_iso` (timestamp of
    that confirmation, or None if not yet acked). When `acked_seq < seq`
    the GUI knows the hint is in flight; when `acked_seq == seq` the
    bridge has published it to /initialpose and RTAB-Map should converge
    momentarily (or the hint was wrong). Either way the user has a
    signal they didn't have before.
    """
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
            "acked_seq":      0,        # bumped by POST .../ack
            "acked_iso":      None,
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


@app.route("/set_initial_pose/<robot_id>/ack", methods=["POST"])
def ack_initial_pose(robot_id):
    """Bridge -> server: confirm a hint `seq` was published to /initialpose.

    Called by ros_pose_bridge.py after a successful publish on the
    /initialpose topic. The GUI watches /pose_hint_status/<id> to know
    whether the bridge actually consumed the hint or whether the hint is
    still queued (bridge crashed, network dropped, etc.).

    Body: {"seq": int}

    Returns:
      200 {"status": "acked", ...}        normal case
      200 {"status": "no_pending_hint"}   server has no record (legitimate after restart)
      200 {"status": "stale_ack", ...}    seq < current acked_seq (ignored)
      400                                 missing seq

    Always returns 200 except for malformed input — bridges that target
    an old server version still get a graceful answer when this endpoint
    is missing (404 → bridge silently retries on next tick), so this
    feature is opt-in on both sides."""
    data = request.get_json(silent=True) or {}
    if "seq" not in data:
        return jsonify({"error": "missing seq in JSON body"}), 400
    seq = int(data["seq"])
    with _initial_pose_lock:
        snap = pending_initial_poses.get(robot_id)
        if snap is None:
            return jsonify({"status":   "no_pending_hint",
                            "robot_id": robot_id,
                            "seq":      seq}), 200
        if seq < snap.get("acked_seq", 0):
            return jsonify({"status":            "stale_ack",
                            "robot_id":          robot_id,
                            "seq":               seq,
                            "current_acked_seq": snap.get("acked_seq", 0)}), 200
        snap["acked_seq"] = seq
        snap["acked_iso"] = datetime.now(timezone.utc).isoformat()
    print(f"[server] hint #{seq} for {robot_id} acked by bridge")
    return jsonify({"status":   "acked",
                    "robot_id": robot_id,
                    "seq":      seq}), 200


@app.route("/pose_hint_status/<robot_id>", methods=["GET"])
def pose_hint_status_for(robot_id):
    """GUI poll: latest hint seq + ack state for this robot.

    Returns:
      {
        "available":         bool,    # false if no hint ever queued for <id>
        "robot_id":          str,
        "seq":               int,     # latest hint queued
        "acked_seq":         int,     # latest hint the bridge confirmed
        "acked":             bool,    # acked_seq >= seq (hint was published)
        "age_since_queue_s": float,   # wall-clock age of latest hint
        "age_since_ack_s":   float | null,  # wall-clock age of last ack
      }

    The GUI uses this to drive the 📍 Set Initial Pose button label: after
    the user clicks, the button polls this endpoint until either `acked`
    becomes true (success) or a timeout elapses (bridge probably down)."""
    with _initial_pose_lock:
        snap = pending_initial_poses.get(robot_id)
    if snap is None:
        return jsonify({"available": False,
                        "robot_id":  robot_id,
                        "seq":       0,
                        "acked_seq": 0,
                        "acked":     False}), 200
    try:
        queued_t = datetime.fromisoformat(snap["wall_clock_iso"]).timestamp()
        age_q    = max(0.0, time.time() - queued_t)
    except (KeyError, ValueError):
        age_q = None
    age_a = None
    if snap.get("acked_iso"):
        try:
            acked_t = datetime.fromisoformat(snap["acked_iso"]).timestamp()
            age_a   = max(0.0, time.time() - acked_t)
        except ValueError:
            age_a = None
    seq       = int(snap["seq"])
    acked_seq = int(snap.get("acked_seq", 0))
    return jsonify({
        "available":         True,
        "robot_id":          robot_id,
        "seq":               seq,
        "acked_seq":         acked_seq,
        "acked":             acked_seq >= seq,
        "age_since_queue_s": age_q,
        "age_since_ack_s":   age_a,
    }), 200


def register_goal_callback(fn):
    """Register a function to be called when a new goal is received."""
    goal_callbacks.append(fn)


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
