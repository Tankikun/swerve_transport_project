"""
server.py
Flask server that serves the web UI and point cloud data.
Also receives the selected goal from the browser and forwards to ros_bridge.

Usage:
    python3 server.py --map map.json --port 5000
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

# --- Global state ---
map_data       = None
overlay_data   = None       # optional second map for visual A/B (e.g. Scaniverse .obj)
latest_goal    = None
goal_callbacks = []         # functions to call when a new goal is received
latest_pose         = None  # most recent pose dict from ros_pose_bridge
latest_pose_recv_t  = 0.0   # server-side wall clock when pose was received

# Initial-pose hint (RViz "2D Pose Estimate" equivalent — sent by the GUI's
# 📍 button, picked up by ros_pose_bridge.py and republished to /initialpose
# for RTAB-Map). The bridge polls /set_initial_pose at its tick rate; it
# only acts when `seq` increments past its `last_seen_seq`.
pending_initial_pose = None       # latest pose hint from GUI, awaiting bridge pickup
initial_pose_seq     = 0          # monotonic sequence so bridge knows when there's a new one


# --- Routes ---

@app.route("/")
def index():
    """Serve the main web UI"""
    return send_from_directory(".", "index.html")


@app.route("/map")
def get_map():
    """Serve the primary point cloud JSON to the browser."""
    if map_data is None:
        return jsonify({"error": "Map not loaded"}), 500
    return jsonify(map_data)


@app.route("/map_overlay")
def get_map_overlay():
    """Serve a second map as an overlay for visual A/B comparison.

    Returns 204 if no overlay was loaded, so the frontend can branch.
    """
    if overlay_data is None:
        return ("", 204)
    return jsonify(overlay_data)


@app.route("/goal", methods=["POST"])
def receive_goal():
    """
    Receive selected destination from the browser.
    Expected JSON body:
    {
        "x": 1.2,
        "y": 0.0,
        "z": 0.8,
        "orientation": 90.0   (degrees, yaw only)
    }
    """
    global latest_goal
    data = request.get_json()

    if not data or "x" not in data or "z" not in data:
        return jsonify({"error": "Invalid goal data"}), 400

    latest_goal = {
        "x"           : float(data["x"]),
        "y"           : float(data.get("y", 0.0)),
        "z"           : float(data["z"]),
        "orientation" : float(data.get("orientation", 0.0))
    }

    print(f"\n--- New Goal Received ---")
    print(f"X: {latest_goal['x']:.3f}m")
    print(f"Y: {latest_goal['y']:.3f}m")
    print(f"Z: {latest_goal['z']:.3f}m")
    print(f"Orientation: {latest_goal['orientation']:.1f}°")
    print("-------------------------\n")

    # Notify any registered callbacks (e.g. ros_bridge)
    for cb in goal_callbacks:
        threading.Thread(target=cb, args=(latest_goal,), daemon=True).start()

    return jsonify({"status": "ok", "goal": latest_goal})


@app.route("/goal", methods=["GET"])
def get_latest_goal():
    """Let other scripts poll for the latest goal"""
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
    latest_pose = data
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
    The bridge polls /set_initial_pose (GET) at its tick rate; when seq
    increments, it publishes one PoseWithCovarianceStamped to /initialpose
    for RTAB-Map. This is the same topic AMCL / RViz "2D Pose Estimate"
    use, so RTAB-Map seeds its pose at the hint and converges in 1-2 sec
    instead of doing a slow global re-localization search.
    """
    global pending_initial_pose, initial_pose_seq
    data = request.get_json(silent=True)
    if data is None or "x" not in data or "y" not in data:
        return jsonify({"error": "missing x/y in JSON body"}), 400
    initial_pose_seq += 1
    pending_initial_pose = {
        "seq":      initial_pose_seq,
        "x":        float(data["x"]),
        "y":        float(data["y"]),
        "yaw_rad":  float(data.get("yaw_rad", 0.0)),
        "frame":    "map",
        "wall_clock_iso": datetime.now(timezone.utc).isoformat(),
    }
    print(f"[server] initial pose hint #{initial_pose_seq}: "
          f"x={pending_initial_pose['x']:.2f}, "
          f"y={pending_initial_pose['y']:.2f}, "
          f"yaw={math.degrees(pending_initial_pose['yaw_rad']):.0f}deg")
    return jsonify({"status": "queued", "seq": initial_pose_seq})


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
    """Register a function to be called when a new goal is received"""
    goal_callbacks.append(fn)


# Catch-all static-file route — lets the browser fetch the mesh's .obj /
# .mtl / .jpg trio (or any other file in this folder) directly. Defined
# LAST so it doesn't shadow the specific routes above.
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
