"""
server.py
Flask server that serves the web UI and point cloud data.
Also receives the selected goal from the browser and forwards to ros_bridge.

Usage:
    python3 server.py --map map.json --port 5000
"""

import argparse
import json
import threading
from flask import Flask, jsonify, request, send_from_directory
from flask_cors import CORS

app = Flask(__name__, static_folder=".")
CORS(app)

# --- Global state ---
map_data    = None
latest_goal = None
goal_callbacks = []  # functions to call when a new goal is received


# --- Routes ---

@app.route("/")
def index():
    """Serve the main web UI"""
    return send_from_directory(".", "index.html")


@app.route("/map")
def get_map():
    """Serve the point cloud JSON to the browser"""
    if map_data is None:
        return jsonify({"error": "Map not loaded"}), 500
    return jsonify(map_data)


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


def register_goal_callback(fn):
    """Register a function to be called when a new goal is received"""
    goal_callbacks.append(fn)


def load_map(path):
    global map_data
    print(f"Loading map: {path}")
    with open(path, "r") as f:
        map_data = json.load(f)
    print(f"Map loaded: {map_data['metadata']['point_count']} points")


def run(host="0.0.0.0", port=5002):
    print(f"Server running at http://localhost:{port}")
    print("Open this URL in your browser.")
    app.run(host=host, port=port, debug=False, use_reloader=False)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Map visualization server")
    parser.add_argument("--map",  default="map.json", help="Path to preprocessed map JSON")
    parser.add_argument("--port", type=int, default=5002, help="Port to serve on")
    args = parser.parse_args()

    load_map(args.map)
    run(port=args.port)