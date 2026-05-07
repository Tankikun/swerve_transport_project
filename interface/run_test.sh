#!/bin/bash
# run_test.sh — Mac-side end-to-end navigation test launcher.
#
# Starts:
#   * mock_ros_bridge.py  on ws://localhost:9090   (rosbridge stand-in)
#   * server.py           on http://localhost:5002 (web UI)
#
# Browse to http://localhost:5002, click on the 2D map, watch the robot move.
# Ctrl-C stops both processes.

set -e
cd "$(dirname "$0")"

PY=/usr/bin/python3
MAP=${1:-map.json}

if [ ! -f "$MAP" ]; then
  echo "ERROR: $MAP not found in $(pwd)" >&2
  exit 1
fi

# Free the ports if anything's still bound from a previous run
lsof -ti:5002 | xargs kill 2>/dev/null || true
lsof -ti:9090 | xargs kill 2>/dev/null || true
sleep 1

cleanup() {
  echo
  echo "[run_test] stopping..."
  if [ -n "$BRIDGE_PID" ]; then kill $BRIDGE_PID 2>/dev/null || true; fi
  if [ -n "$SERVER_PID" ]; then kill $SERVER_PID 2>/dev/null || true; fi
  exit 0
}
trap cleanup INT TERM

echo "[run_test] starting mock_ros_bridge (ws://localhost:9090)..."
$PY mock_ros_bridge.py --map "$MAP" --port 9090 &
BRIDGE_PID=$!

echo "[run_test] starting server (http://localhost:5002)..."
$PY server.py --map "$MAP" --port 5002 &
SERVER_PID=$!

sleep 2
echo
echo "================================================================"
echo "  ✅  All services up."
echo "      Web UI:        http://localhost:5002"
echo "      ROS bridge:    ws://localhost:9090"
echo "      Loaded map:    $MAP"
echo
echo "  Click anywhere on the 2D map (green floor) to send a goal."
echo "  The robot marker will appear and move toward the goal,"
echo "  curving around obstacles via APF + velocity ramp."
echo
echo "  Press Ctrl-C to stop."
echo "================================================================"
echo

wait
