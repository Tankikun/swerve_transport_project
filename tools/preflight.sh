#!/bin/bash
# tools/preflight.sh
# ------------------
# Run on the LAPTOP after both Pi launches have come up. Verifies
# every link in the demo's topic chain before you hit "go".
#
# Usage:
#   bash tools/preflight.sh                  # defaults: leader=tb3_1, follower=tb3_0
#   LEADER=tb3_1 FOLLOWER=tb3_0 bash tools/preflight.sh
#
# Exit code 0 = green light to start the demo.
# Exit code 1 = something is wrong; fix it before running goal_driver.

set -u

LEADER="${LEADER:-tb3_1}"
FOLLOWER="${FOLLOWER:-tb3_0}"
HZ_TIMEOUT=5      # seconds to sample each /hz check
MIN_CMD_HZ=15     # per-robot cmd_vel rate (laplacian publishes at 20)
MIN_DEPTH_HZ=10   # depth image rate (depthai_ros_driver targets 15)

GREEN='\033[0;32m'
RED='\033[0;31m'
YEL='\033[0;33m'
NC='\033[0m'
fail=0

# ── helpers ────────────────────────────────────────────────────────────────

ok()    { printf "${GREEN}OK    ${NC}  %s\n" "$1"; }
bad()   { printf "${RED}FAIL  ${NC}  %s\n" "$1"; fail=$((fail + 1)); }
warn()  { printf "${YEL}WARN  ${NC}  %s\n" "$1"; }
note()  { printf "        %s\n" "$1"; }

# Parse the integer Hz value from `ros2 topic hz` output. Returns 0 on no
# data (timeout). Captures stderr because ros2 prints the rate there.
sample_hz() {
    local topic="$1"
    local out
    out=$(timeout "${HZ_TIMEOUT}" ros2 topic hz "$topic" 2>&1 | grep -m1 'average rate' || true)
    if [[ -z "$out" ]]; then
        echo 0
        return
    fi
    # average rate: 14.876
    echo "$out" | awk -F'[: ]+' '/average rate/ { print int($4 + 0.5) }'
}

require_topic() {
    local topic="$1"
    local pubs
    pubs=$(ros2 topic info "$topic" 2>/dev/null | awk -F': ' '/Publisher count/ {print $2}')
    if [[ -z "$pubs" ]] || [[ "$pubs" == "0" ]]; then
        bad "topic $topic has no publisher"
        return 1
    fi
    ok  "topic $topic has $pubs publisher(s)"
}

require_topic_hz() {
    local topic="$1"
    local min="$2"
    local hz
    hz=$(sample_hz "$topic")
    if (( hz >= min )); then
        ok "topic $topic ${hz}Hz (≥ ${min})"
    else
        bad "topic $topic ${hz}Hz (< ${min})"
    fi
}

# ── checks ─────────────────────────────────────────────────────────────────

echo "── ROS environment ───────────────────────────────────────────────"
[[ -n "${ROS_DISTRO:-}" ]] && ok "ROS_DISTRO=$ROS_DISTRO" || bad "ROS_DISTRO not set — did you source /opt/ros/humble/setup.bash?"
ok "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-(unset, defaults to 0)}"
[[ "${ROS_DOMAIN_ID:-0}" != "30" ]] && warn "ROS_DOMAIN_ID is not 30 — Pi launches use 30"
[[ -n "${FASTRTPS_DEFAULT_PROFILES_FILE:-}" ]] && ok "FASTRTPS_DEFAULT_PROFILES_FILE=$FASTRTPS_DEFAULT_PROFILES_FILE" || warn "FASTRTPS_DEFAULT_PROFILES_FILE unset — discovery may fail"

echo
echo "── Robot nodes alive ─────────────────────────────────────────────"
nodes=$(ros2 node list 2>/dev/null)
for needed in \
    "/conveyor_base_node_${LEADER}" \
    "/conveyor_base_node_${FOLLOWER}" \
    "/laplacian_formation_node_${LEADER}" \
    "/laplacian_formation_node_${FOLLOWER}" \
    "/obstacle_avoidance_node_${LEADER}" \
; do
    if grep -qx "$needed" <<<"$nodes"; then
        ok  "node $needed visible"
    else
        bad "node $needed NOT visible"
    fi
done

echo
echo "── Topic publishers ──────────────────────────────────────────────"
require_topic "/${LEADER}/odom"
require_topic "/${FOLLOWER}/odom"
require_topic "/${LEADER}/camera/depth/image_raw"
require_topic "/${LEADER}/camera/rgb/camera_info"
require_topic "/virtual_center/cmd_vel"
require_topic "/${LEADER}/cmd_vel"
require_topic "/${FOLLOWER}/cmd_vel"

echo
echo "── Topic rates (this takes ~${HZ_TIMEOUT}s × 4) ──────────────────"
require_topic_hz "/${LEADER}/cmd_vel"               "$MIN_CMD_HZ"
require_topic_hz "/${FOLLOWER}/cmd_vel"             "$MIN_CMD_HZ"
require_topic_hz "/${LEADER}/camera/depth/image_raw" "$MIN_DEPTH_HZ"

echo
echo "── Reset odom services ───────────────────────────────────────────"
for r in "$LEADER" "$FOLLOWER"; do
    if ros2 service list 2>/dev/null | grep -qx "/${r}/reset_odom"; then
        ok  "service /${r}/reset_odom available"
    else
        bad "service /${r}/reset_odom NOT available"
    fi
done

echo
echo "── Obstacle-avoidance state ──────────────────────────────────────"
state_line=$(timeout 2 ros2 topic echo --once /obstacle_avoidance/state 2>/dev/null || true)
if [[ -n "$state_line" ]]; then
    ok "$(echo "$state_line" | grep -E 'data:' | head -1 | sed 's/^.*data: //')"
else
    bad "/obstacle_avoidance/state silent — obstacle node may not be subscribed"
fi

echo
if (( fail == 0 )); then
    printf "${GREEN}── ALL GREEN — safe to run goal_driver_node. ─────────────────────${NC}\n"
    exit 0
else
    printf "${RED}── %d CHECK(S) FAILED — fix before running the demo. ────────────${NC}\n" "$fail"
    exit 1
fi
