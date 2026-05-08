# tools/demo_aliases.sh
# ---------------------
# Source this on the laptop AND each Pi, then a single named command per
# terminal does the right thing for the markerless / mapless two-robot
# transport demo (branch feature/depth-obstacle-avoid).
#
# On every machine:
#     source /opt/ros/humble/setup.bash
#     source ~/ros2_ws/install/setup.bash
#     source ~/ros2_ws/src/swerve_transport_project/tools/demo_aliases.sh
#                       ^^ adjust the path to wherever you cloned the repo
#
# Then per-terminal:
#     pi1 (tb3_0, follower): demo_follower
#     pi2 (tb3_1, leader)  : demo_leader
#     laptop (preflight)   : demo_preflight
#     laptop (run demo)    : demo_run     # 2.5 m forward, then stops
#     laptop (replay)      : demo_replay  # re-arms the goal driver
#     laptop (dry run)     : demo_dryrun  # fake camera, no robots
#     anywhere (kill all)  : demo_estop
#
# Tweak the env vars at the top to match your network and physical layout.

# ── per-deployment config (edit me) ───────────────────────────────────────

export DEMO_LEADER_ID="${DEMO_LEADER_ID:-tb3_1}"
export DEMO_FOLLOWER_ID="${DEMO_FOLLOWER_ID:-tb3_0}"

# Measured camera mount on tb3_1 (per CLAUDE.md). Re-measure for tb3_0
# if you swap the leader role.
export DEMO_CAM_X="${DEMO_CAM_X:-0.128}"
export DEMO_CAM_Y="${DEMO_CAM_Y:-0.000}"
export DEMO_CAM_Z="${DEMO_CAM_Z:--0.0175}"

# Formation geometry. tb3_1 (leader) on right, tb3_0 (follower) on left,
# 0.5 m centre-to-centre. These are the offsets verified in
# feature/two-robot-test-seven HANDOFF_TO_TAN.md §6.
export DEMO_LEADER_OFFSET="${DEMO_LEADER_OFFSET:-0.0,-0.25}"
export DEMO_FOLLOWER_OFFSET="${DEMO_FOLLOWER_OFFSET:-0.0,0.25}"

# Demo behaviour.
export DEMO_GOAL_M="${DEMO_GOAL_M:-2.5}"
export DEMO_FORWARD_SPEED="${DEMO_FORWARD_SPEED:-0.10}"
export DEMO_AVOID_RANGE_MM="${DEMO_AVOID_RANGE_MM:-1200}"
export DEMO_LATERAL_GAIN="${DEMO_LATERAL_GAIN:-0.10}"

# OpenCR USB device (override per-Pi if yours enumerates differently).
export DEMO_USB_PORT="${DEMO_USB_PORT:-/dev/ttyACM0}"

# ── helper: apply networking env (Pi vs laptop) ───────────────────────────

_demo_net_env() {
    unset RMW_IMPLEMENTATION RMW_ZENOH_CONFIG_FILE
    export ROS_DOMAIN_ID=30
    if [[ -z "${FASTRTPS_DEFAULT_PROFILES_FILE:-}" ]]; then
        # Pick the first existing peers file we can find.
        for cand in \
            "/home/$USER/fastdds_peers.xml" \
            "$HOME/fastdds_peers.xml" \
            "/home/pi1/fastdds_peers.xml" \
            "/home/pi2/fastdds_peers.xml" \
            "/home/tankikun/fastdds_peers.xml" \
            "/home/toodmuk/fastdds_peers.xml" \
        ; do
            if [[ -r "$cand" ]]; then
                export FASTRTPS_DEFAULT_PROFILES_FILE="$cand"
                break
            fi
        done
    fi
    echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID  FASTRTPS_DEFAULT_PROFILES_FILE=${FASTRTPS_DEFAULT_PROFILES_FILE:-(unset)}"
}

# ── per-Pi launches ───────────────────────────────────────────────────────

demo_leader() {
    _demo_net_env
    echo "→ Launching LEADER ${DEMO_LEADER_ID} on $(hostname)…"
    ros2 launch swerve_bringup demo_robot.launch.py \
        robot_id:="${DEMO_LEADER_ID}" is_leader:=true \
        my_offset:="${DEMO_LEADER_OFFSET}" \
        neighbors:="${DEMO_FOLLOWER_ID}" \
        neighbor_offsets:="${DEMO_FOLLOWER_OFFSET}" \
        usb_port:="${DEMO_USB_PORT}" \
        cam_x:="${DEMO_CAM_X}" cam_y:="${DEMO_CAM_Y}" cam_z:="${DEMO_CAM_Z}" \
        avoid_range_mm:="${DEMO_AVOID_RANGE_MM}" \
        lateral_gain:="${DEMO_LATERAL_GAIN}"
}

demo_follower() {
    _demo_net_env
    echo "→ Launching FOLLOWER ${DEMO_FOLLOWER_ID} on $(hostname)…"
    ros2 launch swerve_bringup demo_robot.launch.py \
        robot_id:="${DEMO_FOLLOWER_ID}" is_leader:=false \
        my_offset:="${DEMO_FOLLOWER_OFFSET}" \
        neighbors:="${DEMO_LEADER_ID}" \
        neighbor_offsets:="${DEMO_LEADER_OFFSET}" \
        usb_port:="${DEMO_USB_PORT}"
}

# ── laptop commands ───────────────────────────────────────────────────────

demo_preflight() {
    _demo_net_env
    local repo_root
    repo_root="$(_demo_repo_root)"
    LEADER="${DEMO_LEADER_ID}" FOLLOWER="${DEMO_FOLLOWER_ID}" \
        bash "${repo_root}/tools/preflight.sh"
}

demo_run() {
    _demo_net_env
    echo "→ Resetting odom on both robots…"
    ros2 service call "/${DEMO_LEADER_ID}/reset_odom"   std_srvs/srv/Trigger \
        > /dev/null
    ros2 service call "/${DEMO_FOLLOWER_ID}/reset_odom" std_srvs/srv/Trigger \
        > /dev/null
    sleep 0.3
    echo "→ Driving formation forward ${DEMO_GOAL_M} m at ${DEMO_FORWARD_SPEED} m/s…"
    ros2 run swerve_formation goal_driver_node --ros-args \
        -p leader_robot_id:="${DEMO_LEADER_ID}" \
        -p goal_distance_m:="${DEMO_GOAL_M}" \
        -p forward_speed:="${DEMO_FORWARD_SPEED}" \
        -p ramp_up_s:=1.0 \
        -p start_immediately:=true
}

demo_replay() {
    _demo_net_env
    ros2 service call /goal_driver/start std_srvs/srv/Trigger
}

demo_estop() {
    # Publish zeros for half a second on every command topic. The
    # OpenCR's 5-second watchdog is the safety backstop if this misses.
    _demo_net_env
    for topic in \
        "/virtual_center/cmd_vel/raw" \
        "/virtual_center/cmd_vel" \
        "/${DEMO_LEADER_ID}/cmd_vel" \
        "/${DEMO_FOLLOWER_ID}/cmd_vel" \
    ; do
        ros2 topic pub --rate 20 --times 10 "$topic" geometry_msgs/Twist \
            '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' \
            > /dev/null &
    done
    wait
    echo "ESTOP: zeros published. OpenCR watchdog is the backstop."
}

# ── laptop dry-run (no robots needed) ─────────────────────────────────────

demo_dryrun() {
    _demo_net_env
    echo "→ Laptop dry-run: fake camera + obstacle_avoidance + a forward Twist."
    echo "  Watch /obstacle_avoidance/state in another terminal:"
    echo "    ros2 topic echo /obstacle_avoidance/state"
    echo "  And the modified twist:"
    echo "    ros2 topic echo /virtual_center/cmd_vel"
    echo
    # Background fake depth + obstacle nodes; foreground a steady forward Twist.
    ros2 run swerve_formation fake_depth_publisher_node --ros-args \
        -p leader_robot_id:="${DEMO_LEADER_ID}" -p scenario:=sweep &
    fake_pid=$!
    ros2 run swerve_formation obstacle_avoidance_node --ros-args \
        -p leader_robot_id:="${DEMO_LEADER_ID}" \
        -p avoid_range_mm:="${DEMO_AVOID_RANGE_MM}" \
        -p lateral_gain:="${DEMO_LATERAL_GAIN}" &
    avoid_pid=$!
    trap "kill $fake_pid $avoid_pid 2>/dev/null; trap - INT TERM EXIT" INT TERM EXIT
    sleep 1.5
    echo "  (Ctrl-C to stop. Publishing forward 0.10 m/s on /virtual_center/cmd_vel/raw…)"
    ros2 topic pub --rate 10 /virtual_center/cmd_vel/raw geometry_msgs/Twist \
        '{linear: {x: 0.10, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
}

# ── meta ──────────────────────────────────────────────────────────────────

_demo_repo_root() {
    # The directory this aliases file lives in is <repo>/tools.
    local self="${BASH_SOURCE[0]}"
    cd "$(dirname "$self")/.." 2>/dev/null && pwd
}

demo_help() {
    cat <<EOF
swerve transport demo aliases — feature/depth-obstacle-avoid

ON THE LEADER PI (tb3_1, where the OAK-D is):
    demo_leader

ON THE FOLLOWER PI (tb3_0):
    demo_follower

ON THE LAPTOP:
    demo_preflight    Run all sanity checks. Green = safe to run demo.
    demo_run          Reset odom on both robots, drive ${DEMO_GOAL_M} m forward.
    demo_replay       Re-arm goal driver for another run.
    demo_estop        Publish zeros (kill switch), OpenCR watchdog backs up.
    demo_dryrun       Fake camera + obstacle node, no robots.
                      Watch /obstacle_avoidance/state in another shell.

CONFIG (env vars, set before sourcing this file):
    DEMO_LEADER_ID         (= ${DEMO_LEADER_ID})
    DEMO_FOLLOWER_ID       (= ${DEMO_FOLLOWER_ID})
    DEMO_LEADER_OFFSET     (= ${DEMO_LEADER_OFFSET})
    DEMO_FOLLOWER_OFFSET   (= ${DEMO_FOLLOWER_OFFSET})
    DEMO_GOAL_M            (= ${DEMO_GOAL_M})
    DEMO_FORWARD_SPEED     (= ${DEMO_FORWARD_SPEED})
    DEMO_AVOID_RANGE_MM    (= ${DEMO_AVOID_RANGE_MM})
    DEMO_LATERAL_GAIN      (= ${DEMO_LATERAL_GAIN})
    DEMO_USB_PORT          (= ${DEMO_USB_PORT})
    DEMO_CAM_X / Y / Z     (= ${DEMO_CAM_X}, ${DEMO_CAM_Y}, ${DEMO_CAM_Z})
EOF
}

# Print a brief banner on source so the user knows the aliases loaded.
echo "swerve_demo aliases loaded — try: demo_help"
