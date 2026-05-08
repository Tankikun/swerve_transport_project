# Markerless Two-Robot Transport Demo — Runbook

**Branch:** `feature/depth-obstacle-avoid`
**Approach:** Proven feedforward Laplacian formation control + depth-based
obstacle avoidance on the leader. **No markers, no map, no SLAM, no EKF,
no leader election.**
**Tested target:** carry a payload ~2.5 m straight ahead, swerve around
one obstacle in the path, stop at goal distance.

---

## What this demo is (and isn't)

**It is** the simplest possible end-to-end pipeline that exercises both
robots cooperatively transporting a payload in formation while reacting
to a sensor:

```
[goal_driver_node (laptop)]                    /virtual_center/cmd_vel/raw
              │
              ▼
[obstacle_avoidance_node (leader Pi)] ◄────── /{leader}/camera/depth/image_raw
              │
              ▼  /virtual_center/cmd_vel  (modified Twist)
              │
[laplacian_formation_node ×2]  pure FF, consensus OFF  (proven on floor)
              │
              ▼  /{robot_id}/cmd_vel
              │
[conveyor_base_node ×2] ──serial──► OpenCR ──► swerve motors
```

**It is not** a self-localising mobile platform. There is no `map` frame,
no shared world coordinates, no global goal pose. The leader stops when
its own encoder odometry has integrated to the configured `goal_distance_m`.
That's enough for a 2-3 m straight-with-detour run; longer runs would need
SLAM back.

**Components reused as-is:**
- `conveyor_base_node` (lifecycle serial bridge — proven)
- `laplacian_formation_node` in pure-FF mode (proven on floor —
  `feature/two-robot-test-seven` HANDOFF_TO_TAN.md §4)
- `oak_camera.launch.py` (depthai_ros_driver Camera component —
  current main)

**Components added on this branch:**
- `obstacle_avoidance_node` — depth-image swathe → modified twist
- `goal_driver_node` — odometry-integrating forward driver
- `demo_robot.launch.py` — minimal per-robot launch

---

## Prerequisites

* Both Pis have the workspace at `~/ros2_ws` and have already built
  `swerve_formation` + `swerve_bringup` at least once.
* OpenCR firmware on each robot is the encoder-odometry build (the
  one that emits `POSE x y theta vx vy wz` at ~33 Hz). This is the
  current `opencr_firmware/swerve_kinematics/turtlebot3_conveyor.ino`
  on `main`.
* OAK-D Lite physically on the leader robot, connected via USB 3.
  We default to **tb3_1** as leader because that's where the camera
  is confirmed working in CLAUDE.md's "Camera Mount TF" section
  (cam_x=0.128, cam_y=0, cam_z=-0.0175).
* Both Pis discover each other on `ROS_DOMAIN_ID=30` over FastDDS
  with the existing `~/fastdds_peers.xml` setup. Same network as
  your previous formation tests.

---

## Deploy

From the laptop, in the repo root:

```bash
git checkout feature/depth-obstacle-avoid

# Sync code to both Pis (adjust IPs to yours)
rsync -av --exclude='__pycache__' ros2_ws/src/ pi1@192.168.1.101:~/ros2_ws/src/
rsync -av --exclude='__pycache__' ros2_ws/src/ pi2@192.168.1.102:~/ros2_ws/src/

# Build on each Pi
ssh pi1@192.168.1.101 'cd ~/ros2_ws && source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install --packages-select swerve_formation swerve_bringup'
ssh pi2@192.168.1.102 'cd ~/ros2_ws && source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install --packages-select swerve_formation swerve_bringup'
```

(The `--symlink-install` means future Python edits won't need a rebuild;
edits to `setup.py` or new YAMLs do.)

---

## Run the demo

### 1. Place the robots

* Both fronts pointing the same direction (the goal direction).
* tb3_0 on the LEFT (per `my_offset:=0.0,0.25`).
* tb3_1 on the RIGHT (per `my_offset:=0.0,-0.25`). **Leader.**
* About **0.5 m centre-to-centre.**
* Place the payload across both robot tops.
* Place an obstacle (cardboard box, ~30 cm tall) ~1.5 m ahead of the
  formation, offset ~30 cm to one side. The leader must see it in
  the front swathe of its OAK-D.

### 2. Start the follower (tb3_0) — terminal 1, SSH'd to pi1

```bash
ssh pi1@192.168.1.101         # password: raspberry
unset RMW_IMPLEMENTATION RMW_ZENOH_CONFIG_FILE
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=30
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/pi1/fastdds_peers.xml

ros2 launch swerve_bringup demo_robot.launch.py \
    robot_id:=tb3_0 is_leader:=false \
    my_offset:=0.0,0.25 neighbors:=tb3_1 neighbor_offsets:=0.0,-0.25 \
    usb_port:=/dev/ttyACM0
```

Wait for:

```
[conveyor_base_node_tb3_0]: ConveyorBaseNode activated — forwarding commands to OpenCR
[laplacian_formation_node_tb3_0]: laplacian_formation_node ready: id=tb3_0 ...
```

### 3. Start the leader (tb3_1) — terminal 2, SSH'd to pi2

```bash
ssh pi2@192.168.1.102
unset RMW_IMPLEMENTATION RMW_ZENOH_CONFIG_FILE
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=30
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/pi2/fastdds_peers.xml

ros2 launch swerve_bringup demo_robot.launch.py \
    robot_id:=tb3_1 is_leader:=true \
    my_offset:=0.0,-0.25 neighbors:=tb3_0 neighbor_offsets:=0.0,0.25 \
    usb_port:=/dev/ttyACM0 \
    cam_x:=0.128 cam_y:=0.0 cam_z:=-0.0175
```

Wait for the conveyor + laplacian lines AND the camera signals (per
the depthai_ros_driver migration commit):

```
[component_container]: Loaded component into 'oak'
[obstacle_avoidance_node_tb3_1]: obstacle_avoidance_node ready  leader=tb3_1 ...
```

Quick camera sanity check from the laptop:

```bash
ros2 topic hz /tb3_1/camera/depth/image_raw       # ≈ 15 Hz
ros2 topic echo /obstacle_avoidance/state         # one line per cycle
```

### 4. Start the goal driver — terminal 3, on the laptop

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=30
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/tankikun/fastdds_peers.xml

# Reset both robots' integrated odometry first
ros2 service call /tb3_0/reset_odom std_srvs/srv/Trigger
ros2 service call /tb3_1/reset_odom std_srvs/srv/Trigger

# Drive the formation forward 2.5 m at 0.10 m/s (steady state).
ros2 run swerve_formation goal_driver_node --ros-args \
    -p leader_robot_id:=tb3_1 \
    -p goal_distance_m:=2.5 \
    -p forward_speed:=0.10 \
    -p ramp_up_s:=1.0 \
    -p start_immediately:=true
```

The formation should:
1. Ramp from rest to 0.10 m/s over the first second.
2. Move forward in formation. Both robots' wheel speeds match (modulo
   the rigid-body offset terms; for a pure forward command those are
   zero, so identical wheel speeds).
3. As the leader's depth swathe sees the obstacle:
   - `/obstacle_avoidance/state` flips from `clear …` to
     `AVOID closest=… col_u=… push_y=…`.
   - The formation's `linear.y` jumps by ±0.10 m/s and `linear.x` is
     attenuated. Both robots strafe sideways together.
4. Once past the obstacle, the swathe goes clear again and the
   command falls back to pure forward.
5. When `goal_driver_node` reports `REACHED — distance=2.51 m`, it
   publishes a zero Twist. The OpenCR's 5-second watchdog will also
   zero motors if the goal driver dies for any reason.

---

## Verifying it's working from the laptop

In yet another terminal:

```bash
ros2 topic echo /virtual_center/cmd_vel/raw    # what goal_driver puts in
ros2 topic echo /virtual_center/cmd_vel        # what obstacle_avoidance puts out
ros2 topic echo /tb3_0/cmd_vel                 # what laplacian gives tb3_0
ros2 topic echo /tb3_1/cmd_vel                 # what laplacian gives tb3_1
ros2 topic echo /obstacle_avoidance/state      # human-readable state
ros2 topic hz   /obstacle_avoidance/closest_m  # ≈ 15 Hz when seeing something
```

For a quick "is the formation rigid?" sanity check during a forward
command (linear.x only, no avoidance), `/tb3_0/cmd_vel` and
`/tb3_1/cmd_vel` should match exactly. With a non-zero `angular.z` they
differ symmetrically.

---

## Tuning knobs

If the formation panics (jumps too aggressively) or hits the obstacle
(too timid), tune these on `obstacle_avoidance_node`. Pass via
`-p name:=value` at launch, or `ros2 param set` at runtime.

| Parameter | Default | Effect |
|---|---|---|
| `avoid_range_mm` | 1200 | Distance at which avoidance kicks in. Lower = let the obstacle get closer before reacting. |
| `lateral_gain` | 0.10 | Peak lateral velocity (m/s) of the avoidance push. Higher = swerve harder. Stay below `MAX_WHEEL_LINEAR=0.18`. |
| `speed_scale_floor` | 0.40 | Forward velocity at urgency=1. 0 = stop completely when very close; 1 = don't slow down at all. |
| `swathe_top_frac` | 0.40 | Top of the depth-image vertical band. Increase to ignore higher obstacles (overhead pipes, etc.). |
| `swathe_bot_frac` | 0.65 | Bottom of the band. Decrease to ignore the floor closer in. |
| `min_valid_mm` | 250 | Reject depths closer than this — the OAK-D Lite stereo baseline can't measure < 25 cm reliably. |

---

## Failure modes & what to do

* **Formation drifts apart.** Check that both robots received the same
  `/virtual_center/cmd_vel` message — `ros2 topic echo` on each Pi.
  If only one is receiving, FastDDS peers are mis-configured. Verify
  `FASTRTPS_DEFAULT_PROFILES_FILE` is set on both Pis and includes
  every participant's IP.

* **Camera publishes but `/obstacle_avoidance/state` says
  `depth-stale`.** Topic name mismatch. Confirm:
  `ros2 topic info /tb3_1/camera/depth/image_raw` shows ≥ 1 publisher
  and ≥ 1 subscriber. If 0 subscribers, the obstacle node didn't start
  — check terminal 2 for an error.

* **Robot goes the wrong way around the obstacle.** The `col_u → push_y`
  sign assumes the camera's optical-from-body rotation in
  `oak_camera.launch.py` (roll = pitch = -π/2). If the camera is
  mounted upside-down or rotated, the lateral push will go the wrong
  way. Either physically reorient the camera or invert the sign of
  `lateral_gain`.

* **Formation slows but never swerves.** `avoid_range_mm` may be too
  high, or the obstacle is in the centre of the swathe (col_u ≈ 0)
  so push_dir collapses. Move the obstacle slightly off-axis at
  start or set a small initial bias on `linear.y` in `goal_driver_node`.

* **OpenCR watchdog kicks in mid-run.** The 5 s watchdog zeroes motors
  if no command arrives. Check `ros2 topic hz /tb3_*/cmd_vel`; the
  `laplacian_formation_node` runs at 20 Hz so this should always be
  fine. If it isn't, suspect WiFi loss between leader and follower.

* **Pi 4 thermal throttle.** Without RTAB-Map running, the Pi sits at
  ~50 °C. If yours runs hot, `vcgencmd measure_temp` and consider
  lowering camera fps to 10.

---

## Aborting safely

* Easiest: kill the goal driver (`Ctrl+C` on terminal 3). Both robots
  receive a zero Twist within 50 ms; OpenCR watchdog backs that up.
* If the formation goes off the rails: `Ctrl+C` either pi launch.
  The conveyor_base_node sends a zero on shutdown, and the OpenCR
  watchdog zeroes after 5 s as a safety backstop.
* Hardware E-stop: the OpenCR power switch.

---

## After a successful run

```bash
# Optional: capture a rosbag for the demo video
ros2 bag record \
    /virtual_center/cmd_vel/raw \
    /virtual_center/cmd_vel \
    /tb3_0/cmd_vel /tb3_1/cmd_vel \
    /tb3_0/odom    /tb3_1/odom \
    /obstacle_avoidance/state /obstacle_avoidance/closest_m
```

To re-run, repeat steps 4 (or call the service):

```bash
ros2 service call /goal_driver/start std_srvs/srv/Trigger
```

---

## Files added on `feature/depth-obstacle-avoid`

```
ros2_ws/src/swerve_formation/swerve_formation/obstacle_avoidance_node.py
ros2_ws/src/swerve_formation/swerve_formation/goal_driver_node.py
ros2_ws/src/swerve_formation/setup.py             (entry_points updated)
ros2_ws/src/swerve_bringup/launch/demo_robot.launch.py
MARKERLESS_DEMO_RUNBOOK.md                         (this file)
INVESTIGATION.md                                   (Agent B's reuse/drop/risk report)
```

Nothing on the existing main branch is modified or removed. The full
SLAM/EKF/leader-election stack still works via `conveyor.launch.py` if
we ever want it back.
