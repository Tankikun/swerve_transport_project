# Agent B Investigation Report — Swerve Transport Reuse/Drop/Risk Analysis

**Branch examined:** `interface/v3-offline-server`
**Date:** 2026-05-08
**Author:** Agent B (Explore subagent)
**Scope:** Deep read of existing 15-node ROS 2 stack to identify reusable components and concrete failure modes for the new ArUco leader-follower approach.

---

## 1. Reuse List (Verbatim or Trivial Edits)

### opencr_firmware/swerve_kinematics/turtlebot3_conveyor.ino
**Path:** `opencr_firmware/swerve_kinematics/turtlebot3_conveyor.ino` (280+ lines)
**Serial protocol (lines 6–35):**
- **Receive:** `"x_dot y_dot gamma_dot\n"` (floats: m/s, m/s, rad/s)
- **Reply:** `"OK d:δ0,δ1,δ2,δ3 w:ω0,ω1,ω2,ω3\n"`
- **Odometry:** `"POSE x y theta vx vy wz\n"` at ~33 Hz (ODOM_DIV=1, line 92)
- **Reset:** `"R\n"` zeroes odometry
- **Watchdog:** CMD_TIMEOUT_MS = 5000 ms (line 66)

**Key feature:** Encoder-based FK (line 163–188) reads measured motor positions and velocities, not commanded values. Falls back to commanded FK on read failure. This eliminates phantom odometry when wheels free-spin on boxes.

**Status:** Keep firmware as-is. Serial protocol is final.

---

### ros2_ws/src/swerve_formation/swerve_formation/conveyor_base_node.py
**Subscriptions:** `/{robot_id}/cmd_vel` (Twist, line 135–137)
**Publications:** `/{robot_id}/odom` (Odometry, line 138); TF `{robot_id}_odom → {robot_id}_base_link` (line 139)
**Lifecycle behavior (lines 112–189):**
- **on_configure:** opens serial at 115200 baud, creates timers (50 Hz read at line 146, 5 Hz watchdog)
- **on_activate:** enables forwarding, logs `"ConveyorBaseNode activated — forwarding commands to OpenCR"`
- **on_deactivate:** sends zero velocity
- **Reset service:** `/{robot_id}/reset_odom` sends `'R\n'` to firmware (line 151–152)

**POSE parsing:** Serial read at line 237–238 delegates to `_handle_pose(line)` (parses "POSE x y theta vx vy wz").

**Status:** Reuse verbatim. Namespace-safe by design. Works for both robot1 and robot2 with different `robot_id` parameters.

---

### ros2_ws/src/swerve_formation/swerve_formation/oak_camera_node.py
**Published topics (lines 94–107):**
- `/{robot_id}/camera/rgb/image_raw` (Image, BGR8, 640×400@15 Hz, line 155)
- `/{robot_id}/camera/rgb/camera_info` (CameraInfo with intrinsics K, line 133–147)
- `/{robot_id}/camera/depth/image_raw` (Image, 16UC1 mm, aligned to RGB if `use_stereo_align=true`, line 171)
- `/{robot_id}/camera/depth/camera_info` (CameraInfo, aligned frame if depth aligned, line 98–101)

**Frame IDs:**
- RGB: `{robot_id}_oak_rgb_camera_optical_frame` (line 97)
- Depth: same as RGB if aligned (line 98–101)

**Intrinsics delivery:** K matrix at 3×3 from `calib.getCameraIntrinsics()` (lines 243–246), packed in `camera_info.k[9]` as flattened array (line 140). **This is what aruco_tracker_node needs for solvePnP.**

**Key parameters:** `robot_id`, `fps` (default 15), `rgb_size` (default 640x400), `stereo_size` (default 640x400, **must be multiple of 16 width** per line 220), `use_stereo_align` (default True).

**Depthai 3.x quirks:** Preset names tried in order (line 208); `setOutputSize()` mandatory or stereo crashes (line 220–227); no XLinkOut nodes, queues created directly (line 235–236).

**Status:** Drop-in reuse. **Critical:** ensure aruco_tracker_node subscribes to exact topic names and caches `camera_info` for intrinsics before calling solvePnP.

---

### ros2_ws/src/swerve_bringup/launch/conveyor.launch.py
**Key pattern:** Per-robot node naming with `_{robot_id}` suffix (line 57) to prevent DDS collisions.
**Launch args:** `robot_id`, `usb_port`, `k_gain`, `neighbors`, `my_offset`, `neighbor_offsets`, `offset_init_mode`, `db_path`, `fps`, `cam_x`, `cam_y`, `cam_z`.

**Status:** Keep launch infrastructure and naming pattern. Remove SLAM/EKF/formation nodes in new branch.

---

### ros2_ws/src/swerve_bringup/launch/oak_camera.launch.py
**Brings up:** `oak_camera_node` (line 51–63) + static TF base_link→optical (line 72–83).
**Static TF params:** cam_x/y/z translation; rotation roll=−π/2, pitch=0, yaw=−π/2.

**Status:** Reuse directly in new aruco launch files.

---

## 2. Drop List (Do Not Launch for Demo)

| Component | Why Dropping |
|-----------|------|
| EKF + RTAB-Map | SLAM relocalization is the fragile subsystem (README line 11); ArUco is vision-marker-only |
| Laplacian consensus | Requires shared world frame; EKF init fails (README line 356) |
| Leader election | Hardcode roles for two-robot pair; no need for Bully |
| Formation size / alignment | Fixed object being carried; no dynamic sizing |
| Navigation APF | Will rewrite as simpler `apf_simple_node` |
| AI camera / SLAM 3D / simulators | Not on critical path |

---

## 3. Smoking-Gun Register

| Failure | Evidence | File:Line |
|---------|----------|-----------|
| EKF init timeout | `"Offset init timeout — no valid EKF pose received"` | README.md:356 |
| Odometry drift | `"Keep runs under 30 seconds"` without SLAM correction | README.md:460 |
| POSE rate mismatch | Firmware was 3 Hz, laplacian 20 Hz → closed-loop wobble | HANDOFF_TO_TAN.md:309 |
| Phantom odometry | FK on commanded (not measured) wheel speeds when boxes free-spin | HANDOFF_TO_TAN.md:286 |
| Node name collisions | Identical `/laplacian_formation_node` on both robots breaks DDS discovery | HANDOFF_TO_TAN.md:201 |

---

## 4. Things That Worked

- **Encoder odometry + formation kinematics:** Floor-tested with keyboard teleop. Orbit and translation verified (HANDOFF_TO_TAN.md:274–286).
- **OAK-D camera:** Working on pi2, depthai 3.x stable at 15 fps (RTAB_SESSION_SUMMARY.md:81).
- **Firmware IMU:** MPU-9250 Madgwick yaw at ~11 Hz over serial (HANDOFF_TO_TAN.md:105).

---

## 5. Top-5 Risks for New ArUco Approach

1. **Camera topic naming:** `aruco_tracker_node` must subscribe to exact path `/{robot_id}/camera/rgb/image_raw` (BGR8). Frame ID mismatch → solvePnP fails silently.

2. **Missing intrinsics:** `aruco_tracker_node` **must** cache `/{robot_id}/camera/rgb/camera_info` (contains K matrix) before calling OpenCV solvePnP. Skipping this = garbage pose.

3. **Namespace safety:** All new nodes require `_{robot_id}` suffix in launch `name=` parameter. Missing this → DDS collisions → subscriptions fail. Verify: `ros2 node list` should show distinct entries per robot.

4. **USB bandwidth saturation:** OAK-D Lite @ 640×400@15fps is at USB 2.0 edge. Higher res or fps risks frame drops and thermal throttle. Monitor `vcgencmd measure_temp` during tests.

5. **Holonomic yaw saturation:** Corner wheels can saturate if follower demands high angular velocity while moving. Use low yaw gains (< 0.5 rad/s per rad error) until floor-tuned.

---

## 6. Critical Agent C Inputs

**OAK-D topics (final):**
- RGB: `/{robot_id}/camera/rgb/image_raw` (BGR8, 640×400@15Hz)
- Depth: `/{robot_id}/camera/depth/image_raw` (16UC1 mm, aligned to RGB)
- Intrinsics: `/{robot_id}/camera/rgb/camera_info` (K matrix in msg.k[9])
- Frames: `{robot_id}_oak_rgb_camera_optical_frame`

**Serial protocol:**
- Send: `"x_dot y_dot gamma_dot\n"` (m/s, m/s, rad/s)
- Recv: `"POSE x y theta vx vy wz\n"` @ 33 Hz
- Watchdog: 5000 ms (firmware command hold time)

**Namespace pattern:** Node name must include `_{robot_id}` suffix in launch file to prevent DDS collisions.

**Lifecycle:** `conveyor_base_node` is a lifecycle node; wrap with existing lifecycle manager.
