# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What This Is

A multi-robot cooperative transport system built on ROS 2 Humble. Two (scaling to 3+) holonomic swerve-drive robots carry a shared rigid payload in formation. All control is decentralized — no master node ever.

## Hardware (per robot)

- **Chassis**: TurtleBot3 Conveyor (swerve/holonomic)
- **Motors**: 8x Dynamixel XL430-W250 — 4 steering (position mode) + 4 drive (velocity mode), all on a single TTL bus via U2D2 Power Hub Board
- **Motor IDs**: drive = [7, 3, 5, 1], steering = [8, 4, 6, 2]
- **Microcontroller**: OpenCR (STM32) — runs custom swerve IK firmware in C++
- **Compute**: Raspberry Pi 4 (username `pi1`/`pi2`, workspace `~/ros2_ws`)
- **Camera**: Luxonis OAK-D Lite (RGB + stereo depth) — **only one host connection allowed at a time**; the `depthai_ros_driver` `Camera` component (loaded into the OAK container in `oak_camera.launch.py`) owns the device exclusively. Never propose code that opens a second depthai device handle.
- **Dev machine**: ROG-Strix-G513QE, Ubuntu 22.04, ROS 2 Humble, username `tankikun`

## OpenCR Firmware

- Receives `"x_dot y_dot gamma_dot\n"` over USB-CDC at 115200 baud
- Sends `"POSE x y theta vx vy wz\n"` back at ~33 Hz (current firmware) or legacy `"ODOM j:... w:..."` (old firmware — handled by the `_handle_odom_legacy` fallback path in `conveyor_base_node.py`)
- Steering constrained to `[-π/2, π/2]` with drive direction flip + angle-minimizing selection
- 500 ms watchdog: zeros drive motors on command timeout
- GroupSyncWrite at 1 Mbps (~200-300 µs per write, fits in 20 ms control loop)

## Build & Test

All commands run from `ros2_ws/` on whichever machine (Pi or laptop) you're building for.

```bash
# First build (or after adding new nodes, launch files, or config YAMLs)
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash

# Python logic changes don't need a rebuild after --symlink-install.
# The following DO require a rebuild:
#   - Adding a new entry_point in setup.py
#   - Adding a new file to data_files in setup.py (e.g. a new YAML config)

# Run linting / style tests
colcon test --packages-select swerve_formation
colcon test-result --verbose

# Run a single test file directly
cd ros2_ws/src/swerve_formation
python3 -m pytest test/test_flake8.py -v
```

Tests live in `ros2_ws/src/swerve_formation/test/` and only cover lint/style (flake8, pep257, copyright). There are no unit tests yet.

## Software Architecture (per robot)

All nodes are Python 3. Always use numpy for matrix math in control loops (Pi 4 is compute-limited).

| Node | Role |
|---|---|
| `conveyor_base_node` | Lifecycle node; serial bridge to OpenCR over USB-CDC. Publishes `/{robot_id}/odom` + TF `{robot_id}_odom → {robot_id}_base_link` |
| `ekf_node` | Extended Kalman filter: prediction from `/{robot_id}/odom`, correction from `/{robot_id}/slam/pose`. Publishes authoritative `/{robot_id}/ekf/odom`. Fixed observation noise `R = diag(0.05, 0.05, 0.02)`. **In localization (production), this is the sole consumer of raw `/odom`** — rtabmap reads `/ekf/odom` instead. During mapping, rtabmap reads raw `/odom` directly (ekf is typically not running there). |
| OAK camera (`depthai_ros_driver::Camera` component) | RGB + aligned stereo depth. Loaded into a `ComposableNodeContainer` by `oak_camera.launch.py`, configured by `swerve_bringup/config/depthai_oak_d_lite.yaml`. Owns the OAK-D USB device exclusively. Publishes `/{robot_id}/camera/rgb/image_raw`, `/{robot_id}/camera/rgb/camera_info`, `/{robot_id}/camera/depth/image_raw`, `/{robot_id}/camera/depth/camera_info`. Depth aligned to RGB optical frame. |
| `ai_camera_node` | Subscribes to published detection topics from `oak_camera_node`. Does NOT open its own depthai device — the OAK-D only allows one host connection. |
| `slam_pose_relay_node` | Type-conversion glue: converts `/{robot_id}/rtabmap/localization_pose` (`PoseWithCovarianceStamped`) → `/{robot_id}/slam/pose` (`PoseStamped`) for `ekf_node`. Covariance is dropped; `ekf_node` uses a fixed R matrix. Do not remove this node — the message types are incompatible and cannot be fixed with a remap alone. |
| `laplacian_formation_node` | Rigid-body feedforward + optional Laplacian consensus correction. Publishes `/{robot_id}/cmd_vel` and `/formation/state`. Consensus (`enable_consensus`) is **OFF by default** — see node docstring for why. |
| `leader_election_node` | Bully-inspired election over `/formation/heartbeat`. Lowest-priority active robot wins. Publishes `/formation/leader`. |
| `navigation_node` | Runs on all robots; only the elected leader activates its APF + velocity-ramp control loop. Drives `/virtual_center/cmd_vel`. |
| `formation_size_node` | Leader-only. Computes formation bounding envelope from `/formation/state` + camera object size; publishes footprint to navigation. |
| `alignment_node` | Pre-run depth-based spacing correction. Leader measures depth to payload via OAK-D central ROI, nudges all robots to equal depth, then publishes final offset PoseArray. |
| `fake_swerve_simulator` | Software-only robot for local nav testing — no hardware needed. |

**Pose data flow (localization, i.e. production runtime)**: `/{robot_id}/odom` (raw) → `ekf_node` (prediction); `ekf_node` publishes `/{robot_id}/ekf/odom` → RTAB-Map (used as a clean prior; the static `.db` prevents feedback drift). RTAB-Map → `/{robot_id}/rtabmap/localization_pose` → `slam_pose_relay_node` → `/{robot_id}/slam/pose` → `ekf_node` (correction) → `/{robot_id}/ekf/odom` (authoritative). Raw `/odom` has exactly one consumer: `ekf_node`.

**Pose data flow (mapping)**: `/{robot_id}/odom` (raw) → RTAB-Map directly. ekf is typically not running. There is no SLAM-correction loop yet — rtabmap is building the `.db`, not consulting it — so raw odom is the right input and matches the `{robot_id}_odom → base_link` TF that `conveyor_base_node` publishes.

**Command data flow**: `/virtual_center/cmd_vel` (formation centre) → `laplacian_formation_node` → `/{robot_id}/cmd_vel` → `conveyor_base_node` → OpenCR.

## Localization Stack (RTAB-Map)

RTAB-Map (`rtabmap_slam`) is used for visual localization. 3DGS is retained only as a visualization asset and plays no role at runtime.

### Mapping (done once on the laptop)

One robot is driven manually through the room. The laptop runs `rtabmap_laptop_mapping.launch.py`, which subscribes to the Pi's camera and odometry topics over the network. The output is a single `.db` file, e.g. `~/maps/lab.db`. This file is then copied to every Pi at `/home/pi1/maps/lab.db` and `/home/pi2/maps/lab.db` via rsync.

```bash
# Split-mode mapping (recommended — keeps Pi cool at ~40°C vs 70-80°C on-Pi)
# Terminal 1: on the Pi
ros2 launch swerve_bringup rtabmap_pi_sensors.launch.py robot_id:=tb3_1

# Terminal 2: on the laptop
ros2 launch swerve_bringup rtabmap_laptop_mapping.launch.py robot_id:=tb3_1

# After mapping, copy the .db to all Pis
rsync ~/maps/lab.db pi1@<ip>:~/maps/lab.db
rsync ~/maps/lab.db pi2@<ip>:~/maps/lab.db
```

**Mapping config** (`rtabmap_mapping.yaml`): `Mem/IncrementalMemory true`, `Kp/MaxFeatures 600`, `Mem/ImagePreDecimation 1` (full res), `Vis/MinInliers 20`, `Rtabmap/DetectionRate 1 Hz`, `RGBD/LinearUpdate 0.1 m`, `RGBD/AngularUpdate 0.1 rad`, `ProximityBySpace true`, `ProximityByTime true`, `Optimizer/Strategy 1` (g2o), `Optimizer/Robust true`.

### Localization (runtime, per robot)

Each Pi runs an independent RTAB-Map instance in localization-only mode against the shared `.db`. Robots share only fused EKF poses over the ROS 2 middleware — map data is never shared at runtime.

**Localization config** (`rtabmap_localization.yaml`): `Mem/IncrementalMemory false`, `Mem/InitWMWithAllNodes true`, `Mem/ImagePreDecimation 2`, `Kp/MaxFeatures 400`, `Rtabmap/DetectionRate 5 Hz`, `Reg/Force3DoF true` (flat floor assumption), `Vis/EstimationType 1` (PnP), `Vis/MinInliers 15`, `ProximityBySpace false`, `ProximityByTime false` (saves CPU).

**Feature detector**: GFTT + ORB (`Kp/DetectorStrategy 6`). ORB is used instead of BRIEF because the holonomic swerve drive can execute fast in-place rotations that break BRIEF's non-rotation-invariant descriptors.

**Initial localization delay**: RTAB-Map scans the entire stored map on startup looking for a visual match. Until it finds one (5–30 seconds), no `/rtabmap/localization_pose` is published and `ekf_node` falls back to pure dead-reckoning. Drop the robot in an area with visual variety — blank white walls will fail.

**RTAB-Map output topics** are remapped per-robot to avoid collisions when two robots run simultaneously:
- `localization_pose` → `/{robot_id}/rtabmap/localization_pose`
- `info` → `/{robot_id}/rtabmap/info`
- `cloud_map` → `/{robot_id}/rtabmap/cloud_map`

### Camera Configuration (OAK-D Lite)

- **Resolution**: RGB streams at 960×540 (sensor 1080P with ISP scale 1/2; the IMX214 sensor does not support a native 720P/540P mode). The ISP downscale runs on the OAK's VPU at zero Pi CPU cost. Stereo streams at 640×400 native (OAK-D Lite mono cameras). Width must be a multiple of 16 — both 960 and 640 satisfy this. RTAB-Map further halves the effective processing resolution to 480×270 via `Mem/ImagePreDecimation 2` in the localization config.
- **Frame rate**: 15 Hz. This gives RTAB-Map (running at 5 Hz) fresh frames with margin and reduces motion blur during fast swerve rotations.
- **Depth alignment**: depth is aligned to the RGB optical frame (`stereo.i_align_depth: true`). This is required for RTAB-Map's RGBD subscription — do not disable it.
- **USB**: USB 3 required for sustained 15 fps streaming. Configured explicitly via `camera.i_usb_speed: "SUPER"` so init fails loudly if the cable/port negotiates USB 2.

### Camera Mount TF

A static TF must be published from `{robot_id}_base_link` to `{robot_id}_oak_rgb_camera_optical_frame`. `base_link` is defined at the top of the chassis (where the payload rests). Rotation is the standard ROS optical-from-body convention: `roll=-π/2, pitch=0, yaw=-π/2`.

Translation lives in a per-robot table — `_CAMERA_MOUNT` at the top of `swerve_bringup/launch/oak_camera.launch.py`. Currently:

| robot_id | cam_x [m] | cam_y [m] | cam_z [m] |
|---|---|---|---|
| `tb3_0` | +0.128 | 0.000 | -0.0175 |
| `tb3_1` | +0.128 | 0.000 | -0.0175 |

The cam_z is negative because the camera is 17.5 mm *below* `base_link` (which sits at the payload-rest plane). Both robots share a chassis design, so the same measurement applies to both — re-measure and update the table if a mount is changed.

Resolution rule (in `oak_camera.launch.py:_resolve_cam`):
- If any of `cam_x`/`cam_y`/`cam_z` is passed explicitly to the launch, those values win (missing components default to 0).
- Otherwise the launch looks up `_CAMERA_MOUNT[robot_id]`.
- If `robot_id` is not in the table and no explicit args are given, the launch raises `RuntimeError` rather than silently using a placeholder.

Errors in this TF translate directly into localization errors. When adding a new robot, measure first and add a row.

### Config Files

YAML configs live in `swerve_bringup/config/`: `rtabmap_localization.yaml`, `rtabmap_mapping.yaml`, `depthai_oak_d_lite.yaml`. New YAML files must be registered in `setup.py` under `data_files` using `glob('config/*.yaml')`. Build with `--symlink-install` so YAML edits don't require rebuilds.

## Packages

- `swerve_formation` — all node logic; `entry_points` in `setup.py` are the source of truth for node names
- `swerve_bringup` — launch files + YAML config in `config/`; no node logic
- `turtlebot3_conveyor_bridge` — legacy standalone serial bridge and teleop; predates `conveyor_base_node`. Keep in sync if you change the serial protocol.

Launch files and config files must appear in `setup.py` under `data_files` or they won't install to the share directory:

```python
(os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
(os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
```

## Launch File Architecture

Two SLAM topologies exist. Choose based on whether Pi 4 thermal headroom is available.

**On-Pi (single launch):**
```bash
Pi: ros2 launch swerve_bringup rtabmap_mapping.launch.py      # builds .db
Pi: ros2 launch swerve_bringup conveyor.launch.py             # full stack, localization-only
```

**Split (Pi sensors + laptop SLAM):** Recommended for mapping. Pi runs at ~40°C instead of 70-80°C.
```bash
Pi:     ros2 launch swerve_bringup rtabmap_pi_sensors.launch.py robot_id:=tb3_1
Laptop: ros2 launch swerve_bringup rtabmap_laptop_mapping.launch.py robot_id:=tb3_1
Laptop: ros2 launch swerve_bringup rtabmap_laptop_localization.launch.py robot_id:=tb3_1
```

`conveyor.launch.py` is the production full-robot launch: brings up base, EKF, camera, RTAB-Map localization, leader election, navigation, formation controller, and alignment node for a single robot.

**Simulation (no hardware):**
```bash
ros2 launch swerve_bringup sim_navigation.launch.py
ros2 run swerve_formation send_goal_node --ros-args -p x:=4.5 -p y:=3.5
```

## TF Frame Naming

Each robot uses prefixed frames to coexist in the same TF tree:

- `map` → `{robot_id}_odom` (published by RTAB-Map)
- `{robot_id}_odom` → `{robot_id}_base_link` (published by `conveyor_base_node`)
- `{robot_id}_base_link` → `{robot_id}_oak_rgb_camera_optical_frame` (static TF in `oak_camera.launch.py`)

The full chain `map → {robot_id}_odom → {robot_id}_base_link → {robot_id}_oak_rgb_camera_optical_frame` must be complete and consistent before RTAB-Map will start. A broken link anywhere in the chain causes a silent startup failure.

## Middleware

- **Current RMW: `rmw_fastrtps_cpp`** (FastDDS). The project temporarily reverted to FastDDS due to time constraints; treat it as the active middleware for everything below (discovery, QoS tuning, debugging, deployment).
- Discovery on the laptop is configured via `FASTRTPS_DEFAULT_PROFILES_FILE=~/fastdds_peers.xml` (exported in `~/.bashrc`). The `ROS_DOMAIN_ID` is `30`. The Pis use analogous `~/fastdds_peers.xml` files; see `LOCALIZATION_RUN_LAPTOP.md` and `MAPPING_RUN_LAPTOP.md` for the canonical locator structure.
- **Zenoh migration is deferred.** The legacy Zenoh config files (`zenoh_client.json5`, any `RMW_ZENOH_CONFIG_FILE` references) are retained in the repo and shell history for the eventual switch back, but they are **not active** right now. Do not propose Zenoh-specific debugging steps, do not assume `rmw_zenohd` is running, and do not set `RMW_IMPLEMENTATION=rmw_zenoh_cpp` unless the user explicitly resumes the migration.

## Namespacing

- Robot namespaces are `tb3_0`, `tb3_1`, etc. (illustrative; roles transfer dynamically)
- Use `PushRosNamespace` OR explicit `/{robot_id}/` prefixes in topic strings — never both, or you get double-namespaced paths like `/robot_1/robot_1/odom`
- `laplacian_formation_node` uses parameter-driven neighbor/offset construction — hardcoded robot name keys cause `KeyError` at runtime
- Node names get a `_{robot_id}` suffix (e.g. `laplacian_formation_node_tb3_0`) so two robots on the same network don't collide on DDS discovery
- RTAB-Map output topics must be remapped per-robot (see Localization Stack section) to prevent robots consuming each other's localization poses

## Leader Election

- Elected leader = active robot with the lowest `priority` parameter (defaults to the trailing digit of `robot_id`)
- Heartbeat on `/formation/heartbeat` format: `"robot_id:priority"`
- Peer missing for `PEER_TIMEOUT=2.0 s` triggers re-election
- `tb3_0`/`tb3_1` labels are illustrative — roles transfer automatically on disconnect

## Deployment Workflow

```bash
# Sync code to Pi (run from repo root)
rsync -av --exclude='__pycache__' ros2_ws/src/ pi1@<ip>:~/ros2_ws/src/

# Build on Pi
colcon build --symlink-install
source install/setup.bash

# Copy map database to all Pis after a mapping run
rsync ~/maps/lab.db pi1@<ip>:~/maps/lab.db
rsync ~/maps/lab.db pi2@<ip>:~/maps/lab.db

# Inspect RTAB-Map database (laptop only — heavy GUI)
rtabmap-databaseViewer ~/maps/lab.db

# Verify localization is running (from laptop)
ros2 topic hz /tb3_1/slam/pose          # should rise to ~1-5 Hz once localized
ros2 topic echo /tb3_1/ekf/odom         # smooth, drift-corrected
ros2 run tf2_ros tf2_echo map tb3_1_base_link

# Live odometry display on Pi
python3 ~/odom_watch.py /tb3_0
```

Workspace source order: ROS 2 base → TurtleBot3 ws → project ws (all in `~/.bashrc`).

## Key Rules

- These are **holonomic** robots — always swerve IK, never differential drive math
- **Never propose a central master node** — all control must be distributed
- Use numpy heavily in control loops
- Formation controller and serial bridge are intentionally separate nodes (allows sim/hardware swap)
- **OAK-D Lite only allows one host connection.** The `depthai_ros_driver` Camera component owns the device. `ai_camera_node` and any other node must subscribe to published ROS topics, not open a depthai device handle directly.
- Consensus correction in `laplacian_formation_node` is **off by default** (`enable_consensus:=false`). Turn it on only after confirming every robot loads the **same** `.db` file. Different databases produce incompatible world frames and the correction silently corrupts the formation.
- RTAB-Map multi-robot requirement: all robots must localize against **the same `.db`** for `/formation/leader`-gated consensus to be meaningful.
- **`slam_pose_relay_node` is not redundant glue.** It exists because RTAB-Map publishes `PoseWithCovarianceStamped` and `ekf_node` subscribes to `PoseStamped` — these cannot be bridged with a remap alone. Do not remove it.
- **Camera mount TF values must be physically measured** for each robot before mapping. Errors here propagate directly into every downstream pose estimate. Add new robots to `_CAMERA_MOUNT` in `swerve_bringup/launch/oak_camera.launch.py` rather than passing one-off launch args — the table is the single source of truth.
- `conveyor_base_node` is a **lifecycle node** — it must be configured then activated. The `main()` function handles this automatically, but manual lifecycle management in tests requires explicit `trigger_configure()` + `trigger_activate()` calls.
- New YAML config files in `swerve_bringup/config/` must be registered in `setup.py` `data_files` and the package must be rebuilt before they are accessible at runtime.
