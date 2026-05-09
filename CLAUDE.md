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

## Software Architecture

The system is split across **the laptop** and **every robot**. All robot Pis run an *identical* node stack — there is no leader-only or follower-only variant of the runtime image, only runtime *state* differs (the leader has its `path_follower_node` active, others have it dormant). All nodes are Python 3. Always use numpy for matrix math in control loops (Pi 4 is compute-limited).

### Laptop nodes

| Node | Role |
|---|---|
| `rmw_zenohd` | Zenoh router on TCP `:7447`. Every Pi connects to this router as a client. Must be up before any robot or interface node starts. |
| `flask_frontend` + `goal_publisher` | Locally-hosted Flask UI (built by a teammate) that lets a human submit a target. Publishes `/goal_pose` (`PoseStamped`) when the user clicks submit. **This is the production user interface** — RViz is debug-only and is not in any control-loop data path. |
| `formation_manager_node` | Two responsibilities: (1) ONE-SHOT — at startup, snapshots `/tb3_0/pose` and `/tb3_1/pose`, computes virtual centre `C = (P0 + P1)/2` and per-robot offsets `δᵢ = Pᵢ − C`, and publishes `/formation/offsets` (`Float64MultiArray`, latched / `transient_local`). (2) CONTINUOUS — recomputes the formation's convex-hull `/formation/footprint` (`Polygon`) on every pose update and feeds it to `path_planner_node`. Replaces the old `formation_size_node`. |
| `path_planner_node` | Subscribes to `/goal_pose` and `/formation/footprint`. Plans a footprint-aware path and publishes `/formation/path` (`PoseArray`) once per goal, latched. |
| (optional) monitoring tools | `rqt`, `rviz2`, dashboards. Strictly observational — never in a control loop. |

### Per-robot nodes (identical stack on tb3_0 and tb3_1)

| Node | Role |
|---|---|
| `conveyor_base_node` | Lifecycle node; serial bridge to OpenCR over USB-CDC. Publishes `/{robot_id}/odom` + TF `{robot_id}_odom → {robot_id}_base_link`. |
| `rtabmap_node` | Localization-only against the shared `.db`. GFTT+ORB features, 5 Hz detection. Publishes `/{robot_id}/visual_pose` (the per-robot localization pose). |
| `ai_camera_node` | **DEFERRED — stub only.** Architecture preserved: when re-enabled it will subscribe to `depthai_ros_driver` detection topics. The OAK-D allows only one host connection, so this node must never open its own depthai device handle. **No current node depends on its output.** |
| `ekf_node` | Fuses `/{robot_id}/odom` (prediction) + `/{robot_id}/visual_pose` (correction). Publishes `/{robot_id}/pose` at 20 Hz. Fixed observation noise `R = diag(0.05, 0.05, 0.02)`. |
| `laplacian_formation_node` | Graph Laplacian consensus on `/{robot_id}/pose` from all peers + `/formation/offsets` + `/virtual_center/cmd_vel`. Publishes `/{robot_id}/cmd_vel` to `conveyor_base_node`. |
| `path_follower_node` | Runs on **every** robot. **ACTIVE on the elected leader** — consumes `/formation/path`, publishes `/virtual_center/cmd_vel` at 20 Hz. **DORMANT on followers** — caches the latest `/formation/path` so on leader handover the newly elected robot activates against cached path with zero re-init. (This is "Option B": identical stack everywhere, runtime activation gated by `/formation/leader`.) |
| `leader_election_node` | Bully/Raft-style election over `/formation/heartbeat` (`Empty`, 2 Hz). Lowest-priority active robot wins. Publishes `/formation/leader`. |
| `slam_pose_relay_node` | Type-conversion glue: converts RTAB-Map's native `PoseWithCovarianceStamped` to the `PoseStamped` consumed by `ekf_node`. Do not remove — the message types cannot be bridged with a remap alone. |
| OAK camera (`depthai_ros_driver::Camera` component) | RGB + aligned stereo depth. Loaded into a `ComposableNodeContainer` by `oak_camera.launch.py`, configured by `swerve_bringup/config/depthai_oak_d_lite.yaml`. Owns the OAK-D USB device exclusively. |
| `fake_swerve_simulator` | Software-only robot for local nav testing — no hardware needed. |

### Cross-machine topics

| Topic | Type | Rate | Producer → Consumer |
|---|---|---|---|
| `/tb3_0/pose` | `PoseStamped` | 20 Hz | `ekf_node@tb3_0` → `laplacian_formation_node@all`, `formation_manager@laptop`, `path_planner@laptop` |
| `/tb3_1/pose` | `PoseStamped` | 20 Hz | `ekf_node@tb3_1` → `laplacian_formation_node@all`, `formation_manager@laptop`, `path_planner@laptop` |
| `/formation/offsets` | `Float64MultiArray` | one-shot (latched, `transient_local`) | `formation_manager@laptop` → `laplacian_formation_node@all` |
| `/formation/footprint` | `Polygon` | on-change | `formation_manager@laptop` → `path_planner@laptop` |
| `/formation/path` | `PoseArray` | one-shot per goal (latched) | `path_planner@laptop` → `path_follower_node@all` |
| `/virtual_center/cmd_vel` | `Twist` | 20 Hz | `path_follower_node@leader` → `laplacian_formation_node@all` |
| `/formation/heartbeat` | `Empty` | 2 Hz | `leader_election@leader` → `leader_election@all` |
| `/goal_pose` | `PoseStamped` | on-submit | `flask_frontend@laptop` → `path_planner@laptop` |

### Pose data flow (production runtime, localization mode)

`/{robot_id}/odom` (raw) → `ekf_node` (prediction). `ekf_node` publishes `/{robot_id}/pose`, which is shared over the network and consumed by every robot's `laplacian_formation_node` and the laptop's `formation_manager` + `path_planner`. RTAB-Map → `slam_pose_relay_node` → `/{robot_id}/visual_pose` → `ekf_node` (correction). Raw `/odom` has exactly one consumer: the local `ekf_node`.

### Pose data flow (mapping)

`/{robot_id}/odom` (raw) → RTAB-Map directly. `ekf_node` is typically not running. There is no SLAM-correction loop yet — rtabmap is *building* the `.db`, not consulting it — so raw odom is the right input and matches the `{robot_id}_odom → base_link` TF.

### Command data flow

Goal: `flask_frontend → /goal_pose → path_planner → /formation/path` (latched). Then on the leader: `path_follower_node → /virtual_center/cmd_vel → laplacian_formation_node → /{robot_id}/cmd_vel → conveyor_base_node → OpenCR`. Followers receive the same `/virtual_center/cmd_vel` and the same `/formation/offsets`, so each robot independently arrives at its own corrected wheel commands without any per-robot dispatch from the laptop.

## Initialization sequence

The system comes up in five distinct phases. A failure in an earlier phase should *halt* the later phases (no partial activation).

1. **Bringup + election.** Laptop brings up `rmw_zenohd`, `flask_frontend`, `formation_manager`, `path_planner`. Each Pi launches its full node stack. `leader_election_node` instances exchange `/formation/heartbeat` and converge on a leader.
2. **Localization.** Each Pi's `rtabmap_node` scans the shared `.db` for a visual match (5–30 s typical). Until matched, `/{robot_id}/visual_pose` is silent and `ekf_node` runs as pure dead-reckoning. Phase 3 must not start until *all* robots are localized.
3. **Pose-sharing + formation init.** All `ekf_node`s publish `/{robot_id}/pose` at 20 Hz. `formation_manager_node` snapshots the first synchronized pose pair, computes `C` and `δᵢ`, latches `/formation/offsets`. Every `laplacian_formation_node` receives the offsets via `transient_local` QoS.
4. **Goal + planning.** User submits a target via the Flask frontend → `/goal_pose` → `path_planner_node` plans against `/formation/footprint`, publishes `/formation/path` (latched). All `path_follower_node`s cache it; only the leader activates against it.
5. **Execution.** Leader's `path_follower_node` publishes `/virtual_center/cmd_vel` at 20 Hz; followers' `path_follower_node`s remain dormant but keep the cached path warm.

## Failure modes

- **Laptop dies mid-mission.** Robots continue executing the cached `/formation/path` (latched, `transient_local`). The laptop is not in any tight control loop — losing it does not stop motion. Recovery: relaunch laptop nodes; latched topics re-establish.
- **Leader dies.** `leader_election_node` detects the missing heartbeat after `PEER_TIMEOUT=2.0 s` and promotes the next-priority follower. The newly elected robot's `path_follower_node` activates against its already-cached `/formation/path` — no re-planning, no re-init.
- **Network hiccup.** `/virtual_center/cmd_vel` stops arriving at the Pis. The OpenCR's 500 ms watchdog zeroes the drive motors. Steering holds. When the network recovers, motion resumes from the next received command.
- **Partial localization (one robot still searching the `.db`).** Phase 3 should not start until both robots publish `/{robot_id}/pose` from a fused (not pure-dead-reckoning) state, otherwise the `formation_manager` snapshot is poisoned and `δᵢ` will be wrong for the rest of the mission.

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

`conveyor.launch.py` is the production full-robot launch and brings up the per-robot stack listed in the Software Architecture section: `conveyor_base_node`, `rtabmap_node` (localization-only), the OAK camera composable container, `slam_pose_relay_node`, `ekf_node`, `laplacian_formation_node`, `path_follower_node`, and `leader_election_node`. `formation_manager_node` and `path_planner_node` are launched from the laptop, not here. The laptop launch additionally starts `rmw_zenohd` and the Flask frontend.

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

- **Current RMW: `rmw_zenoh_cpp`.** The laptop runs `rmw_zenohd` on TCP `:7447` as the discovery/transport router; every Pi connects to it as a Zenoh client. `rmw_zenohd` must be up before any robot or interface node starts, or DDS-layer discovery will silently never converge.
- Per-Pi Zenoh client config is selected via the **`RMW_ZENOH_CONFIG_FILE`** environment variable (point it at `~/zenoh_client.json5`). Do not confuse this with `ZENOH_CONNECT` or `ZENOH_CONFIG_OVERRIDE` — those are upstream Zenoh CLI vars and have no effect on `rmw_zenoh_cpp`.
- `ROS_DOMAIN_ID` is `30`.
- **FastDDS fallback is retained but inactive.** The legacy `fastdds_peers.xml` files and `FASTRTPS_DEFAULT_PROFILES_FILE` references in `LOCALIZATION_RUN_LAPTOP.md` / `MAPPING_RUN_LAPTOP.md` are kept for the case where `rmw_zenohd` is down or under investigation, but they are **not active** right now. Do not propose FastDDS-specific debugging steps unless the user explicitly switches back.

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
ros2 topic hz /tb3_1/visual_pose        # should rise to ~1-5 Hz once localized
ros2 topic echo /tb3_1/pose             # 20 Hz, smooth, drift-corrected
ros2 run tf2_ros tf2_echo map tb3_1_base_link

# Live odometry display on Pi
python3 ~/odom_watch.py /tb3_0
```

Workspace source order: ROS 2 base → TurtleBot3 ws → project ws (all in `~/.bashrc`).

## Key Rules

- These are **holonomic** robots — always swerve IK, never differential drive math.
- **Never propose a central master node.** The laptop runs planning and the UI, but it is not in any control loop and the system survives the laptop dying.
- **Identical node stack on every robot.** Do not propose a leader-only or follower-only build. Roles are runtime state, not a deployment artifact. `path_follower_node` activates/deactivates against `/formation/leader`; it does not get conditionally launched.
- **`ai_camera_node` is deferred.** No production node currently consumes its output. Keep the architecture wired so it can be re-enabled, but do not add dependencies on it.
- **The user interface is the Flask frontend, not RViz.** RViz is debug-only and must never appear in a control-loop data path.
- Use numpy heavily in control loops (Pi 4 is compute-limited).
- Formation controller and serial bridge are intentionally separate nodes (allows sim/hardware swap).
- **OAK-D Lite only allows one host connection.** The `depthai_ros_driver` Camera component owns the device. `ai_camera_node` (when re-enabled) and any other node must subscribe to published ROS topics, not open a depthai device handle directly.
- RTAB-Map multi-robot requirement: all robots must localize against **the same `.db`** — different databases produce incompatible world frames.
- **`slam_pose_relay_node` is not redundant glue.** It exists because RTAB-Map publishes `PoseWithCovarianceStamped` and `ekf_node` subscribes to `PoseStamped` — these cannot be bridged with a remap alone. Do not remove it.
- **Latched topics for one-shot data.** `/formation/offsets` and `/formation/path` use `transient_local` QoS so a late-joining or newly-elected leader receives the latest value without a re-publish. Do not change these to volatile.
- **Camera mount TF values must be physically measured** for each robot before mapping. Errors here propagate directly into every downstream pose estimate. Add new robots to `_CAMERA_MOUNT` in `swerve_bringup/launch/oak_camera.launch.py` rather than passing one-off launch args — the table is the single source of truth.
- `conveyor_base_node` is a **lifecycle node** — it must be configured then activated. The `main()` function handles this automatically, but manual lifecycle management in tests requires explicit `trigger_configure()` + `trigger_activate()` calls.

### Workflow conventions

- **Namespace via `PushRosNamespace` at launch time; topics are relative inside nodes.** Never combine `PushRosNamespace` with explicit `/{robot_id}/` prefixes inside a node — you will get double-namespaced paths like `/tb3_0/tb3_0/odom`. The cross-machine topic table above shows the *resolved* names; nodes themselves should subscribe to relative names like `pose`, not `/tb3_0/pose`.
- **Middleware env var is `RMW_ZENOH_CONFIG_FILE`**, not `ZENOH_CONNECT` and not `ZENOH_CONFIG_OVERRIDE`. The latter two are upstream Zenoh CLI vars and do nothing for `rmw_zenoh_cpp`.
- **Always rsync the full `src/` tree before `colcon build` on a Pi.** Partial syncs leave stale `.pyc` and orphaned `setup.py` entries that cause silent runtime errors.
- **`setup.py` `data_files` must `glob('launch/*.py')` and `glob('config/*.yaml')`.** New launch files and YAML configs are invisible to `ros2 launch` until they install to the share directory.
- **Workspace sourcing order in `~/.bashrc`:** ROS 2 base → TurtleBot3 ws → project ws. Each layer overrides the previous; reversing the order loses the project's overrides.
