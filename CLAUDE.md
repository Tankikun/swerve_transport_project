# Swerve Transport — Project Context for Claude Code

## What This Is

A multi-robot cooperative transport system built on ROS 2 Humble. Two (scaling to 3+) holonomic swerve-drive robots carry a shared rigid payload in formation. All control is decentralized — no master node ever.

## Hardware (per robot)

- **Chassis**: TurtleBot3 Conveyor (swerve/holonomic)
- **Motors**: 8x Dynamixel XL430-W250 — 4 steering (position mode) + 4 drive (velocity mode), all on a single TTL bus via U2D2 Power Hub Board
- **Motor IDs**: drive = [7, 3, 5, 1], steering = [8, 4, 6, 2]
- **Microcontroller**: OpenCR (STM32) — runs custom swerve IK firmware in C++
- **Compute**: Raspberry Pi 4 (username `pi1`, workspace `~/turtlebot3_ws`)
- **Camera**: Luxonis OAK-D Lite (RGB + stereo depth)
- **Dev machine**: ROG-Strix-G513QE, Ubuntu 22.04, ROS 2 Humble, username `tankikun`

## OpenCR Firmware

- Receives `"x_dot y_dot gamma_dot\n"` over USB-CDC at 115200 baud
- Steering constrained to `[-π/2, π/2]` with drive direction flip + angle-minimizing selection
- 500 ms watchdog: zeros drive motors on command timeout
- GroupSyncWrite at 1 Mbps (~200-300 µs per write, fits in 20 ms control loop)

## Software Architecture (per robot)

All nodes are Python 3 unless noted. Always use numpy for matrix math in control loops (Pi 4 is compute-limited).

| Node | Role |
|---|---|
| `conveyor_base_node` | Lifecycle node; serial bridge to OpenCR over USB-CDC |
| `laplacian_formation_node` | Graph Laplacian formation controller; publishes `/formation/state` |
| `ai_camera_node` | OAK-D Lite RGB + stereo depth pipeline |
| `3d_slam_node` | Publishes pose estimate |
| `ekf_node` | Fuses raw `/odom` + SLAM pose; **single authoritative pose source** |
| `navigation_node` | Runs on all robots; only the elected leader activates its control loop |
| `formation_size_node` | Leader-only; subscribes to `/formation/state` + camera object size; computes system bounding envelope; publishes footprint to navigation |
| `leader_election_node` | Bully/Raft-inspired dynamic election; scales to 3+ robots |

**Pose data flow**: raw `/odom` -> `ekf_node` -> authoritative pose consumed by laplacian + navigation nodes. Nothing reads raw `/odom` except `ekf_node`.

## Middleware

- **rmw_zenoh_cpp** v0.1.8 (not standard FastDDS)
- Zenoh router (`rmw_zenohd`) runs on the laptop
- Robots are Zenoh clients; config via `RMW_ZENOH_CONFIG_FILE` pointing to `zenoh_client.json5` with `mode: "client"` and explicit `connect.endpoints`
- Do NOT use `ZENOH_CONNECT` or `ZENOH_CONFIG_OVERRIDE` env vars — they are ignored by rmw_zenoh_cpp

## Packages

- `swerve_formation` — all node logic
- `swerve_bringup` — launch files only; separate from logic on purpose

Launch files must be listed in `setup.py` under `data_files` as `launch/*.py` or they won't install to the share directory.

## Namespacing

- Robot namespaces are `tb3_0`, `tb3_1`, etc. (illustrative; roles transfer dynamically)
- Use `PushRosNamespace` OR explicit `/{robot_id}/` prefixes in topic strings — never both, or you get double-namespaced paths like `/robot_1/robot_1/odom`
- `laplacian_formation_node` uses parameter-driven neighbor/offset construction — hardcoded robot name keys cause `KeyError` at runtime

## Leader Election

- Elected leader publishes heartbeat on `/formation/heartbeat`
- Robots missing heartbeats past timeout trigger re-election
- `tb3_0`/`tb3_1` labels are illustrative — roles transfer automatically on disconnect

## Deployment Workflow

```bash
# Sync code to Pi
rsync -av --exclude='__pycache__' <src>/ pi1@<ip>:~/turtlebot3_ws/src/

# Build on Pi (Python changes after initial build don't need rebuild)
colcon build --symlink-install
source install/setup.bash
```

Workspace source order: ROS 2 base -> TurtleBot3 ws -> project ws (all in `~/.bashrc`).

## Key Rules

- These are **holonomic** robots — always swerve IK, never differential drive math
- **Never propose a central master node** — all control must be distributed
- Use numpy heavily in control loops
- Formation controller and serial bridge are intentionally separate nodes (allows sim/hardware swap)