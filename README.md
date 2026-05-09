# Swerve Transport Project

Two (scaling to 3+) holonomic swerve-drive TurtleBots cooperatively transport a shared rigid payload. ROS 2 Humble on the robot side, custom OpenCR C++ firmware below it, a Flask frontend on the laptop for the human operator. **All robot-side control is decentralized** — the laptop runs planning and the UI, but the system survives the laptop dying mid-mission.

For the full architecture (per-node responsibilities, cross-machine topic table, init sequence, failure modes), see **[CLAUDE.md](CLAUDE.md)** and **ARCHITECTURE.md**. The three companion diagrams `01_topology.png`, `02_initialization.png`, `03_runtime.png` give the visual reference.

## High-level architecture

**On the laptop:**
- `rmw_zenohd` — Zenoh router on TCP `:7447`, the discovery/transport hub
- Flask frontend + `goal_publisher` — the operator UI; emits `/goal_pose` on submit
- `formation_manager_node` — snapshots initial poses, latches `/formation/offsets`, publishes `/formation/footprint` from the live convex hull
- `path_planner_node` — plans against `/formation/footprint`, latches `/formation/path`

**On every robot (identical stack on tb3_0 and tb3_1):**
- `conveyor_base_node` — lifecycle serial bridge to OpenCR, publishes `/odom`
- `rtabmap_node` — visual localization-only against the shared `.db`, publishes `/visual_pose`
- `ekf_node` — fuses `/odom` + `/visual_pose` into authoritative `/pose` at 20 Hz
- `laplacian_formation_node` — Graph Laplacian consensus on neighbor poses + offsets, publishes `/cmd_vel`
- `path_follower_node` — **active on the elected leader** (publishes `/virtual_center/cmd_vel`), **dormant on followers** (caches `/formation/path` for instant promotion)
- `leader_election_node` — Bully/Raft heartbeat, publishes `/formation/leader`
- `ai_camera_node` — **deferred (stub)**; reserved for object-detection input later
- OAK camera composable container — owns the OAK-D Lite USB device exclusively

The leader's `/virtual_center/cmd_vel` flows to every robot's `laplacian_formation_node`, which combines it with the latched `/formation/offsets` to produce that robot's local `/cmd_vel`. No per-robot dispatch from the laptop. If the leader dies, the next-priority follower promotes itself, activates against its already-cached `/formation/path`, and motion continues with no re-init.

## Repository Layout

```
swerve-transport/
├── ros2_ws/
│   └── src/
│       ├── swerve_formation/        # All node logic (Python 3)
│       ├── swerve_bringup/          # Launch files + YAML config
│       └── turtlebot3_conveyor_bridge/  # Legacy serial bridge / teleop
├── opencr_firmware/
│   └── swerve_kinematics/           # OpenCR C++ firmware (swerve IK)
├── interface/                       # Flask frontend, map viewer, ros_pose_bridge
├── CLAUDE.md                        # Authoritative architecture + conventions
└── ARCHITECTURE.md                  # Companion to the three PNG diagrams
```

`entry_points` in `swerve_formation/setup.py` are the source of truth for node names. New launch files and YAML configs must be registered in `setup.py` `data_files` (`glob('launch/*.py')`, `glob('config/*.yaml')`) or they will not install.

---

## First-Time Setup (Do This Once)

### Prerequisites

Make sure you have the following installed before cloning:

- Ubuntu 22.04
- ROS 2 Humble — [install guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
- TurtleBot3 packages (see step below)
- Python 3, pip, git

### 1. Install TurtleBot3 Packages

If you do not already have a `turtlebot3_ws`, install the TurtleBot3 packages:

```bash
sudo apt update
sudo apt install ros-humble-turtlebot3 ros-humble-turtlebot3-msgs
```

If your teammate gave you a `turtlebot3_ws/` folder instead, source that workspace directly (see step 4).

### 2. Clone the Repository

```bash
mkdir -p ~/projects
cd ~/projects
git clone <your-repo-url> swerve-transport
cd swerve-transport
```

### 3. Install Python Dependencies

```bash
pip install numpy
```

### 4. Configure Your Shell Environment

This is the most important step. You need to source three things in the correct order every time you open a terminal. The easiest way is to add them permanently to your `~/.bashrc`:

```bash
# ROS 2 base
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# TurtleBot3 workspace — choose ONE of the following two lines:

# Option A: if you installed via apt (recommended)
echo "export TURTLEBOT3_MODEL=waffle" >> ~/.bashrc

# Option B: if you have a local turtlebot3_ws/ folder
echo "source ~/turtlebot3_ws/install/setup.bash" >> ~/.bashrc

# Your project workspace (built in the next step, source after building)
echo "source ~/projects/swerve-transport/ros2_ws/install/setup.bash" >> ~/.bashrc

# Reload your shell
source ~/.bashrc
```

> **Order matters.** ROS 2 base → TurtleBot3 → your project. Each layer builds on the previous one.

### 5. Build the Project Workspace

```bash
cd ~/projects/swerve-transport/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

`--symlink-install` means Python file edits take effect immediately — no rebuild needed unless you change `setup.py` or `package.xml`.

### 6. Verify the Setup

```bash
ros2 pkg list | grep swerve_formation
```

You should see `swerve_formation` in the output. If you do, setup is complete.

---

## Daily Workflow

### Opening VS Code

Open the **repo root**, not a subfolder:

```bash
code ~/projects/swerve-transport
```

This gives you one window with visibility into both the ROS 2 Python nodes and the OpenCR firmware.

### Recommended VS Code Extensions

- **ROS** (`ms-iot.vscode-ros`) — topic introspection, launch file support
- **Python** (`ms-python.python`) — linting and IntelliSense
- **C/C++** (`ms-vscode.cpptools`) — for OpenCR firmware

### Git Workflow

```bash
# 1. Sync with main before starting
git checkout main
git pull origin main

# 2. Switch to your feature branch
git checkout feature/laplacian-python   # Python side
# or
git checkout feature/opencr-swerve      # Firmware side

# 3. Make changes, then commit in small working chunks
git status
git add ros2_ws/src/swerve_formation/...
git commit -m "<descriptive message>"

# 4. Push
git push origin feature/laplacian-python
```

---

## Running the Simulation

Open **three separate terminals**. Each one must have the environment sourced (handled automatically if you completed step 4 above).

**Terminal 1 — Robot 1 (simulator):**
```bash
ros2 run swerve_formation fake_swerve_simulator \
  --ros-args -p robot_id:=tb3_0 -p start_x:=0.0 -p start_y:=0.5
```

**Terminal 2 — Robot 2 (simulator):**
```bash
ros2 run swerve_formation fake_swerve_simulator \
  --ros-args -p robot_id:=tb3_1 -p start_x:=0.0 -p start_y:=-0.5
```

**Terminal 3 — Formation controller:**
```bash
ros2 run swerve_formation laplacian_formation_node \
  --ros-args -p robot_id:=tb3_0
```

**Terminal 4 — Teleoperation (optional):**
```bash
ros2 topic pub /virtual_center/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.2, y: 0.0, z: 0.0}}"
```

**Visualization:**
```bash
rviz2
```
Add a `Marker` display and set the topic to `/tb3_0/marker` and `/tb3_1/marker` to see the robots as cylinders.

---

## Build Reference

```bash
# Full build
cd ~/projects/swerve-transport/ros2_ws
colcon build --symlink-install
source install/setup.bash

# Build only the formation package (faster)
colcon build --symlink-install --packages-select swerve_formation
source install/setup.bash
```

---

## Branching Rules

| Branch | Purpose |
|--------|---------|
| `main` | Tested code only — real robots or validated simulation |
| `feature/laplacian-python` | Python formation and transport logic (RPi4 nodes) |
| `feature/opencr-swerve` | OpenCR motor and kinematics firmware |

## Pull Request Policy

- Open a PR to merge your feature branch into `main`
- Require at least one teammate review
- Merge only after simulation or hardware verification

---

## Running the Two-Robot System

This is the production runtime. The startup order matters — see [CLAUDE.md → Initialization sequence](CLAUDE.md) for the five phases and what each one waits for.

### Prerequisites

- Ubuntu 22.04 + ROS 2 Humble on every machine, sourced in this order: ROS 2 base → TurtleBot3 ws → project ws.
- `ros-humble-rmw-zenoh-cpp` installed on the laptop and both Pis.
- A Zenoh client config at `~/zenoh_client.json5` on each Pi that points at the laptop's IP on TCP `:7447`. Each Pi's shell exports `RMW_ZENOH_CONFIG_FILE=~/zenoh_client.json5` (note: not `ZENOH_CONNECT` and not `ZENOH_CONFIG_OVERRIDE`).
- `ROS_DOMAIN_ID=30` on every machine.
- A built `.db` map (see Mapping below), copied to `/home/pi1/maps/lab.db` and `/home/pi2/maps/lab.db`.

### Deploy code to a Pi

```bash
# From the laptop, full sync (do not partial-sync — orphaned files cause silent failures)
rsync -av --delete --exclude='__pycache__' \
  ros2_ws/src/ pi1@<pi-ip>:~/ros2_ws/src/

# Then on the Pi
ssh pi1@<pi-ip>
cd ~/ros2_ws && colcon build --symlink-install && source install/setup.bash
```

### Bring up the laptop side first

Start the Zenoh router, the Flask frontend, and the laptop-side planning nodes:

```bash
# Terminal 1 — Zenoh router (must be first; leave running)
ros2 run rmw_zenoh_cpp rmw_zenohd

# Terminal 2 — laptop nodes (formation_manager, path_planner, frontend)
ros2 launch swerve_bringup laptop.launch.py
```

If `rmw_zenohd` fails with "address in use", clear stale routers: `pkill rmw_zenohd`.

### Bring up each robot

```bash
# On tb3_0
ros2 launch swerve_bringup conveyor.launch.py robot_id:=tb3_0

# On tb3_1
ros2 launch swerve_bringup conveyor.launch.py robot_id:=tb3_1
```

This launches the per-robot stack listed in the architecture section above. Each robot independently localizes against the shared `.db` (5–30 seconds is normal — RTAB-Map needs to find a visual match before publishing `/visual_pose`).

### Verify the system is healthy

From the laptop, confirm the topic graph:

```bash
ros2 topic hz /tb3_0/pose /tb3_1/pose          # ~20 Hz once both are localized
ros2 topic echo /formation/offsets --once      # latched, should print on subscribe
ros2 topic echo /formation/leader --once       # current elected leader
```

If `/tb3_*/pose` is silent, the robot has not yet localized — check the Pi's RTAB-Map log for "loop closure detected" / "global localization succeeded" before assuming a real fault.

### Drive the formation

Open the Flask frontend in a browser (URL printed in the laptop launch log) and submit a goal. `path_planner_node` plans a footprint-aware path against the live `/formation/footprint`, latches `/formation/path`, and the elected leader's `path_follower_node` activates against it. Followers stay dormant but cache the path so leader handover is instant.

### Mapping (one-time, before the first run)

Drive one robot manually around the room while the laptop runs RTAB-Map in mapping mode:

```bash
# On the Pi
ros2 launch swerve_bringup rtabmap_pi_sensors.launch.py robot_id:=tb3_1

# On the laptop
ros2 launch swerve_bringup rtabmap_laptop_mapping.launch.py robot_id:=tb3_1
```

When done, copy the resulting `.db` to every Pi:

```bash
rsync ~/maps/lab.db pi1@<tb3_0-ip>:~/maps/lab.db
rsync ~/maps/lab.db pi2@<tb3_1-ip>:~/maps/lab.db
```

All robots **must** localize against the same `.db` — different databases produce incompatible world frames and the formation will silently corrupt.

---

## Stopping cleanly

Stop in reverse startup order: kill each Pi launch first (lifecycle deactivation zeroes the motors via the serial link), then the laptop nodes, then `rmw_zenohd` last. The OpenCR's 500 ms watchdog is a safety backstop, not a replacement for clean shutdown.

---

## Troubleshooting

**Pis can't see each other's topics.** `rmw_zenohd` is not running on the laptop, or `RMW_ZENOH_CONFIG_FILE` on the Pi does not point at a valid client config. Check the laptop terminal for the router log; on the Pi, `echo $RMW_ZENOH_CONFIG_FILE` and verify the file's `connect.endpoints` matches the laptop IP.

**`/tb3_*/pose` never appears.** RTAB-Map has not localized yet. Confirm the `.db` is at `~/maps/lab.db` on the Pi and the camera is producing images (`ros2 topic hz /tb3_0/camera/rgb/image_raw`). Drop the robot in an area with visual variety — blank walls fail.

**Build fails with a missing dependency.** Run `rosdep install --from-paths src --ignore-src -r -y` inside `~/ros2_ws/` and rebuild.

**Serial port `/dev/ttyACM0` not accessible on a Pi.** The `pi1` user must be in the `dialout` group: `sudo usermod -aG dialout pi1`, then log out and back in.

**`ros2: command not found`.** ROS 2 base is not sourced. Either run `source /opt/ros/humble/setup.bash` or fix `~/.bashrc`.

**`Package 'swerve_formation' not found`.** Project workspace is not sourced: `source ~/ros2_ws/install/setup.bash`.