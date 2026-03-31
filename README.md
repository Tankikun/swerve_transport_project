# Swerve Transport Project

Single team repository for high-level ROS 2 control and low-level OpenCR firmware.

## Repository Layout

```
swerve-transport/
├── ros2_ws/
│   └── src/
│       └── swerve_formation/     # High-level Python ROS 2 package
│           ├── swerve_formation/
│           │   ├── laplacian_formation_node.py
│           │   └── fake_swerve_simulator.py
│           ├── package.xml
│           └── setup.py
└── opencr_firmware/
    └── swerve_kinematics/        # Low-level OpenCR C++ firmware
```

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

## Troubleshooting

**`ros2: command not found`**
You forgot to source ROS 2. Run `source /opt/ros/humble/setup.bash` or check your `~/.bashrc`.

**`Package 'swerve_formation' not found`**
You need to source the project workspace: `source ~/projects/swerve-transport/ros2_ws/install/setup.bash`

**Nodes can't see each other's topics**
Make sure Zenoh is being used as the RMW layer. Set this in your `~/.bashrc`:
```bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
```

**Build fails with missing dependency**
Run `rosdep install --from-paths src --ignore-src -r -y` from inside `ros2_ws/` to auto-install missing ROS dependencies.