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

# Two-Robot Formation Test (Keyboard Teleop)

This section covers everything needed to pull the latest code, deploy it to both robots, and run the Laplacian formation test using the twist keyboard to drive the virtual center. No camera or SLAM is required for this test — offset initialization uses odometry from the robots' starting positions.

---

## What This Test Does

The keyboard on your laptop publishes velocity commands to `/virtual_center/cmd_vel`. Both robots' `laplacian_formation_node` instances subscribe to that topic and independently compute corrected wheel commands to maintain their starting offset from each other. The formation geometry is derived from wherever you physically place the robots before launching — the `alignment_node` reads their initial EKF poses and locks in the offsets automatically.

---

## Prerequisites

Before starting, confirm you have the following on your laptop:

- Ubuntu 22.04 with ROS 2 Humble installed and sourced
- `rmw_zenoh_cpp` installed (`sudo apt install ros-humble-rmw-zenoh-cpp`)
- `nav2_lifecycle_manager` installed (`sudo apt install ros-humble-nav2-lifecycle-manager`)
- SSH access to both Raspberry Pis on the lab network
- The repo cloned and the workspace built at least once

The two Pis are:

| Robot | Hostname / IP | ROS namespace |
|-------|--------------|---------------|
| tb3_0 (leader) | `pi1@<tb3_0-ip>` | `tb3_0` |
| tb3_1 (follower) | `pi1@<tb3_1-ip>` | `tb3_1` |

Replace `<tb3_0-ip>` and `<tb3_1-ip>` with the actual fixed IPs assigned to each Pi by the lab router.

---

## Step 1: Pull and Deploy Code to Both Pis

On your laptop, pull the latest changes from main first:

```bash
cd ~/turtlebot3_ws   # or wherever your project workspace lives
git pull origin main
```

Then sync the source to each Pi. Run both commands from the same terminal (they are fast):

```bash
# Deploy to tb3_0
rsync -av --exclude='__pycache__' \
  src/ pi1@<tb3_0-ip>:~/turtlebot3_ws/src/

# Deploy to tb3_1
rsync -av --exclude='__pycache__' \
  src/ pi1@<tb3_1-ip>:~/turtlebot3_ws/src/
```

Because the workspace was built with `--symlink-install`, Python-only changes take effect immediately. You only need to rebuild on the Pi if `setup.py` or `package.xml` changed. If you are unsure, rebuild anyway:

```bash
# On each Pi (run in separate SSH terminals)
cd ~/turtlebot3_ws
colcon build --symlink-install
source install/setup.bash
```

---

## Step 2: Start the Zenoh Router on Your Laptop (Skip if using different middleware)

Open a dedicated terminal for the router and leave it running for the entire test. The router must be up before any robot or teleop node starts.

```bash
# Terminal 1 — Zenoh router
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
ros2 run rmw_zenoh_cpp rmw_zenohd
```

You should see log output like `Zenoh router started`. If you see an error about the port already in use, kill any stray `rmw_zenohd` processes first: `pkill rmw_zenohd`.

Each Pi's `zenoh_client.json5` should already point to your laptop's IP. If this is a different laptop than usual, you need to update the `connect.endpoints` in that file on each Pi before continuing:

```json5
// ~/zenoh_client.json5 on each Pi
{
  mode: "client",
  connect: {
    endpoints: ["tcp/<YOUR_LAPTOP_IP>:7447"]
  }
}
```

---

## Step 3: Place the Robots

Position the two robots where you want them. They should be facing the same direction (both pointing forward along the x-axis of your test area). A side-by-side arrangement works well — roughly 0.5 to 0.8 m apart laterally.

Do not move them after this point until the nodes are fully up. The alignment node reads their starting EKF poses to compute the formation offsets.

---

## Step 4: Launch on tb3_0 (Leader)

SSH into tb3_0 and run:

```bash
# SSH terminal for tb3_0
ssh pi1@<tb3_0-ip>

# On the Pi:
source ~/.bashrc
cd ~/turtlebot3_ws
ros2 launch swerve_bringup conveyor.launch.py \
  robot_id:=tb3_0 \
  neighbors:=tb3_1 \
  my_offset:=0.0,0.3 \
  neighbor_offsets:=0.0,-0.3 \
  usb_port:=/dev/ttyACM0 \
  offset_init_mode:=odom
```

Wait for this line in the output before moving on:

```
[alignment_node]: Offset init from odometry complete
```

If it says `Offset init timeout — no valid EKF pose received`, the EKF has not had time to publish yet. Kill the launch, wait five seconds, and try again. This usually happens if the Zenoh router was not up yet when the node started.

---

## Step 5: Launch on tb3_1 (Follower)

In a second SSH terminal, connect to tb3_1 and run:

```bash
# SSH terminal for tb3_1
ssh pi1@<tb3_1-ip>

# On the Pi:
source ~/.bashrc
cd ~/turtlebot3_ws
ros2 launch swerve_bringup conveyor.launch.py \
  robot_id:=tb3_1 \
  neighbors:=tb3_0 \
  my_offset:=0.0,-0.3 \
  neighbor_offsets:=0.0,0.3 \
  usb_port:=/dev/ttyACM0 \
  offset_init_mode:=odom
```

Wait for the same alignment confirmation line.

Note the offsets are mirrored: tb3_0 is offset +0.3 m in y, tb3_1 is offset -0.3 m. This means the virtual center sits between them. If your robots are further than ~0.6 m apart, adjust these values to match half the actual physical gap.

---

## Step 6: Verify Topics Are Visible on the Laptop

Before touching the keyboard, confirm the full topic graph is healthy. On your laptop (new terminal):

```bash
# Terminal 2 — topic verification
source ~/.bashrc
ros2 topic list | grep -E "odom|cmd_vel|formation|offsets"
```

You should see at minimum:

```
/tb3_0/odom
/tb3_0/ekf/odom
/tb3_0/cmd_vel
/tb3_1/odom
/tb3_1/ekf/odom
/tb3_1/cmd_vel
/virtual_center/cmd_vel
/formation/offsets
/formation/state
```

If `/tb3_1/ekf/odom` or `/tb3_0/ekf/odom` are missing, the EKF node on that robot is not publishing yet. Check that the lifecycle manager activated the `conveyor_base_node` on that Pi — look for the line `ConveyorBaseNode activated` in the Pi's launch output.

You can also echo the formation state to confirm both robot poses are being reported:

```bash
ros2 topic echo /formation/state --once
```

You should see two `poses` entries with distinct x/y coordinates.

---

## Step 7: Reset Odometry on Both Robots

Before driving, zero out both robots' odometry so the formation controller works from a clean baseline.

```bash
# On your laptop:
ros2 service call /tb3_0/reset_odom std_srvs/srv/Trigger
ros2 service call /tb3_1/reset_odom std_srvs/srv/Trigger
```

Both should respond with `success: True`. If you get `Node not active`, the lifecycle manager has not finished activating `conveyor_base_node` yet — wait a few more seconds and retry.

---

## Step 8: Start the Keyboard Teleop

Open a new terminal on your laptop. The teleop node must publish to `/virtual_center/cmd_vel`, not to a robot-specific `cmd_vel`:

```bash
# Terminal 3 — teleop
source ~/.bashrc
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r cmd_vel:=/virtual_center/cmd_vel
```

Key bindings for holonomic control:

| Key | Motion |
|-----|--------|
| `i` | Forward |
| `,` | Backward |
| `j` | Rotate left (CCW) |
| `l` | Rotate right (CW) |
| `J` (shift+j) | Strafe left |
| `L` (shift+l) | Strafe right |
| `k` | Stop |
| `q` / `z` | Increase / decrease speed |

Start with a low speed. The default is 0.10 m/s which is safe for a first run. Keep runs under 30 seconds because wheel odometry drifts without SLAM correction active.

---

## Step 9: What to Watch For

During the test, the formation is working correctly if:

- Both robots move in the same direction when you press forward/back
- The lateral gap between them stays roughly constant (within ~5 cm drift over 30 s)
- Neither robot oscillates or spins unexpectedly

A few things that indicate problems:

- One robot moves and the other stands still: the Zenoh bridge is not delivering `/virtual_center/cmd_vel` to that Pi. Check the router is still running.
- Both robots move but drift apart: the `laplacian_formation_node` is not receiving the neighbor's EKF odom. Run `ros2 topic hz /tb3_0/ekf/odom` from the laptop to check the publish rate — it should be around 10-20 Hz.
- Oscillation or jitter: `k_gain` is too high for the current offset distance. Kill the launch on both Pis and relaunch with `k_gain:=0.8` instead of the default 1.5.

---

## Stopping the Test

Press `Ctrl+C` in the teleop terminal first. This sends a zero velocity, which triggers the OpenCR watchdog and stops both robots. Then `Ctrl+C` the launch on each Pi. The lifecycle manager will call `deactivate` on `conveyor_base_node`, which zeroes the motors through the serial link before shutting down.

Do not kill the Pi launch while the robots are still moving — the watchdog timeout is 500 ms so the motors will stop on their own, but it is cleaner to let the lifecycle shutdown handle it.

---

## Troubleshooting

**Teleop key presses have no effect on the robots**
Check that `/virtual_center/cmd_vel` is publishing: `ros2 topic hz /virtual_center/cmd_vel`. If the rate is 0, the remap in the teleop launch command may be wrong. Confirm the `-r` argument is exactly `cmd_vel:=/virtual_center/cmd_vel`.

**`ros2 service call /tb3_0/reset_odom` times out**
The `conveyor_base_node` lifecycle is not in the active state. Look at the Pi output for `ConveyorBaseNode activated`. If it never appears, the lifecycle manager may have failed to configure the node — usually because the serial port `/dev/ttyACM0` is not accessible. Run `ls -la /dev/ttyACM*` on the Pi to confirm the OpenCR shows up and that the `pi1` user has read/write permission on it (it should be in the `dialout` group: `sudo usermod -aG dialout pi1`).

**`colcon build` fails on the Pi with a missing package**
Run `rosdep install --from-paths src --ignore-src -r -y` inside `~/turtlebot3_ws/` to auto-install any missing ROS dependencies, then rebuild.

**The alignment node times out every time**
This means the EKF is not producing a valid pose within 10 seconds of launch. The most common cause is that `conveyor_base_node` has not activated yet, so no raw `/odom` is flowing into `ekf_node`. Watch the launch output for `ConveyorBaseNode activated` and confirm it appears before the alignment timeout fires. Increasing the lifecycle manager's `bond_timeout` or adding a longer `TimerAction` delay before the lifecycle manager starts can also help on a slow Pi boot.

**Robots drive in opposite directions**
The `my_offset` and `neighbor_offsets` values are likely swapped between the two launch commands. Double-check that tb3_0 has `my_offset:=0.0,0.3` and `neighbor_offsets:=0.0,-0.3`, while tb3_1 has the opposite. The rule is simple: your offset and your neighbor's offset must sum to zero.