# Swerve Transport Demo Runbook — `interface/v5-final`

Two ways to run the demo:

| Mode | What moves | When to use |
|---|---|---|
| **A — Fake** | Robots animate **only on screen** (laptop only) | Practice the GUI flow, dry-run before bringing the robots online, or fall back if the network/hardware is misbehaving on the day of the demo. |
| **B — Real** | Robots **physically drive** along the planned path; GUI mirrors their live position | The actual demo: place the two robots on the floor, set their poses in the GUI, click goal, watch them carry the object to it. |

The GUI flow (steps 1–8 in §3) is **identical** in both modes. Only the
infrastructure that runs underneath differs.

---

## 0. One-time setup

Run once per machine touched by either mode.

### Laptop

```bash
# Open Ubuntu (WSL) on Windows
wsl -d ubuntu-22.04

# Confirm branch
cd ~/swerve_transport_project
git status                  # → On branch interface/v5-final
git log --oneline -1

# Python deps for fake mode + the Flask server
pip3 install flask numpy scipy requests

# ROS deps for real mode (skip if you only want fake mode)
sudo apt install -y ros-humble-rosbridge-server ros-humble-tf2-ros
```

### Each Raspberry Pi (only needed for Mode B)

```bash
ssh pi1@192.168.1.101   # or pi2@192.168.1.102
cd ~/swerve_transport_project
git pull                 # picks up the new conveyor_base_node, ekf_node, navigation_node
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```

The OpenCR firmware (in `opencr_firmware/swerve_kinematics/`) is
**already flashed with IMU support**. You don't need to touch the
Arduino IDE.

---

# Mode A — Fake demo (laptop only)

Two terminals on the laptop, no robots, no ROS.

## A.1 Terminal 1 — start the GUI server

```bash
wsl -d ubuntu-22.04
cd ~/swerve_transport_project/interface
python3 server.py
```

Wait for:

```
Map loaded: 453438 points
Server running at http://localhost:5002
```

## A.2 Terminal 2 — fake pose publisher in walk mode

```bash
wsl -d ubuntu-22.04
cd ~/swerve_transport_project/interface
python3 -u fake_pose_publisher.py --walk --speed 0.18 \
    --r1-pose -0.59 -1.10 84 \
    --r2-pose -0.30 -1.10 84
```

`--r1-pose X Y YAW_DEG` and `--r2-pose X Y YAW_DEG` set where R1 and R2
appear on the map. The distance between them IS the carried-object
length.

## A.3 Browser

`http://localhost:5002`. Now jump to §3 and follow steps 1–8.

---

# Mode B — Real-robot demo

Five terminals total: 1 laptop GUI, 1 laptop rosbridge, 1 laptop pose-bridge,
2 Pis (one each).

## B.1 Place the robots on the floor

Pick two real-world spots that fit inside the room map. Default values
match what the GUI shows:

| Robot | Floor position (m) | Yaw |
|---|---|---|
| R1 (`tb3_0`) | x = -0.59, y = -1.10 | 84° (CCW from +X) |
| R2 (`tb3_1`) | x = -0.30, y = -1.10 | 84° |

Distance between them = 29 cm. Put the rigid object across both robots
once they're powered up but before you click Send Goal — see §3 step 4.

## B.2 Power the robots, sanity-check serial

On each Pi:

```bash
ssh pi1@192.168.1.101
ls -l /dev/ttyACM0           # exists → OpenCR is enumerated
dmesg | tail | grep ttyACM   # last line should mention /dev/ttyACM0
```

If `/dev/ttyACM0` is missing, replug the OpenCR USB and check `dmesg`
again. Don't proceed until both Pis see the device.

## B.3 Network env vars (on **every** terminal that runs ROS — both Pis and laptop)

```bash
unset RMW_IMPLEMENTATION RMW_ZENOH_CONFIG_FILE
export ROS_DOMAIN_ID=30
export FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/fastdds_peers.xml
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash      # on Pi
# OR on laptop:
# source ~/swerve_transport_project/ros2_ws/install/setup.bash
```

The `fastdds_peers.xml` file lists the unicast peers (laptop +
both Pis). The template is at `interface/fastdds_peers.xml.example` —
copy it to `~/fastdds_peers.xml` on each machine and edit the IPs to
match your network. Both Pis and the laptop must list each other.

Sanity check from the laptop after sourcing:

```bash
ros2 daemon stop && ros2 daemon start
ros2 topic list             # if both Pis aren't powered yet, this is empty — that's fine
```

## B.4 Pi 1 (tb3_0) — bring up the robot

```bash
ssh pi1@192.168.1.101
# (do the env-var block from §B.3 first)

ros2 launch swerve_bringup conveyor.launch.py \
    robot_id:=tb3_0 \
    enable_slam:=false \
    neighbors:=tb3_1 \
    my_offset:=0.0,0.145 \
    neighbor_offsets:=0.0,-0.145
```

What `enable_slam:=false` does:
- skips RTAB-Map and the camera entirely
- sets `ekf_node.use_slam:=false` so the EKF runs on wheel odom + IMU only
- adds a static `map → tb3_0_odom` identity TF so the GUI's `map`-frame
  click can be interpreted as the EKF's state directly

You should see a stream of `POSE …` and `IMU …` log lines from
`conveyor_base_node` (mirroring the OpenCR's serial output) plus the
usual leader-election heartbeats. **Leave this terminal running.**

## B.5 Pi 2 (tb3_1) — same launch with the offsets flipped

```bash
ssh pi2@192.168.1.102
# (env-var block from §B.3)

ros2 launch swerve_bringup conveyor.launch.py \
    robot_id:=tb3_1 \
    enable_slam:=false \
    neighbors:=tb3_0 \
    my_offset:=0.0,-0.145 \
    neighbor_offsets:=0.0,0.145
```

The two robots' `my_offset` values are mirror images of each other.
Both `0.145` numbers describe a **29 cm**-apart formation; if you want
a different object length, set both robots' offsets to **half** the new
length (e.g. for a 50 cm object: `0.0,0.25` on tb3_0 and `0.0,-0.25` on
tb3_1).

Once the second launch is up, in **either** Pi terminal you should see
`leader_election_node` settle on one robot:

```
[leader_election_node_tb3_0]: I am leader (priority 0)
```

The lower-priority robot becomes the formation "brain" — it computes
`/virtual_center/cmd_vel`, both robots' `laplacian_formation_node`s
follow it.

## B.6 Laptop terminal 1 — GUI server

```bash
wsl -d ubuntu-22.04
cd ~/swerve_transport_project/interface
python3 server.py
```

Same as fake mode. Wait for `Server running at http://localhost:5002`.

## B.7 Laptop terminal 2 — rosbridge (WebSocket → ROS)

```bash
wsl -d ubuntu-22.04
# env-var block from §B.3
ros2 run rosbridge_server rosbridge_websocket
```

This bridges the GUI's `/formation/path` WebSocket publication into the
ROS network. Without this, Send Goal still computes the path and shows
it on screen, but the navigation node never receives it.

You should see `Rosbridge WebSocket server started on port 9090`.

## B.8 Laptop terminal 3 — pose bridge (one per robot)

`ros_pose_bridge.py` reads the EKF pose and POSTs it to the GUI. Run
**one instance per robot** (two terminals total, or use `tmux`):

```bash
# Terminal 3a — R1
wsl -d ubuntu-22.04
# env-var block from §B.3
cd ~/swerve_transport_project
python3 interface/ros_pose_bridge.py --ros-args \
    -p robot_id:=tb3_0 \
    -p use_ekf_topic:=true

# Terminal 3b — R2  (separate terminal)
wsl -d ubuntu-22.04
# env-var block from §B.3
cd ~/swerve_transport_project
python3 interface/ros_pose_bridge.py --ros-args \
    -p robot_id:=tb3_1 \
    -p use_ekf_topic:=true
```

`use_ekf_topic:=true` is the no-SLAM mode — the bridge takes the pose
directly from `/{robot_id}/ekf/odom` instead of doing a TF lookup
through RTAB-Map. The GUI's "Set Initial Pose" hints reset the EKF
state and the bridge sees the new value on the next tick.

## B.9 Browser

`http://localhost:5002`. Now follow §3.

---

# 3. The demo — 8 steps (same in both modes)

## Step 1 — Open the GUI

`http://localhost:5002`. The 3D map renders.

## Step 2 — Set initial pose for R1

1. Click **📍 Set Initial Pose** → pick **R1**.
2. Click on the floor at the spot where R1 physically is.
3. Drag to set the yaw arrow, click again to confirm.

Wait ~2 s. R1's pill flips **LOST → LIVE**.

- **Mode A:** the published pose is the `--r1-pose` value, regardless
  of where you clicked (the click is just the trigger).
- **Mode B:** the EKF on tb3_0 gets a hard-reset to (your click x, y,
  yaw). The robot model in the GUI snaps to that location. The real
  robot doesn't move yet — its motors are idle and `cmd_vel` is zero.

## Step 3 — Set initial pose for R2

Same flow, pick **R2**. After confirm + ~2 s the R2 pill turns LIVE.

## Step 4 — Place the object on the robots

**Physical step**, Mode B only. Carefully lay the rigid object across
both robots so it spans both. They're idle and won't drive away from
under it.

(In Mode A, just imagine it.)

## Step 5 — Click the goal location

Click anywhere clear on the floor in the GUI. A goal marker drops.
Optionally adjust orientation with the slider.

## Step 6 — Click "Send Goal"

The button shows "Planning…" for a fraction of a second. You should see:

- ~50 yellow waypoint dots forming a smooth U-curve on the floor.
- Laptop GUI server logs:
  `[plan] wrote path_plan.json seq=N (NN VC waypoints, X.YZ m)`
- **Mode A:** Terminal 2 logs `[walk] new plan loaded: NN waypoints, …`
- **Mode B:** the leader's Pi logs
  `Waypoint sequence loaded: 56 points.` from `navigation_node`
  followed by `Became leader — navigation active.` (if it wasn't
  already).

## Step 7 — Watch the cooperative walk

- **Mode A:** the GUI animates both robots through the U-curve in
  ~14 s. No real motion.
- **Mode B:** the **real motors spin up** ~50 ms after Send Goal. Both
  robots strafe and rotate along the path keeping the rigid 29 cm
  formation. The GUI's robot icons mirror their live positions in real
  time.

Roughly 15 s for a 2.5 m path at 0.18 m/s.

## Step 8 — Done

Robots stop at the goal in formation. Object stays put.

- Run another goal: just click a new goal location and Send Goal again.
  No need to reset anything.
- Run from a fresh starting pose: click 📍 Set Initial Pose for either
  robot, confirm, then Send Goal.

---

# 4. Stopping

## Mode A

`Ctrl-C` in each terminal. Order doesn't matter.

## Mode B

1. **First** `Ctrl-C` the rosbridge / pose-bridge / GUI on the laptop
   (this stops `/virtual_center/cmd_vel` updates from reaching the
   robots — the OpenCR's 5 s watchdog will then zero motors).
2. **Then** `Ctrl-C` each Pi's `conveyor.launch.py` — this also zeroes
   motors via the `on_deactivate` lifecycle.

Order matters: stopping the robot launch first leaves the GUI shouting
into a void, which is harmless. Stopping the laptop side first relies
on the firmware watchdog, which is also fine.

**Emergency stop**: power-off the OpenCR (yank the USB if you must).
Motors stop instantly. The Pis can be resumed; firmware will home back
to center on next launch.

---

# 5. Tweak cheat-sheet

| Want to change | Where |
|---|---|
| Initial X/Y/yaw of R1 or R2 (Mode A) | `--r1-pose` / `--r2-pose` on the fake publisher |
| **Object length** (inter-robot distance, Mode A) | same — set R1 and R2 farther apart |
| Object length (Mode B) | `my_offset` / `neighbor_offsets` launch args on each Pi (set both robots to ±half the desired length) |
| Walking speed (Mode A) | `--speed 0.25` on Terminal 2 |
| Walking speed (Mode B) | `MAX_LINEAR` constant in `navigation_node.py` (default 0.18 — keep below the OpenCR's ~0.198 m/s firmware clamp) |
| Re-localization delay (Mode A) | `--delay-sec 0.5` on the fake publisher |
| Path smoother gains (advanced) | `apf_smooth_path()` defaults in `astar_planner.py` |

---

# 6. Troubleshooting

### Mode A and B — R1/R2 pill stays LOST

- Mode A: check Terminal 2 for `[fake] tb3_0: now LIVE`. If you don't
  see it, the GUI's hint POST never reached the server. Look at the
  GUI server log for `POST /set_initial_pose/tb3_0`.
- Mode B: check the pose-bridge terminal for that robot. The bridge
  must log a startup line like
  `ros_pose_bridge: robot_id=tb3_0 -> http://localhost:5002 …`. If you
  see `TF map -> tb3_0_base_link not available` or
  `/tb3_0/ekf/odom not received yet`, the Pi's launch is incomplete.
  `ros2 topic hz /tb3_0/ekf/odom` from the laptop should report ~33 Hz
  once it's up.

### Mode B — No yellow waypoints after Send Goal

- Check the GUI server (laptop terminal 1) for `[plan] A* failed: …`.
  Most common: start or goal too close to walls. Click somewhere with
  more clearance.
- Check rosbridge (terminal 2) — must say
  `[INFO] Started server.` and a `Client connected` line when the
  browser tab is open. If the browser console (DevTools → Console)
  shows `WebSocket connection failed`, rosbridge isn't running or the
  port (9090) is blocked.

### Mode B — Robots don't move

- `ros2 topic echo /virtual_center/cmd_vel --once` on the laptop should
  show non-zero values immediately after Send Goal. If it's silent,
  the `navigation_node` never received the path.
  - Check `ros2 topic echo /formation/path --once` after Send Goal.
    Should show 50+ poses. If empty, rosbridge's WebSocket→ROS
    forwarding isn't reaching the Pi.
- `ros2 topic echo /tb3_0/cmd_vel` should show the per-robot twist
  (different x/y for each robot due to the offset). If silent, the
  `laplacian_formation_node` isn't running on that Pi.
- `ros2 topic echo /formation/leader` must publish exactly one robot
  ID. If empty, leader election isn't quorum-stable — usually a
  network problem (peers can't see each other). Re-check the
  `fastdds_peers.xml` on every machine.

### Mode B — Robot moves but GUI position is wrong

- Most likely the OpenCR firmware POSE has drifted from EKF state. The
  EKF resets on each Set Initial Pose hint, but firmware doesn't —
  this is fine for short runs. For very long runs, click Set Initial
  Pose again to re-anchor.
- Verify `ros_pose_bridge` is in `use_ekf_topic:=true` mode (look at
  its startup log). In TF mode it would lag or drift since SLAM isn't
  running.

### Mode B — Robot drifts away from formation

- The Laplacian consensus correction is **off** by default
  (`enable_consensus:=false` in `laplacian_formation_node.py`). With
  IMU + odom only, each robot's pose drifts independently — over a
  ~14 s run this is usually <10 cm, fine for the demo. If it's worse,
  re-flash the OpenCR after disconnecting the USB cable from the
  Dynamixel hub side first (sometimes the SCB sees a stale handshake).

### Server crashes with `Address already in use`

```bash
pkill -f "interface/server.py"
```

---

# 7. What's running under the hood

## Mode A (fake)

```
Browser GUI (index.html)
   │  POST /set_initial_pose/<id>
   │  POST /pose          ← polled GET
   │  POST /plan
   ▼
server.py (Flask :5002) → astar_planner.compute_plan()
   │  writes path_plan.json on every Send Goal
   ▼
fake_pose_publisher.py --walk
   • polls /set_initial_pose/<id> for hints
   • polls path_plan.json mtime for new plans
   • walks a virtual centre along the dense waypoints with a trapezoidal
     speed ramp; each robot's pose = vc ⊕ R(vc_heading) · offset_local
   • POSTs /pose/<id> at 10 Hz
```

## Mode B (real)

```
Browser GUI (index.html)
  │  HTTP: POST /set_initial_pose/<id>
  │  HTTP: poll  /pose/<id>
  │  HTTP: POST /plan
  │  WebSocket → /formation/path
  ▼
laptop:
  ├─ server.py             :5002    Flask
  ├─ rosbridge_server      :9090    WebSocket ↔ ROS
  ├─ ros_pose_bridge tb3_0          /tb3_0/ekf/odom → POST /pose
  └─ ros_pose_bridge tb3_1          /tb3_1/ekf/odom → POST /pose
        ▲             │
        │             │
        │             ▼  /tb3_X/initialpose, /formation/path  (over ROS)
        │
ROS network (FastDDS, ROS_DOMAIN_ID=30)
        │
        ▼
each Pi (conveyor.launch.py enable_slam:=false):
  ├─ conveyor_base_node    /tb3_X/cmd_vel ──serial→ OpenCR
  │                        OpenCR ──serial→ /tb3_X/odom + /tb3_X/imu + TF
  ├─ ekf_node              /odom + /imu + /initialpose → /tb3_X/ekf/odom
  ├─ leader_election_node  /formation/leader
  ├─ navigation_node       /formation/path → /virtual_center/cmd_vel  (leader-only)
  ├─ laplacian_formation   /virtual_center/cmd_vel + my_offset → /tb3_X/cmd_vel
  ├─ formation_size_node   formation envelope (leader-only)
  └─ static_transform_publisher  map → tb3_X_odom (identity, no-SLAM stand-in)

OpenCR firmware (already flashed):
  Receives  "x_dot y_dot gamma_dot\n"
  Sends     "POSE  x y theta vx vy wz\n"  (33 Hz)
            "IMU   ax ay az gx gy gz yaw\n"  (~11 Hz)
```
