# Swerve Transport Demo Runbook — `interface/v5-final`

Two ways to run the demo:

| Mode | What moves | When to use |
|---|---|---|
| **A — Fake** | Robots animate **only on screen** (laptop only) | Practice the GUI flow, dry-run before bringing the robots online, or fall back if the network/hardware is misbehaving on the day of the demo. |
| **B — Real** | Robots **physically drive** along the planned path; GUI mirrors their live position | The actual demo: place the two robots on the floor, set their poses in the GUI, click goal, watch them carry the object to it. |

The GUI flow (steps 1–8 in §3) is **identical** in both modes. Only the
infrastructure that runs underneath differs.

> **First time on a Pi?** Jump to **§C First-time Pi setup** at the
> bottom and finish all of C.1–C.9 on each Pi before attempting Mode B.
> Without C.5 (FastDDS peers) and C.6 (bashrc env vars) the Pi will
> not be discoverable from the laptop.

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

### Each Raspberry Pi (Mode B only)

If your Pi has **never been set up for this project**, jump to **§C
First-time Pi setup** at the bottom of this file. Do that first, come
back here.

If the Pi was already set up in a previous session, just refresh:

```bash
ssh pi1@192.168.1.101   # or pi2@192.168.1.102
cd ~/swerve_transport_project
git fetch origin
git checkout interface/v5-final     # only if you weren't already on it
git pull                            # picks up new code
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

## B.1 Place the robots on the floor — these positions ARE the demo

These two coordinates are **hardcoded** in three places (the GUI's
`FIXED_INITIAL_POSE`, the bridge's `fixed_pose` param in §B.8, and
your physical placement on the floor). All three must match — the
robots cannot localize, so we anchor everything to these known spots:

| Robot | Floor position (m) | Yaw | Side |
|---|---|---|---|
| R1 (`tb3_0`) | x = +0.453, y = -1.437 | 103° (CCW from +X) | left |
| R2 (`tb3_1`) | x = +0.940, y = -1.325 | 103°               | right |

Distance between centres = 50 cm — that's the carried-object length.
At yaw=103°, both robots face the same direction with R1 on the left
side of the formation and R2 on the right.

**Place the robots first, then everything else.** Use a tape measure
or marked floor spots — drift in physical placement maps directly into
where the path "really" ends up vs. where the GUI shows the goal. ±5 cm
is fine; ±20 cm and the robots may end up far from the GUI's goal
marker.

Put the rigid object across both robots once their motors are idle
(after §B.4–B.5) but before you click Send Goal — see §3 step 4.

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
    my_offset:=0.0,0.25 \
    neighbor_offsets:=0.0,-0.25
```

`my_offset:=0.0,0.25` says R1 sits **+0.25 m on the formation's body Y
axis** (the left side). Half of the 50 cm object length.

What the defaults give you (**no need to type these**):

- **`enable_slam:=false`** (you typed this above) — skips RTAB-Map /
  camera; EKF runs on wheel odom + IMU only; a static `map →
  tb3_0_odom` identity TF stands in for the SLAM-published one.
- **`enable_consensus:=true`** (default) — laplacian_formation_node
  uses **both** robots' `/ekf/odom` to compute the inter-robot pose
  error and adds a small velocity correction to keep the formation
  rigid against wheel-odom drift. With consensus off, the two robots
  rely entirely on feedforward decoding of `/virtual_center/cmd_vel`
  — fine for a few seconds but accumulates a few cm of drift over
  the demo run.
- **`k_gain:=0.1`** (default) — the consensus gain, in
  velocity-per-metre-of-error units. 0.1 gives gentle mm/s
  corrections for cm-scale errors. If the formation visibly wobbles
  during the run, drop to `k_gain:=0.05`. If it drifts apart by more
  than ~10 cm, raise to `0.2`.

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
    my_offset:=0.0,-0.25 \
    neighbor_offsets:=0.0,0.25
```

The two robots' `my_offset` values are mirror images of each other.
Both `0.25` numbers describe a **50 cm**-wide formation; if you want a
different object length, set both robots' offsets to **half** the new
length (and update the §B.1 placement coords + §B.8 `fixed_pose`
values to match).

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

`ros_pose_bridge.py` does two jobs:
1. When the user clicks "Set Initial Pose", **publishes the FIXED
   coords** (matching §B.1) to `/{rid}/initialpose`, anchoring the
   EKF — and therefore navigation — at the robot's known physical
   spot. The user's click position itself is ignored — see
   `fixed_pose` below.
2. **Continuously POSTs `/{rid}/ekf/odom` back to the GUI** so the
   robot's marker on the map tracks the EKF state as the wheels turn.

Run **one instance per robot** (two terminals total, or use `tmux`):

```bash
# Terminal 3a — R1
wsl -d ubuntu-22.04
# env-var block from §B.3
cd ~/swerve_transport_project
python3 interface/ros_pose_bridge.py --ros-args \
    -p robot_id:=tb3_0 \
    -p use_ekf_topic:=true \
    -p fixed_pose:='0.453,-1.437,103'

# Terminal 3b — R2  (separate terminal)
wsl -d ubuntu-22.04
# env-var block from §B.3
cd ~/swerve_transport_project
python3 interface/ros_pose_bridge.py --ros-args \
    -p robot_id:=tb3_1 \
    -p use_ekf_topic:=true \
    -p fixed_pose:='0.940,-1.325,103'
```

Param meanings:

- **`use_ekf_topic:=true`** — no-SLAM mode: take the GUI pose feed
  directly from `/{robot_id}/ekf/odom` instead of looking up a TF
  chain through RTAB-Map (which isn't running).
- **`fixed_pose:='x,y,yaw_deg'`** — replaces the user's clicked
  coordinates with these hardcoded values. The "Set Initial Pose"
  click then becomes a *trigger* only: it tells the system "yes, R1
  is now at its pre-arranged spot." Required for this no-localisation
  flow. The values **must match §B.1** and the GUI's
  `FIXED_INITIAL_POSE` in `interface/index.html`.

On startup each bridge logs:

```
Fixed-pose override active: x=+0.453 y=-1.437 yaw=+103.0°
(/set_initial_pose hints will be replaced with this).
```

If you don't see that line, the bridge will use whatever the user
clicks — and the EKF will reset to a wrong position, putting the
real robot's path far from the GUI's path.

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
| Initial X/Y/yaw of R1 or R2 (Mode B) | `fixed_pose:='x,y,yaw_deg'` on the bridge (§B.8) **AND** the matching `FIXED_INITIAL_POSE` constants in `interface/index.html` |
| Object length (Mode B) | `my_offset` / `neighbor_offsets` launch args on each Pi (both robots to ±half the desired length); update §B.1 placement to match |
| Walking speed (Mode A) | `--speed 0.25` on Terminal 2 |
| Walking speed (Mode B) | `MAX_LINEAR` constant in `navigation_node.py` (default 0.18 — keep below the OpenCR's ~0.198 m/s firmware clamp) |
| Formation drift correction strength (Mode B) | `k_gain:=0.1` on each Pi's launch (raise → tighter formation but risk oscillation, lower → looser but smoother) |
| Disable formation drift correction (Mode B) | `enable_consensus:=false` on each Pi's launch — robots run pure feedforward, drift independently |
| Number of waypoints in the planned path | `target_spacing` parameter in `apf_smooth_path()` in `interface/astar_planner.py` (smaller → more dots, larger → fewer) |
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
- Verify `fixed_pose:=…` is set on each bridge instance — the startup
  log must say `Fixed-pose override active: …`. If absent, the bridge
  is using whatever the user clicked, which won't match the physical
  placement.

### Mode B — Robots drive, but in the wrong direction

- Almost always: the physical placement (§B.1) doesn't match the
  hardcoded `fixed_pose` values. Re-measure the robots' floor
  positions and orientations. If they're 30° off in yaw the path
  curves the wrong way.
- Or `my_offset` / `neighbor_offsets` (§B.4 / §B.5) don't match the
  formation geometry. R1 should be at body +Y (left), R2 at body -Y
  (right) for the default placement.

### Mode B — Robot drifts away from formation

- The Laplacian consensus correction is **on** by default in this
  branch (`enable_consensus:=true` in `conveyor.launch.py`). It uses
  both robots' `/ekf/odom` to compute the inter-robot pose error
  and applies a small velocity correction to keep the formation rigid
  against wheel-odom drift. Verify it's actually running:
  ```bash
  ros2 topic echo /formation/state --once   # should show 2 poses
  ros2 param get /laplacian_formation_node_tb3_0 enable_consensus  # → true
  ```
  If `enable_consensus:=false` somehow leaked through (you passed it
  on the launch line), the two robots fall back to pure feedforward
  and accumulate independent drift.
- If the formation **wobbles or oscillates**, the gain is too high.
  Restart the launch on each Pi with `k_gain:=0.05`.
- If the formation **drifts apart by >10 cm** during the run, the
  gain is too low. Restart with `k_gain:=0.2`. Don't go above ~0.3 —
  the consensus-correction term will dominate the feedforward and
  the robots oscillate around each other.
- For consensus to fire, both robots must have published a recent
  `/{rid}/ekf/odom` within 1.0 s. If only one robot is up (you're
  testing alone), the laplacian node falls back to feedforward
  silently — that's expected.

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
  ├─ ros_pose_bridge tb3_0          fixed_pose:='0.453,-1.437,103'
  │                                 click → /tb3_0/initialpose at FIXED
  │                                 /tb3_0/ekf/odom → POST /pose
  └─ ros_pose_bridge tb3_1          fixed_pose:='0.940,-1.325,103'
                                    click → /tb3_1/initialpose at FIXED
                                    /tb3_1/ekf/odom → POST /pose
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
  ├─ laplacian_formation   feedforward: /virtual_center/cmd_vel + my_offset
  │                        consensus  : /tb3_0/ekf/odom + /tb3_1/ekf/odom
  │                                     (closes loop on inter-robot pose)
  │                        → /tb3_X/cmd_vel
  ├─ formation_size_node   formation envelope (leader-only)
  └─ static_transform_publisher  map → tb3_X_odom (identity, no-SLAM stand-in)

OpenCR firmware (already flashed):
  Receives  "x_dot y_dot gamma_dot\n"
  Sends     "POSE  x y theta vx vy wz\n"  (33 Hz)
            "IMU   ax ay az gx gy gz yaw\n"  (~11 Hz)
```

---

# C. First-time Pi setup (one-time, persists)

Skip this entire section if a previous session already set up the Pi.
Symptom that you can skip: `ros2 pkg list | grep swerve` from inside
an SSH session prints `swerve_bringup` and `swerve_formation`.

If anything below is already in place on your Pi (because of an
earlier project), just rerun the missing steps and ignore the rest.

## C.1 Prerequisites I'm assuming are done

- Ubuntu 22.04 installed on the Pi (Server or Desktop, doesn't matter).
- ROS 2 Humble installed at `/opt/ros/humble/` — check with
  `ls /opt/ros/humble/setup.bash`. If missing, follow the official
  install: <https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html>.
- Pi has a fixed IP on the lab network: pi1 = `192.168.1.101`, pi2 =
  `192.168.1.102`. Test with `ping 192.168.1.114` (the laptop) and
  `ping 192.168.1.10X` from the laptop.
- SSH works: `ssh pi1@192.168.1.101` from the laptop, password
  `raspberry` (or whatever you've set).
- The OpenCR's USB cable is connected to the Pi. `ls /dev/ttyACM0`
  on the Pi shows the device when the OpenCR is powered.

## C.2 Clone the repo

```bash
ssh pi1@192.168.1.101                    # or pi2@192.168.1.102
cd ~
git clone https://github.com/Tankikun/swerve_transport_project.git
cd swerve_transport_project
git checkout interface/v5-final
```

If you'd rather use SSH (so you don't get prompted for HTTPS auth on
push):

```bash
git remote set-url origin git@github.com:Tankikun/swerve_transport_project.git
```

## C.3 Install missing apt packages

The lab network's apt mirror has been flaky in past sessions
(HANDOFF_TO_TAN.md §3.2). If `apt update` complains about hash
mismatches, follow the .deb workaround in that doc — but try the
straight install first:

```bash
sudo apt update
sudo apt install -y \
    ros-humble-tf2-ros \
    ros-humble-tf2-ros-py \
    ros-humble-tf2-py \
    python3-pip \
    python3-serial \
    python3-numpy \
    python3-scipy
```

`tf2-ros-py` and `tf2-py` are needed by `conveyor_base_node`. Without
them you'll get `ModuleNotFoundError: No module named 'tf2_ros'` at
launch.

## C.4 Install Dynamixel SDK (Python)

The Pi side doesn't actually drive the motors itself (the OpenCR does
that), so this is only needed if you want to run any of the diagnostic
tools that talk to the bus directly:

```bash
pip3 install --user dynamixel-sdk
```

## C.5 FastDDS peers list

This is **required** — without it the Pi cannot discover the laptop
or the other Pi on a non-multicast network.

Create `~/fastdds_peers.xml` on the Pi (NOT in the repo):

```bash
cat > ~/fastdds_peers.xml << 'EOF'
<?xml version="1.0" encoding="UTF-8"?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
  <participant profile_name="default" is_default_profile="true">
    <rtps>
      <builtin>
        <initialPeersList>
          <locator><udpv4><address>127.0.0.1</address></udpv4></locator>
          <locator><udpv4><address>192.168.1.101</address></udpv4></locator>
          <locator><udpv4><address>192.168.1.102</address></udpv4></locator>
          <locator><udpv4><address>192.168.1.114</address></udpv4></locator>
        </initialPeersList>
      </builtin>
    </rtps>
  </participant>
</profiles>
EOF
```

The list is **identical on both Pis and the laptop** — every machine
includes its own IP plus all peers. `127.0.0.1` is critical for
intra-host discovery (omitting it caused subscriber-not-found bugs
last session per HANDOFF_TO_TAN.md §2). Adjust the laptop IP
(`192.168.1.114`) if your laptop's actual address differs — find it
with `ip addr` on the laptop's WSL.

## C.6 Bashrc — make ROS env vars permanent

```bash
cat >> ~/.bashrc << 'EOF'

# === swerve_transport_project ===
unset RMW_IMPLEMENTATION RMW_ZENOH_CONFIG_FILE
export ROS_DOMAIN_ID=30
export FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/fastdds_peers.xml
source /opt/ros/humble/setup.bash
# Source the project workspace if present (created by C.7)
[ -f ~/swerve_transport_project/ros2_ws/install/setup.bash ] && \
    source ~/swerve_transport_project/ros2_ws/install/setup.bash
EOF

# Apply now
source ~/.bashrc
```

After this, every new SSH session has the right env automatically.
You no longer need to retype the §B.3 block.

## C.7 Build the workspace

```bash
cd ~/swerve_transport_project/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

`--symlink-install` means future `git pull`s with Python edits don't
need a rebuild — they're picked up next time you launch a node.
Setup.py changes (new entry points, new YAMLs) DO need a rebuild.

## C.8 Serial port permissions

The user that runs the launch needs `dialout` group access for
`/dev/ttyACM0`. Check first:

```bash
groups
# If "dialout" is NOT in the list:
sudo usermod -aG dialout $USER
sudo reboot
# After reconnect, re-check with `groups`.
```

## C.9 Verify everything

After C.8's reboot (or fresh SSH session if no reboot was needed):

```bash
# 1. ROS env
echo $ROS_DOMAIN_ID                  # 30
echo $FASTRTPS_DEFAULT_PROFILES_FILE # /home/piN/fastdds_peers.xml

# 2. Workspace built and sourced
ros2 pkg list | grep swerve          # swerve_bringup, swerve_formation

# 3. Serial device exists and is readable
ls -l /dev/ttyACM0                   # crw-rw---- root dialout
groups | grep dialout                # confirms group access

# 4. ROS daemon and topics work
ros2 daemon stop && ros2 daemon start
ros2 topic list                      # may be empty (nothing else launched)

# 5. Launch dry-run — confirms our edited launch file parses
ros2 launch swerve_bringup conveyor.launch.py --show-args | grep enable_slam
# Should print:
#   'enable_slam':
#       When false, skip RTAB-Map / camera, ...
#       (default: 'true')
```

If all five pass, the Pi is ready. Repeat C.1–C.9 for the other Pi
(swap IP and hostname; everything else is identical).

## C.10 Pi-to-Pi + laptop discovery test

After both Pis are set up AND the laptop has the same env (see §B.3
or replicate C.5–C.6 in WSL):

On the laptop:
```bash
ros2 daemon stop && ros2 daemon start
ros2 topic list           # empty until launches start
```

On pi1:
```bash
ros2 launch swerve_bringup conveyor.launch.py robot_id:=tb3_0 enable_slam:=false
```

Back on the laptop:
```bash
ros2 topic list           # should now show /tb3_0/odom, /tb3_0/imu, /tb3_0/cmd_vel, etc.
ros2 topic hz /tb3_0/odom # ~33 Hz
```

If `ros2 topic list` from the laptop **doesn't** see the Pi's topics:
- Verify FastDDS peer file lists the laptop's IP correctly.
- Verify `ROS_DOMAIN_ID=30` matches on both.
- Check firewall on both sides (`sudo ufw status` should be `inactive` for the lab).
- Verify ping in both directions.

Then `Ctrl-C` the launch on pi1, repeat on pi2. Once both robots
respond independently, you're cleared to run §B end-to-end.
