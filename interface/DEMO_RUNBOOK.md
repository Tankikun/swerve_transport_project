# Swerve Transport Demo Runbook — `interface/v6-fuckyea`

SLAM-anchored cooperative-transport demo. Two robots carry a rigid
object from a clicked goal to wherever you click in the GUI; their
pose comes from RTAB-Map matching the OAK-D camera image against a
pre-built map of the lab.

This branch **requires localization**. There is no fake-pose fallback.
If RTAB-Map can't relocalize, the robots stay LOST in the GUI and
nothing moves. The branch that supports a no-localization fake demo
is [`interface/v5-final`](https://github.com/Tankikun/swerve_transport_project/tree/interface/v5-final).

---

## 0. One-time setup

### Laptop

```bash
wsl -d ubuntu-22.04
cd ~/swerve_transport_project
git status                    # → On branch interface/v6-fuckyea
git log --oneline -1

pip3 install flask numpy scipy requests
sudo apt install -y ros-humble-rosbridge-server ros-humble-tf2-ros
```

### Each Raspberry Pi

If your Pi has never been set up for this project, jump to **§C
First-time Pi setup** at the bottom. Otherwise just refresh:

```bash
ssh pi1@192.168.1.101                # or pi2@192.168.1.102
cd ~/swerve_transport_project
git fetch origin
git checkout interface/v6-fuckyea    # only if you weren't already on it
git pull
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```

The OpenCR firmware in `opencr_firmware/swerve_kinematics/` is
already flashed. No Arduino IDE needed.

### Pre-built map

You need a usable RTAB-Map `.db` at `~/maps/lab.db` (or override via
`db_path:=…`) on each Pi. If your teammate is still building it, no
demo. To build one: `ros2 launch swerve_bringup rtabmap_mapping.launch.py`
(see `MAPPING_RUN_LAPTOP.md`), drive a robot through the room, copy
the resulting `.db` to each Pi.

---

## 1. Place the robots on the floor

Anywhere inside the mapped lab. RTAB-Map handles the rest.

That said — placing them in a **visually rich** area (corner with
chairs, a posters wall, etc.) speeds up relocalization. A blank wall
or empty floor stalls the visual match.

Once both Pis are launched, the bridges are running, and the GUI is
open, hitting "📍 Set Initial Pose" lets you give RTAB-Map a hint
about where the robot roughly is — that converts a 30 s scan into a
1–2 s match.

Place the rigid object across both robots after the launch (§B.4–B.5)
but before clicking Send Goal — see Step 4 in §3.

---

## 2. Power the robots, sanity-check serial

On each Pi:

```bash
ssh pi1@192.168.1.101
ls -l /dev/ttyACM0           # exists → OpenCR is enumerated
dmesg | tail | grep ttyACM   # last line should mention /dev/ttyACM0
```

Don't proceed until both Pis see the device.

---

## 3. Network env vars (every ROS terminal)

```bash
unset RMW_IMPLEMENTATION RMW_ZENOH_CONFIG_FILE
export ROS_DOMAIN_ID=30
export FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/fastdds_peers.xml
source /opt/ros/humble/setup.bash
source ~/swerve_transport_project/ros2_ws/install/setup.bash
```

Each Pi already has the right `~/fastdds_peers.xml` and
`~/.bashrc` from §C. If you put it in bashrc, new SSH sessions
inherit it automatically and you can skip this step.

---

## 4. Launch terminals (5 total)

### 4.1 — Pi 1 (tb3_0)

```bash
ssh pi1@192.168.1.101
ros2 launch swerve_bringup conveyor.launch.py \
    robot_id:=tb3_0 \
    neighbors:=tb3_1 \
    my_offset:=0.0,0.25 \
    neighbor_offsets:=0.0,-0.25 \
    db_path:=$HOME/maps/lab.db
```

What gets spawned (per `conveyor.launch.py`):

- `conveyor_base_node` — serial bridge to OpenCR
- `ekf_node` — `/odom` + `/imu` + `/slam/pose` → `/ekf/odom` + `/pose`
- `laplacian_formation_node` — formation control with consensus on (k_gain=0.1)
- `leader_election_node` — bully election
- `path_follower_node` — leader-only, follows `/formation/path`
- `formation_size_node` — leader-only, publishes `/formation/footprint`
- `alignment_node` — leader-only, depth-based pre-run alignment
- `oak_camera` + `rtabmap_localization` — the visual SLAM stack

For the first 5–30 s after launch you'll see RTAB-Map scanning the
stored map for a visual match. Once it finds one:

```
[rtabmap]: Initialization successful! Loop closure: <NN>
```

— and `/tb3_0/rtabmap/localization_pose` starts publishing.

**Leave the terminal running.**

### 4.2 — Pi 2 (tb3_1)

```bash
ssh pi2@192.168.1.102
ros2 launch swerve_bringup conveyor.launch.py \
    robot_id:=tb3_1 \
    neighbors:=tb3_0 \
    my_offset:=0.0,-0.25 \
    neighbor_offsets:=0.0,0.25 \
    db_path:=$HOME/maps/lab.db
```

Same node set, mirrored offsets (50 cm formation).

### 4.3 — Laptop terminal 1: GUI server

```bash
wsl -d ubuntu-22.04
cd ~/swerve_transport_project/interface
python3 server.py
```

Wait for `Server running at http://localhost:5002`.

### 4.4 — Laptop terminal 2: rosbridge_server

```bash
wsl -d ubuntu-22.04
ros2 run rosbridge_server rosbridge_websocket
```

This is what makes the GUI's `/goal_pose` publish actually reach the
ROS network. Without it, Send Goal becomes a no-op.

### 4.5 — Laptop terminals 3+4: pose bridges

One per robot:

```bash
# Terminal 3a
python3 ~/swerve_transport_project/interface/ros_pose_bridge.py \
    --ros-args -p robot_id:=tb3_0

# Terminal 3b (separate terminal)
python3 ~/swerve_transport_project/interface/ros_pose_bridge.py \
    --ros-args -p robot_id:=tb3_1
```

Each bridge does:

1. **TF lookup** `map → tb3_X_base_link` at 10 Hz → POST `/pose/tb3_X`.
   Until RTAB-Map has relocalized, this lookup fails and the bridge
   posts `localized: false` (the GUI shows a red LOST pill).
2. **Polls** `/set_initial_pose/tb3_X`. When the user clicks "Set
   Initial Pose", republishes the click as `PoseWithCovarianceStamped`
   to **both** `/initialpose` (RTAB-Map seed) and
   `/tb3_X/initialpose` (ekf_node hard-reset).
3. **Tracks** `/tb3_X/slam/pose` arrivals to drive the GUI's
   "fresh visual match" badge colour.

### 4.6 — Laptop terminal 5: path_planner_node

```bash
wsl -d ubuntu-22.04
ros2 run swerve_formation path_planner_node --ros-args \
    -p map_path:=$HOME/swerve_transport_project/interface/map.json
```

Wait for:

```
Map loaded: HHHxWWW cells @ 0.020 m, X[…] Y[…]
path_planner_node ready. yaw_policy='free', target_spacing=0.15 m,
default formation_radius=0.45 m. Waiting for /goal_pose, /tb3_0/pose,
/tb3_1/pose.
```

### 4.7 — Browser

`http://localhost:5002`. The 3D map renders; both robot pills are
red **LOST** (correct — RTAB-Map hasn't relocalized yet, and there
is no fake fallback).

---

## 5. The demo (8 steps)

### Step 1 — Open the GUI

`http://localhost:5002`. The 3D map renders.

### Step 2 — Wait or hint Set Initial Pose for R1

Two paths:

**Patient.** Wait for RTAB-Map to find R1 on its own. Watch its Pi
terminal for `Initialization successful! Loop closure: <NN>`. R1's
pill flips LOST → LIVE within 5–30 s. Skip to Step 4.

**Hinted.** Click "📍 Set Initial Pose" → pick **R1** → click on
the GUI map at the spot where R1 physically is → drag yaw → click.
The bridge publishes the click to `/initialpose`. RTAB-Map
re-localizes against your hint instead of doing a full scan; expect
a match in 1–2 s.

The hint does NOT make the GUI render R1 anywhere. The icon only
appears once SLAM has actually matched.

### Step 3 — Same for R2

Same flow; R2's pill flips to LIVE once SLAM matches.

Once both are LIVE, the GUI shows the live formation box between
the two robots. `path_planner_node` is now receiving `/tb3_0/pose`
and `/tb3_1/pose` and is one click away from being able to plan.

### Step 4 — Place the object on the robots

Physical step. Lay the rigid object across both. Motors are idle
until Send Goal.

### Step 5 — Click the goal

Click anywhere clear on the floor. A goal marker drops. Adjust
orientation with the slider — that's the **final yaw the formation
will face on arrival.**

### Step 6 — Click "Send Goal"

The GUI publishes `/goal_pose` (PoseStamped, with the slider's yaw
in the quaternion) over rosbridge. You should see:

- **path_planner_node terminal:**
  ```
  goal received: (X.XX, Y.YY, yaw=NNN°)
  planning: VC=(cx,cy) → goal=(gx,gy), radius=0.45 m, yaw_policy='free'
  plan #N published: NN poses, vc_distance=X.YZ m
  ```
- **Leader's Pi terminal** (whichever robot wins leader election):
  ```
  Received NN waypoints.
  Became leader — path-follower active.
  ```

The dense waypoint dots render on the floor in the browser.

### Step 7 — Watch the cooperative walk

Motors spin up ~50 ms after Send Goal. Both robots strafe and rotate
along the path keeping the rigid 50 cm formation. The GUI's
markers follow the live SLAM-anchored pose at 10 Hz.

When the leader reaches the final waypoint xy, status flips
`FOLLOWING → ALIGNING` and the formation rotates in place to face
the slider yaw. Once within ±5°, status reports `REACHED`.

Roughly 15–20 s for a 2.5 m path at 0.18 m/s.

### Step 8 — Done

Robots stop in formation, facing the goal yaw. Object stays put.

To go again: just click a new goal, then Send Goal. The bridges keep
posting live SLAM poses; the planner replans from the current
formation centre.

---

## 6. Stopping

1. **First** `Ctrl-C` the rosbridge / pose-bridge / planner / GUI on
   the laptop — this stops `/virtual_center/cmd_vel` updates.
   The OpenCR's 5 s watchdog will then zero motors.
2. **Then** `Ctrl-C` each Pi's `conveyor.launch.py` — also zeros
   motors via `on_deactivate`.

**Emergency stop**: power-off the OpenCR (yank the USB if you must).
Motors stop instantly.

---

## 7. Tweak cheat-sheet

| Want to change | Where |
|---|---|
| Object length (inter-robot distance) | `my_offset` / `neighbor_offsets` on each Pi (set both to ±half the desired length) |
| Walking speed | `MAX_LINEAR` constant in `path_follower_node.py` |
| Formation drift correction strength | `k_gain:=0.1` on each Pi (raise → tighter, lower → smoother) |
| Disable formation drift correction | `enable_consensus:=false` |
| Number of waypoints in the planned path | `target_spacing` in `apf_smooth_path()` (smaller → more dots) |
| Map source | `db_path:=` on each Pi (RTAB-Map db) and `map_path:=` on path_planner_node (the planner's grid) |
| Camera mount calibration | `cam_x` / `cam_y` / `cam_z` on each Pi launch — measured per robot |

---

## 8. Troubleshooting

### Pills stay LOST forever after both Pis launch

RTAB-Map hasn't relocalized.

- Check the Pi's RTAB-Map log for `Initialization successful! Loop closure`.
  If you don't see that, RTAB-Map can't match. Move the robot to a
  more visually distinctive part of the room and try a hint click.
- Confirm the `.db` file exists: `ls -l ~/maps/lab.db` on the Pi.
- Confirm the camera is publishing: `ros2 topic hz /tb3_0/camera/rgb/image_raw`
  → ~15 Hz. If 0, USB problem or `oak_camera_node` failed.
- Confirm the camera mount TF is right (CLAUDE.md §"Camera Mount TF").
  Errors here translate directly to localization errors.

### Pills go LIVE then back to LOST during driving

- RTAB-Map briefly lost track. Most often: motion blur during fast
  in-place rotations, or pointing at a blank wall. The EKF
  dead-reckons through the gap; usually re-acquires within seconds.
- If it doesn't recover, the formation will drift. Stop, re-click
  Set Initial Pose, try again.

### `/formation/path` empty after Send Goal

- `ros2 topic echo /goal_pose --once` on the laptop should fire on
  every Send Goal. If empty, rosbridge isn't bridging the WebSocket.
  Restart Terminal 4.4.
- `path_planner_node`'s log will say "deferring plan" until both
  robot poses arrive. If it sits there, the EKFs aren't yet
  publishing `/tb3_X/pose` — check that both robot pills are LIVE.
- `ros2 topic info /formation/path` must show
  `Durability: TRANSIENT_LOCAL` on both publisher and subscriber.

### Robots receive `/virtual_center/cmd_vel` but motors don't spin

- `dialout` group missing — check §C.8.
- `conveyor_base_node` lifecycle didn't activate — Pi log should say
  `ConveyorBaseNode activated — forwarding commands to OpenCR`.

### Server crashes with `Address already in use`

```bash
pkill -f "interface/server.py"
```

---

## 9. What's running under the hood

```
Browser GUI (interface/index.html)
  │  HTTP    : POST /set_initial_pose/<id>   (Set Initial Pose click)
  │  HTTP    : poll /pose/<id>               (live robot marker, 5 Hz)
  │  WebSock : publish /goal_pose            (Send Goal click)
  ▼
laptop:
  ├─ server.py             :5002    Flask — serves the GUI + /pose mailbox
  ├─ rosbridge_server      :9090    WebSocket ↔ ROS bridge
  ├─ ros_pose_bridge tb3_0          ► click → /initialpose + /tb3_0/initialpose
  │                                 ► TF lookup map → tb3_0_base_link → POST /pose
  ├─ ros_pose_bridge tb3_1          (same for tb3_1)
  └─ path_planner_node              ► reads /goal_pose, /tb3_0/pose, /tb3_1/pose,
                                          /formation/footprint
                                    ► virtual centre C = (P0+P1)/2
                                    ► /formation/path (LATCHED, transient_local)
        ▲                  │
        │                  │
        │                  ▼  /tb3_X/initialpose, /goal_pose, /formation/path
        │                       (all over ROS, FastDDS, ROS_DOMAIN_ID=30)
        │
each Pi (conveyor.launch.py):
  ├─ conveyor_base_node     /tb3_X/cmd_vel ──serial→ OpenCR
  │                         OpenCR ──serial→ /tb3_X/odom + /tb3_X/imu + TF
  ├─ rtabmap_localization   OAK-D image vs lab.db → /tb3_X/rtabmap/localization_pose
  │                                                   → /tb3_X/slam/pose (via relay)
  │                         publishes map → tb3_X_odom (continuous correction)
  ├─ ekf_node               /odom (predict) + /imu (yaw) + /slam/pose (full)
  │                         + /initialpose (hard reset) → /tb3_X/ekf/odom + /tb3_X/pose
  ├─ leader_election_node   /formation/leader
  ├─ path_follower_node     /formation/path → /virtual_center/cmd_vel  (leader-only)
  ├─ laplacian_formation    feedforward + consensus → /tb3_X/cmd_vel
  ├─ formation_size_node    /formation/footprint  (leader-only)
  └─ alignment_node         depth-based pre-run spacing  (leader-only)

OpenCR firmware:
  Receives  "x_dot y_dot gamma_dot\n"
  Sends     "POSE  x y theta vx vy wz\n"  (33 Hz)
            "IMU   ax ay az gx gy gz yaw\n"  (~11 Hz)
```

The "where does pose come from" question, in this build, has one
answer: **RTAB-Map**. The EKF integrates IMU + odom for smooth
high-rate output, but the world-frame anchor is the visual match
against `lab.db`. Without that anchor the robot is LOST and nothing
moves.

---

# C. First-time Pi setup (one-time, persists)

Skip if `ros2 pkg list | grep swerve` already prints `swerve_bringup`
and `swerve_formation` from a previous session.

## C.1 Prerequisites

- Ubuntu 22.04 + ROS 2 Humble at `/opt/ros/humble/`.
- Pi has a fixed IP: pi1 = `192.168.1.101`, pi2 = `192.168.1.102`.
- SSH works.
- OpenCR USB cable plugged in. `ls /dev/ttyACM0` shows the device.

## C.2 Clone the repo

```bash
ssh pi1@192.168.1.101
cd ~
git clone https://github.com/Tankikun/swerve_transport_project.git
cd swerve_transport_project
git checkout interface/v6-fuckyea
```

## C.3 Install apt packages

```bash
sudo apt update
sudo apt install -y \
    ros-humble-tf2-ros \
    ros-humble-tf2-ros-py \
    ros-humble-tf2-py \
    ros-humble-rtabmap-ros \
    ros-humble-depthai-ros-driver \
    python3-pip \
    python3-serial \
    python3-numpy \
    python3-scipy
pip3 install --user dynamixel-sdk
```

## C.4 FastDDS peers

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

Replace `192.168.1.114` with your laptop's actual LAN IP.

## C.5 Bashrc — make ROS env vars permanent

```bash
cat >> ~/.bashrc << 'EOF'

# === swerve_transport_project ===
unset RMW_IMPLEMENTATION RMW_ZENOH_CONFIG_FILE
export ROS_DOMAIN_ID=30
export FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/fastdds_peers.xml
source /opt/ros/humble/setup.bash
[ -f ~/swerve_transport_project/ros2_ws/install/setup.bash ] && \
    source ~/swerve_transport_project/ros2_ws/install/setup.bash
EOF
source ~/.bashrc
```

## C.6 Build the workspace

```bash
cd ~/swerve_transport_project/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## C.7 Serial port permissions

```bash
groups            # check 'dialout' is listed
# If not:
sudo usermod -aG dialout $USER
sudo reboot
```

## C.8 Map database

Copy the latest `lab.db` from the laptop to each Pi:

```bash
# On the laptop:
rsync ~/maps/lab.db pi1@192.168.1.101:~/maps/lab.db
rsync ~/maps/lab.db pi2@192.168.1.102:~/maps/lab.db
```

(Build it once with `rtabmap_mapping.launch.py` — see
`MAPPING_RUN_LAPTOP.md`.)

## C.9 Camera mount calibration

The OAK-D mount values must match the physical install on each robot.
Edit them per Pi when launching `conveyor.launch.py`:

```bash
ros2 launch swerve_bringup conveyor.launch.py \
    robot_id:=tb3_0 \
    cam_x:=0.128 cam_y:=0.000 cam_z:=-0.0175 \
    ...
```

(The default 0.10 / 0.00 / 0.15 is a placeholder — measure your robot.)

## C.10 Verify

```bash
echo $ROS_DOMAIN_ID                  # 30
ros2 pkg list | grep swerve          # swerve_bringup, swerve_formation
ls -l /dev/ttyACM0                   # crw-rw---- root dialout
ros2 launch swerve_bringup conveyor.launch.py --show-args | head -20
```

If all four pass, repeat for the other Pi. Then on the laptop:

```bash
ros2 daemon stop && ros2 daemon start
ros2 topic list                      # should be empty until launches start
```

Once both Pis launch and RTAB-Map relocalizes:

```bash
ros2 topic hz /tb3_0/pose            # ~33 Hz
ros2 topic hz /tb3_0/slam/pose       # ~5 Hz once SLAM matches
ros2 run tf2_ros tf2_echo map tb3_0_base_link
```

Now you're cleared to run §4 → §5 from a fresh state.
