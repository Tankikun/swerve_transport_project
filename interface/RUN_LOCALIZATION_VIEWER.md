# Live Localization Viewer — Self-Contained Runbook

This guide is **everything you need** to verify localization works using the GUI. Don't open any other doc.

You'll end up with a browser window showing:

- A **status pill** (green = localized, orange = dead-reckoning, red = lost)
- A **cyan cone** on the 3D map that moves as you drive the robot
- A simple visual proof that "the robot knows where it is"

**Time:** ~10 minutes from cold start.

---

## Terminal layout

You'll need **5 terminals on the laptop** + 1 browser window. Arrange them ahead of time:

| Terminal | Role |
|---|---|
| T1 | SSH'd into pi2 — runs Pi sensor stack |
| T2 | Laptop — Flask server (web GUI backend) |
| T3 | Laptop — RTAB-Map in localization mode |
| T4 | Laptop — ROS pose bridge (TF → HTTP) |
| T5 | Laptop — teleop keyboard |
| Browser | Chrome/Firefox at `http://localhost:5002` |

---

## What you need on disk before starting

Tick all four:

- [ ] A working `.db` from a successful mapping run at `~/maps/tb3_1_room.db` (10+ MB)
- [ ] A pre-generated `interface/map.json` matching that `.db` (preprocessing is out of scope for this doc — generate it with whatever tool your branch uses; a healthy `map.json` is 2–5 MB)
- [ ] Python packages: `pip install flask flask-cors requests`
- [ ] (Cross-host workflow only) `~/fastdds_peers.xml` exists. If you don't have it yet, copy `interface/fastdds_peers.xml.example` to `~/fastdds_peers.xml` and replace `YOUR_LAPTOP_LAN_IP` with the output of `ip -4 addr show | grep 192.168.`. Skip this if you're using **Step 0 alt** (Single-Laptop Simulation Mode).
- [ ] You're on a branch whose `interface/` folder contains `index.html`, `server.py`, and `ros_pose_bridge.py`. Verify:
  ```bash
  ls interface/index.html interface/server.py interface/ros_pose_bridge.py
  ```

---

## Step 0 (alt) — Single-Laptop Simulation Mode (no Pi, no robot)

If you only want to verify the **GUI ↔ bridge ↔ server** chain — for example to debug the web UI, fake a robot pose, or test `Set Initial Pose` without unplugging anything — skip the Pi, skip RTAB-Map, and skip the Fast-DDS XML entirely.

> **Important — do NOT export `FASTRTPS_DEFAULT_PROFILES_FILE` in this mode.**  
> The XML hardcodes `192.168.1.114` / `.101` / `.102`; on a laptop with a different LAN IP (or with the cable unplugged) Fast-DDS tries to bind to nonexistent interfaces and may silently fail discovery. Default multicast (`224.0.0.1:7400`) handles same-host discovery just fine.

Open **4 terminals** on the laptop. In every one, source ROS but leave `FASTRTPS_DEFAULT_PROFILES_FILE` unset:

```bash
unset FASTRTPS_DEFAULT_PROFILES_FILE
source /opt/ros/humble/setup.bash
source ~/swerve_transport_project/ros2_ws/install/setup.bash
```

Then in **T1**, fake the `map → tb3_1_base_link` TF that RTAB-Map would normally publish:

```bash
ros2 run tf2_ros static_transform_publisher \
    --x 1.0 --y 2.0 --z 0.0 \
    --yaw 0.5 --pitch 0.0 --roll 0.0 \
    --frame-id map --child-frame-id tb3_1_base_link
```

In **T2**, start the Flask server:

```bash
cd ~/swerve_transport_project/interface
python3 server.py --map map.json --port 5002
```

In **T3**, start the bridge:

```bash
cd ~/swerve_transport_project/interface
python3 ros_pose_bridge.py --ros-args -p robot_id:=tb3_1
```

In **T4**, sanity check:

```bash
ros2 run tf2_ros tf2_echo map tb3_1_base_link    # should print the transform
curl http://localhost:5002/pose                  # should return "localized": true
```

Open **`http://localhost:5002`** in your browser. You should see the cyan cone at (1.0, 2.0). Click **📍 Set Initial Pose** to verify the hint round-trip; the bridge will log `Published initial pose: x=… seq=…`.

To **make the cone move**, restart T1 with new `--x` / `--y` / `--yaw` values, or publish `/tf` directly in a loop:

```bash
ros2 topic pub -r 10 /tf tf2_msgs/msg/TFMessage \
  "{transforms: [{header: {frame_id: 'map'}, child_frame_id: 'tb3_1_base_link',
    transform: {translation: {x: 1.5, y: 2.0, z: 0.0},
                rotation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}]}"
```

To turn the LOC pill **green** (LIVE) instead of orange (DEAD-RECK), also publish a heartbeat on `/{robot_id}/slam/pose` — the bridge uses any message arrival on that topic as the "fresh visual match" signal:

```bash
ros2 topic pub -r 2 /tb3_1/slam/pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 2.0, z: 0.0}, orientation: {w: 1.0}}}"
```

When you're done with simulation mode, **skip directly to "Stopping the run"** at the bottom of this file. Resume from Step 1 below only when you're testing with the real Pi + robot.

---

## Step 1 — Physical setup

1. **Place the robot inside the area you mapped.** RTAB-Map can only localize where it has stored keyframes. If you put the robot in a corner you didn't drive through during mapping, the GUI badge will stay red forever.
2. **Turn lights on.** Visual SLAM matches the conditions the map was made under. Different lighting → different visual features → no match.
3. **Power on the robot** (toggle the OpenCR power switch). Wait ~5 seconds for the OpenCR LEDs to stabilize.
4. **Make sure all 8 Dynamixel motors are responding** — listen for the brief click as they apply torque.

---

## Step 2 — Confirm the OAK-D camera is connected to pi2

From your laptop:

```bash
ssh pi2@192.168.1.102 "lsusb | grep Movidius"
```

Expected output:

```
Bus 002 Device 003: ID 03e7:2485 Intel Movidius MyriadX
```

If the output is empty, the camera isn't enumerated. Re-seat the OAK-D's USB-C cable on the **blue USB-3 port** of pi2, wait 10 sec, and re-run.

---

## Step 3 — Sync pi2's clock

The Pi has no battery-backed RTC, so its clock resets every reboot. RTAB-Map will silently drop every frame if the clocks differ by more than a few seconds.

Run this once after every Pi reboot (skip if pi2 hasn't been rebooted since you last did it):

```bash
ssh -t pi2@192.168.1.102 "sudo systemctl disable --now systemd-timesyncd && sudo date -u -s \"$(date -u +'%Y-%m-%d %H:%M:%S')\""
```

Verify (the two epoch numbers should be within 5 of each other):

```bash
echo "laptop: $(date -u +%s)" ; ssh pi2@192.168.1.102 "echo \"pi2:    \$(date -u +%s)\""
```

Pi sudo password: `raspberry`.

---

## Step 4 — T1: Start the Pi sensor stack

In **Terminal 1**, SSH into pi2:

```bash
ssh pi2@192.168.1.102
```

Once you see `pi2@ubuntu:~$`, paste the following block all at once:

```bash
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/pi2/fastdds_peers.xml
export ROS_DOMAIN_ID=30
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch swerve_bringup rtabmap_pi_sensors.launch.py robot_id:=tb3_1 cam_x:=0.128 cam_y:=0.000 cam_z:=-0.0175
```

**Wait ~15 seconds** (the ~5-second pause after `Serial /dev/ttyACM0 @ 115200 opened.` is normal — OpenCR is homing). Then check that all 4 of these lines have appeared:

```
[oak_camera_node-1]    ... oak_camera_node ready (tb3_1) — rgb=640x400@15fps  ... depthai=3.5.0
[oak_camera_node-1]    ... pipeline running. device=OAK-D-LITE
[conveyor_base_node-3] ... ConveyorBaseNode activated
[ekf_node-4]           ... EKF node ready for tb3_1
```

**Leave T1 running for the rest of the session.** Don't Ctrl+C.

---

## Step 5 — T2: Start the Flask server

In **Terminal 2** (laptop, fresh shell):

```bash
cd ~/swerve_transport_project/interface
python3 server.py --map map.json --port 5002
```

Expected output:

```
Loading map: map.json
Map loaded: 60000 points
Server running at http://localhost:5002
```

**Leave T2 running.** This is your web backend.

The server exposes:

- `GET /` and `GET /map` for the GUI
- `POST /pose` and `GET /pose` for the live-pose relay
- `POST /set_initial_pose` and `GET /set_initial_pose` for the GUI's "Set Initial Pose" tool

---

## Step 6 — T3: Start RTAB-Map in localization mode

In **Terminal 3** (laptop, fresh shell). First source the ROS env:

```bash
source /opt/ros/humble/setup.bash
source ~/swerve_transport_project/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=30
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/$USER/fastdds_peers.xml   # skip in Single-Laptop Simulation Mode (Step 0 alt)
```

Then verify pi2's topics are visible:

```bash
ros2 daemon stop ; sleep 2 ; ros2 daemon start ; sleep 6
ros2 topic list | grep -E '/tb3_1/(camera|ekf/odom|odom$)'
```

You should see exactly **6 topics**:

```
/tb3_1/camera/depth/camera_info
/tb3_1/camera/depth/image_raw
/tb3_1/camera/rgb/camera_info
/tb3_1/camera/rgb/image_raw
/tb3_1/ekf/odom
/tb3_1/odom
```

If empty, see [Troubleshooting → Discovery](#discovery).

Now launch RTAB-Map:

```bash
ros2 launch swerve_bringup rtabmap_laptop_localization.launch.py \
    robot_id:=tb3_1 \
    db_path:=~/maps/tb3_1_room.db
```

Wait for `RTAB-Map started` (or `Initialization complete!`). Then RTAB-Map enters "global re-localization" mode — silently scanning the `.db` for a visual match to whatever the camera is currently seeing. **For the first 5–30 seconds you'll see no `/slam/pose` output — that's normal.**

**Leave T3 running.**

> Use `~`, NOT `$HOME`, for `db_path` — `os.path.expanduser` only expands `~` when it's the first character.

> **About the odom feedback loop:** the launch defaults to `odom:=/tb3_1/ekf/odom` which is technically a feedback loop, but in practice it works fine for verification. If you see jumpy poses (meter-scale jumps), see [Why the override](#why-the-override) for the proper fix.

---

## Step 7 — T4: Start the ROS pose bridge

This is the small Python service that reads the robot's pose from ROS TF and POSTs it to your Flask server every 100 ms. Without it, the GUI badge stays at `LOC: NO BRIDGE`.

In **Terminal 4** (laptop, fresh shell):

```bash
source /opt/ros/humble/setup.bash
source ~/swerve_transport_project/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=30
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/$USER/fastdds_peers.xml   # skip in Single-Laptop Simulation Mode (Step 0 alt)
cd ~/swerve_transport_project/interface
python3 ros_pose_bridge.py --ros-args -p robot_id:=tb3_1
```

Expected output (one line, then silence):

```
[INFO] [ros_pose_bridge]: ros_pose_bridge: robot_id=tb3_1 -> http://localhost:5002/pose @ 10.0 Hz
                          (initial-pose poll: http://localhost:5002/set_initial_pose)
```

If you see this every 5 seconds:

```
[WARN] TF map -> tb3_1_base_link not available; robot not localized yet
```

…that's NORMAL until RTAB-Map finds its first match. Drive the robot for 30 seconds in Step 10 (or use Step 9 to give it a hint), and the warning will stop.

**Leave T4 running.**

> The bridge talks to the server in **two directions**:
>
> - `POST /pose` → server stores the current TF every tick. The browser polls `GET /pose` to drive the LOC pill and the cyan cone.
> - `GET /set_initial_pose` → bridge polls the server for new GUI hints and republishes them once on `/initialpose` for RTAB-Map. This is what makes the "Set Initial Pose" button work.

---

## Step 8 — Open the GUI

In any browser:

```
http://localhost:5002
```

You should see:

- **Full panel**: 3D point cloud of your mapped room with a floor grid + axes gnomon at world origin
- **Header (right side)**: three status pills — `Awaiting map…` (turns green when the cloud loads), `ROS Disconnected` (informational; this app only uses ROS via the HTTP bridge for localization), and `LOC: …` (the localization pill)
- **Bottom bar**: X/Y/Z readouts, an orientation slider, and a **Send Goal** button
- A **📍 Set Initial Pose** button in the header

Until you give RTAB-Map a hint or drive a bit so it matches on its own, the cyan cone won't appear. That's expected.

**Useful 3D controls:**

- **Drag** to orbit
- **Scroll** to zoom
- **Double-click** to refit the camera to the cloud
- A short **click on the floor** sets a goal at that world position; drag-clicks (anything past 5 px of motion) are filtered out so they only orbit

---

## Step 9 — Click to set initial pose (this is the magic step)

Without this step, RTAB-Map has to do a global re-localization search through every keyframe in the map — slow and unreliable in real rooms. With this step, RTAB-Map gets a hint and converges in 1–2 seconds.

1. **Look at the 3D point cloud.** Find where the robot actually is in the room (visually estimate based on landmarks).
2. **Click the 📍 Set Initial Pose button** in the header. The button highlights and the cursor turns into a crosshair. The state is now `PLACING`.
3. **Click on the floor in the 3D view** at the position the robot is currently at. A cyan dot appears at that spot. The state moves to `SETTING_YAW`.
4. **Move the mouse without clicking.** As the cursor moves over the floor, an arrow grows from the dot showing the yaw direction (where the robot is facing). Camera orbit is auto-disabled during this step so the mouse can move freely.
5. **Click again** to confirm. The button briefly shows `✓ Pose sent — RTAB-Map matching…`.

To cancel without setting a pose: press `Escape`, or click the button again.

Within 1–3 seconds the status pill should turn 🟢 `LOC: LIVE` and the cyan cone should appear at your hinted location (possibly with a small refinement).

If it doesn't turn green within 5 seconds:

- Wrong room location — try clicking somewhere else
- Hint too far from any keyframe — drive the robot a bit so RTAB-Map sees fresh frames near the hint
- Map and current view differ too much — regenerate `map.json` from a current `.db` and reload the browser

---

## Step 10 — T5: Drive the robot until it localizes

In **Terminal 5** (laptop, fresh shell):

```bash
source /opt/ros/humble/setup.bash
source ~/swerve_transport_project/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=30
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/$USER/fastdds_peers.xml   # skip in Single-Laptop Simulation Mode (Step 0 alt)
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/tb3_1/cmd_vel
```

If "Package not found": `sudo apt install -y ros-humble-teleop-twist-keyboard` then re-run.

**Click on T5's window to focus it** (the keyboard input only works while T5 is the foreground window). Then:

1. Press `z` 5 times to slow max speed to ~0.07 m/s — safer for verification.
2. Drive a small loop:

   | Key | Action |
   |---|---|
   | `i` | forward |
   | `,` | backward |
   | `j` / `l` | rotate CCW / CW |
   | `u` `o` `m` `.` | diagonals |
   | `k` or space | stop |

3. Drive ~1–2 meters through visually rich parts of the room (varied walls, furniture, posters — NOT a featureless corner).

**Watch the browser tab** (you don't need to put it in the foreground — it polls every 200 ms).

---

## Step 11 — Watch the GUI

Within 30 seconds of the robot starting to drive, you should see:

| Sign | What it means |
|---|---|
| **Status pill turns green** with `LOC: LIVE x.xx,y.yy` | RTAB-Map matched a keyframe — localization is working ✅ |
| **A cyan cone appears** on the 3D point cloud at the robot's location and moves as you teleop | Pose is being broadcast and the bridge → server → browser chain works |
| **A green sphere appears too** (if rosbridge is running on `ws://localhost:9090`) | Independent confirmation from the rosbridge `/tb3_0/pose` subscription. Optional — the cyan cone alone is sufficient. |

If after **60 seconds** of driving the pill is still `SEARCHING`, see [Troubleshooting → SEARCHING never resolves](#searching).

---

## Status pill reference

| Pill text | Color | Meaning | What to do |
|---|---|---|---|
| `LOC: LIVE x.xx,y.yy` | 🟢 green | Localized; visual match within last 5 s | Done. You can stop here. |
| `LOC: DEAD-RECK Xs` | 🟠 orange | TF still tracking but no visual match in X seconds | Drive into a more featureful area; if it persists, the map has a hole there |
| `LOC: SEARCHING Xs` | 🔴 red | No `map → tb3_1_base_link` TF yet | Use Set Initial Pose, OR drive 30 cm in any direction with view of varied features |
| `LOC: STALE Xs` | 🔴 red | Pose data on the server is older than 2 s | T4 (bridge) died — restart it |
| `LOC: NO BRIDGE` | 🔴 red | Server up but no pose ever received | T4 was never started — go back to Step 7 |
| `LOC: SERVER DOWN` | 🔴 red | Browser can't reach `/pose` endpoint | T2 (server) died — restart it |

---

## Stopping the run

Stop in this order (commands stop first, sensor last):

1. **T5** (teleop) — `Ctrl+C`
2. **Browser** — close the tab
3. **T4** (pose bridge) — `Ctrl+C`
4. **T3** (rtabmap) — `Ctrl+C`
5. **T2** (Flask server) — `Ctrl+C`
6. **T1** (Pi sensors) — `Ctrl+C`

---

## <a name="why-the-override"></a>Why the odom feedback loop is theoretically a problem

The launch file's default remap tells RTAB-Map to read `/tb3_1/ekf/odom` as its odometry prior. But `ekf_node` itself fuses RTAB-Map's correction back in — that's a feedback loop:

```
RTAB-Map (correction) → ekf_node → /tb3_1/ekf/odom → RTAB-Map (input) → ...
```

In practice the loop gain is small and the default works fine for verification. **If you see jumpy poses** (the cyan cone jumping meters at a time), break the loop by editing `ros2_ws/src/swerve_bringup/launch/rtabmap_laptop_localization.launch.py` and changing:

```python
('odom', f'/{robot_id}/ekf/odom'),
```

to:

```python
('odom', f'/{robot_id}/odom'),
```

Then rebuild your laptop workspace:

```bash
cd ~/swerve_transport_project/ros2_ws && colcon build --packages-select swerve_bringup
source install/setup.bash
```

Re-launch rtabmap. RTAB-Map's odometry prior is now the **raw** wheel odom, which breaks the loop.

The canonical pose flow:

```
/tb3_1/odom (raw)  ──────►  ekf_node  ◄──── /tb3_1/slam/pose (RTAB correction)
                              │
                              ▼
                      /tb3_1/ekf/odom  (authoritative — read by Nav2 + laplacian)
```

`slam_pose_relay_node` (auto-started by the launch) is what converts RTAB's `PoseWithCovarianceStamped` to the `PoseStamped` that `ekf_node` already subscribes to.

---

## Troubleshooting

### <a name="searching"></a>SEARCHING never resolves

After driving for 60 s the pill is still red `SEARCHING`. Possible causes:

- **Use Step 9 first.** A click-to-hint usually resolves SEARCHING faster than blind driving.
- **Robot is outside the mapped area.** Pick it up and put it inside the area you drove through during mapping. RTAB-Map can't match a view it's never seen.
- **Lighting differs from mapping.** Re-map under the actual operating lighting.
- **Map is too small.** Re-do mapping with a longer drive (more keyframes).
- **`.db` is broken.** Verify by listing keyframe + link counts; want > 0 links.

If `ros2 topic echo /tb3_1/slam/pose` (run in any sourced terminal) is silent, RTAB-Map definitely hasn't matched. If it's actually publishing but the GUI still shows SEARCHING, the bridge is broken — see next.

### Bridge logs `TF map -> tb3_1_base_link not available` forever

The TF doesn't exist because RTAB-Map hasn't matched. Same fix as "SEARCHING never resolves" above. **Try Step 9 first.**

### `Set Initial Pose` button does nothing

Two things to check:

- Is the **map.json loaded**? (Status pill says `<N> pts · floor … · bounds …`.) If not, `Set Initial Pose` has nothing to raycast against.
- Is **T4 (the bridge) running**? Without the bridge, the hint is queued on the server but nothing ever publishes it to `/initialpose`. `LOC: NO BRIDGE` is the giveaway.

### Marker is in the wrong corner of the room

Map frame mismatch: `map.json` was generated from a different `.db` than what RTAB-Map is loading. Regenerate `map.json` from the matching `.db` and reload the browser.

### Marker is at the right place but rotated 180°

Yaw sign convention regression. The OpenCR firmware should negate the FK's `wz` per REP-103. Verify the patch is in place by grepping the firmware:

```bash
grep "sum_wz_num / sum_wz_den" "$HOME/swerve_transport_project/opencr_firmware/swerve_kinematics/turtlebot3_conveyor.ino"
```

Expected: `wz = (sum_wz_den > 1e-6f) ? -(sum_wz_num / sum_wz_den) : 0.0f;` (note the leading minus). If the minus is missing, the firmware needs re-flashing.

### `ros_pose_bridge.py` crashes immediately

Likely `requests` is missing in the Python you're running:

```bash
pip install requests
```

Or wrong ROS sourcing — `echo $ROS_DISTRO` should print `humble`.

### `requests.exceptions.ConnectionError` floods the bridge log

T2 (Flask) is unreachable. Confirm:

```bash
curl http://localhost:5002/
```

…returns the HTML. If not, T2 isn't running or port 5002 is in use.

### Status pill says `LIVE` but no cone appears

Browser console error. F12 → Console. Look for `mapData.metadata` undefined or similar. Reload the page; if persistent, regenerate `map.json` and reload again.

### Cone disappears when robot moves

The pose left the bounding box of `map.json`. Either the robot really did leave the mapped area, OR the map was generated with too tight a `--bbox`. Regenerate with a wider bounding box.

### <a name="discovery"></a>Discovery: Pi topics not visible from laptop in Step 6

- `~/fastdds_peers.xml` is missing the `127.0.0.1` loopback entries
- Or the laptop's IP in `<defaultUnicastLocatorList>` doesn't match your actual LAN IP

Find your laptop's LAN IP:

```bash
ip -4 addr show | grep 192.168.1.
```

Canonical `~/fastdds_peers.xml` (replace `192.168.1.114` with what `ip -4` printed):

```xml
<initialPeersList>
  <locator><udpv4><address>127.0.0.1</address>      <port>14910</port></udpv4></locator>
  <locator><udpv4><address>127.0.0.1</address>      <port>14912</port></udpv4></locator>
  <locator><udpv4><address>192.168.1.101</address>  <port>14910</port></udpv4></locator>
  <locator><udpv4><address>192.168.1.101</address>  <port>14912</port></udpv4></locator>
  <locator><udpv4><address>192.168.1.102</address>  <port>14910</port></udpv4></locator>
  <locator><udpv4><address>192.168.1.102</address>  <port>14912</port></udpv4></locator>
</initialPeersList>
<metatrafficUnicastLocatorList>
  <locator><udpv4><address>192.168.1.114</address></udpv4></locator>
</metatrafficUnicastLocatorList>
<defaultUnicastLocatorList>
  <locator><udpv4><address>192.168.1.114</address></udpv4></locator>
</defaultUnicastLocatorList>
```

After fixing: restart T3 (rtabmap), then `ros2 daemon stop ; sleep 2 ; ros2 daemon start`.

### Pi clock has slipped (`delay=` is huge in T3 log, or RTAB-Map drops every frame)

Re-do Step 3. If `systemd-timesyncd` re-enabled itself (some distros do this on every boot), the Step 3 command's `disable --now` makes it permanent.

### TF says "two or more unconnected trees"

Stale processes from a previous run. Restart T1.

---

## Cross-network setup (running server and bridge on different machines)

If T2 (server) is on machine A and T4 (bridge) is on machine B, point the bridge at A's IP:

```bash
python3 interface/ros_pose_bridge.py --ros-args \
    -p robot_id:=tb3_1 \
    -p server_url:=http://<machine-A-ip>:5002/pose \
    -p initial_pose_url:=http://<machine-A-ip>:5002/set_initial_pose
```

The server already listens on `0.0.0.0` (all interfaces). Just open port 5002 on machine A's firewall.

---

## What success looks like

- All 5 terminals running, no error spam in any of them
- Browser shows the 3D point cloud filling the panel
- Status pill is solid 🟢 `LOC: LIVE x.xx,y.yy`
- Cyan cone appears in the 3D scene at the robot's location and follows as you teleop
- Cone position roughly matches where the robot physically is in the room
- (Optional) Click anywhere on the floor in 3D → bottom bar shows world (X, Y, Z); **Send Goal** publishes that to ROS as `/goal_pose`

If all of the above is true, **localization is verified working**. The robot can now be plugged into Nav2, formation control, or whatever's next.
