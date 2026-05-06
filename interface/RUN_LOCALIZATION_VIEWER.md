# Live Localization Viewer — Self-Contained Runbook

This guide is **everything you need** to verify localization works using the GUI. Don't open any other doc.

You'll end up with a browser window showing:
- A **status pill** (green = localized, orange = dead-reckoning, red = lost)
- A **cyan triangle** on the 2D map and a **cyan cone** on the 3D map that moves as you drive the robot
- A simple visual proof that "the robot knows where it is"

**Time**: ~10 minutes from cold start (longer the first time you run `regenerate_map.sh`).

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
- [ ] `rtabmap-export` installed (in WSL: `sudo apt install -y ros-humble-rtabmap`)
- [ ] Python packages: `pip install flask flask-cors requests`
- [ ] You're on branch `feature/localization-seven` so the `interface/` folder has the new files (`ros_pose_bridge.py`, `regenerate_map.sh`):
  ```bash
  cd "/mnt/c/Users/seven/OneDrive/เดสก์ท็อป/turtlebot3_conveyor/_gitrepo"
  git fetch origin
  git checkout feature/localization-seven
  git pull origin feature/localization-seven
  ```

---

## Step 1 — Physical setup

1. **Place the robot inside the area you mapped.** RTAB-Map can only localize where it has stored keyframes. If you put the robot in a corner you didn't drive through during mapping, the GUI badge will stay red forever.
2. **Turn lights on**. Visual SLAM matches the same conditions the map was made under. Different lighting = different visual features = no match.
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

## Step 4 — Generate `map.json` from your `.db`

**Skip this step** if you already have a `map.json` in `interface/` from the same `.db` you'll be localizing against. Otherwise, do it once per fresh map.

In a temporary terminal:

```bash
cd "/mnt/c/Users/seven/OneDrive/เดสก์ท็อป/turtlebot3_conveyor/_gitrepo/interface"
chmod +x regenerate_map.sh
bash regenerate_map.sh ~/maps/tb3_1_room.db map.json
```

Takes ~30 sec. End with a line like:

```
[regenerate_map] done. 3.2M at map.json
```

If you see "ERROR: db not found", check that `~/maps/tb3_1_room.db` actually exists. If `rtabmap-export` is missing, install it:
```bash
sudo apt install -y ros-humble-rtabmap
```

Sanity check the file size — a healthy `map.json` is **2–5 MB**. Under 200 KB usually means the cleanup over-filtered the cloud; see `interface/README.md` for tuning the filter flags.

---

## Step 5 — T1: Start the Pi sensor stack

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

## Step 6 — T2: Start the Flask server

In **Terminal 2** (laptop, fresh shell):

```bash
cd "/mnt/c/Users/seven/OneDrive/เดสก์ท็อป/turtlebot3_conveyor/_gitrepo/interface"
python3 server.py --map map.json --port 5002
```

Expected output:
```
Loading map: map.json
Map loaded: 60000 points
Server running at http://localhost:5002
```

**Leave T2 running.** This is your web backend.

---

## Step 7 — T3: Start RTAB-Map in localization mode

In **Terminal 3** (laptop, fresh shell). First source ROS env:

```bash
source /opt/ros/humble/setup.bash
source ~/swerve_transport_project/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=30
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/$USER/fastdds_peers.xml
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

> **About the odom feedback loop**: the launch defaults to `odom:=/tb3_1/ekf/odom` which is technically a feedback loop, but in practice it works fine for verification. If you see jumpy poses (meter-scale jumps), see [Why the override](#why-the-override) for the proper fix.

---

## Step 8 — T4: Start the ROS pose bridge

This is the small Python service that reads the robot's pose from ROS TF and POSTs it to your Flask server every 100 ms. Without it, the GUI badge stays at `LOC: NO BRIDGE`.

In **Terminal 4** (laptop, fresh shell):

```bash
source /opt/ros/humble/setup.bash
source ~/swerve_transport_project/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=30
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/$USER/fastdds_peers.xml
cd "/mnt/c/Users/seven/OneDrive/เดสก์ท็อป/turtlebot3_conveyor/_gitrepo/interface"
python3 ros_pose_bridge.py --ros-args -p robot_id:=tb3_1
```

Expected output (one line, then silence):
```
[INFO] [ros_pose_bridge]: ros_pose_bridge: robot_id=tb3_1 -> http://localhost:5002/pose @ 10.0 Hz
```

If you see this every 5 seconds:
```
[WARN] TF map -> tb3_1_base_link not available; robot not localized yet
```
…that's NORMAL until RTAB-Map finds its first match. Drive the robot for 30 seconds in Step 10 and the warning will stop.

**Leave T4 running.**

---

## Step 9 — Open the GUI

In any browser:

```
http://localhost:5002
```

You should see:
- **Left panel**: 3D point cloud of your mapped room
- **Right panel**: 2D occupancy grid (top-down view of the map)
- **Header (far right)**: a status pill that initially says `LOC: SEARCHING` (red)

Until you drive the robot a bit, RTAB-Map hasn't matched, so the cyan markers don't appear yet. That's expected.

---

## Step 9.5 — Click to set initial pose (this is the magic step)

Without this step, RTAB-Map has to do a global re-localization search
through every keyframe in the map — slow and unreliable in real rooms.
With this step, RTAB-Map gets a hint and converges in 1–2 seconds.

1. **Look at the 2D map on the right** of the GUI. Find where the robot
   actually is in the room (visually estimate based on landmarks).
2. **Click the "📍 Set Initial Pose" button** in the header. The button
   highlights and the cursor turns into a crosshair.
3. **Click on the 2D map** at the position the robot is currently at.
   A cyan dot appears.
4. **Move the mouse** to indicate which way the robot is facing — an
   arrow extends from the dot showing the yaw.
5. **Click again** to confirm.
6. The button briefly shows "✓ Pose sent — RTAB-Map matching…"

Within 1–3 seconds the status pill should turn 🟢 LOC: LIVE and the
cyan robot marker should appear on the map at your hinted location
(possibly with a small refinement).

If it doesn't turn green within 5 seconds:
- Wrong room location — try clicking somewhere else
- Hint too far from any keyframe — drive the robot a bit so RTAB-Map sees
  fresh frames near the hint
- Map and current view differ too much — see Step 4 (regenerate map.json)
  and ensure the .db is current

To cancel without setting a pose: press Escape, or click the button again.

---

## Step 10 — T5: Drive the robot until it localizes

In **Terminal 5** (laptop, fresh shell):

```bash
source /opt/ros/humble/setup.bash
source ~/swerve_transport_project/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=30
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/$USER/fastdds_peers.xml
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
| **A cyan triangle appears** on the 2D map and moves as you teleop | Pose is being broadcast and the bridge → server → browser chain works |
| **A cyan cone appears** on the 3D point cloud at the robot's location | 3D rendering is working |

If after **60 seconds** of driving the pill is still `SEARCHING`, see [Troubleshooting → SEARCHING never resolves](#searching).

---

## Status pill reference

| Pill text | Color | Meaning | What to do |
|---|---|---|---|
| `LOC: LIVE x.xx,y.yy` | 🟢 green | Localized; visual match within last 5 s | Done. You can stop here. |
| `LOC: DEAD-RECK Xs` | 🟠 orange | TF still tracking but no visual match in X seconds | Drive into a more featureful area; if it persists, the map has a hole there |
| `LOC: SEARCHING Xs` | 🔴 red | No `map → tb3_1_base_link` TF yet | Drive 30 cm in any direction with view of varied features |
| `LOC: STALE Xs` | 🔴 red | Pose data on the server is older than 2 s | T4 (bridge) died — restart it |
| `LOC: NO BRIDGE` | 🔴 red | Server up but no pose ever received | T4 was never started — go back to Step 8 |
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

In practice the loop gain is small and the default works fine for verification. **If you see jumpy poses** (the cyan marker jumping meters at a time), break the loop by editing `ros2_ws/src/swerve_bringup/launch/rtabmap_laptop_localization.launch.py` and changing the line:

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

The canonical pose flow per `CLAUDE.md`:

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
- **Robot is outside the mapped area.** Pick it up and put it inside the area you drove through during mapping. RTAB-Map can't match a view it's never seen.
- **Lighting differs from mapping.** Re-map under the actual operating lighting.
- **Map is too small.** Re-do mapping with a longer drive (more keyframes).
- **`.db` is broken.** Verify with the keyframe count check in `MAPPING_RUN_LAPTOP.md` Step 13. Want > 0 links.

If `ros2 topic echo /tb3_1/slam/pose` (run in any sourced terminal) is silent, RTAB-Map definitely hasn't matched. If it's actually publishing but the GUI still shows SEARCHING, the bridge is broken — see next.

### Bridge logs `TF map -> tb3_1_base_link not available` forever
The TF doesn't exist because RTAB-Map hasn't matched. Same fix as "SEARCHING never resolves" above.

### Marker is in the wrong corner of the room
Map frame mismatch: `map.json` was generated from a different `.db` than what RTAB-Map is loading. Re-run Step 4 with the matching `.db`.

### Marker is at the right place but rotated 180°
Yaw sign convention regression. The OpenCR firmware should negate the FK's `wz` per REP-103. Verify the patch is in place by grepping the firmware:
```bash
grep "sum_wz_num / sum_wz_den" "/mnt/c/Users/seven/OneDrive/เดสก์ท็อป/turtlebot3_conveyor/_gitrepo/opencr_firmware/swerve_kinematics/turtlebot3_conveyor.ino"
```
Expected: `wz = (sum_wz_den > 1e-6f) ? -(sum_wz_num / sum_wz_den) : 0.0f;` (note the leading minus). If the minus is missing, the firmware needs re-flashing — see `MAPPING_RUN_LAPTOP.md` for the procedure.

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

### Status pill says `LIVE` but no marker appears
Browser console error. F12 → Console. Look for `mapData.metadata` undefined or similar. Reload the page; if persistent, regenerate `map.json` (Step 4).

### Marker disappears when robot moves
The pose left the bounding box of `map.json`. Either the robot really did leave the mapped area, OR the map was generated with too tight a `--bbox`. Re-run Step 4 with a wider bounding box per `interface/README.md`.

### <a name="discovery"></a>Discovery: Pi topics not visible from laptop in Step 7
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
    -p server_url:=http://<machine-A-ip>:5002/pose
```

The server already listens on `0.0.0.0` (all interfaces). Just open port 5002 on machine A's firewall.

---

## What success looks like

- All 5 terminals running, no error spam in any of them
- Browser shows the 3D point cloud + 2D map
- Status pill is solid 🟢 `LOC: LIVE x.xx,y.yy`
- Cyan triangle on the 2D map moves smoothly as you teleop
- Cyan cone on the 3D map follows
- The cyan triangle's position roughly matches where the robot physically is in the room

If all of the above is true, **localization is verified working**. The robot can now be plugged into Nav2, formation control, or whatever's next.
