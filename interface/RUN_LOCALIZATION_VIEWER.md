# Live Localization Viewer — Self-Contained Runbook

This guide is **everything you need** to verify localization works using the GUI. Don't open any other doc.

You'll end up with a browser window showing:

- A **status pill** (green = localized, orange = dead-reckoning, red = lost)
- A **cyan cone** on the 3D map that moves as you drive the robot
- A simple visual proof that "the robot knows where it is"

**Time:** ~10 minutes from cold start (faster on subsequent runs).

> **Architecture (what runs where).** RTAB-Map runs **on the Pi**, not on the laptop. The Pi has the `.db`, the camera, the wheels — it does the entire localization pipeline locally and publishes `map → tb3_1_base_link` over the LAN. The laptop only runs the **GUI server** (`server.py`) and the **TF→HTTP bridge** (`ros_pose_bridge.py`). This is faster and more reliable than the old split-mode where the laptop did the SLAM — no cross-network image streaming.

---

## Terminal layout

You'll need **4 terminals** + 1 browser. Arrange them ahead of time:

| Terminal | Where | Role |
|---|---|---|
| **T1** | SSH'd into pi2 | RTAB-Map all-on-Pi (sensors + EKF + localization) |
| **T2** | Laptop | Flask server (web GUI backend) |
| **T3** | Laptop | ROS pose bridge (TF → HTTP, GUI hint → /initialpose) |
| **T4** | Laptop | Teleop keyboard |
| **Browser** | Laptop | Chrome/Firefox at `http://localhost:5002` |

---

## What you need on disk before starting

Tick all five:

- [ ] A working `.db` from a successful mapping run **on pi2** at `~/maps/tb3_1_room.db` (10+ MB). See the [New `.db`?](#new-db) section if you just regenerated one on the laptop and need to copy it across.
- [ ] A pre-generated `interface/map.json` **on the laptop** matching that `.db` (the GUI's 3D point cloud comes from this file — preprocessing is out of scope here; whatever script in this folder is current). Healthy size: 2–5 MB.
- [ ] Python packages on the laptop: `pip install flask flask-cors requests`
- [ ] (Cross-host workflow) `~/fastdds_peers.xml` exists on **both** the laptop and pi2. If you don't have it yet, copy `interface/fastdds_peers.xml.example` → `~/fastdds_peers.xml` and replace `YOUR_LAPTOP_LAN_IP` with the output of `ip -4 addr show | grep 192.168.`. Skip this if you're using **Step 0 alt** (Single-Laptop Simulation Mode).
- [ ] You're on a branch whose `interface/` folder contains `index.html`, `server.py`, and `ros_pose_bridge.py`:
  ```bash
  ls interface/index.html interface/server.py interface/ros_pose_bridge.py
  ```

---

## <a name="new-db"></a>Got a new `.db` file? Read this first

Every time you do a fresh mapping run (laptop-side, with `rtabmap_laptop_mapping.launch.py`), you end up with a new `.db` on the **laptop** at `~/maps/tb3_1_room.db`. To use it for localization with this guide, you must:

### 1. Copy the new `.db` to pi2

The Pi runs RTAB-Map locally and reads its `.db` from its own filesystem.

```bash
rsync -avh ~/maps/tb3_1_room.db pi2@192.168.1.102:~/maps/
```

Verify it landed:
```bash
ssh pi2@192.168.1.102 "ls -lh ~/maps/tb3_1_room.db"
```

### 2. Regenerate the GUI's `map.json` on the laptop

The browser's 3D point cloud is rendered from `interface/map.json`. If the underlying `.db` changed, this needs to be regenerated so the cloud and the live pose share the same world frame.

```bash
cd ~/swerve_transport_project/interface
# Use whichever preprocessing script is in your branch — examples:
python3 db_to_map_json.py --db ~/maps/tb3_1_room.db --out map.json
# or whatever your branch's pipeline calls for.
```

Healthy result: `interface/map.json` is 2–5 MB, no errors.

### 3. (Optional) Verify both copies match

```bash
md5sum ~/maps/tb3_1_room.db
ssh pi2@192.168.1.102 "md5sum ~/maps/tb3_1_room.db"
```

The two hashes must match. If they differ, the rsync didn't complete — re-run.

> **Why this matters**: if the laptop's `map.json` was generated from one `.db` and pi2 is localizing against a different `.db`, the cyan robot cone will appear in random parts of the room because the two frames don't align. Always regenerate `map.json` and rsync the `.db` together.

After this prep, continue with Step 1 below.

---

## Step 0 (alt) — Single-Laptop Simulation Mode (no Pi, no robot)

If you only want to verify the **GUI ↔ bridge ↔ server** chain — for example to debug the web UI or test `Set Initial Pose` without unplugging anything — skip the Pi, skip RTAB-Map, and skip the Fast-DDS XML entirely.

> **Important — do NOT export `FASTRTPS_DEFAULT_PROFILES_FILE` in this mode.**
> The XML hardcodes `192.168.1.114` / `.101` / `.102`; on a laptop with a different LAN IP (or with the cable unplugged) Fast-DDS tries to bind to nonexistent interfaces and may silently fail discovery. Default multicast (`224.0.0.1:7400`) handles same-host discovery just fine.

Open **3 terminals** on the laptop. In every one, source ROS but leave `FASTRTPS_DEFAULT_PROFILES_FILE` unset:

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

Open the browser at `http://localhost:5002`. The cyan cone should appear at world `(1.0, 2.0)` with yaw 0.5 rad. Click "📍 Set Initial Pose" and confirm the bridge log prints `Published initial pose`.

Skip the rest of this doc — that's the simulation mode complete.

---

## Step 1 — Physical setup

1. **Place the robot inside the area you mapped.** RTAB-Map can only localize where it has stored keyframes. If you put the robot in a corner you didn't drive through during mapping, the GUI badge will stay red forever (RTAB-Map can never match).
2. **Turn lights on.** Visual SLAM matches the same conditions the map was made under. Different lighting = different visual features = no match.
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

## Step 4 — T1: Start the all-on-Pi localization stack

This single launch on the Pi brings up: camera + camera→base_link TF + wheel odometry + EKF + RTAB-Map + slam_pose_relay. Everything in one command. The laptop only runs the GUI.

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
ros2 launch swerve_bringup rtabmap_localization.launch.py \
    robot_id:=tb3_1 \
    db_path:=~/maps/tb3_1_room.db \
    cam_x:=0.128 cam_y:=0.000 cam_z:=-0.0175
```

> **Use `~`, NOT `$HOME`** for `db_path` — `os.path.expanduser` only expands `~` when it's the first character of the string.

> **Camera mount values** (`cam_x/cam_y/cam_z`) must match the values used during mapping. The defaults here are tb3_1's measured values (camera 12.8 cm forward, on-centre, 1.75 cm below base_link).

**Wait ~20–30 seconds** for the boot sequence. Watch for these milestones in T1's output:

| Line (in roughly this order) | What it means |
|---|---|
| `oak_camera_node ready (tb3_1) — rgb=...@15fps  ... depthai=3.5.0` | Camera process up |
| `pipeline running. device=OAK-D-LITE` | Camera is streaming |
| `Serial /dev/ttyACM0 @ 115200 opened.` | OpenCR serial connected |
| (5-second pause is normal — OpenCR is homing) | |
| `ConveyorBaseNode activated` | Wheel odom flowing |
| `EKF node ready for tb3_1` | EKF subscribed |
| `RTAB-Map started` (or `Initialization complete!`) | Localization node running |

Once `RTAB-Map started` appears, the Pi enters "global re-localization" mode — silently scanning the `.db` for a visual match to whatever the camera is currently seeing. **For the first 5–30 seconds you'll see no `/slam/pose` output — that's normal. Use Step 8 (Set Initial Pose) to skip the wait.**

**Leave T1 running for the rest of the session.** Don't Ctrl+C unless you intentionally restart.

---

## Step 5 — T2: Start the Flask server (laptop)

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

- `GET /` and `GET /map` — for the GUI
- `POST /pose` and `GET /pose` — live-pose relay (bridge writes; browser reads)
- `POST /set_initial_pose` and `GET /set_initial_pose` — GUI's "Set Initial Pose" mailbox (browser writes; bridge reads)

---

## Step 6 — T3: Start the ROS pose bridge (laptop)

This is the small Python service that:
- Reads the robot's `map → tb3_1_base_link` TF every 100 ms and POSTs it to the Flask server (drives the LOC pill and the cyan cone)
- Polls the GUI's pending initial-pose hint and republishes it once on `/initialpose` for RTAB-Map (this is what makes the "Set Initial Pose" button work)

In **Terminal 3** (laptop, fresh shell):

```bash
source /opt/ros/humble/setup.bash
source ~/swerve_transport_project/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=30
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/$USER/fastdds_peers.xml
cd ~/swerve_transport_project/interface
python3 ros_pose_bridge.py --ros-args -p robot_id:=tb3_1
```

Expected output (one line, then mostly silence):

```
[INFO] [ros_pose_bridge]: ros_pose_bridge: robot_id=tb3_1 -> http://localhost:5002/pose @ 10.0 Hz
                          (initial-pose poll: http://localhost:5002/set_initial_pose)
```

If you see this every 5 seconds:

```
[WARN] TF map -> tb3_1_base_link not available; robot not localized yet
```

…that's NORMAL until RTAB-Map finds its first match. Use Step 8 (Set Initial Pose) and the warning will stop within 1–3 seconds.

**Leave T3 running.**

---

## Step 7 — Open the GUI

In any browser:

```
http://localhost:5002
```

You should see:

- **Full panel**: 3D point cloud of your mapped room with a floor grid + axes gnomon at world origin
- **Header (right side)**: three status pills — `Awaiting map…` (turns green when the cloud loads), `ROS Disconnected` (informational; this app uses ROS via the HTTP bridge, not directly), and `LOC: …` (the localization pill — initially red `SEARCHING`)
- **Bottom bar**: X/Y/Z readouts, an orientation slider, and a **Send Goal** button
- A **📍 Set Initial Pose** button in the header

Until you give RTAB-Map a hint or drive a bit so it matches on its own, the cyan cone won't appear. That's expected.

**Useful 3D controls:**

- **Drag** to orbit
- **Scroll** to zoom
- **Double-click** to refit the camera to the cloud
- A short **click on the floor** sets a goal at that world position; drag-clicks (anything past 5 px of motion) are filtered out so they only orbit

---

## Step 8 — Click to set initial pose (this is the magic step)

Without this step, RTAB-Map has to do a global re-localization search through every keyframe in the map — slow and unreliable in real rooms. With this step, RTAB-Map gets a hint and converges in 1–2 seconds.

1. **Look at the 3D point cloud.** Find where the robot actually is in the room (visually estimate based on landmarks).
2. **Click the 📍 Set Initial Pose button** in the header. The button highlights and the cursor turns into a crosshair. The state is now `PLACING`.
3. **Click on the floor in the 3D view** at the position the robot is currently at. A cyan dot appears at that spot. The state moves to `SETTING_YAW`.
4. **Move the mouse without clicking.** As the cursor moves over the floor, an arrow grows from the dot showing the yaw direction (where the robot is facing). Camera orbit is auto-disabled during this step so the mouse can move freely.
5. **Click again** to confirm. The button briefly shows `✓ Pose sent — RTAB-Map matching…`.

To cancel without setting a pose: press `Escape`, or click the button again.

Within 1–3 seconds the status pill should turn 🟢 `LOC: LIVE` and the cyan cone should appear at your hinted location (possibly with a small refinement). Pi-side, the T1 terminal will print something like `Localization succeeded` once the match lands.

If it doesn't turn green within 5 seconds:

- Wrong room location — try clicking somewhere else
- Hint too far from any keyframe — drive the robot a bit (Step 9) so RTAB-Map sees fresh frames near the hint
- Map and current view differ too much — regenerate `map.json` from a current `.db` and reload the browser
- `.db` on pi2 doesn't match `map.json` on laptop — see [New `.db`?](#new-db) above

---

## Step 9 — T4: Drive the robot (verify localization survives motion)

In **Terminal 4** (laptop, fresh shell):

```bash
source /opt/ros/humble/setup.bash
source ~/swerve_transport_project/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=30
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/$USER/fastdds_peers.xml
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/tb3_1/cmd_vel
```

If "Package not found": `sudo apt install -y ros-humble-teleop-twist-keyboard` then re-run.

**Click on T4's window to focus it** (the keyboard input only works while T4 is the foreground window). Then:

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

**Watch the browser tab.** The cyan cone should track your movement smoothly. If the cone freezes or lags far behind the robot's actual position, RTAB-Map lost the lock — re-do Step 8.

---

## Step 10 — Watch the GUI

You should see:

| Sign | What it means |
|---|---|
| Status pill solid 🟢 `LOC: LIVE x.xx,y.yy` | RTAB-Map is matching and `map → tb3_1_base_link` TF is fresh |
| Cyan cone moves with the robot | Bridge → server → browser chain works end to end |
| Pill flickers to 🟠 `DEAD-RECK Xs` briefly during fast motion | Normal — the visual matcher just hasn't caught up yet; will return to green within a few seconds |

If after **60 seconds** of driving the pill is still `SEARCHING`, see [Troubleshooting → SEARCHING never resolves](#searching).

---

## Status pill reference

| Pill text | Color | Meaning | What to do |
|---|---|---|---|
| `LOC: LIVE x.xx,y.yy` | 🟢 green | Localized; visual match within last 5 s | Done. You can stop here. |
| `LOC: DEAD-RECK Xs` | 🟠 orange | TF still tracking but no visual match in X seconds | Drive into a more featureful area; if it persists, the map has a hole there |
| `LOC: SEARCHING Xs` | 🔴 red | No `map → tb3_1_base_link` TF yet | Use Set Initial Pose (Step 8), OR drive 30 cm with view of varied features |
| `LOC: STALE Xs` | 🔴 red | Pose data on the server is older than 2 s | T3 (bridge) died — restart it |
| `LOC: NO BRIDGE` | 🔴 red | Server up but no pose ever received | T3 was never started — go back to Step 6 |
| `LOC: SERVER DOWN` | 🔴 red | Browser can't reach `/pose` endpoint | T2 (server) died — restart it |

---

## Stopping the run

Stop in this order (commands stop first, sensor last):

1. **T4** (teleop) — `Ctrl+C`
2. **Browser** — close the tab
3. **T3** (pose bridge) — `Ctrl+C`
4. **T2** (Flask server) — `Ctrl+C`
5. **T1** (Pi all-in-one) — `Ctrl+C` in the SSH session

---

## <a name="why-the-override"></a>About the odom feedback loop (advanced)

The launch file's default tells RTAB-Map to read `/tb3_1/ekf/odom` as its odometry prior. But `ekf_node` itself fuses RTAB-Map's correction back in — that's a feedback loop:

```
RTAB-Map (correction) → ekf_node → /tb3_1/ekf/odom → RTAB-Map (input) → ...
```

In practice the loop gain is small and the default works fine for verification. **If you see jumpy poses** (the cyan cone jumping meters at a time), break the loop by editing `ros2_ws/src/swerve_bringup/launch/rtabmap_localization.launch.py` and changing:

```python
('odom', f'/{robot_id}/ekf/odom'),
```

to:

```python
('odom', f'/{robot_id}/odom'),
```

Then rebuild **on pi2** (because that's where this launch runs):

```bash
ssh pi2@192.168.1.102 "source /opt/ros/humble/setup.bash && cd ~/ros2_ws && colcon build --packages-select swerve_bringup"
```

Re-launch (T1). RTAB-Map's odometry prior is now the **raw** wheel odom, which breaks the loop.

The canonical pose flow:

```
/tb3_1/odom (raw)  ──────►  ekf_node  ◄──── /tb3_1/slam/pose (RTAB correction)
                              │
                              ▼
                         /tb3_1/ekf/odom (authoritative)
```

---

## Troubleshooting

### <a name="searching"></a>SEARCHING never resolves

After 60 sec of driving + a Set Initial Pose hint, the pill is still red.

**Most likely causes (in order):**

1. **`.db` on pi2 doesn't match `map.json` on laptop.** Re-run the [New `.db`?](#new-db) prep — rsync the .db AND regenerate map.json.
2. **Robot is in a part of the room you didn't map.** Drive it back to a known mapped area, click Set Initial Pose there.
3. **Lighting changed since mapping.** Re-map under current lighting.
4. **OAK-D wedged** — check on pi2: `ssh pi2@192.168.1.102 "lsusb | grep Movidius"`. If empty, replug the camera.
5. **RTAB-Map didn't actually start.** Check T1's output for `RTAB-Map started`. If it crashed, scroll back for the error.

### Browser shows "Awaiting map…" forever

The Flask server isn't serving `interface/map.json`. Check:

- Is `map.json` present? `ls interface/map.json`
- Did `python3 server.py` print `Map loaded: N points`?
- Try `curl http://localhost:5002/map | head -c 200` — should return JSON, not an HTML error

### Set Initial Pose click doesn't trigger anything

- Did the cursor turn into a crosshair after you clicked the 📍 button? If not, the JS isn't running — open the browser dev console and look for errors.
- Is the bridge logging `Published initial pose` after you confirm the click? If not, check T3's console for `requests.exceptions` — server is unreachable.

### Cyan cone lags badly behind the real robot

- Network latency too high. Confirm laptop and pi2 are on the same LAN, not going through a slow Wi-Fi.
- Bridge running on a slow machine. Check CPU usage — if `ros_pose_bridge.py` is at 100%, the queue is overflowing.

### `LOC: STALE` or `LOC: NO BRIDGE`

- `STALE`: T3 (bridge) is running but hasn't POSTed in >2 sec. Restart T3.
- `NO BRIDGE`: T2 (server) is running but T3 was never started. Run Step 6.

### TF says "two or more unconnected trees"

The TF chain `map → tb3_1_odom → tb3_1_base_link` is broken somewhere. Restart T1 — the Pi may have stale TF publishers from a previous run.

### <a name="discovery"></a>Pi topics not visible from the laptop

Symptoms: `ros2 topic list` from the laptop shows no `/tb3_1/*` topics.

Causes (in order):

1. **`~/fastdds_peers.xml` missing the `127.0.0.1` loopback entry** — see the example file in the repo.
2. **Laptop IP in `<defaultUnicastLocatorList>` is wrong** — find yours with `ip -4 addr show | grep 192.168.1.`
3. **Pi's peers file points to a stale laptop IP** — same fix on pi2's `~/fastdds_peers.xml`.

After fixing: restart T1, T2, T3, AND `ros2 daemon stop ; sleep 2 ; ros2 daemon start` on the laptop.

### Pi clock has slipped

Symptoms in T1: rtabmap log shows huge `delay=` values, or "drops every frame."

Re-do Step 3. If `systemd-timesyncd` keeps re-enabling itself, the disable command's `--now` should make it permanent across boots.

---

## Cross-network setup (running server and bridge on different machines)

If T2 (server) is on machine A and T3 (bridge) is on machine B, point the bridge at A's IP:

```bash
python3 interface/ros_pose_bridge.py --ros-args \
    -p robot_id:=tb3_1 \
    -p server_url:=http://<machine-A-ip>:5002/pose \
    -p initial_pose_url:=http://<machine-A-ip>:5002/set_initial_pose
```

The server already listens on `0.0.0.0` (all interfaces). Just open port 5002 on machine A's firewall.

---

## What success looks like

- **All 4 terminals running**, no error spam in any of them
- Browser shows the 3D point cloud, no "Awaiting map…" stuck pill
- Status pill is solid 🟢 `LOC: LIVE x.xx,y.yy`
- Cyan cone on the 3D map moves smoothly as you teleop
- The cyan cone's position roughly matches where the robot physically is in the room

If all of the above is true, **localization is verified working**. The robot can now be plugged into Nav2, formation control, or whatever's next.
