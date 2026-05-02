# Mapping Run — Laptop Side (Self-Contained)

Builds a 3D map (`.db` file) by driving the robot through the room.
Everything is done from your laptop — the Pi side is driven via
SSH from one of the laptop terminals. **You do not need to open
any other guide.**

> **Architecture: SPLIT.** Laptop runs `rtabmap_slam` (heavy CPU);
> Pi runs only sensors (camera, TF, odom, EKF). Map (`.db`) is
> built on the laptop. Pi 4 stays cool.

You'll use **4 terminals on the laptop**:

| Terminal | Role |
|---|---|
| T1 | SSH'd into pi2 — runs the sensor launch |
| T2 | Native laptop — runs `rtabmap_slam` |
| T3 | Native laptop — runs teleop |
| T4 | Native laptop — (optional) monitor topic rates and rtabmap progress |

Follow the steps in order.

---

## Step 0. Pre-flight (do once, before opening terminals)

Confirm the room is ready:
- Lights on (visual SLAM needs features)
- No big mirrors / glass walls
- Move chairs / people that won't be present during the actual
  transport job

Plan a driving path that:
- Covers every area you'll later transport through
- Returns to the start point so RTAB-Map can close the loop
- Avoids sharp 90° turns at full speed

Have a tape mark on the floor for the robot's start position.

---

## Step 1. Open Terminal 1, SSH into pi2, start the sensor launch

```bash
ssh pi2@192.168.1.102

# Once you're in (prompt shows pi2@...):
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/pi2/fastdds_peers.xml
export ROS_DOMAIN_ID=30
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch swerve_bringup rtabmap_pi_sensors.launch.py \
    robot_id:=tb3_1 \
    cam_x:=0.128  cam_y:=0.000  cam_z:=-0.0175
```

**Wait for these lines to appear in the log** (in roughly this
order, ~10–15 sec total):
```
[oak_camera_node-1]    ... oak_camera_node ready (tb3_1) — depthai=3.5.0
[oak_camera_node-1]    ... pipeline running. device=OAK-D-LITE
[static_transform_publisher-2] ... Spinning until stopped - publishing transform
[conveyor_base_node-1] ... Serial /dev/ttyACM0 @ 115200 opened.
[conveyor_base_node-1] ... ConveyorBaseNode activated
[ekf_node-3]           ... EKF node ready for tb3_1
```

**Leave Terminal 1 alone for the rest of the session.** This
terminal stays SSH'd to pi2; the sensor launch keeps publishing
camera + odom + tf to the network.

If anything in the log is red / missing, scroll up and stop here —
do not continue to Step 2.

---

## Step 2. Open Terminal 2, set env, verify pi topics are visible

In a NEW laptop terminal (do NOT SSH this one):

```bash
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/toodmuk/fastdds_peers.xml
export ROS_DOMAIN_ID=30
source /opt/ros/humble/setup.bash
source ~/swerve_transport_project/ros2_ws/install/setup.bash
```

> Workspace path is `ros2_ws/install`, NOT `install/`. The legacy
> `~/swerve_transport_project/install/` lacks the rtabmap launches
> and will fail with "file not found in the share directory".

Sanity check the env:
```bash
echo "DOMAIN=$ROS_DOMAIN_ID  PROFILE=$FASTRTPS_DEFAULT_PROFILES_FILE"
```
Expected: `DOMAIN=30  PROFILE=/home/toodmuk/fastdds_peers.xml`

Now check that pi2's topics are visible:
```bash
ros2 daemon stop ; sleep 1
ros2 topic list | grep -E '/tb3_1/(camera|odom|ekf)'
```

**Expected (5–6 topics):**
```
/tb3_1/camera/depth/camera_info
/tb3_1/camera/depth/image_raw
/tb3_1/camera/rgb/camera_info
/tb3_1/camera/rgb/image_raw
/tb3_1/ekf/odom
/tb3_1/odom
```

**If empty**, jump to "Step 2a — Discovery troubleshooting" below,
fix it, then come back to Step 3.

---

## Step 2a. Discovery troubleshooting (only if Step 2 was empty)

Empty topic list means the FastDDS peers file has the wrong laptop
IP. The fix is to make sure both peers files (laptop and pi2) point
to the laptop's *actual* current LAN IP.

**Find the laptop's real LAN IP:**
```bash
ip -4 addr show | grep 192.168.1.
ping -c 1 192.168.1.102      # confirms LAN reaches pi2
```
Note the IP — call it `NEW_IP` (e.g. `192.168.1.114`).

**Find the OLD laptop IP currently in your peers file:**
```bash
grep -oE 'address>192\.168\.1\.[0-9]+' ~/fastdds_peers.xml \
    | grep -v '\.101\|\.102' | sort -u
```
This intentionally excludes the robot IPs `.101` and `.102`. Call
the result `OLD_IP`.

**Patch laptop side and pi2 side** (replace `OLD` and `NEW` with
your numbers — do NOT script this with auto-detected IPs):
```bash
sed -i "s/192\.168\.1\.OLD/192.168.1.NEW/g" ~/fastdds_peers.xml
ssh pi2@192.168.1.102 "sed -i 's/192\\.168\\.1\\.OLD/192.168.1.NEW/g' ~/fastdds_peers.xml"
```

**Verify the file still has all 4 robot peer entries** (catches
accidental wipeouts):
```bash
grep -E '192\.168\.1\.(101|102)' ~/fastdds_peers.xml | wc -l
```
Expected: `4`. If less, the file is corrupted — copy the canonical
version from pi2: `scp pi2@192.168.1.102:~/fastdds_peers.xml ~/`
(then re-do the laptop-side patch on it).

**Restart pi2's sensor launch** so it re-reads the peers file:
- Switch to Terminal 1
- `Ctrl+C` to stop the sensor launch
- Re-run the same `ros2 launch swerve_bringup rtabmap_pi_sensors...`
  command from Step 1

**Switch back to Terminal 2**:
```bash
ros2 daemon stop ; sleep 1
ros2 topic list | grep -E '/tb3_1/(camera|odom|ekf)'
```
Should now show the topics. If still empty, see Troubleshooting at
the bottom.

---

## Step 3. Verify the topic rates are healthy

Still in Terminal 2:
```bash
ros2 topic hz /tb3_1/camera/rgb/image_raw    # > 1 Hz expected
ros2 topic hz /tb3_1/ekf/odom                # > 5 Hz expected
```

Press Ctrl+C after a few seconds of each.

If the camera rate is < 1 Hz the network is saturated and mapping
quality will suffer. Either:
- Switch to Terminal 1, Ctrl+C, relaunch the Pi sensor launch with
  `fps:=10` appended to the command from Step 1
- Or fall back to all-on-pi mode (out of scope for this guide)

---

## Step 4. In Terminal 2, launch rtabmap (this is the SLAM brain)

Same Terminal 2 that you used for Steps 2 and 3:
```bash
mkdir -p ~/maps

ros2 launch swerve_bringup rtabmap_laptop_mapping.launch.py \
    robot_id:=tb3_1 \
    db_path:=~/maps/tb3_1_room.db
```

**Wait for `rtabmap started` in the log** before doing anything
else. The .db will be created at `~/maps/tb3_1_room.db`.

**Leave Terminal 2 visible** — rtabmap prints `Added keyframe NN`
status as you drive. Glance at it occasionally.

---

## Step 5. Open Terminal 3, set env, start teleop

In a NEW laptop terminal:

```bash
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/toodmuk/fastdds_peers.xml
export ROS_DOMAIN_ID=30
source /opt/ros/humble/setup.bash
source ~/swerve_transport_project/ros2_ws/install/setup.bash

ros2 run turtlebot3_conveyor_bridge teleop_keyboard_node \
    --ros-args -r cmd_vel:=/tb3_1/cmd_vel
```

(If your teleop node has a different name, substitute. Key
requirement: it must publish to `/tb3_1/cmd_vel`.)

The teleop will print key bindings. Test by tapping forward — the
robot should move slightly.

---

## Step 6. (Optional) Open Terminal 4, monitor progress

In a fourth laptop terminal:

```bash
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/toodmuk/fastdds_peers.xml
export ROS_DOMAIN_ID=30
source /opt/ros/humble/setup.bash
source ~/swerve_transport_project/ros2_ws/install/setup.bash

# Number of keyframes added + last loop closure id
ros2 topic echo /tb3_1/rtabmap/info | grep -E "frameId|loopId"

# OR a simpler liveness check (Ctrl+C after a few seconds)
ros2 topic hz /tb3_1/rtabmap/cloud_map
```

Healthy signs:
- `cloud_map` publishing > 0.5 Hz
- `loopId: NN` (non-zero) appearing — that's a loop closure firing

---

## Step 7. Drive the robot through the room

Now use Terminal 3 (teleop) to drive. The driving rules directly
affect map quality:

- **Slow.** About half your usual nav-test speed. Visual feature
  tracker drops above ~0.15 m/s.
- **Smooth.** Long key holds, not staccato taps.
- **Stop and rotate.** Occasionally pivot in place so the camera
  scans walls from one position.
- **Re-visit.** Drive past the same spot from different angles —
  this is how RTAB-Map confirms its map.
- **Close the loop.** End the driving by returning to within ~30 cm
  of the start position. Single most important step for map
  quality.

While driving, glance at Terminal 2 (rtabmap) — you should see
keyframe count increasing.

---

## Step 8. End the run, in this exact order

1. **Terminal 3 (teleop)** — Ctrl+C
2. **Terminal 2 (rtabmap)** — Ctrl+C. Wait ~5 seconds for the
   `.db` to flush to disk.
3. **Terminal 1 (pi2 sensors)** — Ctrl+C
4. **Terminal 4 (monitoring, if open)** — Ctrl+C

---

## Step 9. Verify the .db is on disk

In Terminal 2 (or any laptop terminal):
```bash
ls -lh ~/maps/
```
Expected: `tb3_1_room.db` of 10–200 MB.

If the file is < 1 MB, mapping didn't capture much — the rtabmap
log in Terminal 2 (scroll back) will show why (no sync, no
keyframes added, etc.). See Troubleshooting.

---

## Step 10. (Optional) Inspect the map

Install the GUI viewer if you don't have it:
```bash
sudo apt install -y ros-humble-rtabmap-viz
```

Open the map:
```bash
rtabmap-databaseViewer ~/maps/tb3_1_room.db
```

Look for:
- A continuous trajectory (not broken into many disconnected
  pieces)
- Loop-closure links shown as coloured edges in the graph
- A point cloud that resembles the actual room

If the map looks fragmented, drive again — see Troubleshooting.

---

## Step 11. (Later) Switch to localization mode

After you have a `.db` you're happy with, switch the runtime stack
from mapping mode to localization mode. Same 4-terminal layout,
slightly different launches.

In Terminal 1 (still SSH'd to pi2): the sensor launch from Step 1
is exactly what's needed for localization too. Restart it if you
stopped it.

In Terminal 2 (after Step 8 stopped the mapping launch):
```bash
ros2 launch swerve_bringup rtabmap_laptop_localization.launch.py \
    robot_id:=tb3_1 \
    db_path:=~/maps/tb3_1_room.db
```

In Terminal 4, verify it's localized:
```bash
ros2 topic hz /tb3_1/slam/pose      # 1–3 Hz once localized
ros2 topic echo /tb3_1/ekf/odom     # smooth, drift-corrected
```

**Initial localization can take 5–30 seconds** — RTAB-Map scans
the entire stored map for a visual match to the current camera
frame. Drop the robot in a part of the room with reasonable visual
variety — avoid blank walls.

---

## Multi-robot — every robot must load the SAME .db

When both robots run their localization launches and you intend to
use the laplacian consensus correction (`enable_consensus:=true`),
every robot's launch MUST point `db_path` at the SAME database
file. Different .db files mean different `map` frames, and any
inter-robot pose feedback produces nonsense (silently — neither
robot will detect it).

To distribute the .db you built on this laptop to the other robot:
```bash
rsync ~/maps/tb3_1_room.db pi1@192.168.1.101:~/maps/room.db
# then point pi1's localization launch at ~/maps/room.db
```

Easiest convention: rename the file (`room.db`) and use that path
on every machine.

---

## Troubleshooting

### `file 'rtabmap_laptop_mapping.launch.py' was not found in the share directory`
You sourced the legacy `~/swerve_transport_project/install/` which
doesn't have the new launches. Re-source:
```bash
source ~/swerve_transport_project/ros2_ws/install/setup.bash
```
(Same fix for any "file not found in share dir" error from the
laptop side.)

### `rtabmap started` never appears in Terminal 2
- Check Terminal 1 — is the Pi sensor launch actually running?
- Re-run Step 3 — if any rate is 0 Hz, rtabmap will sit waiting
  for topic sync and never declare ready
- Try `--ros-args --log-level debug` on the launch line to see why
  sync isn't happening

### Map looks fragmented / no loop closures
- Drive slower next time (Step 7 rules)
- Re-map with a tighter physical loop close
- Edit the launch file and lower `RGBD/AngularUpdate` and
  `RGBD/LinearUpdate` to `0.005` (more keyframes captured per
  motion)

### Localization keeps "jumping" after a few seconds
The `/ekf/odom` remap creates a small feedback loop (rtabmap →
ekf → rtabmap). If you see jumpy poses, edit
`rtabmap_laptop_localization.launch.py` and change
`('odom', f'/{robot_id}/ekf/odom')` back to
`('odom', f'/{robot_id}/odom')`.

### Camera rate drops or rtabmap complains about sync
Network is the bottleneck. In Terminal 1, Ctrl+C and relaunch the
Pi sensor command from Step 1 with `fps:=10` appended.

### Discovery still broken after Step 2a
- Verify `grep -E '192\\.168\\.1\\.(101|102)' ~/fastdds_peers.xml`
  returns 4 lines on the laptop AND on pi2 (`ssh pi2@…`)
- Pi2 wasn't actually restarted after patching its peers file —
  go back to Terminal 1, Ctrl+C, re-run the launch from Step 1
- ROS daemon stuck — `pkill -9 -f ros2-daemon ; ros2 daemon start`
- LAN cable not plugged in or wrong subnet — `ip -4 addr show |
  grep 192.168.1.` must show one address; `ping -c 1 192.168.1.102`
  must succeed
