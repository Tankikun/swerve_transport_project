# Mapping Run — Laptop Side

What you do on the laptop during the mapping run. Pairs with
`MAPPING_RUN_PI.md` (Pi-side procedure must be running first).

> **Architecture: SPLIT.** Laptop runs `rtabmap_slam` (heavy CPU);
> Pi runs only sensors (camera, TF, odom, EKF). Map (`.db`) is built
> on the laptop. Pi 4 stays cool.

Follow the steps in order. Each step is one concrete action plus
how to know it worked.

---

## Step 1. Confirm the Pi sensor side is up

The Pi must already be running `rtabmap_pi_sensors.launch.py` before
the laptop launch will see anything to subscribe to. Check with the
Pi-side operator that they reached the "leave this terminal alone"
stage in `MAPPING_RUN_PI.md`.

If you are *both* operators (one person, two windows), do the Pi
guide first up to and including the launch line.

---

## Step 2. Open Terminal 1 on the laptop, set the env

```bash
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/toodmuk/fastdds_peers.xml
export ROS_DOMAIN_ID=30
source /opt/ros/humble/setup.bash
source ~/swerve_transport_project/ros2_ws/install/setup.bash
```

Sanity check:
```bash
echo "DOMAIN=$ROS_DOMAIN_ID  PROFILE=$FASTRTPS_DEFAULT_PROFILES_FILE"
```
**Expected**: `DOMAIN=30  PROFILE=/home/toodmuk/fastdds_peers.xml`

> Note the workspace path is `ros2_ws/install`, NOT just `install/`.
> The legacy `~/swerve_transport_project/install/` directory does not
> contain the rtabmap launches and will fail with "file not found in
> the share directory of package 'swerve_bringup'".

---

## Step 3. Verify the Pi's topics are visible

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

**If empty**, jump to "Step 3a — Discovery troubleshooting" below,
fix it, then come back here. Don't proceed to Step 4 until topics
are visible.

---

## Step 3a. Discovery troubleshooting (only if Step 3 was empty)

Empty topic list almost always means the FastDDS peers file has the
wrong laptop IP.

**Find your laptop's real LAN IP:**
```bash
ip -4 addr show | grep 192.168.1.
ping -c 1 192.168.1.102      # confirms LAN actually reaches pi2
```
Note the IP — call it `NEW_IP` (e.g. `192.168.1.114`).

**Find the OLD laptop IP currently in your peers file:**
```bash
grep -oE 'address>192\.168\.1\.[0-9]+' ~/fastdds_peers.xml \
    | grep -v '\.101\|\.102' | sort -u
```
This intentionally excludes the robot IPs `.101` and `.102` — those
must NOT change. Call the result `OLD_IP`.

**Patch laptop side and pi2 side** (replace `OLD` and `NEW` with
your actual numbers):
```bash
sed -i "s/192\.168\.1\.OLD/192.168.1.NEW/g" ~/fastdds_peers.xml
ssh pi2@192.168.1.102 "sed -i 's/192\\.168\\.1\\.OLD/192.168.1.NEW/g' ~/fastdds_peers.xml"
```

**Verify the file still has all 4 robot peer entries** (this catches
accidental wipeouts):
```bash
grep -E '192\.168\.1\.(101|102)' ~/fastdds_peers.xml | wc -l
```
Expected: `4`. If less, the peers file is corrupted — restore from
git: `git -C ~/swerve_transport_project show feature/laplacian-consensus:MAPPING_RUN_LAPTOP.md`
to find the canonical example, or copy from the Pi (`scp pi2@…:~/fastdds_peers.xml ~/`).

**Restart pi2's sensor launch** (it cached the old peers file at
launch time — must re-read). On the Pi terminal, Ctrl+C the
sensors launch and re-run it.

**Restart the laptop daemon and recheck**:
```bash
ros2 daemon stop ; sleep 1
ros2 topic list | grep -E '/tb3_1/(camera|odom|ekf)'
```

If still empty after all of this, see "Discovery still broken" at
the bottom of this file.

---

## Step 4. Verify the topic rates are healthy

```bash
ros2 topic hz /tb3_1/camera/rgb/image_raw    # > 1 Hz expected
ros2 topic hz /tb3_1/ekf/odom                # > 5 Hz expected
```

If the camera rate is < 1 Hz the network is saturated and mapping
quality will suffer. Either:
- Tell the Pi operator to relaunch with `fps:=10` instead of 15
- Or fall back to the all-on-pi launch (see `MAPPING_RUN_PI.md`)

---

## Step 5. Launch rtabmap (Terminal 1)

```bash
mkdir -p ~/maps

ros2 launch swerve_bringup rtabmap_laptop_mapping.launch.py \
    robot_id:=tb3_1 \
    db_path:=~/maps/tb3_1_room.db
```

**Wait for the line `rtabmap started`** in the log before doing
anything else. The .db will be created at `~/maps/tb3_1_room.db`.

**Leave Terminal 1 visible** — rtabmap prints `Added keyframe NN`
status as you drive. Glance at it occasionally.

---

## Step 6. Open Terminal 2 — start teleop

In a NEW terminal on the laptop:

```bash
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/toodmuk/fastdds_peers.xml
export ROS_DOMAIN_ID=30
source /opt/ros/humble/setup.bash
source ~/swerve_transport_project/ros2_ws/install/setup.bash

ros2 run turtlebot3_conveyor_bridge teleop_keyboard_node \
    --ros-args -r cmd_vel:=/tb3_1/cmd_vel
```

(If your teleop node has a different name, substitute. The key
requirement is publishing to `/tb3_1/cmd_vel`.)

---

## Step 7. Drive the robot through the room

Driving rules — these directly affect map quality:

- **Slow.** About half your usual nav-test speed. Feature tracker
  drops above ~0.15 m/s.
- **Smooth.** Long key holds, not staccato taps.
- **Stop and rotate.** Occasionally pivot in place so the camera
  scans walls from one position.
- **Re-visit.** Drive past the same spot from different angles —
  this is how RTAB-Map confirms its map.
- **Close the loop.** End by driving back to within ~30 cm of the
  start. Single most important step for map quality.

---

## Step 8. (Optional) Open Terminal 3 — monitor progress

In a third terminal, same env exports as Step 2:

```bash
# Number of keyframes added + last loop closure id
ros2 topic echo /tb3_1/rtabmap/info | grep -E "frameId|loopId"

# OR a simple liveness check
ros2 topic hz /tb3_1/rtabmap/cloud_map
```

Healthy signs:
- `cloud_map` publishing > 0.5 Hz
- `loopId: NN` (non-zero) appearing — that's a loop closure firing
- By the end of a full lap you want at least 1 loop closure event
  per major room area

---

## Step 9. End the mapping run

1. Drive back to start tape mark.
2. **Ctrl+C in Terminal 2** (teleop).
3. **Ctrl+C in Terminal 1** (rtabmap). Wait ~5 seconds for the
   `.db` to flush to disk.
4. Verify the `.db` exists:
   ```bash
   ls -lh ~/maps/
   ```
   Expected: `tb3_1_room.db` of 10–200 MB.
5. Tell the Pi operator they can Ctrl+C the sensor launch.

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
- A continuous trajectory (not broken into many disconnected pieces)
- Loop-closure links shown as coloured edges in the graph
- A point cloud that resembles the actual room

If the map looks fragmented, drive again — see Troubleshooting at
the bottom.

---

## Step 11. (Later) Switch to localization

Once you're happy with the .db, the runtime stack closes the EKF
loop and gives the formation drift-free pose.

Pi operator: keep `rtabmap_pi_sensors.launch.py` running (or
restart it).

Laptop, in Terminal 1 (after the mapping launch is stopped):
```bash
ros2 launch swerve_bringup rtabmap_laptop_localization.launch.py \
    robot_id:=tb3_1 \
    db_path:=~/maps/tb3_1_room.db
```

Verify in Terminal 3 (or any terminal with the env set):
```bash
ros2 topic hz /tb3_1/slam/pose      # 1–3 Hz once localized
ros2 topic echo /tb3_1/ekf/odom     # smooth, drift-corrected
```

**Initial localization** can take 5–30 seconds: RTAB-Map scans the
entire stored map for a visual match to the current camera frame.
Drop the robot in a part of the room with reasonable visual
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

### `rtabmap started` never appears
- Check that the Pi sensor launch is actually running (Step 1)
- Re-run Step 4 — if any rate is 0 Hz, rtabmap will sit waiting for
  topic sync
- Try `--ros-args --log-level debug` on the launch line to see why
  sync isn't happening

### Map looks fragmented / no loop closures
- Drive slower next time (Step 7 rules)
- Re-map with a tighter physical loop close
- Edit `rtabmap_laptop_mapping.launch.py` and lower
  `RGBD/AngularUpdate` and `RGBD/LinearUpdate` to `0.005` (more
  keyframes captured)

### Localization keeps "jumping" after a few seconds
Tan's `/ekf/odom` remap creates a small feedback loop (rtabmap →
ekf → rtabmap). If you see jumpy poses, edit
`rtabmap_laptop_localization.launch.py` and change
`('odom', f'/{robot_id}/ekf/odom')` back to
`('odom', f'/{robot_id}/odom')`.

### Camera rate drops or rtabmap complains about sync
Network is the bottleneck. Either reduce Pi launch fps to 10 (see
Pi guide), drive slower, or fall back to the ALL-ON-PI launch.

### Discovery still broken after Step 3a
Possibilities:
- The peers file got corrupted (Step 3a wipeout warning). Verify
  `grep -E '192\\.168\\.1\\.(101|102)' ~/fastdds_peers.xml` returns
  4 lines on BOTH the laptop and pi2.
- Pi2 wasn't restarted after patching its peers file — it caches
  the file at launch time.
- ROS daemon stuck — `pkill -9 -f ros2-daemon ; ros2 daemon start`.
- LAN cable not plugged in or wrong subnet — `ip -4 addr show |
  grep 192.168.1.` must show one address; `ping -c 1 192.168.1.102`
  must succeed.
