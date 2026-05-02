# Mapping Run Guide (Step 6 of the RTAB-Map plan)

This is the live procedure for **driving the robot through the room
to build a 3D map** (a `.db` file) that subsequent localization runs
will use to know where the robot is.

> **Prerequisite**: pi2 is fully set up (depthai-ros + rtabmap-ros
> apt-installed by Earth, our oak_camera_node + slam_pose_relay built,
> latest `feature/rtab-map` branch synced and built). If anything in
> this guide errors with "package not found", come back to
> `CAMERA_NOTES.md` and re-run the inventory script.

## Two architectures — pick one before starting

**(A) SPLIT — recommended for mapping** (lower thermal load, faster
inspection of result, pi stays cool):
- pi2 runs `rtabmap_pi_sensors.launch.py` (camera + TF + odom + EKF)
- laptop runs `rtabmap_laptop_mapping.launch.py` (rtabmap_slam only)
- The `.db` ends up on the **laptop** at `~/maps/tb3_1_room.db`.
- Network bandwidth: ~5–10 MB/s of image data over WiFi/LAN.

**(B) ALL-ON-PI — fallback** (everything on pi2; one terminal but
heats the Pi):
- pi2 runs `rtabmap_mapping.launch.py` (camera + TF + odom + EKF +
  rtabmap_slam)
- The `.db` lives on pi2.
- Watch `vcgencmd measure_temp` — if it climbs past 75 °C, stop and
  switch to (A).

The rest of this guide shows both paths. Read the (A) section first
unless you have a reason to keep everything on the Pi.

---

## Before you start

### 1. Measure the camera mount

The launch defaults assume the OAK-D is 10 cm forward, 0 cm sideways,
15 cm above the robot's `base_link` origin. **If your mount differs by
more than ~3 cm in any axis, override it** (steps below show how) —
the bigger the mismatch, the more the visual SLAM map is offset from
the true world.

Use the right-hand-rule body frame (X = forward, Y = left, Z = up)
when measuring.

### 2. Make sure the room is mapping-friendly

- Lights on (visual features need light)
- No big mirrors/glass walls (visual SLAM hates them)
- Move chairs/people that won't be there during the actual transport
  job — anything dynamic will end up "smeared" in the map

### 3. Plan your driving path

Sketch a path mentally that:
- Covers every area you'll later transport through
- Returns to the start point at the end (so RTAB-Map can close the
  loop and collapse drift)
- Avoids sharp 90° turns at full speed (visual feature tracker
  prefers smooth continuous motion)

A "lawn-mower" pattern (back and forth in parallel strips) works
well for square rooms. For corridors, drive down once, U-turn at the
end, drive back.

---

## Path (A) SPLIT — Terminal 1 (pi2 sensors)

```bash
ssh pi2@192.168.1.102
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/pi2/fastdds_peers.xml
export ROS_DOMAIN_ID=30
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch swerve_bringup rtabmap_pi_sensors.launch.py \
    robot_id:=tb3_1 \
    cam_x:=0.128  cam_y:=0.000  cam_z:=-0.0175
```

This brings up only the camera + TF + odom + EKF on pi2. Pi 4 thermal
stays around 60 °C with no rtabmap CPU load.

## Path (A) SPLIT — Terminal 2 (laptop rtabmap)

In a NEW terminal on the laptop:

```bash
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/toodmuk/fastdds_peers.xml
export ROS_DOMAIN_ID=30
source /opt/ros/humble/setup.bash
source ~/swerve_transport_project/install/setup.bash
mkdir -p ~/maps

# Quick pre-flight (verify pi2 topics are visible from laptop)
ros2 topic hz /tb3_1/camera/rgb/image_raw    # > 1 Hz expected
ros2 topic hz /tb3_1/ekf/odom                # > 5 Hz expected

ros2 launch swerve_bringup rtabmap_laptop_mapping.launch.py \
    robot_id:=tb3_1 \
    db_path:=~/maps/tb3_1_room.db
```

The .db ends up at `~/maps/tb3_1_room.db` on the **laptop**.

---

## Path (B) ALL-ON-PI — Terminal 1 (everything on pi2)

```bash
ssh pi2@192.168.1.102
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/pi2/fastdds_peers.xml
export ROS_DOMAIN_ID=30
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
mkdir -p ~/maps

ros2 launch swerve_bringup rtabmap_mapping.launch.py \
    robot_id:=tb3_1 \
    db_path:=~/maps/tb3_1_room.db \
    cam_x:=0.128  cam_y:=0.000  cam_z:=-0.0175
```

The .db ends up on **pi2**. Pi 4 thermal can climb 70-80 °C — keep
an eye on `vcgencmd measure_temp` and abort if you see throttling.

### What you should see in the logs

**Path (A) Pi terminal:**
```
[oak_camera_node-1] ... oak_camera_node ready (tb3_1) — depthai=3.5.0
[oak_camera_node-1] ... pipeline running. device=OAK-D-LITE
[static_transform_publisher-2] ... Spinning until stopped - publishing transform
[conveyor_base_node-1] ... Serial /dev/ttyACM0 @ 115200 opened.
[conveyor_base_node-1] ... ConveyorBaseNode activated
[ekf_node-3] ... EKF node ready for tb3_1
```

**Path (A) laptop terminal:**
```
[rtabmap-1] ... rtabmap started
```

**Path (B) single Pi terminal:** all of the above lines together.

**Wait until `rtabmap started` appears (laptop in (A), pi in (B))
before driving.** If it never appears, see troubleshooting.

**Leave the rtabmap terminal visible** — it prints status updates as
keyframes are added. You want to glance at it while driving.

---

## Teleop terminal (on the laptop, both paths)

You have an existing teleop from previous testing. On the laptop:

```bash
# Same network env as anywhere else
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/toodmuk/fastdds_peers.xml
export ROS_DOMAIN_ID=30
source /opt/ros/humble/setup.bash
source ~/swerve_transport_project/install/setup.bash

# Start the teleop you used before — adjust to your setup
ros2 run turtlebot3_conveyor_bridge teleop_keyboard_node \
    --ros-args -r cmd_vel:=/tb3_1/cmd_vel
```

(If that exact command name is wrong, use whichever teleop_keyboard
you've been using all along. The key thing is it must publish to
`/tb3_1/cmd_vel`.)

### Driving rules (matter for map quality)

- **Slow**. About half the speed you used for the navigation tests.
  Visual feature tracking gets confused above ~0.15 m/s.
- **Smooth**. Long press of arrow keys, not staccato taps.
- **Look at every direction**. Stop occasionally and rotate in place
  so the camera scans the surrounding walls.
- **Re-visit**. Drive past the same spot from different angles —
  this is how RTAB-Map confirms its map.
- **Close the loop**. End by driving back to within ~30 cm of the
  start position and stopping. This is the single most important
  step for map quality.

---

## Terminal 3 (optional) — Watch progress

On laptop:

```bash
# Same env
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/toodmuk/fastdds_peers.xml
export ROS_DOMAIN_ID=30
source /opt/ros/humble/setup.bash
source ~/swerve_transport_project/install/setup.bash

# Watch keyframe count grow (note per-robot namespace prefix)
ros2 topic echo /tb3_1/rtabmap/info  | grep -E "frameId|loopId"

# OR a simpler liveness check
ros2 topic hz /tb3_1/rtabmap/cloud_map
```

If `cloud_map` is publishing > 0.5 Hz, mapping is working.
If you see `loopId: NN` (non-zero), a loop closure fired — that's
the good sign. By the end of a full lap you want at least 1 loop
closure event per segment.

> **Note**: rtabmap topics are now per-robot
> (`/{robot_id}/rtabmap/...`) instead of the default global
> `/rtabmap/...`. This is required for safe multi-robot operation —
> see CodeRabbit review on PR #3 / namespace fix in this branch.
> If you read older docs that mention `/rtabmap/...`, mentally
> substitute `/tb3_1/rtabmap/...` (or whichever robot_id you're
> running).

---

## Ending the mapping run

1. Drive the robot back to a position near the start.
2. Stop teleop (Ctrl+C in the teleop terminal).
3. Stop the rtabmap launch (Ctrl+C in the rtabmap terminal —
   laptop for path A, pi for path B).
4. Stop the pi sensors launch (path A only — Ctrl+C in the pi
   sensors terminal).
5. Wait ~5 seconds — RTAB-Map flushes and saves the database.
6. Verify the .db is on disk:
   - **Path (A)**: `ls -lh ~/maps/` on the **laptop**
   - **Path (B)**: `ssh pi2@192.168.1.102 'ls -lh ~/maps/'`
   Expect a file like `tb3_1_room.db` of 10–200 MB.
6. (Optional, on a desktop) Inspect the map visually:
   ```bash
   # Copy .db to laptop:
   scp pi2@192.168.1.102:~/maps/tb3_1_room.db ~/
   # Open with the GUI inspector — install with apt if you don't have it:
   sudo apt install ros-humble-rtabmap-viz
   rtabmap-databaseViewer ~/tb3_1_room.db
   ```
   Look for: continuous trajectory, loop-closure links shown as
   coloured edges in the graph, point cloud that resembles the
   actual room.

---

## After mapping — switch to localization

Once you're happy with the .db, the runtime launch closes the EKF
loop and gives the formation drift-free pose. Same SPLIT vs ALL-ON-PI
choice applies.

### Path (A) SPLIT localization

If you mapped with path (A), the .db is already on the laptop.
Just keep `rtabmap_pi_sensors.launch.py` running on pi2 (or restart
it), then on the laptop:

```bash
ros2 launch swerve_bringup rtabmap_laptop_localization.launch.py \
    robot_id:=tb3_1 \
    db_path:=~/maps/tb3_1_room.db
```

### Path (B) ALL-ON-PI localization

```bash
ssh pi2@192.168.1.102
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/pi2/fastdds_peers.xml
export ROS_DOMAIN_ID=30
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch swerve_bringup rtabmap_localization.launch.py \
    robot_id:=tb3_1 \
    db_path:=~/maps/tb3_1_room.db
```

The .db must be on pi2 for path B — copy from the laptop if needed:
```bash
scp ~/maps/tb3_1_room.db pi2@192.168.1.102:~/maps/
```

### Verifying localization (either path)

```bash
ros2 topic hz /tb3_1/slam/pose      # 1–3 Hz once localized
ros2 topic echo /tb3_1/ekf/odom     # smooth, drift-corrected
```

**Initial localization** can take 5–30 seconds: RTAB-Map is scanning
the entire stored map looking for a visual match to what the camera
sees right now. Drop the robot in a part of the room with reasonable
visual variety — avoid blank walls.

---

## Troubleshooting

### `rtabmap started` never appears in Terminal 1

```bash
# Check rtabmap node logs more carefully
tail -100 /tmp/launch_logs/...
# Or relaunch with --ros-args --log-level debug to see why
```
Common causes:
- depth/image and rgb/image not synchronized → bump `queue_size`
  in launch params (already 30 by default)
- camera publish rate too slow → in mapping launch, fps:=20

### Map looks fragmented / not closing loop

- Drive slower next time
- Re-map and physically close the loop more carefully
- In `rtabmap_mapping.launch.py`, lower `RGBD/AngularUpdate` and
  `RGBD/LinearUpdate` to 0.005 (more keyframes captured)

### Localization keeps "jumping" after a few seconds

This is the feedback-loop concern from Tan's `/ekf/odom` change.
If you see jumps:
1. Stop the localization launch
2. Edit `rtabmap_localization.launch.py` to change
   `('odom', f'/{robot_id}/ekf/odom')` back to
   `('odom', f'/{robot_id}/odom')`
3. Rebuild and relaunch

### Pi2 thermal hits 70°C+

RTAB-Map + camera + serial bridge is more CPU than anything we've run
before. If pi2 throttles (`vcgencmd get_throttled` returns non-zero):
- Stop the mapping launch
- Let it cool for 5 min
- Add a fan or ice-pack to the Pi heatsink
- Re-run with `fps:=10` instead of 15

---

## Files referenced in this guide

| File | Where | Purpose |
|---|---|---|
| `oak_camera_node.py` | `swerve_formation/swerve_formation/` | OAK-D → ROS topics |
| `slam_pose_relay_node.py` | `swerve_formation/swerve_formation/` | rtabmap pose → ekf format |
| `oak_camera.launch.py` | `swerve_bringup/launch/` | camera + TF |
| `rtabmap_mapping.launch.py` | `swerve_bringup/launch/` | THIS guide's main launch |
| `rtabmap_localization.launch.py` | `swerve_bringup/launch/` | runtime localization |
| `~/maps/tb3_1_room.db` | on pi2 | the mapping artifact (output) |
