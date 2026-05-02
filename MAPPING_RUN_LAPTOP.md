# Mapping Run — Laptop Side

What you do on the laptop during the mapping run. Pairs with
`MAPPING_RUN_PI.md` — both must be running for the split-execution
mapping to work.

> **Recommended path is SPLIT** (this guide). The laptop does the
> heavy SLAM compute (rtabmap_slam); the Pi only streams sensor data.
> The .db ends up on this laptop.

---

## Before you start

### 1. Make sure the room is mapping-friendly
- Lights on (visual features need light)
- No big mirrors or glass walls (visual SLAM hates them)
- Move chairs / people that won't be present during the actual
  transport job — anything dynamic will end up "smeared" in the map

### 2. Plan your driving path
- Cover every area you'll later transport through
- Return to the start point at the end so RTAB-Map can close the
  loop and collapse drift
- Avoid sharp 90° turns at full speed — the visual feature tracker
  prefers smooth continuous motion

### 3. Confirm the Pi sensor side is up
The Pi-side launch must already be running before this laptop launch
will produce a map. Confirm with the operator on `MAPPING_RUN_PI.md`
that they've reached the "leave this terminal alone" stage.

---

## Pre-flight (laptop, in any terminal)

```bash
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/toodmuk/fastdds_peers.xml
export ROS_DOMAIN_ID=30
source /opt/ros/humble/setup.bash
source ~/swerve_transport_project/install/setup.bash
```

Now check that the Pi's topics are visible:

```bash
ros2 topic list | grep -E '/tb3_1/(camera|odom|ekf)'
```

Expected (4–5 topics):
```
/tb3_1/camera/depth/camera_info
/tb3_1/camera/depth/image_raw
/tb3_1/camera/rgb/camera_info
/tb3_1/camera/rgb/image_raw
/tb3_1/ekf/odom
/tb3_1/odom
```

If you see only `/parameter_events` and `/rosout`, discovery is broken.
Most common cause: the laptop's IP in the FastDDS peers file is stale.
Patch with the current IP:

```bash
LAN_IP=$(ip -4 addr show | awk '/inet 192\.168\.1\./{print $2}' | cut -d/ -f1)
echo "current LAN IP = $LAN_IP"
sed -i "s/192\.168\.1\.[0-9]\+\(<\)/$LAN_IP\1/g" ~/fastdds_peers.xml
ssh pi2@192.168.1.102 "sed -i \"s/192\\.168\\.1\\.114/$LAN_IP/g\" ~/fastdds_peers.xml"
ros2 daemon stop ; sleep 1
ros2 topic list | grep tb3_1
```

(The Pi sensor launch may need a restart after patching its peers
file — see `MAPPING_RUN_PI.md`.)

Then check the rates are healthy:

```bash
ros2 topic hz /tb3_1/camera/rgb/image_raw    # > 1 Hz expected
ros2 topic hz /tb3_1/ekf/odom                # > 5 Hz expected
```

If the camera rate is below ~1 Hz the network is saturated and
mapping quality will suffer. Drop the Pi launch's `fps` to 10 (see
the Pi guide) or fall back to the all-on-pi launch.

---

## Terminal 1 — Launch rtabmap on the laptop

```bash
mkdir -p ~/maps

ros2 launch swerve_bringup rtabmap_laptop_mapping.launch.py \
    robot_id:=tb3_1 \
    db_path:=~/maps/tb3_1_room.db
```

Wait for `rtabmap started` in the log. The .db is being created at
`~/maps/tb3_1_room.db` on this laptop.

**Leave this terminal visible** — rtabmap prints status updates as
it adds keyframes (`Added keyframe NN`). You want to glance at it
while driving.

---

## Terminal 2 — Teleop

In a NEW terminal on the laptop, with the same env exports as before
(re-run the `source` and `export` lines from Pre-flight):

```bash
# Whichever teleop you've been using all along — it must publish to /tb3_1/cmd_vel
ros2 run turtlebot3_conveyor_bridge teleop_keyboard_node \
    --ros-args -r cmd_vel:=/tb3_1/cmd_vel
```

(If the exact node name differs, substitute the teleop you have.
The key requirement is the topic remap to `/tb3_1/cmd_vel`.)

### Driving rules (matter for map quality)

- **Slow.** About half the speed you used for the navigation tests.
  Visual feature tracking gets confused above ~0.15 m/s.
- **Smooth.** Long press of arrow keys, not staccato taps.
- **Stop and look around.** Occasionally pivot in place so the
  camera scans the surrounding walls from one position.
- **Re-visit.** Drive past the same spot from different angles —
  this is how RTAB-Map confirms its map.
- **Close the loop.** End the run by driving back to within ~30 cm
  of the start position and stopping. This is the single most
  important step for map quality.

---

## Terminal 3 (optional) — Watch progress

In a third laptop terminal (same env):

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
  per segment of the room

---

## Ending the mapping run

1. Drive the robot back near the start tape mark.
2. **Ctrl+C** the teleop terminal.
3. **Ctrl+C** the rtabmap terminal (Terminal 1). Wait ~5 seconds —
   RTAB-Map flushes and saves the database.
4. Verify the .db exists on this laptop:
   ```bash
   ls -lh ~/maps/
   ```
   Expect a file like `tb3_1_room.db` of 10–200 MB.
5. Tell the Pi-side operator they can Ctrl+C the sensor launch.

---

## (Optional) Inspect the map

Install the GUI viewer if you don't have it:
```bash
sudo apt install -y ros-humble-rtabmap-viz
```

Open the map:
```bash
rtabmap-databaseViewer ~/maps/tb3_1_room.db
```

Look for:
- A continuous trajectory (not broken into many disconnected segments)
- Loop-closure links shown as coloured edges in the graph
- A point cloud that resembles the actual room

If the map looks fragmented, drive again — see Troubleshooting.

---

## After mapping — switch to localization

Once you're happy with the .db, the runtime stack closes the EKF
loop and gives the formation drift-free pose.

The Pi-side operator should keep `rtabmap_pi_sensors.launch.py`
running (or restart it). On this laptop:

```bash
ros2 launch swerve_bringup rtabmap_laptop_localization.launch.py \
    robot_id:=tb3_1 \
    db_path:=~/maps/tb3_1_room.db
```

Verify in another terminal:
```bash
ros2 topic hz /tb3_1/slam/pose      # 1–3 Hz once localized
ros2 topic echo /tb3_1/ekf/odom     # smooth, drift-corrected
```

**Initial localization** can take 5–30 seconds: RTAB-Map is scanning
the entire stored map looking for a visual match to what the camera
sees right now. Drop the robot in a part of the room with reasonable
visual variety — avoid blank walls.

### Multi-robot — every robot must load the SAME .db

When both robots are running and you intend to use the laplacian
consensus correction (`enable_consensus:=true` in
`laplacian_formation_node`), every robot's localization launch
MUST point `db_path` at the SAME database file. If robot A loads
`room_v1.db` and robot B loads `room_v2.db`, their `map` frames
are unrelated and any inter-robot pose feedback produces nonsense
(silently — neither robot will detect the inconsistency).

To distribute the .db you built on this laptop to the other robot:
```bash
rsync ~/maps/tb3_1_room.db pi1@192.168.1.101:~/maps/room.db
# then point pi1's localization launch at ~/maps/room.db
```

Easiest convention: rename the file (e.g. `room.db`) and use that
same path everywhere, instead of robot-id-suffixed paths.

---

## Troubleshooting

### `rtabmap started` never appears
- Check that the Pi sensor launch is actually running and publishing
  (see Pre-flight)
- Re-run pre-flight `ros2 topic hz` checks — if any are 0 Hz, rtabmap
  will sit waiting for sync
- Try `--ros-args --log-level debug` to see why it's not syncing

### Map looks fragmented / not closing loop
- Drive slower next time
- Re-map and physically close the loop more carefully
- Edit `rtabmap_laptop_mapping.launch.py` and lower
  `RGBD/AngularUpdate` and `RGBD/LinearUpdate` to 0.005 (more
  keyframes captured)

### Localization keeps "jumping" after a few seconds
- This is the feedback-loop concern from Tan's `/ekf/odom` change.
  If you see it, edit `rtabmap_laptop_localization.launch.py` and
  change `('odom', f'/{robot_id}/ekf/odom')` back to
  `('odom', f'/{robot_id}/odom')`.

### Camera rate drops or rtabmap complains about message sync
- The network is the bottleneck. Either reduce the Pi launch fps to
  10 (see Pi guide), drive slower, or fall back to the ALL-ON-PI
  launch (see `MAPPING_RUN_PI.md` bottom section).
