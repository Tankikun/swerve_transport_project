# Mapping Run — Pi Side (pi2)

What you do on the robot's Raspberry Pi during the mapping run.
Pairs with `MAPPING_RUN_LAPTOP.md` — both must be running for the
split-execution mapping to work.

> **Recommended path is SPLIT** (this guide). The Pi only streams
> sensor data; the laptop runs the heavy SLAM. Pi 4 thermal stays
> ~60 °C instead of the 70-80 °C you'd see if rtabmap_slam ran here.
> An ALL-ON-PI fallback is at the bottom of this file in case the
> network can't keep up.

---

## Before you start

### 1. Measure the camera mount (one-time per robot)

The pi sensor launch takes the camera offset as launch args. Use the
ROS body-frame convention: **X forward, Y left, Z up**. Measure from
the robot's `base_link` origin to the camera's lens (NOT the housing
front).

For tb3_1 the measured values are:

| arg | value | meaning |
|---|---|---|
| `cam_x` | `+0.128 m` | 128 mm forward of base_link |
| `cam_y` | ` 0.000 m` | centred laterally |
| `cam_z` | `-0.0175 m` | 17.5 mm BELOW base_link (because base_link is at the payload-mount point on top of the chassis, and the OAK-D sits below it). Negative z is correct. |

If you re-measure differently, update the launch args below.

### 2. Make sure the OAK-D is plugged in

```bash
ssh pi2@192.168.1.102 'lsusb | grep Movidius'
```
You should see `Bus 001 Device 003: ID 03e7:2485 Intel Movidius MyriadX`.
If not — re-seat the USB cable and re-check.

### 3. Make sure pi2 isn't already running stale ROS processes

If you've recently been doing other ROS launches on pi2:

```bash
ssh pi2@192.168.1.102 'pgrep -af "ros2 launch|swerve_formation/lib"'
```
If anything's running, kill it first or it will fight for `/dev/ttyACM0`.

---

## Launch sensors (this terminal stays open)

```bash
ssh pi2@192.168.1.102

# Network env so the laptop can see pi2's topics
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/pi2/fastdds_peers.xml
export ROS_DOMAIN_ID=30

# ROS env
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch swerve_bringup rtabmap_pi_sensors.launch.py \
    robot_id:=tb3_1 \
    cam_x:=0.128  cam_y:=0.000  cam_z:=-0.0175
```

**What you should see, in roughly this order:**

```
[oak_camera_node-1]    ... oak_camera_node ready (tb3_1) — depthai=3.5.0
[oak_camera_node-1]    ... pipeline running. device=OAK-D-LITE
[static_transform_publisher-2] ... Spinning until stopped - publishing transform
                                    translation: (0.128, 0.000, -0.018)
                                    from 'tb3_1_base_link' to
                                    'tb3_1_oak_rgb_camera_optical_frame'
[conveyor_base_node-1] ... Serial /dev/ttyACM0 @ 115200 opened.
[conveyor_base_node-1] ... ConveyorBaseNode activated
[ekf_node-3]           ... EKF node ready for tb3_1
```

If the static_transform translation doesn't match your measurements
(check rounding), Ctrl+C and relaunch with corrected `cam_*` args.

---

## What to do while it's running

**Leave this terminal alone.** rtabmap on the laptop is now
subscribing to your topics. Don't kill anything.

In a SEPARATE pi2 SSH terminal, you can monitor thermals while the
mapping run is happening:

```bash
ssh pi2@192.168.1.102
watch -n 5 'cat /sys/class/thermal/thermal_zone0/temp | awk "{printf \"%.1f C\\n\", \$1/1000}"; vcgencmd get_throttled'
```

- < 65 °C → fine
- 65-75 °C → warm but OK; consider stopping if it climbs
- 75-80 °C → throttling territory; abort
- `throttled=0x0` is good. Anything else means the Pi has thermally
  throttled at some point in this session

---

## When mapping is done

The laptop side will tell you when to stop (after the operator
finishes driving and Ctrl+Cs the rtabmap launch on the laptop).
Then on this terminal:

1. **Ctrl+C** to stop the sensor launch.
2. Wait ~3 seconds for clean shutdown.
3. Verify nothing is left running:
   ```bash
   pgrep -af "ros2 launch|swerve_formation/lib" | head
   ```
   Should be empty.

The .db file is on the **laptop**, not on pi2. If you later want it
on pi2 (for the all-on-pi localization fallback), the laptop guide
shows how to copy it back.

---

## ALL-ON-PI FALLBACK (skip if SPLIT works)

Use this only if the network can't keep up with image streams (for
example, if you see camera-rate drop below 1 Hz on the laptop side).
This runs everything including rtabmap_slam ON pi2 — heats it up.

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

The .db saves to `/home/pi2/maps/tb3_1_room.db`. When you later need
the laptop to inspect it:

```bash
scp pi2@192.168.1.102:~/maps/tb3_1_room.db ~/maps/
```

In ALL-ON-PI mode the laptop only runs teleop and topic monitoring
(no rtabmap launch).

---

## Multi-robot — every robot must load the SAME .db

When tb3_0 and tb3_1 both run their localization launches and you
want to use the laplacian consensus correction
(`enable_consensus:=true`), both Pis MUST point `db_path` at the
SAME database file. Different .db files mean different `map`
frames, and any inter-robot pose feedback would silently produce
garbage. Distribute the .db to every Pi before runtime, e.g.:

```bash
# from the laptop (where the mapping run produced the .db):
rsync ~/maps/tb3_1_room.db pi1@192.168.1.101:~/maps/room.db
rsync ~/maps/tb3_1_room.db pi2@192.168.1.102:~/maps/room.db
# then both Pis launch with db_path:=~/maps/room.db
```

Easiest convention: drop the per-robot suffix (use plain `room.db`)
so the same path works on every machine.

---

## Troubleshooting

### `pipeline running` never appears
- Try unplugging and replugging the OAK-D cable on the Pi
- Check `ls -la /dev/bus/usb/001/` — owner should allow read/write to
  user `pi2`. If not, re-run the udev rule install from `CAMERA_NOTES.md`

### `Serial /dev/ttyACM0 @ 115200 opened` never appears
- Check `ls -la /dev/ttyACM0` — should exist and be readable
- Check OpenCR has power
- Press the OpenCR RESET button if needed

### Pi temp climbs past 70 °C even though rtabmap is on the laptop
- That shouldn't happen with split mode. If it does, something else
  is loading the Pi. Check `top` or `htop`.

### `topic list` from laptop shows none of pi2's topics
- See `MAPPING_RUN_LAPTOP.md` § "Pre-flight" — usually a peers-file
  IP mismatch, not a Pi problem.
