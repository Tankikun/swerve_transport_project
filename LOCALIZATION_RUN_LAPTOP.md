# Localization Run — From the Laptop

Run RTAB-Map in localization-only mode against your saved `.db`, verify the robot knows where it is, and you're ready for Nav2.

**Prerequisite**: a working `.db` from `MAPPING_RUN_LAPTOP.md` at `~/maps/tb3_1_room.db`.

**Time**: ~5 minutes from terminal-1 to verified-localized.

---

## TL;DR — the one verification

After all 4 terminals are up and you've driven the robot ~2 m, paste this:

```bash
echo "=== /slam/pose hz ==="
timeout 5 ros2 topic hz /tb3_1/slam/pose 2>&1 | tail -2
echo "=== /ekf/odom frame_id (should be tb3_1_odom) ==="
ros2 topic echo --once /tb3_1/ekf/odom nav_msgs/msg/Odometry --field header.frame_id
echo "=== map → tb3_1_base_link (should be non-zero, updating) ==="
timeout 4 ros2 run tf2_ros tf2_echo map tb3_1_base_link 2>&1 | tail -8
```

Pass = green light to move on (Nav2, formation, etc.). See [Verify it works](#verify-it-works) for what each line means.

---

## What you need before starting

Tick all four:
- [ ] `~/maps/tb3_1_room.db` exists (10+ MB, came from a successful mapping run)
- [ ] Pi clock is in sync (Step 1 below — re-do after every Pi reboot)
- [ ] OAK-D is enumerated on pi2: `ssh pi2@192.168.1.102 "lsusb | grep Movidius"` returns a `Movidius MyriadX` line
- [ ] Robot is parked **inside** the area you mapped (RTAB-Map can't localize where you didn't drive)

Camera mount values for `tb3_1` — these MUST match what was used during mapping:

| arg | value |
|---|---|
| `cam_x` | `0.128` |
| `cam_y` | `0.000` |
| `cam_z` | `-0.0175` |

---

## Step 0 — Env (every laptop terminal needs this once)

Source these (or put them in `~/.bashrc`):

```bash
source /opt/ros/humble/setup.bash
source ~/swerve_transport_project/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=30
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/$USER/fastdds_peers.xml
```

If `~/fastdds_peers.xml` doesn't exist or is missing the `127.0.0.1` loopback entries, see [Troubleshooting → Discovery](#discovery).

---

## Step 1 — Sync pi2 clock (skip if already done since pi2 last booted)

```bash
ssh -t pi2@192.168.1.102 "sudo systemctl disable --now systemd-timesyncd && sudo date -u -s \"$(date -u +'%Y-%m-%d %H:%M:%S')\""
```

Verify within 5 s:
```bash
echo "laptop: $(date -u +%s)" ; ssh pi2@192.168.1.102 "echo \"pi2:    \$(date -u +%s)\""
```

---

## Step 2 — T1: start Pi sensor stack

SSH to pi2 and run:

```bash
ssh pi2@192.168.1.102
```

```bash
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/pi2/fastdds_peers.xml
export ROS_DOMAIN_ID=30
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch swerve_bringup rtabmap_pi_sensors.launch.py robot_id:=tb3_1 cam_x:=0.128 cam_y:=0.000 cam_z:=-0.0175
```

Wait ~15 s for these 4 lines (the 5-second pause after `Serial /dev/ttyACM0 @ 115200 opened.` is normal — OpenCR is homing):

```
[oak_camera_node-1]    ... oak_camera_node ready (tb3_1) — rgb=640x400@15fps  ... depthai=3.5.0
[oak_camera_node-1]    ... pipeline running. device=OAK-D-LITE
[conveyor_base_node-3] ... ConveyorBaseNode activated
[ekf_node-4]           ... EKF node ready for tb3_1
```

**Leave T1 alone for the rest of the session.**

---

## Step 3 — T2: verify Pi topics, then launch rtabmap

In a NEW laptop terminal (Step 0 env applied):

```bash
ros2 daemon stop ; sleep 2 ; ros2 daemon start ; sleep 6
ros2 topic list | grep -E '/tb3_1/(camera|ekf/odom|odom$)'
```

Expected — exactly **6 topics**:
```
/tb3_1/camera/depth/camera_info
/tb3_1/camera/depth/image_raw
/tb3_1/camera/rgb/camera_info
/tb3_1/camera/rgb/image_raw
/tb3_1/ekf/odom
/tb3_1/odom
```

If empty, see [Troubleshooting → Discovery](#discovery).

Then launch rtabmap in localization mode:

```bash
ros2 launch swerve_bringup rtabmap_laptop_localization.launch.py \
    robot_id:=tb3_1 \
    db_path:=~/maps/tb3_1_room.db
```

> **Use `~`, not `$HOME`**, for `db_path` — `os.path.expanduser` only expands `~` when it's the first character of the path.

> **About the odom feedback loop**: the launch defaults to `odom:=/tb3_1/ekf/odom`, which is technically a feedback loop (RTAB-Map's correction → ekf_node → ekf/odom → RTAB-Map's prior). In practice it works fine for verification — the loop gain is small. If you see jumpy poses (loop closures jumping the robot meters at a time), see [Why the override](#why-the-override) for the proper fix. For now, just run the command as written.

Wait for `RTAB-Map started` (or `Initialization complete!`). RTAB-Map then enters "global re-localization" mode — silently scanning the `.db` for a match. **This phase takes 5–30 s during which `/slam/pose` publishes nothing.** Once it finds the first match, you'll see `Localization succeeded` lines and `/slam/pose` starts ticking.

**Leave T2 alone.**

---

## Step 4 — T3: teleop and drive ~2 m

In a NEW terminal (Step 0 env):

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/tb3_1/cmd_vel
```

Press `z` 5 times to slow max speed to ~0.07 m/s. Drive a small loop through varied scenery — the robot needs to see different parts of the map for RTAB-Map to find a match. **Drive 1–2 meters before checking results.**

| Key | Action |
|---|---|
| `i` / `,` | forward / back |
| `j` / `l` | rotate CCW / CW |
| `u` / `o` / `m` / `.` | diagonal (holonomic) |
| `k` or space | stop |

---

## Verify it works

In a NEW terminal (Step 0 env), paste this **single verdict block**:

```bash
echo "=== /slam/pose hz (PASS: ~1 Hz steady) ==="
timeout 5 ros2 topic hz /tb3_1/slam/pose 2>&1 | tail -2
echo "=== /ekf/odom frame_id (PASS: tb3_1_odom) ==="
ros2 topic echo --once /tb3_1/ekf/odom nav_msgs/msg/Odometry --field header.frame_id
echo "=== map → tb3_1_base_link (PASS: non-zero, updates as you drive) ==="
timeout 4 ros2 run tf2_ros tf2_echo map tb3_1_base_link 2>&1 | tail -8
```

| Output | Verdict |
|---|---|
| All three blocks return real data | ✅ **Localized — go to Nav2 / formation / next phase** |
| `slam/pose` says "no new messages" | ❌ RTAB-Map hasn't matched yet — drive 30+ s through varied scenery |
| `tf2_echo` says "Could not transform" | ❌ Same — wait for the first match |
| `frame_id` is anything other than `tb3_1_odom` | ❌ Wrong launch or remap — restart T2 |

Why ~1 Hz instead of 5 Hz: see [Known issue 1](#known-issues).

### Optional gold-standard test — kidnap

This is the test that PROVES SLAM correction (wheel odom can never pass it):

1. Note pose: `ros2 topic echo --once /tb3_1/ekf/odom nav_msgs/msg/Odometry --field pose.pose.position`
2. **Physically pick the robot up** and put it down ≥ 1 m away in a part of the room with **different visual features** than where it was.
3. Watch pose for 30–60 s.
4. ✅ PASS: pose JUMPS to the new location. ❌ FAIL: pose stays at old.

Why 30–60 s: detection rate is ~1 Hz default (see [Known issue 1](#known-issues)).

---

## Stopping the run

In this order (commands stop first, sensor last so you can still see motion if needed):

1. T3 (teleop) — `Ctrl+C`
2. T4 (verify) — `Ctrl+C`
3. T2 (rtabmap) — `Ctrl+C`
4. T1 (Pi sensors) — `Ctrl+C`

---

## Why the override

The launch file remaps RTAB-Map's `odom` input to `/tb3_1/ekf/odom`. That's a feedback loop — RTAB-Map publishes corrections that feed `ekf_node`, which publishes `ekf/odom`, which feeds back into RTAB-Map's odometry prior:

```
RTAB-Map (correction) → ekf_node → /tb3_1/ekf/odom → RTAB-Map (input) → ...   FEEDBACK LOOP
```

The canonical pose flow per `CLAUDE.md` should be:

```
/odom (raw) → ekf_node ← /slam/pose (RTAB correction via slam_pose_relay)
                ↓
            /ekf/odom  (authoritative — what Nav2 + laplacian read)
```

In practice the loop gain is small and the default works fine for verification. **If you see jumpy poses** (loop closures jumping the robot meters at a time), edit `ros2_ws/src/swerve_bringup/launch/rtabmap_laptop_localization.launch.py` and change the line:
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
Re-launch rtabmap. Pose feedback will now use raw wheel odom, breaking the loop.

`slam_pose_relay_node` (auto-started by the launch) is not optional — it converts RTAB's `PoseWithCovarianceStamped` to the `PoseStamped` that `ekf_node` subscribes to.

---

## Troubleshooting

### `RTAB-Map started` never appears in T2
- Check T1 is still running (no Ctrl+C, no errors)
- Re-verify the 6 topics from Step 3
- Add `--ros-args --log-level debug` to the launch line

### `/slam/pose` never starts publishing
- Robot is outside the mapped area — move it
- Map is too small — re-map a bigger region
- Lighting differs from mapping — re-map under operating conditions
- `.db` is corrupted — verify `Link` count > 0 (see mapping doc Step 13)

### Pose is jumpy (jumping meters at a time)
False loop closures. Re-map with the bigger area, or stop using `enable_consensus` if you're in formation mode.

### `delay=` huge in rtabmap log
Pi clock slipped. Re-do Step 1. The `delay=` line itself depends on `Rtabmap/PublishStats` which the launch doesn't enable (Known issue 1) — you may not see it at all.

### `MsgConversion.cpp ... TF has two or more unconnected trees`
Stale processes from before per-robot frames. Restart T1.

### <a name="discovery"></a>Discovery: Pi topics not visible from laptop
- `~/fastdds_peers.xml` missing `127.0.0.1` in `<initialPeersList>` — see canonical example below
- Laptop IP in `<defaultUnicastLocatorList>` is wrong — find yours with `ip -4 addr show | grep 192.168.1.`
- After fixing: restart rtabmap (Step 3) AND `ros2 daemon stop ; sleep 2 ; ros2 daemon start`

Canonical `~/fastdds_peers.xml` (replace `192.168.1.114` with your actual laptop LAN IP):

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

---

## Multi-robot — every robot loads the SAME .db

When both robots run their localization launches and you use `enable_consensus:=true` on the laplacian node, each robot's `db_path` MUST point at the SAME `.db` file. Different `.db` files mean different `map` frames → silent nonsense in inter-robot pose feedback.

For the all-on-Pi localization launch (NOT the laptop split mode this guide describes), distribute the `.db` to each Pi:
```bash
rsync ~/maps/tb3_1_room.db pi1@192.168.1.101:~/maps/room.db
rsync ~/maps/tb3_1_room.db pi2@192.168.1.102:~/maps/room.db
```

In laptop split mode (this guide), the `.db` lives only on the laptop — no rsync needed.

---

## Known issues

These are non-blocking but worth knowing about. Flagged for separate cleanup PRs.

1. **`rtabmap_laptop_localization.launch.py` doesn't load `swerve_bringup/config/rtabmap_localization.yaml`.** Detection rate falls back to ~1 Hz (vs 5 Hz in the YAML). Kidnap convergence is 30–60 s (vs 5–15 s). `Rtabmap/PublishStats` is off (no `delay=` lines).
2. **Launch file docstring (line 34) references the legacy path** `~/swerve_transport_project/install/setup.bash`. Real path is `~/swerve_transport_project/ros2_ws/install/setup.bash`.
3. **Middleware mismatch.** `README.md` and `CLAUDE.md` say `rmw_zenoh_cpp`. Every run guide assumes FastDDS. Lab actually runs FastDDS — Zenoh is aspirational.

---

## Live visualization (optional)

If you want to SEE the robot move on the map in a browser instead of grepping topics, run the live-pose viewer in `interface/`. See `interface/RUN_LOCALIZATION_VIEWER.md`.
