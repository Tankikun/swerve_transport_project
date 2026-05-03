# Localization Run — From the Laptop

End-to-end procedure for **running RTAB-Map in localization-only
mode against a pre-built `.db`**, plus a multi-tier verification
that the robot is actually localized (not just running). Everything
is driven from your laptop; the Pi is reached over SSH.

> **Prerequisite**: you have a `.db` from a successful mapping run
> (see `MAPPING_RUN_LAPTOP.md`). Default location:
> `~/maps/tb3_1_room.db` on the laptop.

You will use **4 terminals on the laptop**:

| Terminal | Role |
|---|---|
| T1 | SSH'd into pi2 — runs Pi sensor stack |
| T2 | Native laptop — runs `rtabmap_slam` in localization mode |
| T3 | Native laptop — runs teleop (used during Section "Verify it's working") |
| T4 | Native laptop — verification queries (`ros2 topic echo`, `tf2_echo`) |

---

## 0. Operator-specific knobs

Same setup as the mapping guide. This guide assumes:
- Pi at `192.168.1.102` is named `pi2` and you can `ssh pi2@…`
- Laptop home is `~`, project at
  `~/swerve_transport_project/`
- Workspace is `~/swerve_transport_project/ros2_ws/`
- `~/fastdds_peers.xml` exists with the canonical structure (see
  the example below)

```
WORKSPACE=~/swerve_transport_project/ros2_ws
PEERS=~/fastdds_peers.xml
ROS_DOMAIN_ID=30
ROBOT_ID=tb3_1
```

**Canonical `~/fastdds_peers.xml` for the laptop** — three classes
of locators, all required:

```xml
<initialPeersList>
  <!-- 127.0.0.1: required so laptop processes (rtabmap, daemon,
       monitoring) can discover EACH OTHER. Without this, rtabmap
       runs but its outputs are invisible to other laptop terminals. -->
  <locator><udpv4><address>127.0.0.1</address>      <port>14910</port></udpv4></locator>
  <locator><udpv4><address>127.0.0.1</address>      <port>14912</port></udpv4></locator>
  <!-- robot peers -->
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
(Replace `192.168.1.114` with your laptop's actual LAN IP.)

---

## 1. Pre-flight

Before starting:
- Confirm `~/maps/tb3_1_room.db` exists and is a real map (10+ MB,
  non-zero `Link` count). If you're not sure, see "Verify the .db
  is real" in `MAPPING_RUN_LAPTOP.md`.
- **Place the robot somewhere INSIDE the area you mapped.**
  RTAB-Map can only localize when the camera sees a part of the
  room that's in the database. If the robot is in a part of the
  room you didn't drive through during mapping, localization will
  silently fail.
- Lights should be on (visual match needs the same lighting
  conditions as mapping, ideally).

---

## 2. Disable Pi clock drift (one-time per Pi-boot)

Same step as the mapping guide. Skip if you've already disabled
`systemd-timesyncd` on pi2 since its last reboot.

```bash
date -u +'%Y-%m-%d %H:%M:%S'
```
Copy the output, then run (paste your date in place of `PASTE_DATE`):
```bash
ssh -t pi2@192.168.1.102 "sudo systemctl disable --now systemd-timesyncd && sudo date -u -s 'PASTE_DATE'"
```

Verify:
```bash
echo "laptop: $(date -u +%s)"
ssh pi2@192.168.1.102 "echo \"pi2:    \$(date -u +%s)\""
```
Difference should be < 5 s.

> Why this matters: RTAB-Map matches camera frames against TF/odom
> by timestamp. If the Pi's clock drifts hours away from the
> laptop, every frame is dropped silently. The mapping guide
> documents the same fix.

---

## 3. T1 — start the Pi sensor stack

Same launch as for mapping. SSH into pi2:

```bash
ssh pi2@192.168.1.102
```

Once at `pi2@ubuntu:~$`, paste:

```bash
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/pi2/fastdds_peers.xml
export ROS_DOMAIN_ID=30
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch swerve_bringup rtabmap_pi_sensors.launch.py \
    robot_id:=tb3_1 \
    cam_x:=0.128  cam_y:=0.000  cam_z:=-0.0175
```

Wait for these lines to appear:
```
[oak_camera_node-1]    ... oak_camera_node ready (tb3_1)
[oak_camera_node-1]    ... pipeline running. device=OAK-D-LITE
[static_transform_publisher-2] ... Spinning until stopped
[conveyor_base_node-3] ... ConveyorBaseNode activated
[ekf_node-4]           ... EKF node ready for tb3_1
```

**Leave T1 alone for the rest of the session.**

---

## 4. T2 — open a laptop terminal, set env, verify Pi topics

In a NEW laptop terminal (not SSH):

```bash
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/$USER/fastdds_peers.xml
export ROS_DOMAIN_ID=30
source /opt/ros/humble/setup.bash
source ~/swerve_transport_project/ros2_ws/install/setup.bash
```

Reset the daemon and check Pi topics are visible:
```bash
ros2 daemon stop ; sleep 2 ; ros2 daemon start ; sleep 6
ros2 topic list | grep -E '/tb3_1/(camera|odom|ekf)'
```

Expected (5–6 topics):
```
/tb3_1/camera/depth/camera_info
/tb3_1/camera/depth/image_raw
/tb3_1/camera/rgb/camera_info
/tb3_1/camera/rgb/image_raw
/tb3_1/ekf/odom
/tb3_1/odom
```

If empty, see "Discovery troubleshooting" in
`MAPPING_RUN_LAPTOP.md`.

---

## 5. T2 — launch rtabmap in LOCALIZATION mode

```bash
ros2 launch swerve_bringup rtabmap_laptop_localization.launch.py \
    robot_id:=tb3_1 \
    db_path:=~/maps/tb3_1_room.db
```

Differences from the mapping launch (mostly to be aware of, no
action needed):
- `Mem/IncrementalMemory: false` — does NOT add new keyframes,
  just matches against the existing ones
- Loads the entire `.db` into RAM at startup so global
  re-localization can hit any stored keyframe
- Also runs `slam_pose_relay_node` automatically — converts
  RTAB-Map's `PoseWithCovarianceStamped` output into the
  `PoseStamped` format that `ekf_node` already subscribes to,
  closing the SLAM-correction loop

**Wait for `rtabmap started` in the log.** Then RTAB-Map enters
"global re-localization" mode: it scans the entire stored map
trying to find a visual match for the current camera frame.

Expected log behaviour:
- For the first 5–30 seconds, you'll see `Rate=` lines but pose
  outputs may not yet appear (RTAB-Map is still searching for
  match).
- Once a match is found, you'll see lines like
  `Localization succeeded` or loop closure events. From then on,
  RTAB-Map publishes `/tb3_1/rtabmap/localization_pose` as it
  re-confirms matches.

> If `rtabmap started` never appears, see Troubleshooting at the
> bottom.

---

## 6. T4 — open a verification terminal

In a NEW laptop terminal (env exports as Step 4):

```bash
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/$USER/fastdds_peers.xml
export ROS_DOMAIN_ID=30
source /opt/ros/humble/setup.bash
source ~/swerve_transport_project/ros2_ws/install/setup.bash
```

Now go through the multi-tier verification below.

---

# How to know the robot is successfully localized

Five checks, ordered by strength of evidence. The robot is
properly localized only when **all five pass**. Take each one in
turn — they're cheap.

## ✓ Check 1 — `slam/pose` is publishing (1-3 Hz)

```bash
ros2 topic hz /tb3_1/slam/pose
```
- **PASS**: rate jumps to **1–3 Hz** within ~30 seconds and stays
  there.
- **FAIL**: stays at "no new messages". RTAB-Map can't find a
  visual match. Move the robot to where you started mapping (or
  re-map a bigger area).

This is the first signal — `slam/pose` only publishes after RTAB-Map
finds a match against the stored map.

Press Ctrl+C after a few seconds.

## ✓ Check 2 — TF tree includes `map → tb3_1_base_link`

```bash
ros2 run tf2_ros tf2_echo map tb3_1_base_link
```
- **PASS**: prints a `Translation:` and `Rotation:` updating in
  real time. Translation values should be sane (within a few
  meters of where you mapped).
- **FAIL**: "Could not transform" errors loop endlessly. Means
  the SLAM-pose feedback hasn't reached `ekf_node` yet, or
  Check 1 still fails.

Press Ctrl+C.

This is the cleanest "system thinks it knows where I am" signal.

## ✓ Check 3 — EKF pose changes coherently when the robot moves

In T4, stream the EKF pose:
```bash
ros2 topic echo /tb3_1/ekf/odom --field pose.pose.position
```

Open T3 (a temporary teleop terminal — same env exports as Step 4):
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
    --ros-args -r cmd_vel:=/tb3_1/cmd_vel
```
> If "Package not found": `sudo apt install -y ros-humble-teleop-twist-keyboard`

Test these motions:

| Action | Expected EKF pose behaviour |
|---|---|
| Robot stationary | `x`, `y` stay constant within ±1 cm |
| Tap teleop forward 10 cm | `x` and/or `y` change by ~0.1 m in a sensible direction |
| Tap teleop backward 10 cm | Pose returns close to where it was |
| Drive in a small loop, return to start | Pose returns to within ~5 cm of start value |

- **PASS**: all four behave as expected.
- **FAIL**: pose is jumpy (jumps several meters between samples)
  → false-positive loop closures. Re-map a bigger area, or see
  "Localization keeps jumping" in Troubleshooting.
- **FAIL**: pose doesn't change when robot moves → SLAM
  corrections aren't reaching EKF. Confirm Checks 1 and 2 still
  pass.

Stop teleop and the echo when done.

## ✓ Check 4 — Pose is stable when the robot is stopped

Same `ros2 topic echo /tb3_1/ekf/odom --field pose.pose.position`
in T4. Don't touch teleop.
- **PASS**: pose value drifts < 2 cm/min.
- **FAIL**: pose is constantly jittering by tens of cm. SLAM is
  switching between false matches. Re-map bigger area.

## ✓ Check 5 — The "kidnap test" (gold standard)

This is the test that PROVES SLAM correction is working — wheel
odometry alone can't pass it.

1. Note the current EKF pose: `ros2 topic echo /tb3_1/ekf/odom --once`.
2. Without using teleop, **physically pick the robot up and put
   it back down ~30 cm away** (still inside the mapped area).
3. Watch T4's pose stream.

- **PASS**: Within 5–15 seconds, EKF pose JUMPS to the new
  location. RTAB-Map saw the new view, matched it to a different
  part of the stored map, the relay pushed the corrected pose to
  EKF, and EKF snapped its estimate. Wheel-only EKF can NEVER do
  this — no wheel rotation = no detected motion.
- **FAIL**: pose stays at the old location forever. SLAM
  correction loop is broken somewhere. Restart T2 (Step 5) and
  retry.

This single check is the ultimate proof: **only SLAM can recover
pose after a teleport.**

---

## Important caveat: localization only works in the mapped area

RTAB-Map cannot localize the robot if it's in a part of the room
that wasn't covered during mapping. Symptoms:
- Check 1 fails (`slam/pose` never starts publishing)
- Check 2 fails (no `map → tb3_1_base_link` transform)
- The rtabmap log will repeat `Localization failed` warnings
  every few seconds

Fix:
- Move the robot to where you started mapping
- Or do another mapping run that covers the area you actually
  want to operate in

---

## Stopping the run

In this exact order:
1. **T3 (teleop, if still open)** — Ctrl+C
2. **T4 (verification echoes)** — Ctrl+C
3. **T2 (rtabmap)** — Ctrl+C
4. **T1 (Pi sensors)** — Ctrl+C

---

## Troubleshooting

### `rtabmap started` never appears in T2
- Check T1 — is the Pi sensor launch still running?
- Re-do verification of camera and odom rates from the mapping
  guide's Step 5
- Try `--ros-args --log-level debug` on the launch line in Step 5

### Check 1 fails: `slam/pose` never publishes
- Robot is outside the mapped area — see "Important caveat" above
- The map is too small (< 10 keyframes) — re-map with longer drive
- Visual conditions don't match (mapping was done with lights on,
  now off, etc.) — re-map under the actual operating conditions
- The .db file got corrupted somehow — re-map from scratch

### Localization keeps "jumping" after a few seconds
The `/ekf/odom` remap creates a small feedback loop (rtabmap →
ekf → rtabmap). If you see jumpy poses, edit
`rtabmap_laptop_localization.launch.py` and change
`('odom', f'/{robot_id}/ekf/odom')` back to
`('odom', f'/{robot_id}/odom')`. Rebuild:
```bash
cd ~/swerve_transport_project/ros2_ws && colcon build --packages-select swerve_bringup
source install/setup.bash
```
Then restart T2.

### `delay=` is huge (1000s of seconds) in rtabmap log
Pi clock slipped. Re-do Step 2, then restart T1 (Step 3) and T2
(Step 5). The Pi's `systemd-timesyncd` may have re-enabled itself
— check with `ssh pi2@192.168.1.102 "systemctl is-enabled systemd-timesyncd"`. If "enabled", run Step 2 again
(it includes `disable --now`).

### `MsgConversion.cpp ... TF has two or more unconnected trees`
TF chain broken. Should not happen on current code (per-robot
prefixed frames). If it does, restart T1's launch — old processes
that pre-date the per-robot frame fix may still be running.

### Topics from rtabmap don't appear in `ros2 topic list`
Laptop's `~/fastdds_peers.xml` is missing `127.0.0.1` in
`<initialPeersList>`. See Section 0 for the canonical correct
file. After fixing, restart the rtabmap launch (Step 5) and the
daemon (`ros2 daemon stop ; sleep 2 ; ros2 daemon start`).

### Discovery is broken (Pi topics not visible from laptop in Step 4)
See "Discovery troubleshooting" in `MAPPING_RUN_LAPTOP.md` — same
fix applies.

---

## Multi-robot — every robot must load the SAME .db

When both robots run their localization launches and you intend to
use the laplacian consensus correction (`enable_consensus:=true`),
every robot's launch MUST point `db_path` at the SAME `.db` file.
Different `.db` files mean different `map` frames, and inter-robot
pose feedback would silently produce nonsense.

Distribute the .db (built on the laptop) to every Pi:
```bash
rsync ~/maps/tb3_1_room.db pi1@192.168.1.101:~/maps/room.db
rsync ~/maps/tb3_1_room.db pi2@192.168.1.102:~/maps/room.db
```
Then point every robot's localization launch at `~/maps/room.db`.

Easiest convention: drop the per-robot suffix. Use plain `room.db`
on every machine that runs a localization launch.
