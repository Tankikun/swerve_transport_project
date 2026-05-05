# Localization Run — From the Laptop

End-to-end procedure for **running RTAB-Map in localization-only
mode against a pre-built `.db`**, plus a multi-tier verification
that the robot is actually localized AND ready for Nav2. Everything
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

> ### Known issues (read before you start)
>
> 1. **`rtabmap_laptop_localization.launch.py` does NOT load
>    `swerve_bringup/config/rtabmap_localization.yaml`.** The launch
>    file passes a small inline parameter dict directly to the
>    `rtabmap` node; the YAML described in `CLAUDE.md` is never
>    referenced (the `swerve_bringup/config/` directory is not even
>    present in the install share). As a result:
>    - `Rtabmap/DetectionRate` falls back to the rtabmap default of
>      ~1 Hz instead of the 5 Hz in the YAML. Expect `/slam/pose`
>      around **~1 Hz**, not 1–3 Hz.
>    - Kidnap-test convergence (Check 5) takes **30–60 s**, not
>      5–15 s.
>    - `Rtabmap/PublishStats` and other YAML-only knobs are at their
>      rtabmap defaults.
>    Flagged for a separate cleanup PR — do not fix in this run.
> 2. **Launch-file docstring (line 34) still says
>    `~/swerve_transport_project/install/setup.bash` (legacy path).**
>    The real workspace path is
>    `~/swerve_transport_project/ros2_ws/install/setup.bash`. Same
>    cleanup PR.
> 3. **Middleware mismatch.** `README.md` and `CLAUDE.md` say the
>    project uses `rmw_zenoh_cpp` with a Zenoh router on the laptop.
>    Every run guide (this one + `MAPPING_RUN_LAPTOP.md`) assumes
>    FastDDS with `~/fastdds_peers.xml`. The lab runs FastDDS in
>    practice; the Zenoh story is aspirational. Flagged — do not
>    silently change either side.

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

Confirm camera mount values **for tb3_1** — these MUST match the
values used during mapping, otherwise `map → tb3_1_base_link` will
be off by the camera-mount delta:

| arg | meaning | tb3_1 measured |
|---|---|---|
| `cam_x` | forward offset from base_link [m] | `0.128` |
| `cam_y` | sideways offset (+ left) | `0.000` |
| `cam_z` | vertical offset (+ up) | `-0.0175` |

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
> **Replace `192.168.1.114` with your laptop's actual LAN IP.**
> If you don't know it, see "Discovery troubleshooting → find your
> laptop IP" in `MAPPING_RUN_LAPTOP.md`. The `127.0.0.1`
> port pair (`14910`/`14912`) is the FastDDS unicast metatraffic
> port-pair convention (`PB + DG·domain + PG·participant + offset`)
> for `ROS_DOMAIN_ID=30`. Verify against your actual working setup
> if you previously used a different domain.

---

## 1. Pre-flight

Before starting:
- Confirm `~/maps/tb3_1_room.db` exists and is a real map (10+ MB,
  non-zero `Link` count). If you're not sure, see "Verify the .db
  is real" in `MAPPING_RUN_LAPTOP.md`.
- **Confirm OAK-D is enumerated on pi2:**
  ```bash
  ssh pi2@192.168.1.102 "lsusb | grep Movidius"
  ```
  Expected: a line containing `Movidius MyriadX`. If empty: re-seat
  the USB cable on the Pi (USB 3 port required), or power-cycle the
  camera. Do NOT continue if missing — `oak_camera_node` will fail
  silently mid-launch.
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

Single SSH invocation (avoids paste-the-date races; the `$(...)`
runs locally on the laptop and is interpolated into the SSH
command before transmission):
```bash
ssh -t pi2@192.168.1.102 "sudo systemctl disable --now systemd-timesyncd && sudo date -u -s \"$(date -u +'%Y-%m-%d %H:%M:%S')\""
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

**Wait for these lines** (in roughly this order, **expect ~15 s
before all four blocks appear** — `conveyor_base_node` blocks for
`BOOT_WAIT_S = 5.0 s` after opening the serial port to let OpenCR
finish homing; the camera pipeline also takes a few seconds to spin
up):
```
[oak_camera_node-1]    ... oak_camera_node ready (tb3_1) — rgb=640x400@15fps  depth=640x400  aligned_to_rgb=True  depthai=3.5.0
[oak_camera_node-1]    ... pipeline running. device=OAK-D-LITE rgb_K diag=...
[static_transform_publisher-2] ... Static transform from 'tb3_1_base_link' to 'tb3_1_oak_rgb_camera_optical_frame' with translation: ('0.128', '0.000', '-0.0175')
[conveyor_base_node-3] ... Serial /dev/ttyACM0 @ 115200 opened.
[conveyor_base_node-3] ... Waiting 5.0s for OpenCR boot + homing...
[conveyor_base_node-3] ... Serial ready.
[conveyor_base_node-3] ... ConveyorBaseNode configured for /tb3_1
[ekf_node-4]           ... EKF node ready for tb3_1
```

> Don't Ctrl+C while it looks frozen during the 5 s
> `BOOT_WAIT_S` — that's just the OpenCR homing wait, not a hang.

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

> Workspace path is `ros2_ws/install`, **not** `install/`. The
> launch-file docstring still references the legacy
> `~/swerve_transport_project/install/setup.bash` — that path does
> not exist and will give "file not found in share directory".

Reset the daemon and check Pi topics are visible:
```bash
ros2 daemon stop ; sleep 2 ; ros2 daemon start ; sleep 6
ros2 topic list | grep -E '/tb3_1/(camera|ekf/odom|odom$)'
```

**Expected (exactly 6 topics — 4 camera + `/odom` + `/ekf/odom`):**
```
/tb3_1/camera/depth/camera_info
/tb3_1/camera/depth/image_raw
/tb3_1/camera/rgb/camera_info
/tb3_1/camera/rgb/image_raw
/tb3_1/ekf/odom
/tb3_1/odom
```

> The regex intentionally anchors `odom$` so it does NOT match
> `/tb3_1/cmd_vel` if it ever appears in the list.

If empty, see "Discovery troubleshooting" in
`MAPPING_RUN_LAPTOP.md`.

---

## 5. T2 — launch rtabmap in LOCALIZATION mode

```bash
ros2 launch swerve_bringup rtabmap_laptop_localization.launch.py \
    robot_id:=tb3_1 \
    db_path:=~/maps/tb3_1_room.db \
    --ros-args -r odom:=/tb3_1/odom
```

> **Use `~`, not `$HOME`, in `db_path`.** The launch file expands
> the path with `os.path.expanduser`, which only expands `~` when
> it is the **first character** of the string. `$HOME/...` works
> too because the shell expands it before `ros2 launch` ever sees
> it, but `~/...` is what the launch's WARNING-on-missing-file
> path-check assumes.

### Why the `--ros-args -r odom:=/tb3_1/odom` override

The launch file's default remap is
`('odom', f'/{robot_id}/ekf/odom')`. That creates a feedback loop:
RTAB-Map subscribes to `/tb3_1/ekf/odom`, computes a corrected
pose, the relay node hands it to `ekf_node` as a SLAM
observation, and `ekf_node` re-emits a now-corrected
`/tb3_1/ekf/odom` — which RTAB-Map ingests on the next cycle as
its prediction. The corrected pose sits inside its own correction
input, and any small inconsistency snowballs into "jumpy"
localization.

`CLAUDE.md` defines the canonical pose flow as:

```
/{robot_id}/odom (raw)            ──► ekf_node (prediction step)
                                          │
camera ─► rtabmap (uses RAW odom!) ─► /rtabmap/localization_pose
                                          │
                                  slam_pose_relay_node
                                          │
                                          ▼
                                   /{robot_id}/slam/pose
                                          │
                                          ▼
                                  ekf_node (correction step)
                                          │
                                          ▼
                              /{robot_id}/ekf/odom (authoritative)
```

So RTAB-Map should consume **raw** `/tb3_1/odom`, not the EKF
output. The override at the end of the `ros2 launch` line above
re-points the `odom` topic remap to raw odom and breaks the loop.
This makes the historical "Localization keeps jumping" troubleshooting
note (now removed below) obsolete.

### What this launch is doing

Differences from the mapping launch (mostly to be aware of, no
action needed):
- `Mem/IncrementalMemory: false` — does NOT add new keyframes,
  just matches against the existing ones
- `Mem/InitWMWithAllNodes: true` — loads the entire `.db` into RAM
  at startup so global re-localization can hit any stored
  keyframe. **Cost:** 5–30 s startup time during which `slam/pose`
  publishes nothing while RTAB-Map searches the whole map for the
  first match.
- Also runs `slam_pose_relay_node` automatically — converts
  RTAB-Map's `PoseWithCovarianceStamped` output into the
  `PoseStamped` format that `ekf_node` already subscribes to,
  closing the SLAM-correction loop. **This node is load-bearing
  glue, not an optional helper** — `ekf_node`'s correction step
  reads `/tb3_1/slam/pose`, which only exists because the relay
  publishes it. Per `CLAUDE.md`: "Do not remove this node — the
  message types are incompatible and cannot be fixed with a remap
  alone."

**Wait for `RTAB-Map started` (or `Initialization complete!`) in
the log.** Then RTAB-Map enters "global re-localization" mode: it
scans the entire stored map trying to find a visual match for the
current camera frame.

Expected log behaviour:
- The slam_pose_relay's banner line
  (`slam_pose_relay: /tb3_1/rtabmap/localization_pose
  (PoseWithCovarianceStamped) -> /tb3_1/slam/pose (PoseStamped)`)
  will print once at startup. **The relay is silent after that** —
  it doesn't log per message. So if you see the banner and nothing
  else, that's fine; it just means RTAB-Map hasn't found a match
  yet and there's nothing to forward.
- For the first 5–30 s of incremental match-search (and longer if
  the `.db` is large), pose outputs will not appear.
- Once a match is found, you'll see lines like
  `Localization succeeded` or loop closure events. From then on,
  RTAB-Map publishes `/tb3_1/rtabmap/localization_pose` as it
  re-confirms matches.

> If `RTAB-Map started` never appears, see Troubleshooting at the
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

## 7. Verify odom CHANGES when the robot moves (catches stationary "flow tests")

This single check catches "I forgot to teleop" failures BEFORE the
bigger SLAM checks.

In T4, start streaming odom positions:
```bash
ros2 topic echo /tb3_1/odom --field pose.pose.position
```

In a temporary 5th terminal (env exports + workspace source as in
Step 4), start a brief teleop:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
    --ros-args -r cmd_vel:=/tb3_1/cmd_vel
```
> If "Package not found": `sudo apt install -y ros-humble-teleop-twist-keyboard`

Tap `i` once briefly. Numbers in T4 must change. If they don't:
- Confirm the robot actually moved physically
- Check T1 — `conveyor_base_node` should be alive (no exceptions
  in the log) and emit POSE-derived odom
- Restart T1's sensor launch

When verified, **Ctrl+C T4's echo** and the temporary teleop. Drive
the robot back to the start tape mark. (You'll re-use teleop in
Check 3 below.)

---

# How to know the robot is successfully localized

Six checks, ordered by strength of evidence. The robot is properly
localized AND Nav2-ready only when **all six pass**. Take each one
in turn — they're cheap.

## ✓ Check 1 — `slam/pose` is publishing

```bash
ros2 topic hz /tb3_1/slam/pose
```
- **PASS**: rate jumps to **~1 Hz** within ~30 seconds and stays
  there. (1 Hz, not 5 Hz, because the launch ignores
  `rtabmap_localization.yaml` — see "Known issues" at the top.)
- **FAIL**: stays at "no new messages". RTAB-Map can't find a
  visual match. Move the robot to where you started mapping (or
  re-map a bigger area).

This is the first signal — `slam/pose` only publishes after the
relay forwards a `localization_pose` from RTAB-Map.

If the relay-out topic is silent but you suspect RTAB-Map is
actually matching, check the input side:
```bash
ros2 topic hz /tb3_1/rtabmap/localization_pose
```
If THIS is publishing but `/tb3_1/slam/pose` isn't, the relay node
is down (see Troubleshooting). If neither is publishing, RTAB-Map
itself isn't matching.

Press Ctrl+C after a few seconds.

## ✓ Check 2 — TF tree includes `map → tb3_1_base_link`

```bash
ros2 run tf2_ros tf2_echo map tb3_1_base_link
```
- **PASS**: prints a `Translation:` and `Rotation:` updating in
  real time. Translation values should be non-trivial (within a
  few meters of where you mapped, not all-zeros).
- **FAIL**: "Could not transform" errors loop endlessly.

The `map → tb3_1_odom` half of this chain comes from **rtabmap**,
not from `ekf_node` (which publishes only an `Odometry` message,
no TF — the EKF has no `tf2_ros.TransformBroadcaster`). So a fail
here means either:
1. RTAB-Map isn't matching yet (Check 1 still failing), so it hasn't
   published `map → tb3_1_odom`, OR
2. The `tb3_1_odom → tb3_1_base_link` half (from
   `conveyor_base_node`) is missing — restart T1 if Step 7
   already passed but this transform is gone.

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

Test these motions:

| Action | Expected EKF pose behaviour |
|---|---|
| Robot stationary | `x`, `y` stay constant within ±1 cm |
| Tap teleop forward 10 cm | `x` and/or `y` change by ~0.1 m in a sensible direction |
| Tap teleop backward 10 cm | Pose returns close to where it was |
| Drive in a small loop, return to start | Pose returns to within ~5 cm of start value |

- **PASS**: all four behave as expected.
- **FAIL**: pose is jumpy (jumps several meters between samples)
  → false-positive loop closures. Re-map a bigger area.
- **FAIL** with two distinguishable sub-cases when "pose doesn't
  change while the robot moves":
  - **Wheel odom dead**: re-do Step 7. If `/tb3_1/odom` itself is
    constant during teleop, the OpenCR/serial bridge is dead;
    restart T1.
  - **SLAM correction dead**: if `/tb3_1/odom` updates fine but
    `/tb3_1/ekf/odom` is frozen during motion, the EKF prediction
    step is wedged. Note that `_publish` fires on every `_odom_cb`
    (~33 Hz from the firmware POSE rate), so a truly frozen
    `/tb3_1/ekf/odom` means the EKF subscription itself is broken
    (e.g. crashed). Confirm `ekf_node` is still running on pi2 in
    T1's log.

Stop teleop and the echo when done.

## ✓ Check 4 — Pose is stable when the robot is stopped

Same `ros2 topic echo /tb3_1/ekf/odom --field pose.pose.position`
in T4. Don't touch teleop.
- **PASS**: pose value drifts < **5 cm** over 1 minute. (Wheel
  prediction adds tiny numerical drift each tick; that's expected.)
- **FAIL**: pose is constantly jittering by tens of cm. SLAM is
  switching between false matches. Re-map bigger area.

## ✓ Check 5 — The "kidnap test" (gold standard)

This is the test that PROVES SLAM correction is working — wheel
odometry alone can't pass it.

1. Note the current EKF pose: `ros2 topic echo /tb3_1/ekf/odom --once`.
2. Without using teleop, **physically pick the robot up and put it
   back down at least 1 m away in a part of the room where the
   camera will see DIFFERENT visual features than its current
   keyframe** (i.e., a different keyframe area, not 30 cm shuffled
   in front of the same wall — that displacement may be inside the
   same-keyframe radius and won't trigger a re-match).
3. Watch T4's pose stream.

- **PASS**: Within **30–60 s** (default detection rate ~1 Hz, see
  "Known issues" — would be 5–15 s if the YAML loaded), EKF pose
  JUMPS to the new location. RTAB-Map saw the new view, matched
  it to a different part of the stored map, the relay pushed the
  corrected pose to EKF, and EKF snapped its estimate. Wheel-only
  EKF can NEVER do this — no wheel rotation = no detected motion.
- **FAIL**: pose stays at the old location forever. SLAM
  correction loop is broken somewhere. Restart T2 (Step 5) and
  retry.

This single check is the ultimate proof: **only SLAM can recover
pose after a teleport.**

## ✓ Check 6 — Nav2 readiness (one-shot bundled verdict)

A single shell-pasteable block that bundles the three signals
Nav2 actually consumes. Paste in T4:

```bash
echo "/slam/pose hz:";        timeout 5 ros2 topic hz /tb3_1/slam/pose 2>&1 | tail -2
echo "/ekf/odom frame_id:";   ros2 topic echo --once /tb3_1/ekf/odom nav_msgs/msg/Odometry --field header.frame_id
echo "map → base_link:";      timeout 4 ros2 run tf2_ros tf2_echo map tb3_1_base_link 2>&1 | tail -10
```

PASS criteria (all three must hold):
- `/slam/pose hz`: any line containing `average rate:` with a
  positive value (~1 Hz expected).
- `/ekf/odom frame_id`: prints exactly `tb3_1_odom` (this is what
  Nav2's local costmap will use as its `odom` frame).
- `map → tb3_1_base_link`: prints a `Translation:` block with
  values that are NOT all `0.000` and `Rotation:` quaternion is
  not identity (`0,0,0,1`).

If all three pass, the localization stack is ready to feed Nav2.
If any fail: the corresponding earlier Check identifies which
piece is wrong (`/slam/pose hz` → Check 1; frame_id wrong →
Step 3 ekf_node restart; identity TF → Check 2 / Check 5 retry).

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

> In localization-only mode there's nothing to flush to disk
> (`Mem/IncrementalMemory: false`), so this order is purely a
> safety habit: stop the things that command the robot first, then
> the SLAM brain, then the sensor stack last so you can still see
> motion on the map until the very end of the session.

---

## Troubleshooting

### `RTAB-Map started` never appears in T2
- Check T1 — is the Pi sensor launch still running?
- Re-do verification of camera and odom rates from the mapping
  guide's Step 5
- Try `--ros-args --log-level debug` on the launch line in Step 5
  (note: this is in addition to the existing `--ros-args` flag —
  combine into one block)

### Check 1 fails: `slam/pose` never publishes
- Robot is outside the mapped area — see "Important caveat" above
- The map is too small (< 10 keyframes) — re-map with longer drive
- Visual conditions don't match (mapping was done with lights on,
  now off, etc.) — re-map under the actual operating conditions
- The .db file got corrupted somehow — re-map from scratch

### `slam/pose` is silent but `rtabmap/localization_pose` is publishing
The `slam_pose_relay_node` died. Check T2's log for a Python
traceback in the `slam_pose_relay_tb3_1` node section. Restart
T2 (Step 5).

### `delay=` is huge (1000s of seconds) in rtabmap log
Pi clock slipped. Re-do Step 2, then restart T1 (Step 3) and T2
(Step 5). The Pi's `systemd-timesyncd` may have re-enabled itself
— check with `ssh pi2@192.168.1.102 "systemctl is-enabled systemd-timesyncd"`. If "enabled", run Step 2 again (it includes
`disable --now`).

> Note: the verbose `rtabmap (NN): … delay=…` line frequency
> depends on `Rtabmap/PublishStats`, which is set in the
> (currently unloaded) YAML. With the YAML not loaded you'll see
> default rtabmap stat lines, not necessarily the `delay=` field.
> Look at log timestamps vs the Pi's clock manually if `delay=`
> doesn't appear.

### `MsgConversion.cpp ... TF has two or more unconnected trees`
TF chain broken. Should not happen on current code (per-robot
prefixed frames). If it does, restart T1's launch — old processes
that pre-date the per-robot frame fix may still be running.

### Topics from rtabmap don't appear in `ros2 topic list`
Laptop's `~/fastdds_peers.xml` is missing `127.0.0.1` in
`<initialPeersList>`. See Section 0 for the canonical correct
file. After fixing, restart the rtabmap launch (Step 5) and the
daemon (`ros2 daemon stop ; sleep 2 ; ros2 daemon start`).

> (See "Known issues" #3 — the project's `README.md` claims Zenoh
> middleware. The run guides assume FastDDS. If you actually have
> a working Zenoh setup, this entire troubleshooting section is
> moot — but no operator on this team is currently running Zenoh.)

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

> **Note: the `rsync` step below is for the all-on-Pi localization
> launch (`rtabmap_localization.launch.py` running ON each Pi),
> not for this guide's laptop-side localization.** If you're
> running `rtabmap_laptop_localization.launch.py` per robot from
> one or more laptops, every laptop just needs its own copy of the
> `.db` at the same path; you do NOT need to push the .db to the
> Pis at all (the Pis only run sensors in the split architecture).

Distribute the .db (built on the laptop) to every Pi (only needed
for the all-on-Pi localization launch):
```bash
rsync ~/maps/tb3_1_room.db pi1@192.168.1.101:~/maps/room.db
rsync ~/maps/tb3_1_room.db pi2@192.168.1.102:~/maps/room.db
```
Then point every robot's localization launch at `~/maps/room.db`.

Easiest convention: drop the per-robot suffix. Use plain `room.db`
on every machine that runs a localization launch.
