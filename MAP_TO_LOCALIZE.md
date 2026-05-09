# MAP → LOCALIZE — End-to-end runbook for `feature/map-to-localize`

This branch is **the verified, hands-on path** from a powered-off robot
to a green `LOC: LIVE` pill in the GUI with the cyan robot cone tracking
the real robot. Everything in this document was tested live on pi1 with
robot `tb3_0` on 2026-05-09 and produced visual localization at
**σ ≈ 4–35 mm** (real visual covariance, not sentinel "no confidence"
values).

If anything in this guide doesn't match what you see, **stop and read the
log line that disagrees** — every symptom we hit is documented at the
bottom of this file with the actual fix.

> **What this branch changes vs. main**
>
> 1. `ros2_ws/src/swerve_bringup/config/rtabmap_localization.yaml` — three
>    parameter changes that make RTAB-Map publish confident loc_pose
>    instead of sentinel covariance: `Vis/MinInliers: 10` (was 15),
>    `RGBD/MaxOdomCacheSize: "0"` (NEW), `Rtabmap/LoopThr: "0.08"` (NEW).
> 2. `interface/loc_doctor.py` — numpy covariance bug fix.
> 3. `scripts/drive_box.py` — automated mapping drive pattern.
> 4. `scripts/inject_initialpose.py` — CLI fallback for the GUI's 📍
>    Set Initial Pose button.
> 5. This runbook (`MAP_TO_LOCALIZE.md`).
>
> Nothing in the existing `interface/` GUI is changed — the 📍 button
> in `index.html` already does exactly what we need.

---

## Time budget

| Phase | Time |
|---|---|
| 1. Pre-flight (clock sync, OAK-D check) | 1 min |
| 2. Mapping (teleop drive of the room + save .db) | 5–10 min depending on room size |
| 3. Rsync .db + regenerate map.json | 2 min |
| 4. Localization launch + Set Initial Pose | 2 min |
| **Total cold-start** | **≈ 12–15 min** for a small room |

---

## Terminal layout

You'll need **5 terminals** + a browser. Open them all up front:

| # | Where | Role |
|---|---|---|
| **T1** | SSH'd into the robot's Pi | Mapping launch (Phase 2), then localization launch (Phase 4) |
| **T2** | Laptop | Flask GUI server |
| **T3** | Laptop | ROS pose bridge (TF → Flask, hint → /initialpose) |
| **T4** | Laptop | **Teleop keyboard** for the mapping drive (Phase 2), then for verifying the cone follows the robot (Phase 4.7) |
| **T5** | SSH'd into the Pi | `loc_doctor.py` for live diagnostic readout (always useful) |
| **Browser** | Laptop | Chrome/Firefox at `http://localhost:5002` |

**Robot ID convention used below**: `tb3_0` running on `pi1@172.20.10.9`.
For pi2 with `tb3_1`, swap both names everywhere.

---

## Phase 1 — Pre-flight

### 1.1 Sync the Pi's clock to the laptop

The Pi has no battery-backed RTC, so its clock resets to ~Aug 2025
on every boot. **If the Pi's clock is off, RTAB-Map silently drops
every camera frame** (TF_OLD_DATA warnings, empty .db). Do this every
time the Pi has been rebooted:

```bash
# On the laptop:
NOW=$(date -u +"%Y-%m-%d %H:%M:%S")
ssh pi1@172.20.10.9 "echo raspberry | sudo -S date -u -s '$NOW'"
ssh pi1@172.20.10.9 "date"   # should now read today's date
```

(Sudo password on the Pi is `raspberry` unless you've changed it.)

### 1.2 Verify the OAK-D camera enumerates

```bash
ssh pi1@172.20.10.9 "lsusb | grep Movidius"
# Expected: Bus 001 Device NNN: ID 03e7:2485 Intel Movidius MyriadX
```

If empty: re-seat the OAK-D's USB-C cable on a **blue USB-3 port**, wait
10 sec, retry. If the device shows up but RTAB-Map later prints
`Insufficient permissions to communicate with X_LINK_UNBOOTED device`,
add the udev rule from [Troubleshooting → OAK-D won't boot](#oak-perm).

### 1.3 Confirm this branch is checked out everywhere

```bash
# On the laptop:
cd ~/swerve_transport_project
git fetch origin && git checkout feature/map-to-localize && git pull

# On the Pi:
ssh pi1@172.20.10.9 "cd ~/swerve_transport_project && \
    git fetch origin && git checkout feature/map-to-localize && git pull && \
    cd ros2_ws && colcon build --symlink-install --packages-select swerve_bringup swerve_formation"
```

The `colcon build` is needed on the Pi the first time you switch to
this branch so the patched YAML installs into the share directory.
(Subsequent edits to YAML/Python don't need rebuild thanks to
`--symlink-install`.)

---

## Phase 2 — Mapping

This produces the `.db` that localization later loads.

### 2.1 T1 — Start mapping launch on the Pi

```bash
ssh pi1@172.20.10.9
# Once at the pi1@... prompt, paste this whole block:
unset FASTRTPS_DEFAULT_PROFILES_FILE     # ← important on hotspot/non-LAN networks
export ROS_DOMAIN_ID=30
source /opt/ros/humble/setup.bash
source ~/swerve_transport_project/ros2_ws/install/setup.bash

DB_PATH="$HOME/maps/$(date +%Y%m%d_%H%M%S)_lab.db"
mkdir -p ~/maps
echo "Will save: $DB_PATH"

ros2 launch swerve_bringup rtabmap_mapping.launch.py \
    robot_id:=tb3_0 \
    db_path:=$DB_PATH \
    cam_x:=0.128 cam_y:=0.000 cam_z:=-0.0175
```

> **Camera mount values.** The defaults shown (`0.128 / 0.000 / -0.0175`)
> are tb3_1's measured values. If pi1's robot has a different camera
> mount, **physically measure** and override — errors here propagate
> directly into every keyframe and make localization wrong even when it
> "succeeds." See `CLAUDE.md` for the measurement convention.

**Wait ~30 seconds.** You should see in order:
1. `Camera ready!` (OAK-D up)
2. `ConveyorBaseNode activated` (wheels + odom flowing)
3. `EKF node ready for tb3_0`
4. `RTAB-Map started` and then
   `rtabmap (1): Rate=1.00s ... (local map=1, WM=1)` lines at 1 Hz

**Leave T1 running.** Don't Ctrl-C until Phase 2.4.

### 2.2 T4 — Teleop drive the mapping run yourself

Place the robot at a sensible **starting spot** — somewhere you can
walk back to later (clicking 📍 Set Initial Pose at this same spot is
the easiest way to verify localization works in Phase 4). On the laptop
in T4:

```bash
unset FASTRTPS_DEFAULT_PROFILES_FILE
export ROS_DOMAIN_ID=30
source /opt/ros/humble/setup.bash
source ~/swerve_transport_project/ros2_ws/install/setup.bash

ros2 run teleop_twist_keyboard teleop_twist_keyboard \
    --ros-args -r cmd_vel:=/tb3_0/cmd_vel
```

The default `teleop_twist_keyboard` keys:

| Key | Action |
|---|---|
| `i` | Forward |
| `,` | Backward |
| `j` | Pivot CCW (left) |
| `l` | Pivot CW (right) |
| `u` / `o` | Forward + diagonal turn |
| `m` / `.` | Backward + diagonal turn |
| `q` / `z` | Increase / decrease overall speed |
| `w` / `x` | Increase / decrease linear speed only |
| `e` / `c` | Increase / decrease angular speed only |
| `k` / spacebar | Stop |

> **Set the speeds to safe-for-mapping values BEFORE you start driving.**
> The default speed teleop_twist_keyboard starts at can be too fast for
> good visual SLAM. Press `z` repeatedly to lower the linear speed to
> ~**0.10 m/s** (shown in the terminal as `currently: speed 0.10`), and
> separately tune angular to ~**0.20 rad/s** with the `e` / `c` pair.
> You can verify these match the speeds the firmware will actually
> execute by watching the OpenCR's POSE-rate output in T1.

#### What to drive (the four ingredients of a good map)

In any order, but cover all four:

1. **Drive slowly.** 0.10–0.15 m/s linear. Fast motion → motion blur on
   the OAK-D → bad ORB features → bad keyframes. The Pi 4 also can't
   keep up with 1 Hz RTAB-Map detection at faster speeds (it ends up
   with a 4–5 sec processing backlog and drops most frames).

2. **Pause + spin 360° at every station.** This is the single most
   important thing you can do. The 360° pivot is what gives RTAB-Map's
   loop-closure detector multiple chances at the same physical spot
   from different yaw angles — without it, your map will visit each
   location only from one heading and the matcher won't recognize the
   same place when you come back. **Pivot at the start, at every corner
   of your traversal, and at the end.** The pivots are slow (a full
   360° at 0.20 rad/s takes ~31 sec) — that's normal, don't rush them.

3. **Make multiple passes through the same area.** Drive a route, then
   drive the same route in the opposite direction, or zig-zag through
   it twice. Each repeated pass is a chance for a loop closure to fire
   and tighten the map.

4. **Return to the start.** End your drive at (or very near) the same
   spot you began. The "I'm back where I started" loop closure is the
   one that globally optimizes the whole trajectory and removes drift.

A good 1–2 m × 1–2 m room map takes 3–8 minutes of careful driving.
Aim for **20–30 loop closures** in the resulting `.db` (we'll inspect
this in 2.4). You'll see far fewer in low-texture rooms — that's a
known limitation of this environment, and the `Set Initial Pose` flow
in Phase 4 is designed around that limitation.

#### Watch T1 while you drive

You can tell mapping is going well by the keyframe counter in T1's
`rtabmap (N): ... (local map=K, WM=K)` lines. `K` should grow steadily
as you drive — typically a new keyframe every 0.1 m of translation or
every ~11° of rotation (the `RGBD/LinearUpdate` and `RGBD/AngularUpdate`
thresholds). If `K` stops growing while the robot is moving, your motion
isn't crossing the threshold, or RTAB-Map isn't getting frames — check
T5 (`loc_doctor`) for the failing topic.

> **If you'd rather skip teleop and run a known-quantity automated
> drive** (useful for repeatable A/B testing of YAML changes), the
> branch ships `scripts/drive_box.py` which executes a fixed
> 0.5 × 0.5 m box pattern with four 360° pivots. See [Optional —
> automated box drive](#auto-box) at the end of this doc. For real
> production mapping, teleop is what you want.

### 2.3 T1 — Watch RTAB-Map's verdict

While you're driving in T4, T1's log should show keyframes accumulating:

```
rtabmap (47): Rate=1.00s ... (local map=12, WM=12)
rtabmap (52): Rate=1.00s ... (local map=15, WM=15)
```

If `WM=K` is growing while you're driving, mapping is working. If it
stays at 1 even though you're moving the robot, mapping is broken —
stop driving, check T5's loc_doctor output, fix whatever's red there
before continuing. Typical causes: clock skew (Phase 1.1), camera
permission (Phase 1.2 + the OAK-D udev fix in Troubleshooting), or
T4's teleop publishing to the wrong topic (must be `/tb3_0/cmd_vel`,
note the prefix).

You may also see lines like:

```
[WARN] Rejected loop closure 64 -> 76: Not enough inliers 0/15 (matches=107)
```

These rejections are the textbook low-texture failure mode (lots of
visual matches, zero geometrically consistent). They're a known issue
in this environment — they don't prevent localization from working
*with the GUI's Set Initial Pose hint*. Don't worry about them yet.

### 2.4 T4 — Stop teleop, then T1 — stop mapping cleanly

When you've covered the area you want mapped (drove the route, did
your 360° pivots, returned to start):

1. **In T4: press `k` or spacebar** to stop the robot, then **Ctrl-C**
   to exit teleop_twist_keyboard. The OpenCR's 5 sec watchdog will
   also stop the wheels if you forget.

2. **In T1: Ctrl-C ONCE only.** RTAB-Map's clean shutdown serializes
   the `.db`. You should see:

   ```
   rtabmap: Saving database/long-term memory... (located at /home/pi1/maps/<NAME>.db)
   rtabmap: Saving database/long-term memory...done! (located at .../<NAME>.db, NN MB)
   ```

   A 30-150 MB file is healthy. <5 MB means RTAB-Map didn't add nodes —
   go to the [Troubleshooting → tiny .db](#tiny-db) section.

> **Don't Ctrl-C twice in T1.** The first Ctrl-C tells RTAB-Map to
> serialize the database. A second one before it finishes interrupts
> the save and you'll lose the map. Wait until you see "Saving... done!"
> in the log before doing anything else in T1.

---

## Phase 3 — Get the `.db` and `map.json` aligned

The browser's 3D point cloud is rendered from `interface/map.json` on
the laptop. The cyan robot cone is positioned by RTAB-Map running on
the Pi, which reads its own `.db`. **If those two come from different
.db files, the cone shows up in random locations** because the
coordinate frames don't match.

### 3.1 Rsync the .db to the laptop (so it can regenerate map.json)

```bash
# On the laptop:
rsync -avh pi1@172.20.10.9:/home/pi1/maps/<DB_NAME>.db ~/maps/
```

(Use the actual `<DB_NAME>` you saw in T1's "Saving database" line.)

### 3.2 Regenerate `interface/map.json`

```bash
# On the laptop:
cd ~/swerve_transport_project/interface
python3 db_to_map_json.py --db ~/maps/<DB_NAME>.db --out map.json
```

Healthy result: a new `map.json` of 2-50 MB. The script prints how many
points and what bounds it extracted.

### 3.3 (Optional) Verify the .db hashes match

```bash
md5sum ~/maps/<DB_NAME>.db
ssh pi1@172.20.10.9 "md5sum /home/pi1/maps/<DB_NAME>.db"
```

The two hashes must be identical. If they're not, the rsync didn't
finish — re-run it.

---

## Phase 4 — Localization

### 4.1 T1 — Start the localization launch on the Pi

In T1 (still SSH'd into the Pi):

```bash
# (env vars from 2.1 are still set — no need to re-source unless you've
#  closed and reopened this terminal)
ros2 launch swerve_bringup rtabmap_localization.launch.py \
    robot_id:=tb3_0 \
    db_path:=$HOME/maps/<DB_NAME>.db \
    cam_x:=0.128 cam_y:=0.000 cam_z:=-0.0175
```

After ~25 seconds you should see in the log:

```
rtabmap: Localization mode (Mem/IncrementalMemory=false)
Setting RTAB-Map parameter "RGBD/MaxOdomCacheSize"="0" (rosparam)   ← ← ← KEY
Setting RTAB-Map parameter "Vis/MinInliers"="10" (rosparam)         ← ← ← KEY
[WARN] Transformed map accordingly to last localization pose saved in database (RGBD/OptimizeFromGraphEnd=true)! nearest id = NN of last pose = xyz=...
```

If you do **not** see the two "Setting RTAB-Map parameter ..." lines
above with values `0` and `10`, the YAML didn't get installed. Go to
[Troubleshooting → YAML didn't apply](#yaml-not-applied).

### 4.2 T2 — Start the Flask GUI server

```bash
cd ~/swerve_transport_project/interface
python3 server.py --map map.json --port 5002
```

Expected: `Map loaded: NN points`, `Running on http://0.0.0.0:5002`.

### 4.3 T3 — Start the ROS pose bridge

```bash
unset FASTRTPS_DEFAULT_PROFILES_FILE
export ROS_DOMAIN_ID=30
source /opt/ros/humble/setup.bash
source ~/swerve_transport_project/ros2_ws/install/setup.bash

cd ~/swerve_transport_project/interface
python3 ros_pose_bridge.py --ros-args -p robot_id:=tb3_0
```

Expected:
```
ros_pose_bridge: robot_id=tb3_0 -> http://localhost:5002/pose @ 10.0 Hz
```

The line `[WARN] TF map -> tb3_0_base_link not available` may print
every 5 sec until the next step — that's normal.

### 4.4 T5 — (Recommended) Start loc_doctor on the Pi

```bash
ssh pi1@172.20.10.9
unset FASTRTPS_DEFAULT_PROFILES_FILE
export ROS_DOMAIN_ID=30
source /opt/ros/humble/setup.bash
source ~/swerve_transport_project/ros2_ws/install/setup.bash

python3 ~/swerve_transport_project/interface/loc_doctor.py \
    --ros-args -p robot_id:=tb3_0
```

This prints one diagnostic block per second covering camera Hz, odom,
EKF, rtabmap rate, slam/pose count, and TF state. You'll watch this in
the next step.

### 4.5 Open the browser

Navigate to `http://localhost:5002`.

You'll see:
- The 3D point cloud rendered from `map.json`
- A red status pill in the top-right: `LOC: SEARCHING N.Ns` (red is
  expected at this point — RTAB-Map hasn't been told where the robot
  is yet)
- **No cyan cone visible** (because no confident pose yet)

### 4.6 ★ THE CRITICAL STEP — Click 📍 Set Initial Pose

This is what makes everything light up.

1. **Physically place the robot somewhere on the mapped area.** A spot
   you actually drove through during Phase 2. The simplest choice is
   the **same start position** you began the mapping drive at — that's
   the (0, 0) point in the map's coordinate frame, easy to specify in
   the GUI. Note the robot's real heading too (or just put it pointing
   along the same axis you started with).
2. **Click the 📍 Set Initial Pose button** (top-right of the GUI).
   The cursor turns into a crosshair and a status banner appears.
3. **Click on the floor in the GUI** at the spot the robot is
   physically. The 3D point cloud's coordinate frame matches the .db's
   (because Phase 3.2 made sure they do), so you can identify the spot
   visually.
4. **Move the mouse** — you'll see a yaw-arrow rotate. Set it to match
   the robot's real heading.
5. **Click again to confirm.**

What happens behind the scenes (visible in T3 and the rtabmap log):
```
T3 (bridge):       Published initial pose: x=0.45 y=-0.20 yaw=15deg, seq=1
T1 (rtabmap):      initialpose received: xyz=0.45,-0.20,0 rpy=...
T1 (rtabmap):      Transformed map accordingly to last localization pose saved
                   in database! nearest id = NN of last pose = xyz=...
```

Within 1-2 seconds, in the GUI:
- The pill turns **green**: `LOC: LIVE x.xx,y.yy`
- A **cyan cone marker appears** at the position you clicked, pointing
  in the direction you set
- T5 (loc_doctor) shows σ values like `σ_x=0.013m σ_y=0.013m
  σ_yaw=2.47°` — that's the real visual match confidence

If the σ values shown by loc_doctor are **`σ_x=99.995m`**, the YAML
patch didn't apply — see [Troubleshooting → YAML didn't apply](#yaml-not-applied).

### 4.7 Drive the robot — verify the cone follows

Reuse **T4** (the teleop terminal you used for the mapping drive). If
you closed it, restart with:

```bash
unset FASTRTPS_DEFAULT_PROFILES_FILE
export ROS_DOMAIN_ID=30
source /opt/ros/humble/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
    --ros-args -r cmd_vel:=/tb3_0/cmd_vel
```

Drive a small motion (say, tap `i` once to creep forward ~0.1 m, or
`j`/`l` to pivot a few degrees in place). The cyan cone in the GUI
should track the robot. The pill stays green; loc_doctor's σ values
should stay below ~50 mm during slow motion.

**That's the success criterion.** You now have working
mapping → localization with a confident, GUI-visible robot pose.

---

## CLI fallback if the browser is unavailable

If you can't open a browser (running fully headless, debugging the
GUI, etc.), publish the `/initialpose` hint directly:

```bash
# On the laptop or Pi (anywhere with ROS sourced):
python3 ~/swerve_transport_project/scripts/inject_initialpose.py \
    --ros-args -p x:=0.0 -p y:=0.0 -p yaw_deg:=0.0
```

Adjust `x`, `y`, `yaw_deg` to match the robot's real pose in the map's
coordinate frame. The defaults (0, 0, 0) work if you placed the robot
at the start of the box drive.

---

## Stopping the run

In reverse order of startup:
1. T4 (teleop): press spacebar/`k` to stop the robot, then Ctrl-C to
   exit teleop_twist_keyboard
2. T5 (loc_doctor): Ctrl-C — prints session summary
3. T3 (bridge): Ctrl-C
4. T2 (server): Ctrl-C
5. T1 (rtabmap launch): **Ctrl-C ONCE only.** Wait ~5 sec for
   `Saving database... done!`. Hitting Ctrl-C twice kills before the
   `.db` is flushed and you lose any localization-mode statistics.

---

## Troubleshooting

### <a name="yaml-not-applied"></a>YAML changes didn't apply (σ stays at 99.995 m)

Symptoms: loc_doctor shows `σ_x=99.995m`, the GUI cone is anchored to
wheel-odom dead-reckoning instead of visual matches, and T1's startup
log does NOT contain
`Setting RTAB-Map parameter "RGBD/MaxOdomCacheSize"="0"`.

Cause: the YAML lives in the source tree but `colcon build` never
installed it into the workspace's `share/` directory.

Fix:
```bash
ssh pi1@172.20.10.9 "cd ~/swerve_transport_project/ros2_ws && \
    colcon build --symlink-install --packages-select swerve_bringup && \
    cat install/swerve_bringup/share/swerve_bringup/config/rtabmap_localization.yaml \
        | grep -E 'MinInliers|MaxOdomCache'"
```

The `cat` should print:
```
Vis/MinInliers: "10"
RGBD/MaxOdomCacheSize: "0"
```

Then restart Phase 4.1.

### <a name="tiny-db"></a>The .db is < 5 MB / RTAB-Map didn't add nodes

Look in T1 for:

- `[WARN] Did not receive data since 5 seconds!` → camera or odom isn't
  reaching rtabmap. Run loc_doctor (Phase 4.4) — it'll point at the
  failing topic.
- `TF_OLD_DATA ignoring data from the past for frame tb3_0_odom` → Pi
  clock is wrong. Re-do Phase 1.1.
- `Insufficient permissions to communicate with X_LINK_UNBOOTED device`
  → see next entry.

### <a name="oak-perm"></a>OAK-D won't boot (`Insufficient permissions`)

Add the standard depthai udev rule on the Pi (one-time, persistent):

```bash
ssh pi1@172.20.10.9 'echo raspberry | sudo -S bash -c "cat > /etc/udev/rules.d/80-movidius.rules <<EOF
SUBSYSTEM==\"usb\", ATTRS{idVendor}==\"03e7\", MODE=\"0666\"
EOF
udevadm control --reload-rules && udevadm trigger"'
```

For the *current* USB plug (no replug needed), also chmod the live
device file:

```bash
ssh pi1@172.20.10.9 'DEV=$(lsusb | awk "/Movidius/ {print \"/dev/bus/usb/\" \$2 \"/\" substr(\$4, 1, length(\$4)-1)}"); echo raspberry | sudo -S chmod 666 "$DEV"; ls -la "$DEV"'
```

Then re-launch.

### Pill turns green but cone doesn't move when robot moves

Check loc_doctor's TF section. Two cases:

| Symptom | Meaning | Fix |
|---|---|---|
| `TF map → tb3_0_odom` is `[IDENTITY]` | RTAB-Map publishing the chain but never corrected odom. Visual match weak. | Re-click 📍 Set Initial Pose at the robot's actual location |
| `TF map → tb3_0_base_link` updates but `tb3_0_odom → tb3_0_base_link` stays still | OpenCR not getting cmd_vel | Check that teleop is publishing to `/tb3_0/cmd_vel` (note the namespace prefix) |

### Pill stays `LOC: SEARCHING` after 30 sec + Set Initial Pose hint

Open the database in `rtabmap-databaseViewer` on the laptop:

```bash
rtabmap-databaseViewer ~/maps/<DB_NAME>.db
```

In the Graph View, look at total node count vs loop closure count.
Healthy small-room map: 50-150 nodes, 5-20 loop closures. If you have
< 10 nodes or 0 loop closures, the mapping run failed — re-do Phase 2
with the four 360° pivots.

If the .db looks fine but localization still fails: lighting may have
changed since mapping. Re-map under the current lighting.

### Topics not visible from the laptop (hotspot / cross-network)

If the laptop can SSH to the Pi but `ros2 topic list` shows no
`/tb3_0/*` topics, your FastDDS peer file points at IPs that don't
match the current network.

```bash
unset FASTRTPS_DEFAULT_PROFILES_FILE
ros2 daemon stop && ros2 daemon start
ros2 topic list
```

Default multicast (`224.0.0.1:7400`) handles same-hotspot discovery
fine. The peer file is only needed for fixed LAN setups. **All terminal
commands in this runbook already include `unset FASTRTPS_DEFAULT_PROFILES_FILE`
in the env block** — make sure you actually pasted the whole block.

---

## What "working" actually means in this branch

To be precise about what you're getting:

✅ **Localization works** at any spot on the mapped trajectory **after
clicking Set Initial Pose** at that spot. σ collapses to 4-35 mm
within 1-2 frames; cone tracks robot.

✅ **The GUI is useful** — you click on the floor where the robot is,
and the robot localizes there.

⚠️ **Global re-localization is unreliable** in low-texture / small-map
environments. Without the Set Initial Pose hint, RTAB-Map may pick the
wrong keyframe (especially for closed-loop drives where start ≈ end).
Always seed with the GUI button before expecting confidence.

⚠️ **Loop closures are mostly rejected** during mapping (`0 inliers`
out of `30-50 matches`) due to the white-walls vocabulary fragmentation
issue. The map still works for localization-with-hints, but the graph
isn't optimization-clean. Long-term fix: calibrate the camera mount TF
on each robot, fix the swerve-pivot odometry quirk, and consider
depth-to-laser + Slam Toolbox per `HANDOFF_TO_TAN.md`.

---

## Files in this branch

| Path | Purpose |
|---|---|
| `MAP_TO_LOCALIZE.md` | This runbook |
| `ros2_ws/src/swerve_bringup/config/rtabmap_localization.yaml` | Patched: `Vis/MinInliers=10`, `RGBD/MaxOdomCacheSize=0`, `Rtabmap/LoopThr=0.08` |
| `interface/loc_doctor.py` | Bug fix: numpy covariance truthiness |
| `scripts/drive_box.py` | **Optional** automated mapping drive — see [§ below](#auto-box). Phase 2 uses teleop by default. |
| `scripts/inject_initialpose.py` | CLI fallback for Set Initial Pose |
| `interface/index.html`, `server.py`, `ros_pose_bridge.py` | Unchanged from main — used as-is |

---

## <a name="auto-box"></a>Optional — automated box drive (alternative to teleop)

If you want a **repeatable, hands-off** mapping drive — for A/B testing
YAML changes against the same trajectory, or for unattended demos — use
`scripts/drive_box.py` instead of teleop in Phase 2.2. The driving
shape is fixed:

```
forward 0.5 m  ->  pivot 360° CCW  ->  strafe right 0.5 m  ->  pivot 360° CCW
back    0.5 m  ->  pivot 360° CCW  ->  strafe left  0.5 m  ->  pivot 360° CCW
```

at 0.10 m/s linear and 0.20 rad/s angular. Total ≈ 2 min 35 sec.

To use it:

1. Place the robot in **a clear ~0.7 × 0.7 m floor area** (the box
   pattern stays inside that envelope, with some drift margin).
2. Skip Phase 2.2 (teleop). Instead, in T4 on the laptop:

   ```bash
   unset FASTRTPS_DEFAULT_PROFILES_FILE
   export ROS_DOMAIN_ID=30
   source /opt/ros/humble/setup.bash
   source ~/swerve_transport_project/ros2_ws/install/setup.bash

   python3 ~/swerve_transport_project/scripts/drive_box.py \
       --ros-args -p robot_id:=tb3_0
   ```

3. Watch the `[N/8]` segment progress in T4, and the keyframe count
   growing in T1 just like with teleop.
4. When the script finishes (you'll see `Done. Total elapsed: NNN.Ns`),
   continue from Phase 2.4 (stop mapping cleanly).

> **Why this is in the branch but not the default.** Real production
> maps benefit from human judgment — drive a longer route, pause-and-
> spin where there's interesting geometry, repeat passes through hard
> spots. The box drive is only 0.5 m on a side, which is enough to
> validate the pipeline but not enough for navigation. Use teleop for
> real maps; use `drive_box.py` only when you want a known-quantity
> trajectory you can re-run identically across config changes.
