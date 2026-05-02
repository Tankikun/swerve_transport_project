# Mapping Run — Laptop Side (Self-Contained)

Builds a 3D map (`.db` file) of the room by driving the robot
through it. Map is **saved on the laptop**. Pi just streams sensors.

Everything is done from your laptop — the Pi side is driven via
SSH from one of the laptop terminals. **You do not need to open
any other guide.**

> **Architecture.** Laptop runs `rtabmap_slam` (heavy CPU). Pi runs
> only sensors (camera, TF, wheel odometry, EKF). Image and odom
> data flow over LAN to the laptop. Output `.db` lands on the
> laptop. Pi 4 stays cool (~60 °C).

You'll use **4 terminals on the laptop**:

| Terminal | Role |
|---|---|
| T1 | SSH'd into pi2 — runs the sensor launch |
| T2 | Native laptop — runs `rtabmap_slam` |
| T3 | Native laptop — runs teleop |
| T4 | Native laptop — (optional) monitoring |

> **Two non-obvious things you MUST do** before mapping or this all
> falls apart:
>
> 1. **Sync the laptop and pi2 clocks** (Step 2). RTAB-Map drops
>    every frame if the clocks disagree by more than ~1 s.
> 2. **Actually drive the robot** during mapping (Step 9). RTAB-Map
>    only links keyframes when it sees motion. A stationary "flow
>    test" produces a `.db` file that looks fine on disk but has
>    zero usable links inside.

---

## Step 0. Pre-flight

Confirm the room is mapping-friendly:
- Lights on (visual SLAM needs features)
- No big mirrors / glass walls
- Move chairs / people that won't be present during the actual
  transport job

Plan a driving path that:
- Covers every area you'll later transport through
- Returns to the start point so RTAB-Map can close the loop
- Avoids sharp 90° turns at full speed

Tape mark the floor for the robot's start position.

Make sure the robot's OAK-D camera mount measurements are correct
in the launch args you'll use in Step 3:

| arg | meaning | tb3_1 measured value |
|---|---|---|
| `cam_x` | forward offset from base_link [m] | `0.128` |
| `cam_y` | sideways offset (left positive) | `0.000` |
| `cam_z` | vertical offset (up positive) | `-0.0175` (camera below base_link) |

---

## Step 1. Open Terminal 1, SSH into pi2

```bash
ssh pi2@192.168.1.102
```

You should now be at `pi2@ubuntu:~$`. Don't run anything yet — go
to Step 2 first.

---

## Step 2. Sync clocks (CRITICAL)

In a SEPARATE laptop terminal (so you don't lose your pi2 SSH from
T1), check the clock skew:

```bash
echo "laptop: $(date)"
ssh pi2@192.168.1.102 "echo \"pi2:    \$(date)\""
```

If the times differ by more than ~1 second, sync pi2 to the
laptop's clock:

```bash
ssh -t pi2@192.168.1.102 "sudo date -u -s '$(date -u +'%Y-%m-%d %H:%M:%S')'"
```

Verify:
```bash
echo "laptop: $(date)" && ssh pi2@192.168.1.102 "echo \"pi2: \$(date)\""
```
Two outputs should be within 1 s.

> Why this matters: RTAB-Map matches camera frames against TF/odom
> by timestamp. If pi2's clock is hours behind the laptop's clock
> (the Pi has no working NTP), every frame's timestamp falls
> outside the laptop's TF buffer window, and rtabmap silently
> drops them all. This will manifest later as
> "TF requested time X but earliest data is Y" warnings and a
> map with zero keyframes — fixing it after launch is harder than
> just doing it now.

You can close this throwaway clock-sync terminal now (or reuse it
as T4 later).

---

## Step 3. T1 — start the Pi sensor launch

Back in **Terminal 1** (still SSH'd into pi2):

```bash
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/pi2/fastdds_peers.xml
export ROS_DOMAIN_ID=30
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch swerve_bringup rtabmap_pi_sensors.launch.py \
    robot_id:=tb3_1 \
    cam_x:=0.128  cam_y:=0.000  cam_z:=-0.0175
```

**Wait for these lines** (in roughly this order, ~10–15 s total):
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

**Leave Terminal 1 alone for the rest of the session.** This
terminal stays SSH'd to pi2; the sensor launch keeps publishing
to the network.

If anything is red / missing, scroll up — fix before continuing.

---

## Step 4. Open Terminal 2, set env, verify topics from pi2 are visible

In a NEW laptop terminal (do NOT SSH this one):

```bash
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/toodmuk/fastdds_peers.xml
export ROS_DOMAIN_ID=30
source /opt/ros/humble/setup.bash
source ~/swerve_transport_project/ros2_ws/install/setup.bash
```

> Workspace path is `ros2_ws/install`, **not** `install/`. The
> legacy `~/swerve_transport_project/install/` lacks the rtabmap
> launches.

Sanity check:
```bash
echo "DOMAIN=$ROS_DOMAIN_ID  PROFILE=$FASTRTPS_DEFAULT_PROFILES_FILE"
```
Expected: `DOMAIN=30  PROFILE=/home/toodmuk/fastdds_peers.xml`

Now check pi2's topics are visible:
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

**If empty**, jump to "Discovery troubleshooting" at the bottom,
fix it, then come back.

---

## Step 5. Verify topic rates are healthy

Still in T2:
```bash
ros2 topic hz /tb3_1/camera/rgb/image_raw    # > 1 Hz expected
ros2 topic hz /tb3_1/ekf/odom                # > 5 Hz expected
```
Press Ctrl+C after a few seconds of each.

If camera rate is < 1 Hz the LAN is saturated. In T1, Ctrl+C and
relaunch the sensor command from Step 3 with `fps:=10` appended.

---

## Step 6. Verify odom CHANGES when the robot moves (the "no frozen map" check)

This is the check that catches a previously-painful mistake: if
odom isn't actually changing during teleop, RTAB-Map will write a
.db full of keyframes that all have identical pose, and the map
will be useless.

In T2, start streaming odom positions:
```bash
ros2 topic echo /tb3_1/ekf/odom --field pose.pose.position
```

In a *throwaway* second terminal (or temporarily in T4 if you've
opened it), nudge the robot 10 cm forward via teleop:
```bash
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/toodmuk/fastdds_peers.xml
export ROS_DOMAIN_ID=30
source /opt/ros/humble/setup.bash
source ~/swerve_transport_project/ros2_ws/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
    --ros-args -r cmd_vel:=/tb3_1/cmd_vel
```
Tap `i` once briefly. The values streaming in T2 must change.

If they DON'T change:
- Confirm cmd_vel is reaching the robot — should physically move
- Check T1 — `conveyor_base_node` should print "POSE x y …" lines
- If still frozen, ekf_node may not be subscribing — restart T1's
  launch (Ctrl+C + re-run Step 3)

When odom is confirmed to change with motion, **Ctrl+C T2's echo**
and the temporary teleop. Drive the robot back to the tape mark.

---

## Step 7. T2 — launch rtabmap (the SLAM brain)

Still in T2:
```bash
mkdir -p ~/maps

ros2 launch swerve_bringup rtabmap_laptop_mapping.launch.py \
    robot_id:=tb3_1 \
    db_path:=~/maps/tb3_1_room.db
```

**Wait for `rtabmap started`** in the log before doing anything
else. The .db is created at `~/maps/tb3_1_room.db`.

**Leave T2 visible** — RTAB-Map prints status updates as you
drive. Watch for:
- `Added keyframe NN` — keyframes accumulating (good)
- `loop closure detected` — closing the loop (great)
- `(local map=N, WM=N)` — memory growing as you explore

---

## Step 8. T3 — start teleop

In a new laptop terminal:

```bash
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/toodmuk/fastdds_peers.xml
export ROS_DOMAIN_ID=30
source /opt/ros/humble/setup.bash
source ~/swerve_transport_project/ros2_ws/install/setup.bash

ros2 run teleop_twist_keyboard teleop_twist_keyboard \
    --ros-args -r cmd_vel:=/tb3_1/cmd_vel
```

> If "Package not found": `sudo apt install -y ros-humble-teleop-twist-keyboard`

Key bindings printed by the node:
```
Moving around:    u    i    o
                  j    k    l
                  m    ,    .
```
- `i` / `,` — forward / backward
- `j` / `l` — rotate
- `u` / `o` / `m` / `.` — diagonals (holonomic strafe)
- `space` or `k` — stop
- `q` / `z` — speed up / down

---

## Step 9. Drive the robot through the room (DO THIS — DON'T just sit still)

Driving rules — these directly affect map quality:

- **Slow.** ~0.1 m/s. Visual feature tracker drops above ~0.15 m/s.
- **Smooth.** Long key holds, not staccato taps.
- **Stop and rotate.** Occasionally pivot in place to scan the
  surrounding walls from one position.
- **Re-visit.** Drive past the same spot from different angles —
  this is how RTAB-Map builds confidence in the map.
- **Close the loop.** End the run by driving back to within ~30 cm
  of the start position. Single most important step for map
  quality.

While driving, glance at T2 — keyframe count must keep increasing.
**5–10 minutes of actual driving is the minimum** for a small room.

---

## Step 10. (Optional) T4 — monitor progress

In a fourth laptop terminal (same env exports as Step 4):

```bash
# Liveness check
ros2 topic hz /tb3_1/rtabmap/cloud_map  # should be > 0.5 Hz
```

(Note: the `/tb3_1/rtabmap/*` topics may not appear in `topic list`
even when rtabmap is processing — `topic hz` will still work if
they're publishing. The `.db` file growing on disk is the most
reliable progress signal.)

---

## Step 11. End the run, in this exact order

1. **T3 (teleop)** — Ctrl+C
2. **T2 (rtabmap)** — Ctrl+C. Wait ~5 s for the .db to flush.
3. **T1 (pi2 sensors)** — Ctrl+C
4. **T4 (monitor, if open)** — Ctrl+C

---

## Step 12. Verify the .db is real

In any laptop terminal:
```bash
ls -lh ~/maps/
```
Expected: `tb3_1_room.db` of 10–200 MB.

For a deeper check that the map has actual content (links, not
just disconnected keyframes):
```bash
python3 - <<'PY'
import sqlite3
c = sqlite3.connect('/home/toodmuk/maps/tb3_1_room.db').cursor()
n_nodes = c.execute('SELECT COUNT(*) FROM Node').fetchone()[0]
n_links = c.execute('SELECT COUNT(*) FROM Link').fetchone()[0]
print(f'keyframes: {n_nodes}')
print(f'links:     {n_links}')
print(f'verdict:   {"USABLE MAP" if n_links > 0 else "FROZEN — robot did not move during mapping; redo from Step 7"}')
PY
```

You want **non-zero links**. Zero links means motion wasn't
captured (see verdict above).

---

## Step 13. (Optional) Inspect the map visually

Install the GUI viewer (one-time):
```bash
sudo apt install -y ros-humble-rtabmap-viz
```

Open the map:
```bash
rtabmap-databaseViewer ~/maps/tb3_1_room.db
```

Look for:
- Continuous trajectory (not many disconnected pieces)
- Loop-closure links shown as coloured edges in the graph
- Point cloud that resembles the actual room

---

## Step 14. (Later) Switch to localization

Once you have a `.db` you're happy with, switch the runtime stack
from mapping mode to localization mode.

In T1 (still SSH'd to pi2): the sensor launch from Step 3 is
exactly what's needed for localization too. Restart it if you
stopped it.

In T2 (after Step 11 stopped the mapping launch):
```bash
ros2 launch swerve_bringup rtabmap_laptop_localization.launch.py \
    robot_id:=tb3_1 \
    db_path:=~/maps/tb3_1_room.db
```

Verify in T4:
```bash
ros2 topic hz /tb3_1/slam/pose      # 1–3 Hz once localized
ros2 topic echo /tb3_1/ekf/odom     # smooth, drift-corrected
```

**Initial localization takes 5–30 s** — RTAB-Map scans the entire
stored map for a visual match. Drop the robot in a part of the
room with reasonable visual variety — avoid blank walls.

---

## Multi-robot — every robot must load the SAME .db

When both robots run their localization launches and you want to
use the laplacian consensus correction (`enable_consensus:=true`),
every robot's launch MUST point `db_path` at the SAME .db file.
Different .db files mean different `map` frames, and any
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
doesn't have the new launches. Re-source the right path:
```bash
source ~/swerve_transport_project/ros2_ws/install/setup.bash
```

### `MsgConversion.cpp:2010 ... Lookup would require extrapolation into the past`
Clock skew between laptop and pi2. Re-do Step 2.

### `MsgConversion.cpp:2010 ... TF has two or more unconnected trees`
TF chain is broken (`tb3_1_odom` and `tb3_1_base_link` not
connected). All current code uses per-robot prefixed frames, so
this should not appear on a fresh build. If it does, both pi2 and
laptop need a `colcon build --packages-select swerve_formation`
followed by restarting all launches.

### `rtabmap started` never appears in T2
- Check T1 — is the Pi sensor launch actually running and in the
  "leave alone" state from Step 3?
- Re-do Step 5 — if any rate is 0 Hz, rtabmap will sit waiting
  for topic sync and never declare ready
- Try `--ros-args --log-level debug` on the Step 7 launch line

### Map verdict says "FROZEN — robot did not move"
You ran the mapping but didn't actually drive the robot. Redo
Step 7 onward and physically teleop the robot through the room.
A stationary mapping run is always pointless — RTAB-Map needs
motion to link keyframes.

### Map looks fragmented / no loop closures
- Drive slower next time (Step 9 rules)
- Re-map with a tighter physical loop close
- Edit `rtabmap_laptop_mapping.launch.py` and lower
  `RGBD/AngularUpdate` and `RGBD/LinearUpdate` to `0.005`
  (more keyframes captured per unit motion)

### Localization keeps "jumping" after a few seconds
Tan's `/ekf/odom` remap creates a small feedback loop (rtabmap →
ekf → rtabmap). If you see jumpy poses, edit
`rtabmap_laptop_localization.launch.py` and change
`('odom', f'/{robot_id}/ekf/odom')` back to
`('odom', f'/{robot_id}/odom')`.

### Camera rate drops or rtabmap complains about sync
Network is the bottleneck. In T1, Ctrl+C and relaunch the Pi
sensor command from Step 3 with `fps:=10` appended.

### Discovery troubleshooting (Step 4 was empty)

Empty topic list usually means the FastDDS peers file has the
wrong laptop IP.

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
your numbers — do NOT auto-script with `LAN_IP=$(awk …)`, an
over-broad regex can wipe out the pi entries):
```bash
sed -i "s/192\.168\.1\.OLD/192.168.1.NEW/g" ~/fastdds_peers.xml
ssh pi2@192.168.1.102 "sed -i 's/192\\.168\\.1\\.OLD/192.168.1.NEW/g' ~/fastdds_peers.xml"
```

**Verify the file still has all 4 robot peer entries** (catches
accidental wipeouts):
```bash
grep -E '192\.168\.1\.(101|102)' ~/fastdds_peers.xml | wc -l
```
Expected: `4`. If less, restore from pi2:
`scp pi2@192.168.1.102:~/fastdds_peers.xml ~/`
then re-do the laptop-side patch.

**Restart pi2's sensor launch** (it caches the peers file at
launch time): switch to T1, Ctrl+C, re-run the Step 3 command.

**Restart the laptop daemon and recheck**:
```bash
ros2 daemon stop ; sleep 1
ros2 topic list | grep -E '/tb3_1/(camera|odom|ekf)'
```
