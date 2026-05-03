# Mapping Run — From the Laptop

End-to-end procedure for **building an RTAB-Map `.db` of the room**.
Everything is driven from your laptop; the Pi is reached over SSH.
No file in this repo is required besides this one.

> **Architecture.** Laptop runs the heavy `rtabmap_slam`. Pi runs
> only sensors (camera, TF, wheel odometry, EKF). Map (`.db`)
> lives on the laptop. Pi 4 stays cool (~60 °C).

You will use **4 terminals on the laptop**. They reach the Pi via
SSH where needed. You never need to physically touch the Pi.

| Terminal | Role |
|---|---|
| T1 | SSH'd into pi2 — runs Pi sensor stack |
| T2 | Native laptop — runs `rtabmap_slam` |
| T3 | Native laptop — runs teleop |
| T4 | Native laptop — (optional) monitoring |

> **Two failures cost the previous session a lot of time. Both are
> fixed by following this guide:**
>
> 1. **Pi clock drift.** `systemd-timesyncd` on the Pi quietly
>    rolled the clock back ~14 h every few minutes (couldn't reach
>    `ntp.ubuntu.com` through the lab proxy and fell back to a
>    saved time). RTAB-Map dropped every frame because timestamps
>    were nonsense. Step 2 disables timesyncd permanently and pins
>    the Pi's clock.
> 2. **Stationary "flow tests" produce zero links.** RTAB-Map only
>    creates links between keyframes when it sees the robot move.
>    A test where you just sat the robot down and watched the log
>    will produce a `.db` of the right size with zero usable links.
>    Step 9 pre-flights this; Step 12 verifies the result.

---

## 0. Operator-specific knobs

This guide assumes:
- The Pi at `192.168.1.102` is named `pi2` and you can `ssh pi2@…`
  (Earth's setup; add an SSH key if you haven't yet:
  `ssh-copy-id pi2@192.168.1.102`).
- The laptop's user home is `~`. The project is cloned at
  `~/swerve_transport_project/` with `ros2_ws/` inside it. The
  current operators (Seven, Tan, Run, Earth) all follow that
  layout — adjust `WORKSPACE` below if yours differs.

```
WORKSPACE=~/swerve_transport_project/ros2_ws
PEERS=~/fastdds_peers.xml
ROS_DOMAIN_ID=30
ROBOT_ID=tb3_1
```

`PEERS` (`~/fastdds_peers.xml`) MUST contain THREE classes of
addresses in `<initialPeersList>`:

1. **`127.0.0.1`** (loopback) — required so two laptop processes
   (rtabmap + the daemon, or rtabmap + a monitoring `ros2 topic`
   call) can discover each other. WITHOUT this, rtabmap runs but
   its output topics (`/tb3_1/rtabmap/info`, `cloud_map`,
   `localization_pose`, `mapData`) never appear in the laptop's
   `ros2 topic list`. The mapping itself still works; you just
   can't monitor live.
2. `192.168.1.101` (pi1) and `192.168.1.102` (pi2) — so the laptop
   discovers the robots' nodes.
3. The laptop's own current LAN IP — must appear in
   `<metatrafficUnicastLocatorList>` and `<defaultUnicastLocatorList>`,
   so the robots send discovery replies back to the laptop.

A correct laptop peers file looks like:
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
(Replace `192.168.1.114` with your laptop's actual LAN IP — see
"Discovery troubleshooting" if you need to find it.)

---

## 1. Pre-flight

Confirm the room is mapping-friendly:
- Lights on (visual SLAM needs features)
- No big mirrors / glass walls
- Move chairs / people that won't be present during the actual
  transport job

Plan a driving path:
- Cover every area you'll later transport through
- Return to the start point (so RTAB-Map can close the loop)
- Avoid sharp 90° turns at full speed

Tape mark the floor for the robot's start position.

Confirm camera mount values (these went into firmware once and
shouldn't change):

| arg | meaning | tb3_1 measured |
|---|---|---|
| `cam_x` | forward offset from base_link [m] | `0.128` |
| `cam_y` | sideways offset (+ left) | `0.000` |
| `cam_z` | vertical offset (+ up) | `-0.0175` |

---

## 2. Disable Pi clock drift (one-time, persists across reboots once done)

The Pi has no working NTP because the lab router has no internet,
and `systemd-timesyncd` was making the clock jump backwards every
few minutes. We disable it permanently and set the clock manually
to the laptop's clock.

```bash
date -u +'%Y-%m-%d %H:%M:%S'
```
Copy what this prints (e.g. `2026-05-03 09:03:23`).

Then run the next command, **pasting your copied date in place of
`PASTE_DATE`** (asks for the Pi's sudo password once):

```bash
ssh -t pi2@192.168.1.102 "sudo systemctl disable --now systemd-timesyncd && sudo date -u -s 'PASTE_DATE'"
```

> Why two-step: many terminals (e.g. ones with markdown auto-link)
> munge `$(date)` substitutions inside double-quoted SSH commands.
> Pasting the literal date sidesteps the issue.

Verify:
```bash
echo "laptop: $(date -u +%s)"
ssh pi2@192.168.1.102 "echo \"pi2:    \$(date -u +%s)\""
```
The two epoch numbers should be within ~5 s of each other.

> **You only need to redo Step 2 after a Pi reboot.** Without RTC
> hardware on the Pi 4, the clock resets at every boot. With
> timesyncd disabled (which Step 2 makes permanent), your manual
> setting is the only thing keeping the clock right until next
> boot.

---

## 3. T1 — start the Pi sensor stack

Open Terminal 1 on the laptop and SSH into pi2:

```bash
ssh pi2@192.168.1.102
```

Once you see `pi2@ubuntu:~$`, paste:

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
[static_transform_publisher-2] ... Spinning until stopped
                                  translation: (0.128, 0.000, -0.018)
                                  from 'tb3_1_base_link' to
                                  'tb3_1_oak_rgb_camera_optical_frame'
[conveyor_base_node-3] ... Serial /dev/ttyACM0 @ 115200 opened.
[conveyor_base_node-3] ... ConveyorBaseNode activated
[ekf_node-4]           ... EKF node ready for tb3_1
```

**Leave T1 alone** for the rest of the session. The launch keeps
publishing; do not Ctrl+C unless you intentionally restart.

If anything is red / missing, scroll up and resolve before
continuing.

---

## 4. T2 — open a laptop terminal, source env, verify Pi topics visible

In a NEW laptop terminal (not SSH):

```bash
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/$USER/fastdds_peers.xml
export ROS_DOMAIN_ID=30
source /opt/ros/humble/setup.bash
source ~/swerve_transport_project/ros2_ws/install/setup.bash
```

> Workspace path is `ros2_ws/install`, **not** `install/`. The
> legacy `~/swerve_transport_project/install/` does not contain the
> rtabmap launches and gives "file not found in share directory".

Sanity check the env:
```bash
echo "DOMAIN=$ROS_DOMAIN_ID  PROFILE=$FASTRTPS_DEFAULT_PROFILES_FILE"
```
Expected: `DOMAIN=30  PROFILE=/home/<you>/fastdds_peers.xml`

Now check Pi topics:
```bash
ros2 daemon stop ; sleep 2 ; ros2 daemon start ; sleep 6
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

If empty, jump to "Discovery troubleshooting" (bottom of file),
fix, then come back.

---

## 5. Verify topic rates and TF tree

Still in T2:
```bash
ros2 topic hz /tb3_1/camera/rgb/image_raw    # > 1 Hz expected
ros2 topic hz /tb3_1/ekf/odom                # > 2 Hz expected
```
Press Ctrl+C after a few seconds of each.

If RGB rate < 1 Hz, the LAN is saturated. Lower fps in T1's
launch (Step 3) by appending `fps:=10`.

Check the TF chain rtabmap will need:
```bash
ros2 run tf2_ros tf2_echo tb3_1_odom tb3_1_oak_rgb_camera_optical_frame
```
Expected: prints a `Translation:` and `Rotation:` (the static
camera mount). Press Ctrl+C.

If you see "Could not find a connection between … because they are
not part of the same tree", restart T1's launch (Step 3) — old
ROS processes that pre-date a code update may still be running.

---

## 6. Verify the Pi's clock didn't slip again

```bash
echo "laptop: $(date -u +%s)"
ssh pi2@192.168.1.102 "echo \"pi2:    \$(date -u +%s)\""
```
Difference should be < 5 s. If it's > 60 s, something restarted
the time service — re-do Step 2.

Also check the Pi's published timestamps match its wall clock:
```bash
echo "pi2 wall: $(ssh pi2@192.168.1.102 'date +%s')"
ros2 topic echo /tb3_1/ekf/odom --once 2>/dev/null \
    | grep "sec:" | head -1
```
The two numbers should be within a few seconds.

---

## 7. Verify odom CHANGES when the robot moves (catches stationary "flow tests")

This single check catches what burned ~30 minutes in the last
session. Do it BEFORE starting rtabmap.

In T2, start streaming odom positions:
```bash
ros2 topic echo /tb3_1/ekf/odom --field pose.pose.position
```

In a temporary 5th terminal (env exports + workspace source as in
Step 4), start a brief teleop:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
    --ros-args -r cmd_vel:=/tb3_1/cmd_vel
```
> If "Package not found": `sudo apt install -y ros-humble-teleop-twist-keyboard`

Tap `i` once briefly. Numbers in T2 must change. If they don't:
- Confirm the robot actually moved physically
- Check T1 — `conveyor_base_node` should print "OpenCR: …" lines
- Restart T1's sensor launch

When verified, **Ctrl+C T2's echo** and the temporary teleop.
Drive the robot back to the start tape mark.

---

## 8. T2 — launch rtabmap (the SLAM brain)

Same T2 (still env-set):
```bash
mkdir -p ~/maps

ros2 launch swerve_bringup rtabmap_laptop_mapping.launch.py \
    robot_id:=tb3_1 \
    db_path:=~/maps/tb3_1_room.db
```

Wait for `rtabmap started` in the log. The `.db` is created at
`~/maps/tb3_1_room.db`. Leave T2 visible — RTAB-Map prints status
updates as you drive. Glance for:
- `rtabmap (NN): … delay=X.X` — `X` should stay below ~10 s. If
  it climbs into the 1000s, the Pi's clock slipped (re-do Step 2).
- `(local map=N, WM=N)` — WM grows as you explore.
- `loop closure detected` (later) — re-visit recognition.

> The `Did not receive data since 5 seconds` warning may appear
> sporadically while you're stationary or before the first sync
> completes. It's only a problem if it's *constant*.

---

## 9. T3 — start teleop

In a new laptop terminal (env exports as in Step 4):
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
    --ros-args -r cmd_vel:=/tb3_1/cmd_vel
```

Key bindings (printed by the node):
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

## 10. Drive the room (this is the part that determines map quality)

5–10 minutes is the minimum for a small room. While driving:

- **Slow.** ~0.1 m/s. Visual feature tracker drops above ~0.15 m/s.
- **Smooth.** Long key holds, not staccato taps.
- **Stop and rotate.** Occasionally pivot in place to scan walls
  from one position.
- **Re-visit.** Drive past the same spot from different angles —
  this is how RTAB-Map builds a stitched, self-consistent map.
- **Close the loop.** End the run by driving back to within
  ~30 cm of the start. Single most important step for map quality.

While driving, glance at T2 — you should see `Added keyframe NN`
lines accumulating, and ideally `loop closure detected` as you
re-visit areas.

---

## 11. (Optional) T4 — monitor progress

```bash
# Same env exports as Step 4 — in a new terminal
ros2 topic hz /tb3_1/rtabmap/cloud_map  # > 0.5 Hz when active
```

> Some `/tb3_1/rtabmap/*` topics may not appear in `topic list`
> until the first keyframe is processed and the laptop daemon has
> re-discovered them. The most reliable progress signal is
> the `.db` file's size growing on disk:
> ```bash
> watch -n 5 ls -lh ~/maps/tb3_1_room.db
> ```

---

## 12. End the run, in this exact order

1. **T3 (teleop)** — Ctrl+C
2. **T2 (rtabmap)** — Ctrl+C. Wait ~5 s for the `.db` to flush.
3. **T1 (Pi sensors)** — Ctrl+C
4. **T4 (monitor, if open)** — Ctrl+C

---

## 13. Verify the .db is real (USABLE MAP, not FROZEN)

```bash
ls -lh ~/maps/tb3_1_room.db
```
Expected: 10–200 MB. If smaller, mapping wasn't capturing.

For a real verdict (catches "stationary mapping" failures):
```bash
python3 - <<'PY'
import sqlite3
DB = '/home/' + __import__('os').environ['USER'] + '/maps/tb3_1_room.db'
c = sqlite3.connect(DB).cursor()
n = c.execute('SELECT COUNT(*) FROM Node').fetchone()[0]
l = c.execute('SELECT COUNT(*) FROM Link').fetchone()[0]
print(f'keyframes: {n}')
print(f'links:     {l}')
print('verdict:   ' +
      ('USABLE MAP' if l > 0 else
       'FROZEN — robot did not move during mapping; redo from Step 8'))
PY
```

You want **non-zero links**. Zero links means motion wasn't
captured.

---

## 14. (Optional) Inspect the map visually

One-time install:
```bash
sudo apt install -y ros-humble-rtabmap-viz
```

Open the map:
```bash
rtabmap-databaseViewer ~/maps/tb3_1_room.db
```

What to look for:
- Continuous trajectory (not many disconnected pieces)
- Loop-closure links shown as colored edges in the graph
- Point cloud that resembles the actual room

---

## 15. Next step — runtime localization

Mapping is done. Now you have a `.db` you can use for runtime
localization. Follow `LOCALIZATION_RUN_LAPTOP.md` — separate guide
because the procedure has its own verification flow ("how do I
know the robot is actually localized?") that doesn't belong in a
mapping document.

---

## Multi-robot — all robots must load the SAME .db

When both robots run their localization launches and you intend to
use the laplacian consensus correction (`enable_consensus:=true`),
every robot's launch MUST point `db_path` at the SAME database
file. Different `.db` files mean different `map` frames, and
inter-robot pose feedback would silently produce nonsense.

Distribute the .db (built on the laptop) to every Pi:
```bash
rsync ~/maps/tb3_1_room.db pi1@192.168.1.101:~/maps/room.db
rsync ~/maps/tb3_1_room.db pi2@192.168.1.102:~/maps/room.db
```
Then point every robot's localization launch at `~/maps/room.db`.

Easiest convention: drop the per-robot suffix. Use plain `room.db`
on every machine that runs a localization launch.

---

## Troubleshooting

### `file 'rtabmap_laptop_mapping.launch.py' was not found in the share directory`
You sourced the legacy `~/swerve_transport_project/install/`. Use:
```bash
source ~/swerve_transport_project/ros2_ws/install/setup.bash
```

### `MsgConversion.cpp ... Lookup would require extrapolation into the past`
Pi clock drift. Re-do Step 2. If it keeps happening within minutes,
verify timesyncd is actually disabled:
```bash
ssh pi2@192.168.1.102 "systemctl status systemd-timesyncd | head -3"
```
Should show `inactive (dead)`. If it shows `active`:
```bash
ssh -t pi2@192.168.1.102 "sudo systemctl disable --now systemd-timesyncd"
```

### `MsgConversion.cpp ... TF has two or more unconnected trees`
TF chain is broken. The current code uses per-robot prefixed
frames (`tb3_X_odom`, `tb3_X_base_link`); this error only appears
if old processes pre-date a code update. Restart T1's launch.

### `delay=` in rtabmap log is huge (1000s of seconds)
Same as the extrapolation issue — Pi clock slipped. Re-do Step 2,
then restart T1 (Step 3) so the sensor processes pick up the new
clock, then restart T2 (Step 8).

### rtabmap is processing (log shows `rtabmap (NN): …`) but `/tb3_1/rtabmap/*` topics don't appear in `ros2 topic list`
The laptop's `~/fastdds_peers.xml` doesn't include `127.0.0.1`
in `<initialPeersList>`. Without it, two laptop processes
(rtabmap + the cli daemon) can't discover each other via UDP
unicast — they only send discovery to the Pis. Mapping itself
still works (the camera frames flow Pi→laptop→rtabmap fine via
the Pi-routed peering), you just can't see rtabmap's outputs
locally. See the loopback line in the example peers file at the
top of this guide. After fixing, restart the rtabmap launch
(Step 8) and the daemon (`ros2 daemon stop ; sleep 2 ; ros2 daemon start`).

### `rtabmap started` never appears in T2
- Check T1 — is the Pi sensor launch still running and showing
  the IMU stream?
- Re-do Step 5 — if any rate is 0 Hz, rtabmap will sit waiting
  for sync and never declare ready
- Try `--ros-args --log-level debug` on the Step 8 launch line

### Map verdict says "FROZEN — robot did not move"
You ran the mapping but didn't actually drive. Redo Step 8 onward
and physically teleop the robot. A stationary mapping run is
always pointless — RTAB-Map needs motion to link keyframes.

### Map looks fragmented / no loop closures
- Drive slower next time
- Re-map with a tighter physical loop close
- Edit `rtabmap_laptop_mapping.launch.py` and lower
  `RGBD/AngularUpdate` and `RGBD/LinearUpdate` to `0.005`
  (more keyframes captured per unit motion)

### Localization keeps "jumping" after a few seconds
The `/ekf/odom` remap creates a small feedback loop (rtabmap →
ekf → rtabmap). If you see jumpy poses, edit
`rtabmap_laptop_localization.launch.py` and change
`('odom', f'/{robot_id}/ekf/odom')` back to
`('odom', f'/{robot_id}/odom')`.

### Camera rate drops or rtabmap complains about sync
Network is the bottleneck. In T1 (Step 3), Ctrl+C and relaunch
with `fps:=10` appended.

### Discovery troubleshooting (Step 4 was empty)

Empty topic list usually means the FastDDS peers file has the
wrong laptop IP. The peers file lists every host that should be
reachable.

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
```bash
scp pi2@192.168.1.102:~/fastdds_peers.xml ~/
```
then re-do the laptop-side patch.

**Restart pi2's sensor launch** (it caches the peers file at
launch time): switch to T1, Ctrl+C, re-run Step 3.

**Restart the laptop daemon and recheck**:
```bash
ros2 daemon stop ; sleep 2 ; ros2 daemon start ; sleep 6
ros2 topic list | grep -E '/tb3_1/(camera|odom|ekf)'
```
