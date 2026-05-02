# Handoff: Two-Robot Formation Test Results

**Branch:** [`feature/two-robot-test-seven`](https://github.com/Tankikun/swerve_transport_project/tree/feature/two-robot-test-seven)
**Sessions:** 2026-05-01 (formation control) + 2026-05-02 (firmware encoder odom + IMU)
**Author:** Seven (with help from Claude)
**Status:** ✅ Working — formation orbit and translation verified on the floor with keyboard teleop. Both robots respond to `/virtual_center/cmd_vel` with kinematically correct rigid-body motion.

This document is technical-deep on purpose — it's a brain-dump for you to merge or extend.

---

## TL;DR — what's in the branch

7 commits land on top of `main`:

| SHA | Area | Summary |
|---|---|---|
| `684afb3` | Python | First-pass laplacian gating fix + launch args + alignment_node fix |
| `acda4d9` | Python | Per-robot node naming (`{name}_{robot_id}`) — fixes DDS collision |
| `4b51033` | Python | Pure-feedforward rigid-body laplacian (verified orbit) |
| `b22a2a3` | **Firmware** | OpenCR encoder-based odometry (replaces commanded-value) |
| `6663d8b` | **Firmware** | OpenCR onboard IMU emitted on serial |
| `b33f4a1` | Firmware | Compile fix (`DEG_TO_RAD` rename — collides with OpenCR macro) |
| `700335b` | Python | Reverted closed-loop drift correction (caused wobble at 3 Hz POSE rate) |

The intermediate commit `3ff8229` (closed-loop drift correction) was tried and reverted — see §5 below for the diagnosis. It's gone from the tip but visible in `git log`.

**What needs reflashing:** OpenCR firmware on **both** robots (already done by Seven during the session — encoder odom and IMU lines are confirmed working on the serial monitor).
**What does NOT need reflashing:** anything else.

---

## 1. Code changes — what each file does and why

### 1.1 `opencr_firmware/swerve_kinematics/turtlebot3_conveyor.ino`

Three conceptual blocks were added on top of the original.

#### Encoder-based odometry block (commit `b22a2a3`)

The original firmware computed `POSE` by running FK on `g_modules[]` — i.e., on the values just produced by IK from the operator's command. That was "what we asked the wheels to do", not "what they actually did". On boxes (wheels free) it produced phantom motion that fed back into the laplacian's consensus term and caused jerk.

New flow inside the 30 ms motor cycle:

```cpp
// 1. Compute IK from operator's twist (unchanged)
for (int i = 0; i < 4; i++) compute_module_ik(...);

// 2. Send commands to motors (unchanged GroupSyncWrites)
motor_driver.controlJoints(joint_vals);
motor_driver.controlWheels(wheel_vals);

// 3. NEW: Read actual encoder values back, build measured ModuleState
ModuleState measured[4];
bool used_encoders = read_measured_module_states(measured);

// 4. Run FK on MEASURED states (or fall back to commanded if read failed)
fk_from_module_states(used_encoders ? measured : g_modules, vx, vy, wz);

// 5. Integrate vx,vy,wz into pose (unchanged)
g_odom_x += (c*vx - s*vy) * dt;  // etc.
```

Key helper:

```cpp
static bool read_measured_module_states(ModuleState measured[4])
{
  int32_t joint_raw[4], wheel_raw[4];
  if (!motor_driver.readJointPositions(joint_raw))   return false;
  if (!motor_driver.readWheelVelocities(wheel_raw))  return false;

  for (int m = 0; m < 4; m++) {
    int ik = IK_TO_MOTOR[m];   // self-inverse permutation, valid both ways
    float delta = (joint_raw[m] - STEER_CENTER) / RAD_TO_DXL_POS;
    if (delta >  M_PI / 2.0f) delta =  M_PI / 2.0f;   // match IK clip
    if (delta < -M_PI / 2.0f) delta = -M_PI / 2.0f;
    float omega = (float)wheel_raw[m] / RADS_TO_DXL_VEL;
    measured[ik].delta       = delta;
    measured[ik].drive_speed = omega;
  }
  return true;
}
```

Note the `IK_TO_MOTOR` array is its own inverse permutation `{2,3,0,1}`, so the same table maps both directions. The `delta` clamp matches the IK's `[-π/2, π/2]` window so FK stays consistent if a joint overshoots.

If either GroupSyncRead fails (motor offline, bus glitch), `read_measured_module_states` returns `false` and the caller falls back to commanded values for that cycle. A counter (`g_enc_read_fail_count`) tracks failures and emits a `[WARN] encoder reads failed N times in last ~3 s; using commanded fallback.` line every ~3 s if non-zero. Pi-side parser ignores this line (doesn't start with `POSE`), so it's purely informational.

`fk_from_commanded()` was renamed to `fk_from_module_states(modules, vx, vy, wz)` — same math, takes the array as a parameter so it works for both commanded and measured.

#### IMU block (commit `6663d8b`)

The OpenCR has an onboard MPU-9250 with a built-in Madgwick attitude filter via the `cIMU` library that ships with the OpenCR Arduino board package.

```cpp
#include <IMU.h>
cIMU imu;

// In setup():
imu.begin();   // ~couple hundred ms; spins up SPI + filter

// In every motor cycle:
imu.update();   // keep Madgwick at ~33 Hz

// Every IMU_DIV cycles (= 3 → ~11 Hz), emit:
//   IMU ax ay az gx gy gz yaw
// where:
//   ax,ay,az  body linear acceleration [m/s²]
//   gx,gy,gz  body angular velocity    [rad/s]
//   yaw       Madgwick-filtered yaw    [rad]
float ax = imu.accData[0]  * ACC_LSB_TO_MS2;   // raw int16 → m/s²
float gx = imu.gyroData[0] * GYRO_LSB_TO_RAD;  // raw int16 → rad/s
float yaw = imu.angle[2]   * DEG2RAD;          // filter deg → rad
Serial.print("IMU "); ...
```

Conversion constants assume the cIMU library's defaults (±2 g accel, ±250 dps gyro). If you change the IMU full-scale ranges in the library, update `ACC_LSB_TO_MS2` and `GYRO_LSB_TO_RAD` at the top of the .ino accordingly.

The `IMU` line is distinct from `POSE` so the existing parser doesn't trip. Pi-side currently routes unrecognized OpenCR lines to `get_logger().info('OpenCR: …')` so the IMU lines just appear in launch logs without breaking anything. **No /imu publisher is added on the Pi side yet** — that's a small follow-up (~20 lines in `conveyor_base_node.py`) for whenever the EKF is wired to fuse it.

`DEG_TO_RAD` was originally chosen as the local constant name but it collides with OpenCR's `wiring_constants.h` macro (`#define DEG_TO_RAD 0.017…`). Renamed to `DEG2RAD` in `b33f4a1`.

### 1.2 `opencr_firmware/swerve_kinematics/turtlebot3_conveyor_motor_driver.{h,cpp}`

Added two register defines and two new public methods:

```cpp
#define ADDR_X_PRESENT_VELOCITY   128   // signed int32, 0.229 RPM units
#define ADDR_X_PRESENT_POSITION   132   // unsigned int32, 0..4095 in single-turn mode
#define LEN_X_PRESENT_VELOCITY    4
#define LEN_X_PRESENT_POSITION    4

bool readJointPositions(int32_t *positions_ticks);
bool readWheelVelocities(int32_t *velocities_ticks);
```

Two private members hold the GroupSyncRead handles, allocated in `init()` alongside the existing GroupSyncWrites:

```cpp
groupSyncReadJointPos_ = new dynamixel::GroupSyncRead(
    portHandler_, packetHandler_,
    ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);
groupSyncReadWheelVel_ = new dynamixel::GroupSyncRead(
    portHandler_, packetHandler_,
    ADDR_X_PRESENT_VELOCITY, LEN_X_PRESENT_VELOCITY);
```

Each read method follows the SDK pattern: defensive `clearParam()` → `addParam()` ×4 → `txRxPacket()` → `isAvailable()` ×4 → `getData()` ×4 → `clearParam()`. On `txRxPacket` failure the method returns `false` so the caller can fall back. Output array order matches the existing write methods: `[0=L_R, 1=R_R, 2=L_F, 3=R_F]`.

Velocity is signed (two's complement int32). The SDK's `getData` returns `uint32_t`; a plain `(int32_t)` cast preserves the sign-bit pattern.

Per-call cost: ~200–400 µs for a single GroupSyncRead packet on the 1 Mbps TTL bus. Two reads ≈ 600 µs total; well within the 30 ms motor cycle.

### 1.3 `opencr_firmware/swerve_kinematics/turtlebot3_conveyor.h`

**Unchanged** in this branch. The interesting detail is `IK_TO_MOTOR = {2, 3, 0, 1}` is its own inverse permutation, so we use the same array to map motor index → IK index in `read_measured_module_states`. No `MOTOR_TO_IK` constant needed.

### 1.4 `ros2_ws/src/swerve_formation/swerve_formation/laplacian_formation_node.py`

Replaced wholesale (commit `4b51033`, reaffirmed in `700335b`). Pure-feedforward rigid-body controller:

```python
def _control_loop(self):
    vc_x, vc_y, vc_wz = (operator's virtual_center twist)

    # Formation-wide saturation: across ALL robots' offsets, find worst
    # per-wheel speed; pick a single scale so the rigid shape is preserved.
    max_worst = max over all_offsets of:
        hypot(vc_x - vc_wz*ry, vc_y + vc_wz*rx) + |vc_wz| * CHASSIS_HALF_DIAGONAL
    scale = MAX_WHEEL_LINEAR / max_worst   if max_worst > MAX_WHEEL_LINEAR else 1.0

    # Per-robot rigid-body feedforward
    rx, ry = my_offset
    cmd.linear.x  = (vc_x - vc_wz * ry) * scale
    cmd.linear.y  = (vc_y + vc_wz * rx) * scale
    cmd.angular.z = vc_wz * scale
```

There is **no pose feedback**. The two robots have separate odometry frames (no SLAM, no shared world), so any cross-robot position error term is meaningless before the robots are in motion and produces phantom commands at startup. The instantaneous kinematics is correct: at any moment all robots' commanded body twists are exactly what a rigid body rotating around the virtual centre would dictate.

Trade-off: no consensus, no drift correction. The encoder-based firmware odometry is published to `/odom` and through to `/ekf/odom` for inspection, but the laplacian doesn't use it for control — see §5 for why we tried to and reverted.

`CHASSIS_HALF_DIAGONAL = sqrt(0.15² + 0.15²) ≈ 0.212 m` is the worst-case wheel offset from chassis centre; if you change `HALF_WHEELBASE` or `HALF_TRACK_WIDTH` in the firmware, update this constant too.

`MAX_WHEEL_LINEAR = 0.18 m/s`, just below the firmware's 0.198 m/s motor cap, leaves margin for any future small correction term.

### 1.5 `ros2_ws/src/swerve_bringup/launch/conveyor.launch.py`

Two non-trivial changes (both in commit `acda4d9`):

1. **All README launch args wired through.** Original launch only declared `robot_id`, `usb_port`, `k_gain`. The README's `neighbors`, `my_offset`, `neighbor_offsets`, `offset_init_mode` were silently ignored. Helpers `_parse_xy_list` and `_parse_neighbors` now parse the comma/semicolon syntax and feed the right Node parameters. `OpaqueFunction` is used to do this at runtime so the parsing happens in the launch context.

2. **Per-robot node naming.** Every `Node(name=...)` now appends `_{robot_id}`:
   ```python
   suffix = f'_{robot_id}'
   Node(name='laplacian_formation_node' + suffix, ...)
   Node(name='conveyor_base_node' + suffix, ...)
   # ... and 7 more
   ```
   Without this, both Pis end up with identical `/laplacian_formation_node` etc. on the network. DDS prints `WARNING: Be aware that there are nodes in the graph that share an exact name`, subscription matching gets corrupted, and `conveyor_base_node` silently stops seeing the laplacian's cmd_vel (`Subscription count: 0`). Topics are absolute paths in the source so renaming nodes doesn't change topic names.

### 1.6 `ros2_ws/src/swerve_formation/swerve_formation/alignment_node.py`

Two small fixes (commit `684afb3`):

1. The `_odom_init_sequence`'s readiness check was `np.any(self._my_pose != 0)` which never becomes true if the robot is parked at the origin. Replaced with explicit per-source `_my_pose_received` and `_neighbor_pose_received` flags.
2. The 10 s timeout was too short for staggered SSH launches across two Pis. Bumped to 120 s.

### 1.7 What is NOT in the branch

* No Pi-side IMU parser. The `IMU` lines are emitted by firmware but `conveyor_base_node.py` doesn't recognize them — they currently appear as `OpenCR: IMU 0.021 -0.143 …` in the launch INFO log. Easy follow-up (~20 lines).
* No `/virtual_center/odom` publisher (a real centroid pose). The formation midpoint is implicit in the launch-param offsets — fine for tonight's pure-FF design, would be needed if you re-introduce closed-loop tracking via your `feature/opencrfirmware-odometry` law.
* No encoder-based watchdog or "actual vs commanded" health monitor.
* No formation-state estimator using the new IMU yaw.

---

## 2. Network / discovery setup

The README's Step 2 uses Zenoh middleware. We deliberately skipped it (Seven preferred to use plain FastDDS for this session) — and the README itself permits this with "Skip if using different middleware".

The substitute: a `fastdds_peers.xml` on each participant declaring unicast initial peers so a non-multicast laptop (Seven's WSL2 Ubuntu) can discover the Pis.

`/home/pi1/fastdds_peers.xml` (and a mirror on pi2):

```xml
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
  <participant profile_name="default" is_default_profile="true">
    <rtps>
      <builtin>
        <initialPeersList>
          <locator><udpv4><address>127.0.0.1</address></udpv4></locator>      <!-- localhost (intra-Pi) -->
          <locator><udpv4><address>192.168.1.101</address></udpv4></locator>  <!-- self (or sibling) -->
          <locator><udpv4><address>192.168.1.102</address></udpv4></locator>  <!-- sibling (or self) -->
          <locator><udpv4><address>192.168.1.114</address></udpv4></locator>  <!-- laptop / WSL -->
        </initialPeersList>
      </builtin>
    </rtps>
  </participant>
</profiles>
```

Critical: `127.0.0.1` is needed too — without it, the local node-to-node discovery on a single Pi breaks (we observed `Subscription count: 0` between the laplacian and conveyor_base_node when only LAN peers were listed).

Each launch script exports `FASTRTPS_DEFAULT_PROFILES_FILE=/home/pi1/fastdds_peers.xml` before `ros2 launch`. The laptop's `~/.bashrc` already has the equivalent (`/home/toodmuk/fastdds_peers.xml`), set up earlier by Seven.

This is a non-Zenoh path that satisfies the README's *intent* (laptop teleop reaches both Pis) without the README's *mechanism* (Zenoh router).

---

## 3. Hardware notes

### 3.1 Power

pi1 spent the first session power-cycling every 2–6 minutes whenever the motors drew current. Root cause: Seven was powering pi1 from OpenCR 5V via thin female-to-male jumper wires, which had enough resistance to brownout the Pi during motor current spikes. pi2 was on a proper crimped 2-pin connector (probably JST-XH or 2.54 mm Dupont) and stayed up indefinitely.

Resolution for the second session: Seven swapped pi1's wires for sturdier ones; pi1 stayed up through the rest of testing. Long-term pi1 should get the same crimped connector as pi2.

If a Pi reboots, all the launch scripts in `/tmp/` are wiped — bring them back by re-running the helper `cat > /tmp/relaunch.sh <<'EOF' …` block from the README test runbook in §6 below.

### 3.2 Missing tf2 packages on pi2

pi2 was missing `ros-humble-tf2-py` and `ros-humble-tf2-ros-py`, which crashed `conveyor_base_node` at startup with `ModuleNotFoundError: No module named 'tf2_ros'`. The lab network's apt mirror returns mangled files (`Hash Sum mismatch` / `"NOSPLIT" InRelease`), so `apt install` fails on both Pis.

Workaround used: downloaded the .deb on the laptop (which has internet) and `pscp`-ed to pi2, then `dpkg -i`. Both packages are now installed and persist across reboots — but every future `apt install` on these robots will hit the same problem until the lab network's apt path is fixed (or a working apt-cacher-ng / mirror is configured locally).

### 3.3 Strafe / rotate mechanical calibration

Seven observed during the box test that strafe and rotate motions made the steering joints swivel to "weird" angles. The IK math is correct; this is most likely a mechanical mismatch — the firmware's `init_module_axes()` assumes wheels are tangent to the outward diagonal at home (Dynamixel tick 2048), but the physical mounting of one or more steering joints may not match that assumption. A per-module home offset table in the firmware (or a physical re-zero of the steering joints) would fix it. Not addressed in this branch.

---

## 4. Test results from the floor

After all the changes above:

* **Forward / backward** (`i`, `,`): both robots move identically, gap holds within a few cm over short runs (≤30 s).
* **Strafe** (`J` = shift+j, `L` = shift+l): joints swivel to ±90° and both robots strafe together.
* **Pure rotation** (`j`, `l`): true orbit around the midpoint between robots — outer robot drives forward while inner robot drives backward, both rotating their own bodies at the formation rate. With saturation scaling, hold the key for ~3 s to see a visible 60–80° orbit.
* **Curve** (`u`, `o`, `m`, `.` — combined linear + angular): both robots move in a curve, formation shape preserved; saturation auto-scales when the outer wheel would exceed the firmware speed cap.

After the firmware encoder change (`b22a2a3`):

* On boxes, the wheels can spin freely without the firmware reporting phantom motion. The previously-observed "robot wandered as soon as it became leader" issue from the first session is gone.
* `POSE` x/y/theta stay at zero when the robot is stationary, even with non-zero commands held by the watchdog — the FK now reads measured wheel velocity (zero, because no friction → no actual rotation) instead of commanded velocity.

---

## 5. The closed-loop attempt (and why we backed out)

After the encoder odometry was confirmed to give clean readings, I added a closed-loop drift-correction term to the laplacian (commit `3ff8229`):

```python
peer_in_my_world  = (peer_offset - my_offset) + peer.odom_pose
my_expected       = peer_in_my_world + R(my_heading) @ (my_offset - peer_offset)
err_world         = my_expected - my_pose
correction_world += (k_gain / N_peers) * err_world

# Rotate world → body frame using my heading, add to feedforward
cmd.linear.x = ff_x + (cos(h) * corr_world.x + sin(h) * corr_world.y)
cmd.linear.y = ff_y + (-sin(h) * corr_world.x + cos(h) * corr_world.y)
```

This is the rotation-aware version of the standard formation consensus pull — at any heading, my desired position is the rotated rigid-body offset from peer's actual position. With encoder odom, `peer.odom_pose` is finally trustworthy, so the math should converge.

**Symptom on the floor:** robots wobbled visibly with k_gain = 0.8.

**Diagnosis:** OpenCR firmware sends `POSE` only every `ODOM_DIV = 10` motor cycles → 300 ms → ~3 Hz. The laplacian runs at 20 Hz. So the correction term reacts to the same stale pose for ~6 control cycles in a row. With any meaningful gain (≥ 0.5), this is a textbook recipe for oscillation — controller injects a kick, doesn't see the result for 300 ms, kicks again, then suddenly sees the cumulative result and over-corrects.

**Reverted in `700335b`** — back to pure feedforward (commit `4b51033`'s laplacian).

**Three ways to bring closed loop back without the wobble** (pick one as a follow-up):

1. **Bump `ODOM_DIV` from 10 to 3** in the firmware (POSE at ~10 Hz). Then 6:1 cycle ratio becomes 2:1 and a small gain stops oscillating. Tradeoff: more serial bandwidth and more conveyor_base_node parsing work — measure before deploying.
2. **Switch correction to velocity-based instead of position-based.** Compare per-cycle commanded vs measured velocity (low-pass the diff over ~200 ms) and inject that as a feed-around term. Doesn't depend on long-baseline pose integration so latency matters less.
3. **Add a low-pass filter or deadzone on the position correction term**, so single-step pose updates can't punch the controller. Works but masks the actual problem (stale data).

(1) is the cleanest. The firmware change is two lines.

---

## 6. Reproduction guide — how to run from scratch

### 6.1 One-time setup on each Pi (already in place after this session)

* `/home/pi1/fastdds_peers.xml` and `/home/pi2/fastdds_peers.xml` — peers list (see §2).
* `~/turtlebot3_ws/src/swerve_formation/` and `~/turtlebot3_ws/src/swerve_bringup/` — synced from `feature/two-robot-test-seven`.
* On pi2 only: `ros-humble-tf2-py` and `ros-humble-tf2-ros-py` debs installed (see §3.2).
* OpenCR on each robot: flashed with the firmware in `opencr_firmware/swerve_kinematics/`.

### 6.2 Place the robots

* Both fronts pointing the same direction.
* tb3_0 on the LEFT (per `my_offset:=0.0,0.25` → +Y in ROS = left).
* tb3_1 on the RIGHT.
* About **0.5 m center-to-center.**
* Don't move them after this until both launches are up.

### 6.3 Launch tb3_0 — terminal 1 (laptop SSH to pi1)

```bash
ssh pi1@192.168.1.101            # password: raspberry
unset RMW_IMPLEMENTATION RMW_ZENOH_CONFIG_FILE
source /opt/ros/humble/setup.bash
source ~/turtlebot3_ws/install/setup.bash
export ROS_DOMAIN_ID=30
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/pi1/fastdds_peers.xml
ros2 launch swerve_bringup conveyor.launch.py \
  robot_id:=tb3_0 neighbors:=tb3_1 \
  my_offset:=0.0,0.25 neighbor_offsets:=0.0,-0.25 \
  usb_port:=/dev/ttyACM0 offset_init_mode:=manual \
  k_gain:=0.0
```

Wait until you see both:

```
[laplacian_formation_node_tb3_0]: laplacian_formation_node ready: id=tb3_0 my_offset=[0.0, 0.25] neighbors=['tb3_1']
[conveyor_base_node_tb3_0]: ConveyorBaseNode activated — forwarding commands to OpenCR
```

### 6.4 Launch tb3_1 — terminal 2 (laptop SSH to pi2)

```bash
ssh pi2@192.168.1.102            # password: raspberry
unset RMW_IMPLEMENTATION RMW_ZENOH_CONFIG_FILE
source /opt/ros/humble/setup.bash
source ~/turtlebot3_ws/install/setup.bash
export ROS_DOMAIN_ID=30
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/pi2/fastdds_peers.xml
ros2 launch swerve_bringup conveyor.launch.py \
  robot_id:=tb3_1 neighbors:=tb3_0 \
  my_offset:=0.0,-0.25 neighbor_offsets:=0.0,0.25 \
  usb_port:=/dev/ttyACM0 offset_init_mode:=manual \
  k_gain:=0.0
```

### 6.5 Reset odom + teleop — terminal 3 (laptop WSL Ubuntu)

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=30
unset RMW_IMPLEMENTATION RMW_ZENOH_CONFIG_FILE
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/toodmuk/fastdds_peers.xml

ros2 service call /tb3_0/reset_odom std_srvs/srv/Trigger
ros2 service call /tb3_1/reset_odom std_srvs/srv/Trigger

ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r cmd_vel:=/virtual_center/cmd_vel
```

(If the package isn't installed locally: `sudo apt install ros-humble-teleop-twist-keyboard`.)

Hold rotation keys for ~3 seconds — the saturation cap puts body angular velocity around 0.39 rad/s, so 1 second is only 22° of orbit and is hard to see.

### 6.6 Stopping

* `k` then `Ctrl+C` in teleop terminal.
* `Ctrl+C` in each Pi launch terminal.
* The OpenCR firmware watchdog (5 s on the current build — we use the longer timeout for testing convenience; backup firmware uses 500 ms) is the safety backstop if anything dies unexpectedly.

---

## 7. Known limitations / suggested follow-ups

In rough priority order, easiest first:

1. **Add `/imu` publisher on Pi side.** Parse the new `IMU ax ay az gx gy gz yaw` line in `conveyor_base_node._read_serial_cb` and publish a `sensor_msgs/Imu`. Frame ID = `{robot_id}/base_imu`. ~20 lines.
2. **Bump `ODOM_DIV` 10→3** in firmware (POSE at ~10 Hz). Then re-introduce the closed-loop drift term from `3ff8229` with a low gain (0.2–0.3). Should give the formation tighter shape under wheel slip without wobble.
3. **Re-enable `offset_init_mode:=odom`** in the launch and verify `alignment_node` publishes the right `/formation/offsets` for our setup. Currently we use `manual` and let the launch params own the geometry.
4. **Mechanical re-zero or per-module home offset table** to fix the strafe/rotate visual oddity Seven saw on boxes (see §3.3).
5. **EKF fusion of IMU yaw with wheel-derived yaw.** The wheel-yaw drifts under slip; the IMU yaw drifts on bias. EKF is already in the launch but currently just relays `/odom`. Adding `/imu` as a second sensor source would bound long-run heading error.
6. **True centroid `/virtual_center/odom`.** Average the two robots' EKF positions (after a known startup transform). Use that as the reference frame for the laplacian instead of letting each robot compute the formation midpoint implicitly via its own offset. Necessary if the formation grows beyond 2 robots.
7. **Migrate to Zenoh** as the README originally specified. The FastDDS peers config works but is fragile (each new participant needs its IP added to every other participant's peers list). Zenoh's router-based discovery is cleaner.

---

## 8. Files-to-PR-targets matrix

If you want to merge piecemeal:

| Files | Smallest standalone PR |
|---|---|
| `opencr_firmware/swerve_kinematics/turtlebot3_conveyor_motor_driver.{h,cpp}` + `turtlebot3_conveyor.ino` (encoder odom only) | `b22a2a3` — encoder-based odometry. Standalone. |
| `opencr_firmware/swerve_kinematics/turtlebot3_conveyor.ino` (IMU only, on top of encoder) | `6663d8b` + `b33f4a1` — IMU emit + DEG2RAD fix. Depends on `b22a2a3`. |
| `ros2_ws/src/swerve_formation/swerve_formation/laplacian_formation_node.py` | `4b51033` (and confirmed by `700335b` revert). Standalone. |
| `ros2_ws/src/swerve_bringup/launch/conveyor.launch.py` + `swerve_formation/swerve_formation/alignment_node.py` | `684afb3` + `acda4d9` (per-robot node naming). |

All files in `feature/two-robot-test-seven` are also in the working directory at `_gitrepo/` if you want to diff locally before merging.

---

That's everything. Branch is at [`feature/two-robot-test-seven`](https://github.com/Tankikun/swerve_transport_project/tree/feature/two-robot-test-seven), 7 commits, ready for your review and merge.
