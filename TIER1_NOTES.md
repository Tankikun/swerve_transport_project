# Tier 1 Hardware Test — Verified Notes

Branch: `feature/navigation-real-robot-test`
Date: 2026-05-02 night session
Robot tested: pi2 (`tb3_1`)

## What was verified on hardware

All three holonomic axes of `navigation_node` exercised on a real robot
with `single_robot_nav.launch.py`. Robot drives on a tape mark, EKF
pose recorded, physical position measured with a tape.

| Axis      | Goal     | EKF result | Physical | Error              |
|-----------|----------|------------|----------|--------------------|
| Forward   | 1.00 m   | 0.954 m    | 0.96 m   | 6 mm               |
| Strafe    | 0.50 m   | 0.456 m    | 0.46 m   | 4 mm               |
| Rotation  | 90°      | 92.9°      | ~100°    | ~7° yaw drift      |

Forward and strafe are essentially perfect. Rotation has measurable
yaw drift — see Open Issues below.

## Bugs found and fixed (in this branch)

1. **Heading-alignment overrode the goal heading.** The original nav
   node always rotated the robot to point in the direction of motion
   ("face where you're going"). Correct for diff-drive, *wrong* for
   swerve cooperative transport — the formation would rotate 90°
   to strafe a payload past an obstacle, dropping it. Added
   `holonomic_mode` parameter (default `false` to keep legacy
   behaviour for single-robot exploration). When `true`, the robot
   tracks only the explicit goal heading. **Use `holonomic_mode:=true`
   for any formation work.**

2. **Pure-rotation goals didn't work.** `_apf_velocity()` returned
   `omega=0` whenever position was already at the goal, so a goal
   like `(x=0, y=0, theta=π/2)` produced no `wz` command — the
   robot sat still. Now still emits a heading-tracking omega when
   at goal position.

3. **REACHED check ignored heading.** A pure-rotation goal would
   trigger REACHED on entry (position already at goal) before any
   rotation happened. Added `heading_tolerance` parameter (default
   π — effectively disabled, no behaviour change for existing
   callers). Set to ~0.1 rad (5.7°) when goal heading matters.

## Supporting changes (also in this branch)

- **`send_goal_node`**: was firing 0.5 s after startup with default
  QoS. Cross-machine FastDDS unicast first-discovery handshake takes
  1–2 s, so the goal was silently lost before pi2's nav-node
  subscriber was matched. Now uses TRANSIENT_LOCAL durability and
  polls `get_subscription_count()` (max 5 s ceiling) before
  publishing. Logs the matched-subs count and wait time.

- **`cmd_vel_relay_node`**: 50-line in-tree replacement for
  `topic_tools relay`. We can't `apt install ros-humble-topic-tools`
  on the lab Pis — the apt mirror returns hash-mismatched debs.

- **`single_robot_nav.launch.py`**: parametrises goal_tolerance,
  holonomic_mode, heading_tolerance. Uses the in-tree relay above.

- **`two_robot_formation_nav.launch.py`** (new, *un-tested*): minimal
  Tier-2 stack — `conveyor_base + ekf + laplacian + navigation +
  static_leader` — deliberately omits `slam_3d_node`,
  `ai_camera_node`, `alignment_node`, `formation_size_node` because
  those are camera-dependent stubs that crash on hardware without a
  working camera pipeline. Ready for Tier 2 once both Pis can
  discover each other on the same FastDDS peers config.

## Network gotchas (from this session — important for Tier 2)

- WSL2 in mirrored mode inherits Windows interfaces but Windows
  may have **two default routes** (LAN + WiFi) at the same metric.
  Bumping LAN's `InterfaceMetric` to 9000 (`Set-NetIPInterface`)
  keeps the connected route to `192.168.1.0/24` while letting WiFi
  win for default. This is what allowed simultaneous internet (to
  Claude) + LAN (to robots).
- `fastdds_peers.xml` on **both laptop and each Pi** must list the
  laptop's *current* DHCP-assigned LAN IP. The robot router gives
  out different IPs across plug events; if peers reference a stale
  IP, discovery silently fails (`topic list` shows only
  `/parameter_events` and `/rosout`). Patch with
  `sed -i "s/OLD_IP/NEW_IP/g" ~/fastdds_peers.xml` on every host
  before launching.
- Pi2's `~/.bashrc` exports `ROS_DOMAIN_ID=30` but **not**
  `FASTRTPS_DEFAULT_PROFILES_FILE`. Must be exported in the launch
  shell, otherwise FastDDS falls back to multicast which the laptop
  cannot discover. Same likely true on pi1.
- Multiple concurrent `ros2 launch` instances on the same Pi all
  fight for `/dev/ttyACM0` and corrupt the serial stream. Always
  fully kill prior launches before re-launching. There's a helper
  script at `/tmp/killall.sh` on pi2 (also see `~/killall_pi2.sh`
  on the laptop for the source).

## Open issues (in priority order)

### 1. Wheel-only EKF can't see body rotation accurately

The 7° yaw drift on a 90° rotation is mechanical steering-joint
slop translated into FK error: when the four wheel-direction
encoders disagree slightly, the firmware's encoder→twist
conversion produces a phantom angular component. EKF integrates
this faithfully but the integration is wrong.

**Fix**: parse the IMU stream the firmware already emits
(`OpenCR: IMU ax ay az gx gy gz yaw`) inside `conveyor_base_node`
and either publish it as a `sensor_msgs/Imu` for `ekf_node` to
consume, or feed the yaw directly into `ekf_node` as a correction
step (similar to the existing SLAM correction code path, just
with a simpler observation matrix).

This is the single biggest blocker for Tier 2 reliability — the
formation will accumulate phantom rotation drift over time.

### 2. `slam_3d_node`, `ai_camera_node`, `alignment_node` are stubs

They run but produce no useful output without a working camera
pipeline. Running them in the full bringup launch is harmless but
also pointless. Keep them out of `two_robot_formation_nav.launch.py`
until they actually do something.

### 3. `send_goal_node` publishes goal twice

Initial publish + one TRANSIENT_LOCAL linger publish. Idempotent
for a single-goal test, but worth cleaning up if waypoint sequences
are needed.

### 4. `goal_tolerance` of 0.05 m + `MAX_LINEAR=0.18` makes for
slow approach near goal

The velocity ramp slows the robot to a crawl over the last
~10 cm. Considered fine for now; can be tuned per use case.

## How to reproduce Tier 1 yourself

Network setup (one-time per session):

```powershell
# Run as Administrator if Windows default routes split badly:
$lan = Get-NetAdapter | Where-Object {$_.Name -eq 'Ethernet' -and $_.Status -eq 'Up'}
Set-NetIPInterface -InterfaceIndex $lan.ifIndex -InterfaceMetric 9000 -AddressFamily IPv4
```

Patch peers files with current laptop LAN IP:

```bash
LAN_IP=$(ip -4 addr show | awk '/inet 192\.168\.1\./{print $2}' | cut -d/ -f1)
sed -i "s/192\.168\.1\.[0-9]\+/$LAN_IP/g" ~/fastdds_peers.xml
ssh pi2@192.168.1.102 "sed -i \"s/.*114.*\|.*118.*/<placeholder $LAN_IP>/\" ~/fastdds_peers.xml"
```

(Above sed for pi2 is rough — easier: `cat`, edit, `scp` back.)

Sync code and build on pi2:

```bash
rsync -av --delete --exclude='__pycache__' --exclude='build' --exclude='install' --exclude='log' \
  ~/swerve_transport_project/ros2_ws/src/  pi2@192.168.1.102:~/ros2_ws/src/

ssh pi2@192.168.1.102 'source /opt/ros/humble/setup.bash && cd ~/ros2_ws && \
  colcon build --symlink-install --packages-select swerve_formation swerve_bringup'
```

Launch on pi2:

```bash
ssh pi2@192.168.1.102
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/pi2/fastdds_peers.xml
export ROS_DOMAIN_ID=30
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch swerve_bringup single_robot_nav.launch.py \
  robot_id:=tb3_1 \
  goal_tolerance:=0.05 \
  holonomic_mode:=true \
  heading_tolerance:=0.1
```

From laptop, send a goal:

```bash
export ROS_DOMAIN_ID=30
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/toodmuk/fastdds_peers.xml
source /opt/ros/humble/setup.bash
source ~/swerve_transport_project/install/setup.bash

# Forward 1 m
ros2 run swerve_formation send_goal_node --ros-args -p x:=1.0 -p y:=0.0

# Strafe left 0.5 m (holonomic_mode required)
ros2 run swerve_formation send_goal_node --ros-args -p x:=0.0 -p y:=0.5

# Rotate 90° in place (heading_tolerance:=0.1 required)
ros2 run swerve_formation send_goal_node --ros-args -p x:=0.0 -p y:=0.0 -p theta:=1.5708
```

Between runs: press OpenCR RESET to re-home steering, then restart
the launch on pi2 to reset EKF to (0, 0, 0).
