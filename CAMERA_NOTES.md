# Camera + RTAB-Map Notes

Branch: `feature/rtab-map`
Status: Steps 1–3 + 5 of the RTAB-Map plan complete on pi2. Step 4
(`rtabmap_ros` install) blocked on the lab apt mirror; workaround
documented below. Step 6 (mapping run) is yours to execute.

## What works today on pi2

- **OAK-D Lite camera** plugged in, depthai 3.5.0 SDK working from
  Python, **udev rules added** so non-root userspace can talk to the
  USB device.
- **`oak_camera_node`** (custom, in `swerve_formation` package).
  Publishes 4 ROS topics from the OAK-D using the depthai 3.x SDK
  directly:
    `/{robot_id}/camera/rgb/image_raw`        sensor_msgs/Image (BGR8)
    `/{robot_id}/camera/rgb/camera_info`      sensor_msgs/CameraInfo
    `/{robot_id}/camera/depth/image_raw`      sensor_msgs/Image (16UC1, mm)
    `/{robot_id}/camera/depth/camera_info`    sensor_msgs/CameraInfo
  Default 15 fps at 640×400, depth aligned to the RGB optical frame
  (i.e. depth pixels share intrinsics with RGB — what RTAB-Map wants
  for RGBD subscription).
- **Static TF** `{robot_id}_base_link → {robot_id}_oak_rgb_camera_optical_frame`
  with the standard ROS optical-from-body rotation (roll = pitch =
  yaw = -π/2) and a default mount offset (10 cm forward of rotation
  centre, 15 cm above floor). **You MUST re-measure cam_x / cam_y /
  cam_z for your actual mount** — RTAB-Map's pose accuracy depends
  on this transform being correct.
- **`oak_camera.launch.py`** — brings up just the camera + TF. Used
  for camera-only smoke tests.
- **`rtabmap_mapping.launch.py`** — full mapping stack (camera + TF
  + conveyor_base for wheel odometry + rtabmap_slam). Will fail to
  run until rtabmap is installed (see below).

## Why we wrote our own camera node

The lab apt mirror at Chula does TLS-intercepting MITM at proxy IP
192.168.2.1 with a self-signed cert that pi2 doesn't trust. Even
disabling apt's HTTPS cert verification, large `.deb` downloads
intermittently come back as a 2.4 KB error page (the proxy chokes).
`ros-humble-depthai-ros-driver` was the worst offender — never
finished installing across multiple attempts.

The depthai Python SDK was already installed (`python3-depthai`
3.5.0) and works fine. Our custom `oak_camera_node` is ~250 lines
of Python that publishes exactly what RTAB-Map needs, no apt
dependency.

## Why depthai 3.x bites if you're following old docs

The depthai SDK had a major API change between 2.x and 3.x:

- `dai.node.XLinkOut` is **gone** in 3.x — outputs handle queues
  themselves via `output.createOutputQueue()`.
- Pipeline lifecycle uses `pipeline.start()` and `pipeline.isRunning()`
  inside a `with dai.Pipeline()` context. `dai.Device(pipeline)` may
  still work but the `with` form is the documented 3.x pattern.
- `StereoDepth.PresetMode.HIGH_DENSITY` → `FAST_DENSITY`
  (and other preset names changed). `oak_camera_node` defensively
  iterates the known names so it keeps working across SDK versions.
- `StereoDepth` now requires `setOutputSize(width, height)` with
  width a multiple of 16. Without it the node crashes a few seconds
  after pipeline start with `X_LINK_ERROR`.
- `initialConfig` setters (`setConfidenceThreshold`,
  `setLeftRightCheck`, etc.) may not exist in 3.x. Fall back to
  defaults — they're sensible.

If you ever read official Luxonis examples and they look very
different from `oak_camera_node.py`, check the SDK major version —
most online tutorials are still 2.x.

## RTAB-Map install workaround (laptop-side .deb download)

Because the lab apt mirror corrupts large downloads, `apt install
ros-humble-rtabmap-ros` will fail on pi2 the same way
depthai_ros_driver did. The workaround:

1. **On the laptop's WSL Ubuntu** (which has clean WiFi internet
   when you're NOT on LAN-only):
   ```bash
   # Add arm64 architecture so we can download arm64 .debs
   sudo dpkg --add-architecture arm64

   # Add ROS humble apt repo (HTTP is fine here — your WiFi has clean cert)
   sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
       -o /usr/share/keyrings/ros-archive-keyring.gpg
   echo "deb [arch=arm64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" | \
       sudo tee /etc/apt/sources.list.d/ros2-arm64.list
   echo "deb [arch=arm64] http://ports.ubuntu.com/ubuntu-ports jammy main universe" | \
       sudo tee /etc/apt/sources.list.d/arm64-ports.list
   sudo apt update -o Acquire::AllowInsecureRepositories=true || true

   # Download the .debs (does NOT install on the laptop)
   mkdir -p ~/debs && cd ~/debs
   apt-get download \
       ros-humble-rtabmap-ros:arm64 \
       ros-humble-rtabmap-slam:arm64 \
       ros-humble-rtabmap-conversions:arm64 \
       ros-humble-rtabmap-msgs:arm64 \
       ros-humble-rtabmap-launch:arm64 \
       ros-humble-rtabmap-sync:arm64
   # ↑ may fail to find some — that's OK, we'll grab missing deps below.

   # Resolve the full transitive dependency list
   sudo apt install -y apt-rdepends
   apt-rdepends ros-humble-rtabmap-ros:arm64 2>/dev/null \
       | grep -v '^ ' \
       | grep -v '^Reading' \
       | sort -u > /tmp/rtabmap_deps.txt
   while read pkg; do
       apt-get download "${pkg}:arm64" 2>/dev/null
   done < /tmp/rtabmap_deps.txt
   ```

2. **scp to pi2 and install**:
   ```bash
   ssh pi2@192.168.1.102 'mkdir -p /tmp/debs'
   scp ~/debs/*.deb pi2@192.168.1.102:/tmp/debs/
   ssh pi2@192.168.1.102 'cd /tmp/debs && sudo dpkg -i *.deb; sudo apt -f install -y'
   ```

3. **Verify**:
   ```bash
   ssh pi2@192.168.1.102 'source /opt/ros/humble/setup.bash && ros2 pkg list | grep rtabmap'
   ```
   Should list at least `rtabmap_msgs`, `rtabmap_slam`, `rtabmap_conversions`.

If apt-rdepends recursion gets too greedy and pulls amd64-only
packages, prune the list to ros-humble-* + lib*-arm64 only.

## Mapping run (step 6 — yours to do)

After RTAB-Map is installed, on pi2:

```bash
ssh pi2@192.168.1.102
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/pi2/fastdds_peers.xml
export ROS_DOMAIN_ID=30
source /opt/ros/humble/setup.bash
source /home/pi2/ros2_ws/install/setup.bash

# First override the camera mount measurements with your real values
ros2 launch swerve_bringup rtabmap_mapping.launch.py \
    robot_id:=tb3_1 \
    db_path:=~/maps/tb3_1_room.db
```

Drive the robot slowly around the room (use teleop in another
terminal — the existing `teleop_twist_keyboard` topic works on
`/tb3_1/cmd_vel` if `conveyor_base_node` is running, which the
mapping launch starts by default). Watch:

- Pi2 temperature: `watch -n 5 vcgencmd measure_temp`. RTAB-Map
  with feature extraction will push it. If it goes above 75°C,
  pause and let it cool.
- `/rtabmap/info` topic: tells you keyframes added, loop closures
  fired. Want to see loop closures rise as you re-visit start.
- `/rtabmap/cloud_map`: streams the accumulated point cloud.
  Visualise in RViz on the laptop (set fixed frame = `map`).

End the mapping run by `Ctrl+C` the launch. The .db at
`~/maps/tb3_1_room.db` is now your map. To later run in
localization-only mode against this .db, write a
`rtabmap_localization.launch.py` (template will be the mapping
launch with `Mem/IncrementalMemory: 'False'` and removing
`--delete_db_on_start` from arguments).

## Files added in this branch

```
ros2_ws/src/swerve_formation/swerve_formation/oak_camera_node.py
ros2_ws/src/swerve_formation/setup.py             (registered oak_camera_node)
ros2_ws/src/swerve_bringup/launch/oak_camera.launch.py
ros2_ws/src/swerve_bringup/launch/rtabmap_mapping.launch.py
CAMERA_NOTES.md                                    (this file)
```

## Open issues for next session

1. **Camera mount geometry**: replace the launch-arg defaults
   (`cam_x=0.10 cam_y=0.00 cam_z=0.15`) with measured values from
   the actual OAK-D mount on tb3_1.
2. **RTAB-Map .deb install**: do the laptop-side download workaround
   above when next on a clean network.
3. **Apply same pipeline on tb3_0 (pi1)** once it's online.
4. **Localization-only launch**: write `rtabmap_localization.launch.py`
   for after the .db is built.
5. **Wire `/rtabmap/localization_pose` → `/{robot_id}/slam/pose`**:
   the existing `ekf_node` already subscribes to `/slam/pose` for
   its correction step. One launch remap + we close the EKF drift
   loop documented in `TIER1_NOTES.md`.

## Network / thermal sanity checks during this session

| Probe                        | Value             |
|------------------------------|-------------------|
| Pi2 idle temp                | 52 °C             |
| Pi2 with apt install running | 56–59 °C          |
| Pi2 with camera streaming    | 60–63 °C          |
| Pi2 throttle history         | `0x0` (no events) |
| Pi2 free disk after work     | 48 GB             |
| Camera USB speed             | HIGH (USB 2.0)    |
