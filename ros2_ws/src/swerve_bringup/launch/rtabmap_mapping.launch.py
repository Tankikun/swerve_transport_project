"""
rtabmap_mapping.launch.py
-------------------------
Tier-2 of the RTAB-Map plan: drive the robot around the room with
this launch running, capturing visual SLAM keyframes into a database
file. Switch to localization mode (rtabmap_localization.launch.py)
once the .db is built.

Stack:
  oak_camera_node                 — depthai 3.x publisher (custom)
  static_transform_publisher      — base_link ↔ camera optical frame
  conveyor_base_node              — wheel odometry from OpenCR
                                     publishes /{robot_id}/odom + /imu
  ekf_node                        — fuses /odom (translation) + /imu
                                     (gyro Z, slip-immune yaw rate),
                                     publishes /{robot_id}/ekf/odom
  rtabmap (rtabmap_slam)          — RGB-D SLAM, mapping mode, builds .db

Inputs to rtabmap:
  rgb           : /{robot_id}/camera/rgb/image_raw
  rgb_info      : /{robot_id}/camera/rgb/camera_info
  depth         : /{robot_id}/camera/depth/image_raw
  odom          : /{robot_id}/ekf/odom (IMU-fused, slip-immune yaw)
  TF chain      : odom → base_link → camera_optical (provided by
                  conveyor_base_node + the static TF below)

Output (during mapping):
  /rtabmap/mapData       — incremental keyframes
  /rtabmap/cloud_map     — accumulated coloured point cloud
  /rtabmap/localization_pose — robot pose in map frame (also localizes)

Database file:
  ~/maps/{robot_id}_room.db   (created if missing, appended otherwise)

Mapping run procedure (see TIER1_NOTES / camera README):
  1. ros2 launch swerve_bringup rtabmap_mapping.launch.py
  2. Teleop the robot slowly around the room. Avoid sharp turns —
     visual feature tracker prefers smooth motion. Cover every area
     at least once.
  3. Return to the start position so loop closure can collapse drift.
  4. Ctrl+C the launch. The .db is saved automatically.
  5. Inspect with rtabmap-databaseViewer ~/maps/{robot_id}_room.db
     on a desktop machine (heavy GUI — don't run it on the Pi).

NOTE — RTAB-Map apt install: as of this branch the lab apt mirror
corrupts the depthai_ros_driver download (proxy MITM at
192.168.2.1) and we expect the same problem with
ros-humble-rtabmap-ros. If `ros2 launch ... rtabmap_mapping.launch.py`
errors with "package 'rtabmap_slam' not found", install it via the
laptop-side .deb workaround documented in CAMERA_NOTES.md.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    robot_id     = LaunchConfiguration('robot_id').perform(context)
    usb_port     = LaunchConfiguration('usb_port').perform(context)
    db_path      = LaunchConfiguration('db_path').perform(context)
    fps          = LaunchConfiguration('fps').perform(context)
    enable_base  = LaunchConfiguration('enable_base').perform(context).lower() in ('true', '1', 'yes')
    enable_ekf   = LaunchConfiguration('enable_ekf').perform(context).lower() in ('true', '1', 'yes')
    suffix       = f'_{robot_id}'

    optical_frame = f'{robot_id}_oak_rgb_camera_optical_frame'
    base_link     = f'{robot_id}_base_link'
    odom_frame    = f'{robot_id}_odom'

    # Expand ~ in db_path
    db_path = os.path.expanduser(db_path)
    os.makedirs(os.path.dirname(db_path) or '.', exist_ok=True)

    config_path = os.path.join(
        get_package_share_directory('swerve_bringup'), 'config', 'rtabmap_mapping.yaml'
    )

    actions = []

    # ── Camera (oak_camera.launch.py) ────────────────────────────────────
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('swerve_bringup'), 'launch', 'oak_camera.launch.py'
        ])),
        launch_arguments={
            'robot_id':   robot_id,
            'fps':        fps,
            'cam_x':      LaunchConfiguration('cam_x'),
            'cam_y':      LaunchConfiguration('cam_y'),
            'cam_z':      LaunchConfiguration('cam_z'),
            'rgb_size':   '640x400',
            'depth_size': '640x400',
        }.items(),
    ))

    # ── Wheel odometry source (skip if already running elsewhere) ─────────
    if enable_base:
        actions.append(Node(
            package='swerve_formation',
            executable='conveyor_base_node',
            name='conveyor_base_node' + suffix,
            parameters=[{
                'robot_id':  robot_id,
                'usb_port':  usb_port,
                'baud_rate': 115200,
            }],
            output='screen',
        ))

    # ── EKF — wheel /odom + /imu (gyro Z fusion) → /ekf/odom. Mapping
    # used to consume raw /odom directly; switching to /ekf/odom here
    # gives RTAB-Map a slip-immune yaw signal (the dominant carrier of
    # odom drift on this swerve platform) which fixes loop-closure
    # rejection caused by bad PnP priors.
    if enable_ekf:
        actions.append(Node(
            package='swerve_formation',
            executable='ekf_node',
            name='ekf_node' + suffix,
            parameters=[{
                'robot_id':    robot_id,
                'gyro_z_sign': LaunchConfiguration('gyro_z_sign'),
            }],
            output='screen',
        ))

    # ── RTAB-Map SLAM (mapping mode) ─────────────────────────────────────
    # Subscribes RGB + depth + odom; outputs /rtabmap/cloud_map and the
    # database file. Will fail to launch if ros-humble-rtabmap-slam is
    # not installed — see header note.
    actions.append(Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap' + suffix,
        output='screen',
        parameters=[
            config_path,
            {
                # Robot-specific overrides (cannot live in the shared YAML)
                'frame_id':             base_link,
                'odom_frame_id':        odom_frame,
                'database_path':        db_path,
                # Not in YAML; keep existing behaviour
                'subscribe_rgbd':       False,
                'subscribe_scan':       False,
                'subscribe_scan_cloud': False,
                'RGBD/OptimizeFromGraphEnd': 'True',
            },
        ],
        remappings=[
            ('rgb/image',         f'/{robot_id}/camera/rgb/image_raw'),
            ('rgb/camera_info',   f'/{robot_id}/camera/rgb/camera_info'),
            ('depth/image',       f'/{robot_id}/camera/depth/image_raw'),
            # IMU-fused odom during mapping. ekf_node fuses the gyro Z
            # rate (slip-immune) over the wheel-derived yaw, eliminating
            # the per-turn drift that was poisoning RTAB-Map's PnP
            # priors at loop-closure verification. The
            # {robot_id}_odom→base_link TF is still wheel-derived,
            # mirroring the localization launches; small divergences
            # during a mapping pass are tolerable.
            ('odom',              f'/{robot_id}/ekf/odom'),
            # Per-robot scoping (mirrors rtabmap_localization.launch.py).
            # Even in mapping mode, namespacing keeps logs / cloud_map /
            # info clean if a second robot ever runs concurrently.
            ('localization_pose', f'/{robot_id}/rtabmap/localization_pose'),
            ('info',              f'/{robot_id}/rtabmap/info'),
            ('mapData',           f'/{robot_id}/rtabmap/mapData'),
            ('cloud_map',         f'/{robot_id}/rtabmap/cloud_map'),
        ],
        arguments=['--delete_db_on_start'],   # comment out to APPEND to existing db
    ))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_id', default_value='tb3_1',
            description='Robot ID (tb3_0, tb3_1, ...)'),
        DeclareLaunchArgument(
            'usb_port', default_value='/dev/ttyACM0',
            description='OpenCR USB-CDC device path.'),
        DeclareLaunchArgument(
            'db_path', default_value='~/maps/tb3_1_room.db',
            description='Path to the rtabmap database file. Created if '
                        'absent. Use --delete_db_on_start in node arguments '
                        'for a fresh map; remove that flag to append.'),
        DeclareLaunchArgument(
            'fps', default_value='15',
            description='Camera FPS.'),
        DeclareLaunchArgument(
            'cam_x', default_value='',
            description='Camera optical frame X offset from base_link [m]. '
                        'Leave empty to use measured value from '
                        '_CAMERA_MOUNT in oak_camera.launch.py.'),
        DeclareLaunchArgument(
            'cam_y', default_value='',
            description='Camera Y offset from base_link [m]. Leave empty '
                        'to use _CAMERA_MOUNT.'),
        DeclareLaunchArgument(
            'cam_z', default_value='',
            description='Camera Z offset from base_link [m]. Leave empty '
                        'to use _CAMERA_MOUNT.'),
        DeclareLaunchArgument(
            'enable_base', default_value='true',
            description='Start conveyor_base_node. Set false if odometry '
                        'is being published by a separate launch already.'),
        DeclareLaunchArgument(
            'enable_ekf', default_value='true',
            description='Start ekf_node so RTAB-Map sees IMU-fused odom. '
                        'Set false to feed RTAB-Map raw /odom (legacy '
                        'pre-IMU-fusion behaviour).'),
        DeclareLaunchArgument(
            'gyro_z_sign', default_value='1.0',
            description='Sign of the IMU gyro Z reading. Set to -1.0 if a '
                        'bench yaw test shows ekf yaw decreasing under '
                        'physical CCW rotation.'),
        OpaqueFunction(function=launch_setup),
    ])
