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
                                     publishes /{robot_id}/odom
  rtabmap (rtabmap_slam)          — RGB-D SLAM, mapping mode, builds .db

Inputs to rtabmap:
  rgb           : /{robot_id}/camera/rgb/image_raw
  rgb_info      : /{robot_id}/camera/rgb/camera_info
  depth         : /{robot_id}/camera/depth/image_raw
  odom          : /{robot_id}/odom
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
    suffix       = f'_{robot_id}'

    optical_frame = f'{robot_id}_oak_rgb_camera_optical_frame'
    base_link     = f'{robot_id}_base_link'
    odom_frame    = f'{robot_id}_odom'

    # Expand ~ in db_path
    db_path = os.path.expanduser(db_path)
    os.makedirs(os.path.dirname(db_path) or '.', exist_ok=True)
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

    # ── RTAB-Map SLAM (mapping mode) ─────────────────────────────────────
    # Subscribes RGB + depth + odom; outputs /rtabmap/cloud_map and the
    # database file. Will fail to launch if ros-humble-rtabmap-slam is
    # not installed — see header note.
    actions.append(Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap' + suffix,
        output='screen',
        parameters=[{
            'frame_id':            base_link,
            'odom_frame_id':       odom_frame,
            'map_frame_id':        'map',
            'subscribe_depth':     True,
            'subscribe_rgb':       True,
            'subscribe_rgbd':      False,
            'subscribe_scan':      False,
            'subscribe_scan_cloud': False,
            'approx_sync':         True,
            # Sync settings — see rtabmap_localization.launch.py for
            # rationale. The deprecated `queue_size: 30` is replaced
            # with the explicit topic_queue_size / sync_queue_size
            # names that rtabmap actually wants.
            'approx_sync_max_interval': 0.5,
            'topic_queue_size':    5,
            'sync_queue_size':     10,
            'database_path':       db_path,
            # Mapping mode (NOT localization-only)
            'Mem/IncrementalMemory': 'True',
            # Save database on exit
            'Mem/InitWMWithAllNodes': 'False',
            'RGBD/OptimizeFromGraphEnd': 'True',
            # Tuning cherry-picked from feature/config-rtab (Tan); same
            # mapping-mode profile as rtabmap_laptop_mapping.launch.py
            # (which has the full per-param rationale comments). Pi 4
            # runs hotter under this load; if you see thermal warnings,
            # use the laptop-side mapping launch instead.
            'RGBD/LinearUpdate':          '0.1',
            'RGBD/AngularUpdate':         '0.1',
            'Reg/Force3DoF':              'true',
            'Vis/EstimationType':         '1',
            'Vis/MinInliers':             '20',
            'Kp/DetectorStrategy':        '6',
            'Kp/MaxFeatures':             '800',
            'Mem/ImagePreDecimation':     '1',
            'Mem/DepthDecimation':        '1',
            'Rtabmap/DetectionRate':      '1.0',
            'Rtabmap/LoopThr':            '0.11',
            'Mem/STMSize':                '30',
            'Mem/RehearsalSimilarity':    '0.6',
            'RGBD/ProximityBySpace':      'true',
            'RGBD/ProximityByTime':       'true',
            'RGBD/ProximityMaxGraphDepth': '50',
            'Optimizer/Strategy':         '1',
            'Optimizer/Iterations':       '20',
            'Optimizer/Robust':           'true',
            'Rtabmap/PublishStats':       'true',
            'Rtabmap/PublishLastSignature': 'true',
        }],
        remappings=[
            ('rgb/image',         f'/{robot_id}/camera/rgb/image_raw'),
            ('rgb/camera_info',   f'/{robot_id}/camera/rgb/camera_info'),
            ('depth/image',       f'/{robot_id}/camera/depth/image_raw'),
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
            'cam_x', default_value='0.10',
            description='Camera optical frame X offset from base_link [m].'),
        DeclareLaunchArgument(
            'cam_y', default_value='0.00',
            description='Camera Y offset from base_link [m].'),
        DeclareLaunchArgument(
            'cam_z', default_value='0.15',
            description='Camera Z offset from base_link [m].'),
        DeclareLaunchArgument(
            'enable_base', default_value='true',
            description='Start conveyor_base_node. Set false if odometry '
                        'is being published by a separate launch already.'),
        OpaqueFunction(function=launch_setup),
    ])
