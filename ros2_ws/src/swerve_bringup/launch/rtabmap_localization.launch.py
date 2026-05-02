"""
rtabmap_localization.launch.py
------------------------------
RTAB-Map in **localization-only** mode against an existing room
database. This is what runs during normal operation — every robot
launches this, knows where it is in the map, and feeds that pose
into `ekf_node` which the rest of the stack already trusts.

Use this AFTER you have driven the robot through the room with
`rtabmap_mapping.launch.py` and saved a `.db` you're happy with.

Stack:
  oak_camera.launch.py            — camera + base_link↔optical TF
  conveyor_base_node              — wheel odometry from OpenCR
                                     publishes /{robot_id}/odom
  ekf_node                        — fuses /odom + /slam/pose,
                                     publishes /{robot_id}/ekf/odom
                                     (the authoritative pose for nav
                                     and laplacian)
  rtabmap (rtabmap_slam)          — localization-only against the .db,
                                     publishes /rtabmap/localization_pose
  slam_pose_relay_node            — converts
                                     /rtabmap/localization_pose
                                     (PoseWithCovarianceStamped)
                                     into /{robot_id}/slam/pose
                                     (PoseStamped) which ekf_node
                                     already subscribes to.

Subscriptions:
  rgb           : /{robot_id}/camera/rgb/image_raw
  rgb_info      : /{robot_id}/camera/rgb/camera_info
  depth         : /{robot_id}/camera/depth/image_raw
  odom          : /{robot_id}/odom

Outputs (the chain we actually care about):
  /{robot_id}/slam/pose     PoseStamped — visual localization pose
                              in `map` frame
  /{robot_id}/ekf/odom      Odometry    — fused, drift-free authoritative
                              pose, consumed by laplacian + navigation

The key win: with the slam-pose feedback loop closed, the 7° yaw
drift per 90° rotation we measured in TIER1_NOTES.md goes away.
The EKF still uses wheel odometry as its high-rate prediction step,
but every visual frame nudges it back onto truth.

Database file:
  ~/maps/{robot_id}_room.db  by default (override with db_path arg)
  This file MUST exist — rtabmap will refuse to start otherwise.

Initial pose:
  RTAB-Map's localization-only mode starts in "global re-localization"
  mode: it scans through the database trying to match the current
  camera frame to ANY stored keyframe. Until it succeeds, no
  /rtabmap/localization_pose is published, and ekf_node falls back to
  pure dead-reckoning from wheel odometry. So:
    - Drop the robot in a part of the room that was scanned during
      mapping with reasonable visual variety. White-walls-only spots
      will fail to localize.
    - Watch the rtabmap log for "Localization succeeded" before
      moving the robot.

Usage on each robot Pi:

  ssh pi2@192.168.1.102
  export FASTRTPS_DEFAULT_PROFILES_FILE=/home/pi2/fastdds_peers.xml
  export ROS_DOMAIN_ID=30
  source /opt/ros/humble/setup.bash
  source ~/ros2_ws/install/setup.bash

  ros2 launch swerve_bringup rtabmap_localization.launch.py \\
      robot_id:=tb3_1 \\
      db_path:=~/maps/tb3_1_room.db

Verifying it works (from the laptop):

  ros2 topic hz /tb3_1/slam/pose          # rises to 1-3 Hz once localized
  ros2 topic echo /tb3_1/ekf/odom         # smooth, no jumps
  ros2 run tf2_ros tf2_echo map tb3_1_base_link    # should show robot pose

If localization gets confused (jumps, false matches):
  - Increase Mem/STMSize for more short-term memory
  - Lower RGBD/LoopClosureReextractFeatures false → true to be more
    cautious about loop closures
  - Re-map the problem area with rtabmap_mapping.launch.py
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
    enable_ekf   = LaunchConfiguration('enable_ekf').perform(context).lower() in ('true', '1', 'yes')
    suffix       = f'_{robot_id}'

    base_link  = f'{robot_id}_base_link'
    odom_frame = f'{robot_id}_odom'

    db_path = os.path.expanduser(db_path)
    if not os.path.exists(db_path):
        # Don't fail the launch here — the rtabmap node itself will
        # complain clearly. Just print a heads-up so the user notices.
        print(f'[rtabmap_localization] WARNING: db_path does not exist: '
              f'{db_path}. RTAB-Map will refuse to start in '
              f'localization-only mode without a pre-built database. '
              f'Run rtabmap_mapping.launch.py first.')

    actions = []

    # ── Camera + TF (re-uses oak_camera.launch.py) ───────────────────────
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

    # ── Wheel odometry (skip if started by another launch) ───────────────
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

    # ── EKF — wheel /odom + /slam/pose → /ekf/odom ───────────────────────
    if enable_ekf:
        actions.append(Node(
            package='swerve_formation',
            executable='ekf_node',
            name='ekf_node' + suffix,
            parameters=[{'robot_id': robot_id}],
            output='screen',
        ))

    # ── RTAB-Map (localization-only mode) ────────────────────────────────
    # Differences from rtabmap_mapping.launch.py:
    #   Mem/IncrementalMemory: 'False'   ← read DB, do NOT add new nodes
    #   no --delete_db_on_start argument ← keep the DB
    #   Mem/InitWMWithAllNodes: 'True'   ← load full map into working memory
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
            'queue_size':          30,
            'database_path':       db_path,
            # Localization-only: do NOT add new keyframes, just match.
            'Mem/IncrementalMemory':    'False',
            # Pre-load the entire saved map into RAM at startup so
            # global re-localization can hit any stored keyframe.
            'Mem/InitWMWithAllNodes':   'True',
            # Don't fragment the map if loop closure fails on first frames.
            'RGBD/OptimizeFromGraphEnd': 'True',
            # Localization update rate (only fires after match — fine).
            'RGBD/AngularUpdate':         '0.01',
            'RGBD/LinearUpdate':          '0.01',
        }],
        remappings=[
            ('rgb/image',       f'/{robot_id}/camera/rgb/image_raw'),
            ('rgb/camera_info', f'/{robot_id}/camera/rgb/camera_info'),
            ('depth/image',     f'/{robot_id}/camera/depth/image_raw'),
            ('odom',            f'/{robot_id}/ekf/odom'),
        ],
        # NO --delete_db_on_start.
    ))

    # ── Pose relay: /rtabmap/localization_pose → /{robot_id}/slam/pose ───
    # ekf_node subscribes to /{robot_id}/slam/pose as PoseStamped (see
    # ekf_node.py line 28). RTAB-Map publishes PoseWithCovarianceStamped.
    # This relay strips the covariance and republishes.
    actions.append(Node(
        package='swerve_formation',
        executable='slam_pose_relay_node',
        name='slam_pose_relay' + suffix,
        parameters=[{
            'in_topic':  '/rtabmap/localization_pose',
            'out_topic': f'/{robot_id}/slam/pose',
        }],
        output='screen',
    ))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_id', default_value='tb3_1',
            description='Robot ID (tb3_0, tb3_1, ...).'),
        DeclareLaunchArgument(
            'usb_port', default_value='/dev/ttyACM0',
            description='OpenCR USB-CDC device path.'),
        DeclareLaunchArgument(
            'db_path', default_value='~/maps/tb3_1_room.db',
            description='Path to the rtabmap database (built by '
                        'rtabmap_mapping.launch.py). MUST exist.'),
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
        DeclareLaunchArgument(
            'enable_ekf', default_value='true',
            description='Start ekf_node. Set false if EKF is already '
                        'running from a separate launch.'),
        OpaqueFunction(function=launch_setup),
    ])
