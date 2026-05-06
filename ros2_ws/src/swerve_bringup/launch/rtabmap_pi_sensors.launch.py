"""
rtabmap_pi_sensors.launch.py
----------------------------
Pi2-side half of the SPLIT mapping/localization architecture.

Runs the **sensor stack only** on the Pi 4: camera, TF, wheel
odometry, EKF. Does **NOT** run rtabmap_slam itself — that runs on
the laptop (`rtabmap_laptop_mapping.launch.py` /
`rtabmap_laptop_localization.launch.py`) so the Pi doesn't have to
do CPU-heavy visual feature extraction + graph optimisation.

Why split:
  rtabmap_slam under sustained mapping pushes the Pi 4 to 70-80 °C
  and risks thermal throttling. Image streams + odom + tf compress
  to ~5-10 MB/s over the LAN — well within budget — and the laptop
  has ~10x the spare compute for visual SLAM.

What runs on the Pi:
  oak_camera.launch.py          — camera + base_link↔optical TF
  conveyor_base_node            — wheel odometry from OpenCR,
                                   publishes /{robot_id}/odom
  ekf_node                      — fuses /odom + /slam/pose,
                                   publishes /{robot_id}/ekf/odom

What the laptop subscribes to (across WiFi/LAN):
  /{robot_id}/camera/rgb/image_raw
  /{robot_id}/camera/rgb/camera_info
  /{robot_id}/camera/depth/image_raw
  /{robot_id}/ekf/odom
  /tf, /tf_static

Usage on pi2:

  ssh pi2@192.168.1.102
  export FASTRTPS_DEFAULT_PROFILES_FILE=/home/pi2/fastdds_peers.xml
  export ROS_DOMAIN_ID=30
  source /opt/ros/humble/setup.bash
  source ~/ros2_ws/install/setup.bash

  ros2 launch swerve_bringup rtabmap_pi_sensors.launch.py \\
      robot_id:=tb3_1 \\
      cam_x:=0.128 cam_y:=0.000 cam_z:=-0.0175

Then on the laptop, in a separate terminal:

  ros2 launch swerve_bringup rtabmap_laptop_mapping.launch.py \\
      robot_id:=tb3_1 \\
      db_path:=~/maps/tb3_1_room.db

Bandwidth note: the cross-machine RGB rate we measured in the
all-on-pi setup was ~2.4 Hz over FastDDS unicast (network limited,
not Pi limited). For careful slow mapping that's adequate. If
frame rate is too low, lower fps and rgb resolution in the camera
launch args, or add image_transport compressed plugin support to
`oak_camera_node` later.
"""

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
    robot_id    = LaunchConfiguration('robot_id').perform(context)
    usb_port    = LaunchConfiguration('usb_port').perform(context)
    fps         = LaunchConfiguration('fps').perform(context)
    enable_base = LaunchConfiguration('enable_base').perform(context).lower() in ('true', '1', 'yes')
    enable_ekf  = LaunchConfiguration('enable_ekf').perform(context).lower() in ('true', '1', 'yes')
    suffix      = f'_{robot_id}'

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

    # ── EKF — wheel /odom (+ /slam/pose later, in localization) → /ekf/odom
    if enable_ekf:
        actions.append(Node(
            package='swerve_formation',
            executable='ekf_node',
            name='ekf_node' + suffix,
            parameters=[{'robot_id': robot_id}],
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
            description='Start conveyor_base_node.'),
        DeclareLaunchArgument(
            'enable_ekf', default_value='true',
            description='Start ekf_node.'),
        OpaqueFunction(function=launch_setup),
    ])
