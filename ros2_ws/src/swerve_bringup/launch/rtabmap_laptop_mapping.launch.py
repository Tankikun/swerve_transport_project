"""
rtabmap_laptop_mapping.launch.py
--------------------------------
Laptop-side half of the SPLIT mapping architecture. Runs ONLY
the rtabmap_slam node in mapping mode. The Pi is expected to
already be running `rtabmap_pi_sensors.launch.py` so the camera +
odom + TF topics are flowing on the network.

Why split:
  rtabmap_slam under sustained mapping pushes the Pi 4 to 70-80 °C
  and risks thermal throttling. The laptop has ~10x the spare CPU
  for visual SLAM, and the .db ends up on the laptop where it can
  be inspected with the rtabmap GUI without any extra copying.

Topology:

  pi2 (sensors):              laptop (this launch):
    oak_camera_node             rtabmap_slam (mapping mode)
    conveyor_base_node          → ~/maps/{robot_id}_room.db
    ekf_node
            └────── /tb3_1/camera/...,
                    /tb3_1/ekf/odom,
                    /tf, /tf_static     ─────┘

Usage on the laptop:

  source /opt/ros/humble/setup.bash
  source ~/swerve_transport_project/install/setup.bash
  export ROS_DOMAIN_ID=30
  export FASTRTPS_DEFAULT_PROFILES_FILE=/home/toodmuk/fastdds_peers.xml
  mkdir -p ~/maps

  ros2 launch swerve_bringup rtabmap_laptop_mapping.launch.py \\
      robot_id:=tb3_1 \\
      db_path:=~/maps/tb3_1_room.db

Pre-flight checks (run on the laptop after pi2 sensors are up):

  ros2 topic hz /tb3_1/camera/rgb/image_raw    # should be > 1 Hz
  ros2 topic hz /tb3_1/ekf/odom                # should be > 5 Hz
  ros2 run tf2_ros tf2_echo tb3_1_odom tb3_1_oak_rgb_camera_optical_frame
                                                # should resolve

If the camera rate is too low for good mapping, see the bandwidth
note in `rtabmap_pi_sensors.launch.py` — main mitigation is to
drive slower, drop to 10 fps, or add image_transport compressed
support to oak_camera_node.

Once mapping is finished and the .db is on the laptop:

  # Optional: copy .db back to pi2 for runtime localization
  scp ~/maps/tb3_1_room.db pi2@192.168.1.102:~/maps/

  # Inspect the map (laptop, GUI):
  rtabmap-databaseViewer ~/maps/tb3_1_room.db
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    robot_id = LaunchConfiguration('robot_id').perform(context)
    db_path  = LaunchConfiguration('db_path').perform(context)
    suffix   = f'_{robot_id}'

    base_link  = f'{robot_id}_base_link'
    odom_frame = f'{robot_id}_odom'

    db_path = os.path.expanduser(db_path)
    os.makedirs(os.path.dirname(db_path) or '.', exist_ok=True)

    config_path = os.path.join(
        get_package_share_directory('swerve_bringup'), 'config', 'rtabmap_mapping.yaml'
    )

    return [
        Node(
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
                # Raw wheel odom during mapping — there is no SLAM
                # correction yet, and the {robot_id}_odom→base_link TF
                # (from conveyor_base_node on the Pi) is also raw, so
                # topic and TF agree. Localization-mode launches use
                # /ekf/odom instead.
                ('odom',              f'/{robot_id}/odom'),
                # Per-robot scoping (consistent with the on-pi launches).
                ('localization_pose', f'/{robot_id}/rtabmap/localization_pose'),
                ('info',              f'/{robot_id}/rtabmap/info'),
                ('mapData',           f'/{robot_id}/rtabmap/mapData'),
                ('cloud_map',         f'/{robot_id}/rtabmap/cloud_map'),
            ],
            arguments=['--delete_db_on_start'],   # remove to APPEND to existing db
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_id', default_value='tb3_1',
            description='Robot ID whose camera + odom we subscribe to.'),
        DeclareLaunchArgument(
            'db_path', default_value='~/maps/tb3_1_room.db',
            description='Where to save the map database (on the laptop).'),
        OpaqueFunction(function=launch_setup),
    ])
