"""
rtabmap_laptop_localization.launch.py
-------------------------------------
Laptop-side half of the SPLIT localization architecture. Counterpart
to `rtabmap_laptop_mapping.launch.py` but in **localization-only**
mode against a pre-built .db.

Runs on the laptop:
  rtabmap_slam (localization mode, reads existing .db)
  slam_pose_relay_node (PoseWithCovarianceStamped → PoseStamped for
                         ekf_node on pi2)

The Pi runs `rtabmap_pi_sensors.launch.py` so the camera + odom +
EKF stack is alive. The full feedback loop becomes:

  pi2.camera ──► laptop.rtabmap ──► laptop.slam_pose_relay
        ▲                                    │
        │                                    ▼
        └─── pi2.ekf_node ◄─── /tb3_1/slam/pose ◄──┘
                       │
                       ▼
                 /tb3_1/ekf/odom (drift-corrected)
                       │
                       ▼
              navigation + laplacian use this

Two network hops per cycle (camera up, slam pose down). Latency
adds ~50-100 ms vs the all-on-pi case but the pi stays cool and
the localization quality is the same.

Usage on the laptop:

  source /opt/ros/humble/setup.bash
  source ~/swerve_transport_project/install/setup.bash
  export ROS_DOMAIN_ID=30
  export FASTRTPS_DEFAULT_PROFILES_FILE=/home/toodmuk/fastdds_peers.xml

  ros2 launch swerve_bringup rtabmap_laptop_localization.launch.py \\
      robot_id:=tb3_1 \\
      db_path:=~/maps/tb3_1_room.db

The .db must exist on the laptop. If you mapped on the laptop with
rtabmap_laptop_mapping.launch.py it's already there.

Verification (in another laptop terminal):

  ros2 topic hz /tb3_1/slam/pose      # 1-3 Hz once localised
  ros2 topic echo /tb3_1/ekf/odom     # smooth, drift-corrected

Shared-map requirement (multi-robot):
  When both robots run their own copy of this launch concurrently
  and you intend to use the laplacian consensus correction
  (`enable_consensus:=true` on laplacian_formation_node), every
  robot's launch MUST point `db_path` at the SAME .db file.
  Different .db files mean different `map` frames, which silently
  breaks any inter-robot pose-feedback control. Distribute the same
  .db to every machine that runs a localization launch (the laptop
  in split mode; the Pi in all-on-pi mode), e.g.:
    rsync ~/maps/room.db pi1@192.168.1.101:~/maps/
    rsync ~/maps/room.db pi2@192.168.1.102:~/maps/
"""

import os

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
    if not os.path.exists(db_path):
        print(f'[rtabmap_laptop_localization] WARNING: db_path missing: '
              f'{db_path}. RTAB-Map refuses localization-only without '
              f'a pre-built database. Run rtabmap_laptop_mapping.launch.py '
              f'first.')

    return [
        Node(
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
                # Critical: keep BOTH queues tiny so old frames get
                # dropped instead of accumulating. With sync_queue_size=30
                # (the previous default), the rtabmap log showed steady
                # delay=18.5s — frames were 18 seconds old by the time
                # rtabmap processed them, which made the odom prediction
                # too stale for visual matches to register and the
                # `map → odom` TF correction trailed reality. Tiny queues
                # mean rtabmap always works on near-current frames.
                'topic_queue_size':    1,
                'sync_queue_size':     2,
                'database_path':       db_path,
                # Localization-only.
                'Mem/IncrementalMemory':    'False',
                'Mem/InitWMWithAllNodes':   'True',
                'RGBD/OptimizeFromGraphEnd': 'True',
                'RGBD/AngularUpdate':         '0.01',
                'RGBD/LinearUpdate':          '0.01',
                # Tuning cherry-picked from feature/config-rtab (Tan).
                # See rtabmap_localization.launch.py for per-param
                # rationale; this is the same block applied to the
                # laptop-side rtabmap node so split-mode benefits too.
                'Reg/Force3DoF':              'true',
                'Vis/EstimationType':         '1',
                'Vis/MinInliers':             '15',
                'Kp/DetectorStrategy':        '6',
                'Kp/MaxFeatures':             '400',
                'Mem/ImagePreDecimation':     '2',
                'Mem/DepthDecimation':        '2',
                'Rtabmap/DetectionRate':      '5.0',
                'RGBD/ProximityBySpace':      'false',
                'RGBD/ProximityByTime':       'false',
            }],
            remappings=[
                ('rgb/image',         f'/{robot_id}/camera/rgb/image_raw'),
                ('rgb/camera_info',   f'/{robot_id}/camera/rgb/camera_info'),
                ('depth/image',       f'/{robot_id}/camera/depth/image_raw'),
                ('odom',              f'/{robot_id}/ekf/odom'),
                ('localization_pose', f'/{robot_id}/rtabmap/localization_pose'),
                ('info',              f'/{robot_id}/rtabmap/info'),
                ('mapData',           f'/{robot_id}/rtabmap/mapData'),
                ('cloud_map',         f'/{robot_id}/rtabmap/cloud_map'),
            ],
            # NO --delete_db_on_start.
        ),
        Node(
            package='swerve_formation',
            executable='slam_pose_relay_node',
            name='slam_pose_relay' + suffix,
            parameters=[{
                'in_topic':  f'/{robot_id}/rtabmap/localization_pose',
                'out_topic': f'/{robot_id}/slam/pose',
            }],
            output='screen',
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_id', default_value='tb3_1',
            description='Robot ID whose camera + odom we subscribe to.'),
        DeclareLaunchArgument(
            'db_path', default_value='~/maps/tb3_1_room.db',
            description='Path to .db on the laptop. MUST exist.'),
        OpaqueFunction(function=launch_setup),
    ])
