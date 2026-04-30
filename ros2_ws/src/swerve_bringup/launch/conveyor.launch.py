"""
conveyor.launch.py
------------------
Bringup for one swerve-drive robot in formation mode.

Launch parameters
  robot_id          str    'tb3_0'
  usb_port          str    '/dev/ttyACM0'
  neighbors         str    'tb3_1'          (comma-separated robot IDs)
  my_offset         str    '0.0,0.3'        (x,y in metres from virtual centre)
  neighbor_offsets  str    '0.0,-0.3'       (flat x0,y0,x1,y1,... same order as neighbors)
  k_gain            float  1.5              (legacy single gain, also sets K_peer)
  K_leader          float  5.0
  K_peer            float  0.5
  offset_init_mode  str    'manual'         ('manual' | 'odom')

Nodes started
  conveyor_base_node      Serial bridge: /{robot_id}/cmd_vel → OpenCR
  ekf_node                Integrates /odom → /{robot_id}/ekf/odom
  alignment_node          Locks formation offsets from initial poses
  laplacian_formation_node  /virtual_center/cmd_vel → /{robot_id}/cmd_vel

Usage (per robot, matching README):
  ros2 launch swerve_bringup conveyor.launch.py \\
    robot_id:=tb3_0 \\
    neighbors:=tb3_1 \\
    my_offset:=0.0,0.3 \\
    neighbor_offsets:=0.0,-0.3 \\
    usb_port:=/dev/ttyACM0 \\
    offset_init_mode:=odom
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ── Declare all parameters ────────────────────────────────────────────────
    robot_id         = LaunchConfiguration('robot_id',         default='tb3_0')
    usb_port         = LaunchConfiguration('usb_port',         default='/dev/ttyACM0')
    neighbors        = LaunchConfiguration('neighbors',        default='tb3_1')
    my_offset        = LaunchConfiguration('my_offset',        default='0.0,0.3')
    neighbor_offsets = LaunchConfiguration('neighbor_offsets', default='0.0,-0.3')
    k_gain           = LaunchConfiguration('k_gain',           default='1.5')
    K_leader         = LaunchConfiguration('K_leader',         default='5.0')
    K_peer           = LaunchConfiguration('K_peer',           default='0.5')
    offset_init_mode = LaunchConfiguration('offset_init_mode', default='manual')

    return LaunchDescription([
        DeclareLaunchArgument('robot_id',         default_value='tb3_0'),
        DeclareLaunchArgument('usb_port',         default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('neighbors',        default_value='tb3_1'),
        DeclareLaunchArgument('my_offset',        default_value='0.0,0.3'),
        DeclareLaunchArgument('neighbor_offsets', default_value='0.0,-0.3'),
        DeclareLaunchArgument('k_gain',           default_value='1.5'),
        DeclareLaunchArgument('K_leader',         default_value='5.0'),
        DeclareLaunchArgument('K_peer',           default_value='0.5'),
        DeclareLaunchArgument('offset_init_mode', default_value='manual'),

        # ── EKF: integrates raw /odom → /{robot_id}/ekf/odom ─────────────────
        # (Without active SLAM, correction step is idle — works as odometry relay)
        Node(
            package='swerve_formation',
            executable='ekf_node',
            name='ekf_node',
            parameters=[{'robot_id': robot_id}],
            output='screen',
        ),

        # ── Serial bridge: /{robot_id}/cmd_vel → OpenCR ───────────────────────
        Node(
            package='swerve_formation',
            executable='conveyor_base_node',
            name='conveyor_base_node',
            parameters=[{
                'robot_id': robot_id,
                'usb_port': usb_port,
                'baud_rate': 115200,
            }],
            output='screen',
        ),

        # ── Alignment: computes/locks formation offsets ───────────────────────
        Node(
            package='swerve_formation',
            executable='alignment_node',
            name='alignment_node',
            parameters=[{
                'robot_id':         robot_id,
                'neighbors':        [neighbors],
                'my_offset':        my_offset,
                'neighbor_offsets': neighbor_offsets,
                'offset_init_mode': offset_init_mode,
            }],
            output='screen',
        ),

        # ── Laplacian formation controller ────────────────────────────────────
        # /virtual_center/cmd_vel → /{robot_id}/cmd_vel
        Node(
            package='swerve_formation',
            executable='laplacian_formation_node',
            name='laplacian_formation_node',
            parameters=[{
                'robot_id':         robot_id,
                'neighbors':        [neighbors],
                'my_offset':        [0.0, 0.0],   # overridden by /formation/offsets
                'neighbor_offsets': [0.0, 0.0],   # overridden by /formation/offsets
                'K_leader':         K_leader,
                'K_peer':           K_peer,
                'k_gain':           k_gain,
                'V_MAX':            0.18,
                'vc_odom_topic':    '/tb3_0/ekf/odom',  # leader's pose = VC
            }],
            output='screen',
        ),
    ])
