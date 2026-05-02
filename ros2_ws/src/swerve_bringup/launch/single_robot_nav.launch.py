"""
single_robot_nav.launch.py
--------------------------
Tier-1 hardware test for navigation_node: ONE real robot, NO laplacian,
NO second robot, NO camera, NO SLAM.

What runs:
  conveyor_base_node       — serial bridge to OpenCR
                              subscribes  /{robot_id}/cmd_vel
                              publishes   /{robot_id}/odom (encoder-based)
  ekf_node                  — fuses /{robot_id}/odom only (dead-reckoning,
                              no SLAM corrections — pose will drift but
                              that's OK for short tests)
                              publishes  /{robot_id}/ekf/odom
  navigation_node           — APF + velocity ramp
                              subscribes  /{robot_id}/ekf/odom
                              publishes   /virtual_center/cmd_vel
  static_leader_publisher   — pins this robot as leader so nav activates
  topic_relay (cmd_vel)     — copies /virtual_center/cmd_vel to
                              /{robot_id}/cmd_vel because we are skipping
                              laplacian (only one robot, no formation math
                              needed). For Tier 2 use conveyor.launch.py
                              which runs laplacian_formation_node instead.

Usage on the robot Pi:
  ros2 launch swerve_bringup single_robot_nav.launch.py \\
       robot_id:=tb3_1 usb_port:=/dev/ttyACM0

Sending a goal from the laptop:
  ros2 run swerve_formation send_goal_node --ros-args -p x:=1.0 -p y:=0.0

Watching:
  ros2 topic echo /navigation/status
  ros2 topic echo /{robot_id}/ekf/odom
  ros2 topic echo /{robot_id}/cmd_vel

Safety notes:
  - GOAL_TOL defaults to 0.15 m for hardware (vs 0.05 m in sim) because
    wheel slip + dead-reckoning drift makes a 5 cm tolerance impossible.
  - First test: small forward goal (x:=1.0 y:=0.0). Mark start with tape.
  - If the robot drifts way off heading, kill it (Ctrl+C the launch)
    and check /{robot_id}/ekf/odom theta — wheel-odometry yaw drift is
    the single biggest unknown until we wire IMU into the EKF.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    robot_id  = LaunchConfiguration('robot_id').perform(context)
    usb_port  = LaunchConfiguration('usb_port').perform(context)
    goal_tol  = float(LaunchConfiguration('goal_tolerance').perform(context))
    suffix    = f'_{robot_id}'

    return [
        # ── Serial bridge to OpenCR ──────────────────────────────────────
        Node(
            package='swerve_formation',
            executable='conveyor_base_node',
            name='conveyor_base_node' + suffix,
            parameters=[{
                'robot_id':  robot_id,
                'usb_port':  usb_port,
                'baud_rate': 115200,
            }],
            output='screen',
        ),

        # ── EKF (prediction-only mode — no SLAM yet) ─────────────────────
        Node(
            package='swerve_formation',
            executable='ekf_node',
            name='ekf_node' + suffix,
            parameters=[{'robot_id': robot_id}],
            output='screen',
        ),

        # ── Navigation node (APF + velocity ramp) ────────────────────────
        Node(
            package='swerve_formation',
            executable='navigation_node',
            name='navigation_node' + suffix,
            parameters=[{
                'robot_id':       robot_id,
                'goal_tolerance': goal_tol,
            }],
            output='screen',
        ),

        # ── Static leader so nav activates immediately ───────────────────
        Node(
            package='swerve_formation',
            executable='static_leader_publisher',
            name='static_leader_publisher' + suffix,
            parameters=[{'leader_id': robot_id}],
            output='screen',
        ),

        # ── Bypass laplacian: relay /virtual_center/cmd_vel → /{id}/cmd_vel
        # Single-robot, no formation math, no offsets to apply. Uses the
        # standard topic_tools relay (already shipped with ROS Humble).
        Node(
            package='topic_tools',
            executable='relay',
            name='cmd_vel_relay' + suffix,
            arguments=['/virtual_center/cmd_vel', f'/{robot_id}/cmd_vel'],
            output='screen',
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_id', default_value='tb3_0',
            description='Robot ID / namespace (tb3_0 or tb3_1).'),
        DeclareLaunchArgument(
            'usb_port', default_value='/dev/ttyACM0',
            description='OpenCR USB-CDC device path.'),
        DeclareLaunchArgument(
            'goal_tolerance', default_value='0.15',
            description='Distance (m) within which a goal counts as REACHED. '
                        'Default 0.15 m for hardware (sim uses 0.05 m).'),
        OpaqueFunction(function=launch_setup),
    ])
