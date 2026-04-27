"""
bringup.launch.py
=================
Production launch file for TurtleBot3 Conveyor.
Starts the serial bridge node under a robot namespace.

Usage:
    ros2 launch turtlebot3_conveyor_bridge bringup.launch.py robot_id:=robot1
    ros2 launch turtlebot3_conveyor_bridge bringup.launch.py robot_id:=robot2

Topics created (under namespace):
    /robot1/cmd_vel  ← your synchronization node publishes here
    /robot1/conveyor_serial_bridge/...

The teleop launch (teleop.launch.py) is for manual testing only.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import GroupAction
from launch.conditions import IfCondition


def generate_launch_description():

    # ── Arguments ──────────────────────────────────────────────────────────────
    robot_id_arg = DeclareLaunchArgument(
        'robot_id',
        default_value='robot1',
        description='Robot namespace: robot1 or robot2',
    )
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyACM0',
        description='USB serial port for the OpenCR board',
    )
    teleop_arg = DeclareLaunchArgument(
        'teleop',
        default_value='false',
        description='Set true to also launch keyboard teleop',
    )

    # ── Nodes (wrapped in namespace) ───────────────────────────────────────────
    bringup_group = GroupAction([
        PushRosNamespace(LaunchConfiguration('robot_id')),

        Node(
            package='turtlebot3_conveyor_bridge',
            executable='serial_bridge_node',
            name='conveyor_serial_bridge',
            output='screen',
            parameters=[{
                'serial_port': LaunchConfiguration('serial_port'),
                'baudrate':    115200,
            }],
        ),

        Node(
            package='turtlebot3_conveyor_bridge',
            executable='teleop_keyboard_node',
            name='conveyor_teleop_keyboard',
            output='screen',
            prefix='xterm -e',
            condition=IfCondition(LaunchConfiguration('teleop')),
        ),
    ])

    return LaunchDescription([
        robot_id_arg,
        serial_port_arg,
        teleop_arg,
        bringup_group,
    ])
