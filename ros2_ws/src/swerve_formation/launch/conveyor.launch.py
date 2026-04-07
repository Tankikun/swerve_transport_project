# swerve_formation/launch/swerve_robot_bringup.launch.py

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    robot_id   = LaunchConfiguration('robot_id',  default='robot_1')
    usb_port   = LaunchConfiguration('usb_port',  default='/dev/ttyACM0')
    k_gain     = LaunchConfiguration('k_gain',    default='1.5')
    start_x    = LaunchConfiguration('start_x',   default='0.0')
    start_y    = LaunchConfiguration('start_y',   default='0.0')

    return LaunchDescription([
        DeclareLaunchArgument('robot_id',  default_value=robot_id),
        DeclareLaunchArgument('usb_port',  default_value=usb_port),
        DeclareLaunchArgument('k_gain',    default_value=k_gain),
        DeclareLaunchArgument('start_x',   default_value=start_x),
        DeclareLaunchArgument('start_y',   default_value=start_y),

        PushRosNamespace(robot_id),

        # --- Node 1: Graph Laplacian Formation Controller ---
        # Subscribes to /virtual_center/cmd_vel and neighbor odom.
        # Publishes corrected cmd_vel for THIS robot.
        Node(
            package='swerve_formation',
            executable='laplacian_formation_node',
            name='formation_controller',
            parameters=[{
                'robot_id': robot_id,
                'neighbors': ['robot_2'],   # hardcode or pass as arg
                'k_gain': k_gain,
            }],
            output='screen',
        ),

        # --- Node 2: Serial Bridge ---
        # Subscribes to /{robot_id}/cmd_vel (from Laplacian node above).
        # Writes "x_dot y_dot gamma_dot\n" to OpenCR over USB serial.
        Node(
            package='swerve_formation',
            executable='serial_bridge_node',
            name='serial_bridge',
            parameters=[{
                'robot_id': robot_id,
                'usb_port': usb_port,
                'baud_rate': 115200,
            }],
            output='screen',
        ),
    ])