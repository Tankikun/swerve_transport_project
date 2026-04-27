from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyACM0',
        description='USB serial port for the OpenCR board (e.g. /dev/ttyACM0)',
    )
    baudrate_arg = DeclareLaunchArgument(
        'baudrate',
        default_value='115200',
        description='Serial baudrate — must match OpenCR firmware (115200)',
    )

    serial_bridge = Node(
        package='turtlebot3_conveyor_bridge',
        executable='serial_bridge_node',
        name='conveyor_serial_bridge',
        output='screen',
        parameters=[{
            'serial_port':        LaunchConfiguration('serial_port'),
            'baudrate':           LaunchConfiguration('baudrate'),
            'linear_threshold':   0.05,
            'angular_threshold':  0.05,
        }],
    )

    teleop_keyboard = Node(
        package='turtlebot3_conveyor_bridge',
        executable='teleop_keyboard_node',
        name='conveyor_teleop_keyboard',
        output='screen',
        # prefix='' keeps it in the same terminal; use 'xterm -e' for a popup window
        prefix='',
    )

    return LaunchDescription([
        serial_port_arg,
        baudrate_arg,
        serial_bridge,
        teleop_keyboard,
    ])
