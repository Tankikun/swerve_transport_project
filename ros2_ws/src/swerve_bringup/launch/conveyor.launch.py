from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    robot_id  = LaunchConfiguration('robot_id',  default='tb3_0')
    usb_port  = LaunchConfiguration('usb_port',  default='/dev/ttyACM0')
    k_gain    = LaunchConfiguration('k_gain',    default='1.5')

    return LaunchDescription([
        DeclareLaunchArgument('robot_id',  default_value=robot_id),
        DeclareLaunchArgument('usb_port',  default_value=usb_port),
        DeclareLaunchArgument('k_gain',    default_value=k_gain),

        # Graph Laplacian formation controller
        Node(
            package='swerve_formation',
            executable='laplacian_formation_node',
            name='laplacian_formation_node',
            parameters=[{
                'robot_id': robot_id,
                'k_gain': k_gain,
            }],
            output='screen',
        ),

        # Lifecycle serial bridge → OpenCR
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

        # EKF — fuses raw /odom + SLAM pose; sole consumer of raw odometry
        Node(
            package='swerve_formation',
            executable='ekf_node',
            name='ekf_node',
            parameters=[{'robot_id': robot_id}],
            output='screen',
        ),

        # SLAM (stub; replace with rtabmap_ros launch integration)
        Node(
            package='swerve_formation',
            executable='3d_slam_node',
            name='3d_slam_node',
            parameters=[{'robot_id': robot_id}],
            output='screen',
        ),

        # Leader election — bully algorithm, scales to 3+ robots
        Node(
            package='swerve_formation',
            executable='leader_election_node',
            name='leader_election_node',
            parameters=[{'robot_id': robot_id}],
            output='screen',
        ),

        # Navigation — only the elected leader drives /virtual_center/cmd_vel
        Node(
            package='swerve_formation',
            executable='navigation_node',
            name='navigation_node',
            parameters=[{'robot_id': robot_id}],
            output='screen',
        ),

        # Formation size (leader-only) — computes bounding envelope
        Node(
            package='swerve_formation',
            executable='formation_size_node',
            name='formation_size_node',
            parameters=[{'robot_id': robot_id}],
            output='screen',
        ),

        # OAK-D Lite camera pipeline (degrades gracefully without hardware)
        Node(
            package='swerve_formation',
            executable='ai_camera_node',
            name='ai_camera_node',
            parameters=[{'robot_id': robot_id}],
            output='screen',
        ),
    ])
