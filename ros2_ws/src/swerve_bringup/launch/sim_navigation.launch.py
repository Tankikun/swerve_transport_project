"""
sim_navigation.launch.py
------------------------
Simulation-only launch for testing navigation_node + fake_swerve_simulator
WITHOUT any physical hardware.

What runs:
  fake_swerve_robot (tb3_0) — virtual-center mode
    └─ subscribes  /virtual_center/cmd_vel
    └─ publishes   /tb3_0/odom  and  /tb3_0/ekf/odom

  navigation_node (tb3_0, elected leader)
    └─ reads       /tb3_0/ekf/odom
    └─ reads       /formation/leader  (published by static_leader below)
    └─ reads       /navigation/goal   (send from CLI — see tools/send_goal.py)
    └─ publishes   /virtual_center/cmd_vel

  static_leader_publisher — tiny node that constantly announces tb3_0 as leader

Usage:
  ros2 launch swerve_bringup sim_navigation.launch.py

Then in another terminal, send a goal:
  ros2 run swerve_formation send_goal_node --ros-args -p x:=4.5 -p y:=3.5

Or with obstacles:
  ros2 topic pub /navigation/obstacles geometry_msgs/PoseArray \\
    '{poses: [{position: {x: 2.2, y: 1.8, z: 0.35}},
               {position: {x: 3.2, y: 2.8, z: 0.30}}]}'

Watch in RViz2:
  - Fixed frame: odom
  - Add by topic: /tb3_0/odom (Odometry), /tb3_0/marker (Marker)
  - Add by topic: /navigation/status (String — shows IDLE/NAVIGATING/REACHED)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    leader_id = LaunchConfiguration('leader_id', default='tb3_0')
    start_x   = LaunchConfiguration('start_x',   default='0.5')
    start_y   = LaunchConfiguration('start_y',   default='0.5')

    return LaunchDescription([
        DeclareLaunchArgument('leader_id', default_value=leader_id,
                              description='Robot ID that acts as formation leader'),
        DeclareLaunchArgument('start_x', default_value=start_x,
                              description='Simulation start X position (m)'),
        DeclareLaunchArgument('start_y', default_value=start_y,
                              description='Simulation start Y position (m)'),

        # ── Fake robot: subscribes to /virtual_center/cmd_vel ─────────────────
        # use_virtual_center=True: robot position IS the virtual center
        # (no laplacian node needed for this single-robot navigation test)
        Node(
            package='swerve_formation',
            executable='fake_swerve_simulator',
            name='fake_swerve_tb3_0',
            parameters=[{
                'robot_id':            'tb3_0',
                'start_x':             start_x,
                'start_y':             start_y,
                'start_theta':         0.0,
                'use_virtual_center':  True,   # bypass laplacian, test nav directly
            }],
            output='screen',
        ),

        # ── Navigation node (APF + velocity ramp) ────────────────────────────
        Node(
            package='swerve_formation',
            executable='navigation_node',
            name='navigation_node',
            parameters=[{
                'robot_id': 'tb3_0',
            }],
            output='screen',
        ),

        # ── Static leader publisher ───────────────────────────────────────────
        # Replaces leader_election_node in simulation — just says tb3_0 is leader
        Node(
            package='swerve_formation',
            executable='static_leader_publisher',
            name='static_leader_publisher',
            parameters=[{'leader_id': leader_id}],
            output='screen',
        ),
    ])
