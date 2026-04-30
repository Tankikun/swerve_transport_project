"""
sim_formation.launch.py
-----------------------
Full 2-robot formation simulation — no physical hardware needed.

Tests the complete chain:
  navigation_node (APF + ramp)
       └→ /virtual_center/cmd_vel
              └→ laplacian_formation_node (per robot)
                       └→ /tb3_X/cmd_vel
                              └→ fake_swerve_simulator (per robot)
                                       └→ /tb3_X/odom + /tb3_X/ekf/odom
                                              └→ back to navigation + laplacian

Convention used here: leader's pose IS the Virtual Center pose.
  Leader (tb3_0)   own_offset = (0, 0)        — at VC
  Follower (tb3_1) own_offset = (-0.8, 0)     — 0.8 m behind leader
  Peer offsets are simply each other's own_offset.

Usage:
  ros2 launch swerve_bringup sim_formation.launch.py

  # Send a goal in another terminal:
  ros2 run swerve_formation send_goal_node --ros-args -p x:=4.5 -p y:=3.5

  # Watch formation:
  ros2 topic echo /tb3_0/ekf/odom --field pose.pose.position
  ros2 topic echo /tb3_1/ekf/odom --field pose.pose.position

Expected behaviour:
  - tb3_0 navigates toward goal using APF
  - tb3_1 maintains 0.8 m gap behind tb3_0 throughout the path
  - Both stop within tolerance of goal
"""

from launch import LaunchDescription
from launch_ros.actions import Node


SEPARATION = 0.80   # metres between the two robots


def generate_launch_description():
    return LaunchDescription([

        # ── Fake robots ──────────────────────────────────────────────────────
        Node(
            package='swerve_formation',
            executable='fake_swerve_simulator',
            name='fake_tb3_0',
            parameters=[{
                'robot_id':           'tb3_0',
                'start_x':             0.5,                 # leader at (0.5, 0.5)
                'start_y':             0.5,
                'start_theta':         0.0,
                'use_virtual_center':  False,               # listens to /tb3_0/cmd_vel
            }],
            output='screen',
        ),
        Node(
            package='swerve_formation',
            executable='fake_swerve_simulator',
            name='fake_tb3_1',
            parameters=[{
                'robot_id':           'tb3_1',
                'start_x':             0.5 - SEPARATION,    # follower 0.8 m behind leader
                'start_y':             0.5,
                'start_theta':         0.0,
                'use_virtual_center':  False,
            }],
            output='screen',
        ),

        # ── Static leader announcement ──────────────────────────────────────
        Node(
            package='swerve_formation',
            executable='static_leader_publisher',
            name='static_leader_publisher',
            parameters=[{'leader_id': 'tb3_0'}],
            output='screen',
        ),

        # ── Navigation (runs on leader; activates only when elected) ────────
        Node(
            package='swerve_formation',
            executable='navigation_node',
            name='navigation_node',
            parameters=[{'robot_id': 'tb3_0'}],
            output='screen',
        ),

        # ── Laplacian formation controller for LEADER (tb3_0) ───────────────
        # Leader's own_offset from VC = (0, 0) since leader IS the VC.
        # Peer is tb3_1 at offset (-0.8, 0).
        Node(
            package='swerve_formation',
            executable='laplacian_formation_node',
            name='laplacian_tb3_0',
            parameters=[{
                'robot_id':         'tb3_0',
                'neighbors':        ['tb3_1'],
                'my_offset':        [0.0, 0.0],
                'neighbor_offsets': [-SEPARATION, 0.0],
                'K_leader':         5.0,
                'K_peer':           0.5,
                'V_MAX':            0.18,
                'vc_odom_topic':    '/tb3_0/ekf/odom',
            }],
            output='screen',
        ),

        # ── Laplacian formation controller for FOLLOWER (tb3_1) ─────────────
        # Follower's own_offset = (-0.8, 0) — 0.8 m behind VC (=leader).
        # Peer is tb3_0 at offset (0, 0).
        Node(
            package='swerve_formation',
            executable='laplacian_formation_node',
            name='laplacian_tb3_1',
            parameters=[{
                'robot_id':         'tb3_1',
                'neighbors':        ['tb3_0'],
                'my_offset':        [-SEPARATION, 0.0],
                'neighbor_offsets': [0.0, 0.0],
                'K_leader':         5.0,
                'K_peer':           0.5,
                'V_MAX':            0.18,
                'vc_odom_topic':    '/tb3_0/ekf/odom',
            }],
            output='screen',
        ),
    ])
