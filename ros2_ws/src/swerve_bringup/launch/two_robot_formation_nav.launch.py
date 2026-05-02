"""
two_robot_formation_nav.launch.py
---------------------------------
Tier-2 hardware test: TWO real robots in formation, driven by APF
navigation. No camera, no SLAM, no alignment_node — only the verified
core of the stack.

What runs ON EACH robot:

  conveyor_base_node       — serial bridge to OpenCR
                              subscribes  /{robot_id}/cmd_vel
                              publishes   /{robot_id}/odom
  ekf_node                  — dead-reckoning EKF
                              publishes   /{robot_id}/ekf/odom
  laplacian_formation_node  — rigid-body feedforward; converts the
                              virtual-centre twist into THIS robot's
                              body twist using my_offset
                              subscribes /virtual_center/cmd_vel
                              publishes  /{robot_id}/cmd_vel
  navigation_node           — APF + velocity ramp; active only when
                              /formation/leader == this robot_id
                              publishes  /virtual_center/cmd_vel
  static_leader_publisher   — pins one robot as the leader so nav
                              activates immediately. Replace with
                              leader_election_node once that node is
                              hardware-verified.

Note we deliberately omit slam_3d_node, ai_camera_node, alignment_node,
formation_size_node — those are camera-dependent stubs that will crash
or block on hardware that has no working camera pipeline yet.

Per-robot launch (run on EACH Pi):

  pi1 (left, tb3_0, leader):
    ros2 launch swerve_bringup two_robot_formation_nav.launch.py \
      robot_id:=tb3_0 usb_port:=/dev/ttyACM0 \
      neighbors:=tb3_1 \
      my_offset:=0.0,0.15 \
      neighbor_offsets:=0.0,-0.15 \
      leader_id:=tb3_0

  pi2 (right, tb3_1, follower):
    ros2 launch swerve_bringup two_robot_formation_nav.launch.py \
      robot_id:=tb3_1 usb_port:=/dev/ttyACM0 \
      neighbors:=tb3_0 \
      my_offset:=0.0,-0.15 \
      neighbor_offsets:=0.0,0.15 \
      leader_id:=tb3_0

Sending a formation goal from the laptop:

  ros2 run swerve_formation send_goal_node --ros-args -p x:=1.0 -p y:=0.0

Both robots should drive forward together, maintaining their 30 cm
side-by-side spacing.

Safety:
  - holonomic_mode default true so the formation does NOT rotate to
    face the motion direction (would tear the formation apart).
  - goal_tolerance default 0.15 m for hardware (sim 0.05 m).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _parse_xy_list(s: str):
    """Parse 'x,y' or 'x1,y1;x2,y2;...' into a flat list of floats."""
    out = []
    s = (s or '').strip()
    if not s:
        return out
    for pair in s.split(';'):
        pair = pair.strip()
        if not pair:
            continue
        for v in pair.split(','):
            v = v.strip()
            if v:
                out.append(float(v))
    return out


def _parse_neighbors(s: str):
    s = (s or '').strip()
    return [n.strip() for n in s.split(',') if n.strip()] if s else []


def launch_setup(context, *args, **kwargs):
    robot_id          = LaunchConfiguration('robot_id').perform(context)
    usb_port          = LaunchConfiguration('usb_port').perform(context)
    leader_id         = LaunchConfiguration('leader_id').perform(context)
    goal_tol          = float(LaunchConfiguration('goal_tolerance').perform(context))
    heading_tol       = float(LaunchConfiguration('heading_tolerance').perform(context))
    holonomic         = LaunchConfiguration('holonomic_mode').perform(context).lower() in ('true', '1', 'yes')
    neighbors         = _parse_neighbors(LaunchConfiguration('neighbors').perform(context))
    my_offset         = _parse_xy_list(LaunchConfiguration('my_offset').perform(context))
    neighbor_offsets  = _parse_xy_list(LaunchConfiguration('neighbor_offsets').perform(context))

    if not my_offset:
        my_offset = [0.0, 0.0]
    if not neighbor_offsets:
        neighbor_offsets = [0.0, 0.0] * max(1, len(neighbors))
    if not neighbors:
        neighbors = ['tb3_1' if robot_id == 'tb3_0' else 'tb3_0']

    suffix = f'_{robot_id}'

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

        # ── EKF (dead-reckoning until SLAM lands) ────────────────────────
        Node(
            package='swerve_formation',
            executable='ekf_node',
            name='ekf_node' + suffix,
            parameters=[{'robot_id': robot_id}],
            output='screen',
        ),

        # ── Laplacian formation controller ───────────────────────────────
        # Subscribes /virtual_center/cmd_vel, applies rigid-body
        # transform with my_offset, publishes /{robot_id}/cmd_vel.
        Node(
            package='swerve_formation',
            executable='laplacian_formation_node',
            name='laplacian_formation_node' + suffix,
            parameters=[{
                'robot_id':         robot_id,
                'k_gain':           0.0,    # pure feedforward
                'neighbors':        neighbors,
                'my_offset':        my_offset,
                'neighbor_offsets': neighbor_offsets,
            }],
            output='screen',
        ),

        # ── Navigation node ──────────────────────────────────────────────
        # Runs on every robot, activates only on the elected leader.
        Node(
            package='swerve_formation',
            executable='navigation_node',
            name='navigation_node' + suffix,
            parameters=[{
                'robot_id':          robot_id,
                'goal_tolerance':    goal_tol,
                'holonomic_mode':    holonomic,
                'heading_tolerance': heading_tol,
            }],
            output='screen',
        ),

        # ── Static leader publisher ──────────────────────────────────────
        # Replaces leader_election_node until that's hardware-verified.
        # Runs on every robot but they all publish the SAME leader_id,
        # so there's no contention.
        Node(
            package='swerve_formation',
            executable='static_leader_publisher',
            name='static_leader_publisher' + suffix,
            parameters=[{'leader_id': leader_id}],
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
            'leader_id', default_value='tb3_0',
            description='Which robot acts as formation leader.'),
        DeclareLaunchArgument(
            'neighbors', default_value='',
            description='Comma-separated neighbor IDs, e.g. tb3_1.'),
        DeclareLaunchArgument(
            'my_offset', default_value='0.0,0.0',
            description='This robot offset (x,y in m) from virtual centre.'),
        DeclareLaunchArgument(
            'neighbor_offsets', default_value='0.0,0.0',
            description='Neighbor offsets, semicolon-separated, e.g. '
                        '"0.0,-0.15".'),
        DeclareLaunchArgument(
            'goal_tolerance', default_value='0.15',
            description='Position tolerance (m) for REACHED.'),
        DeclareLaunchArgument(
            'heading_tolerance', default_value='3.14159',
            description='Heading tolerance (rad) for REACHED. Default π '
                        '(disabled — position only).'),
        DeclareLaunchArgument(
            'holonomic_mode', default_value='true',
            description='True for swerve formation transport (default). '
                        'False for diff-drive style heading alignment.'),
        OpaqueFunction(function=launch_setup),
    ])
