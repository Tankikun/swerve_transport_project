"""
mocap_formation.launch.py
-------------------------
Per-robot launch for the 2-robot MoCap accuracy test. Brings up
laplacian_formation_node configured for a side-by-side formation
at the chosen distance D.

Run ONCE on each robot (or in two terminals on the laptop if running
both controllers laptop-side). Pass robot_id and either
formation_position=left|right (default left for tb3_0, right for tb3_1)
or explicit offsets.

Args
----
  robot_id           tb3_0 / tb3_1
  neighbor_id        the OTHER robot (default: derived from robot_id)
  formation_d        Distance between the two robots [m] (default 0.7).
                     Sweep this argument across runs (0.5, 0.7, 0.9).
  formation_position 'left' or 'right' relative to virtual centre,
                     in the formation body frame's +Y axis. Default
                     follows tb3_0=left, tb3_1=right.
  k_gain             consensus gain (default 0.1, off unless
                     enable_consensus is true)
  enable_consensus   pose-feedback term (default false; the trajectory
                     test is feedforward only)

After this launches the laplacian_formation_node, the robot subscribes
to a single shared `/virtual_center/cmd_vel` topic. Whatever is
publishing that topic (the mocap_test/trajectory_publisher.py running
on the laptop) drives the whole formation.

Message flow during the test:

    laptop:                               robot N (each):
    +-----------------------+             +--------------------------+
    | trajectory_publisher  |  Twist      | laplacian_formation_node |
    | (this branch)         | =========>  | (already on main)        |
    +-----------------------+             +--------------------------+
              |                                       |
              |     /virtual_center/cmd_vel           |  /tb3_N/cmd_vel
              |     (single shared topic)             v
                                          +--------------------------+
                                          | conveyor_base_node       |
                                          | (serial -> OpenCR)       |
                                          +--------------------------+

Both robots receive the SAME /virtual_center/cmd_vel and each compute
their own per-robot twist via the rigid-body transform:

    v_robot_x  = vc_vx - vc_wz * my_offset_y
    v_robot_y  = vc_vy + vc_wz * my_offset_x
    v_robot_wz = vc_wz

so the formation maintains itself geometrically.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# Default left/right convention. tb3_0 sits on the formation's +Y side
# (left when the formation faces forward); tb3_1 on the -Y side.
_DEFAULT_NEIGHBOR = {'tb3_0': 'tb3_1', 'tb3_1': 'tb3_0'}
_DEFAULT_POSITION = {'tb3_0': 'left',  'tb3_1': 'right'}


def _resolve(context, *args):
    robot_id    = LaunchConfiguration('robot_id').perform(context)
    neighbor_id = LaunchConfiguration('neighbor_id').perform(context)
    position    = LaunchConfiguration('formation_position').perform(context)
    formation_d = float(LaunchConfiguration('formation_d').perform(context))
    k_gain      = float(LaunchConfiguration('k_gain').perform(context))
    enable_consensus = LaunchConfiguration('enable_consensus').perform(context).lower() in ('1', 'true', 'yes')

    if not neighbor_id:
        neighbor_id = _DEFAULT_NEIGHBOR.get(robot_id, '')
        if not neighbor_id:
            raise RuntimeError(
                f"No default neighbor for robot_id={robot_id!r}. "
                f"Pass neighbor_id:=<other_robot> explicitly."
            )

    if not position:
        position = _DEFAULT_POSITION.get(robot_id, 'left')
    position = position.lower()
    if position not in ('left', 'right'):
        raise RuntimeError(f"formation_position must be 'left' or 'right', got {position!r}")

    half_d = formation_d / 2.0
    # In the formation body frame, +Y is left. "left" robot has +offset,
    # "right" robot has -offset. Side-by-side formation, perpendicular
    # to the direction of motion (no fore-aft offset).
    if position == 'left':
        my_offset       = [0.0, +half_d]
        neighbor_offset = [0.0, -half_d]
    else:
        my_offset       = [0.0, -half_d]
        neighbor_offset = [0.0, +half_d]

    return [Node(
        package='swerve_formation',
        executable='laplacian_formation_node',
        name=f'laplacian_formation_node_{robot_id}',
        parameters=[{
            'robot_id':         robot_id,
            'neighbors':        [neighbor_id],
            'my_offset':        my_offset,
            'neighbor_offsets': neighbor_offset,   # flat list, 2 per neighbor
            'k_gain':           k_gain,
            'enable_consensus': enable_consensus,
        }],
        output='screen',
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_id', default_value='tb3_0',
            description='This robot id (tb3_0 / tb3_1).'),
        DeclareLaunchArgument(
            'neighbor_id', default_value='',
            description='The other robot id. Empty = derived from robot_id.'),
        DeclareLaunchArgument(
            'formation_d', default_value='0.7',
            description='Distance between the two robots [m]. Sweep 0.5, 0.7, 0.9.'),
        DeclareLaunchArgument(
            'formation_position', default_value='',
            description="'left' or 'right' relative to VC. Empty = derived from robot_id."),
        DeclareLaunchArgument(
            'k_gain', default_value='0.1',
            description='Consensus gain (only active when enable_consensus is true).'),
        DeclareLaunchArgument(
            'enable_consensus', default_value='false',
            description='Pose-feedback term. Default false for the open-loop accuracy test.'),
        OpaqueFunction(function=_resolve),
    ])
