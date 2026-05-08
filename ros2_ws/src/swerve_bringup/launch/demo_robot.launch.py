"""
demo_robot.launch.py
--------------------
Per-robot launch for the markerless / mapless two-robot transport demo
(branch `feature/depth-obstacle-avoid`).

What this is, in one breath: the proven feed-forward formation control
from `feature/two-robot-test-seven` plus depth-based obstacle avoidance
on the leader. No EKF, no RTAB-Map, no leader-election, no markers, no
shared map. Two robots holding a payload, formation kinematics
guaranteed by the rigid-body Laplacian feed-forward, leader's OAK-D
modifies `/virtual_center/cmd_vel` to swerve around obstacles.

Run on EACH Pi (one as leader, one as follower):

    # Leader (tb3_1, robot on the right with the OAK-D)
    ros2 launch swerve_bringup demo_robot.launch.py \\
        robot_id:=tb3_1 is_leader:=true \\
        my_offset:=0.0,-0.25 neighbors:=tb3_0 neighbor_offsets:=0.0,0.25 \\
        usb_port:=/dev/ttyACM0 \\
        cam_x:=0.128 cam_y:=0.0 cam_z:=-0.0175

    # Follower (tb3_0, robot on the left)
    ros2 launch swerve_bringup demo_robot.launch.py \\
        robot_id:=tb3_0 is_leader:=false \\
        my_offset:=0.0,0.25 neighbors:=tb3_1 neighbor_offsets:=0.0,-0.25 \\
        usb_port:=/dev/ttyACM0

Then on the laptop, send a goal:

    ros2 run swerve_formation goal_driver_node --ros-args \\
        -p leader_robot_id:=tb3_1 -p goal_distance_m:=2.5

What runs:

  ALWAYS (both robots):
    * conveyor_base_node    — lifecycle serial bridge to OpenCR
    * laplacian_formation_node — pure feedforward (consensus OFF)

  LEADER ONLY (is_leader:=true):
    * oak_camera.launch.py  — depthai_ros_driver Camera component +
                              base_link → optical static TF
    * obstacle_avoidance_node — modifies /virtual_center/cmd_vel based
                              on the depth swathe in front

What is intentionally NOT run:

  * ekf_node, rtabmap_*, slam_pose_relay_node — no SLAM in this demo
  * leader_election_node — leadership is hardcoded via is_leader
  * navigation_node, formation_size_node, alignment_node — replaced
    by goal_driver_node + obstacle_avoidance_node, both far simpler

If you need to test the formation in free air (no obstacle avoidance),
launch with is_leader:=false on both robots and remap the operator
twist directly:

    ros2 run teleop_twist_keyboard teleop_twist_keyboard \\
        --ros-args -r cmd_vel:=/virtual_center/cmd_vel
"""

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _parse_xy_list(s: str):
    """Parse 'x,y' or 'x1,y1;x2,y2;...' into a flat list [x1, y1, x2, y2, ...]."""
    out = []
    s = (s or '').strip()
    if not s:
        return out
    for pair in s.split(';'):
        pair = pair.strip()
        if not pair:
            continue
        parts = [p.strip() for p in pair.split(',') if p.strip()]
        out.extend(float(p) for p in parts)
    return out


def _parse_neighbors(s: str):
    s = (s or '').strip()
    if not s:
        return []
    return [n.strip() for n in s.split(',') if n.strip()]


def _truthy(s: str) -> bool:
    return str(s).strip().lower() in ('true', '1', 'yes', 'on')


def launch_setup(context, *args, **kwargs):
    robot_id          = LaunchConfiguration('robot_id').perform(context)
    is_leader_str     = LaunchConfiguration('is_leader').perform(context)
    is_leader         = _truthy(is_leader_str)
    usb_port          = LaunchConfiguration('usb_port').perform(context)
    neighbors         = _parse_neighbors(LaunchConfiguration('neighbors').perform(context))
    my_offset         = _parse_xy_list(LaunchConfiguration('my_offset').perform(context))
    neighbor_offsets  = _parse_xy_list(LaunchConfiguration('neighbor_offsets').perform(context))
    fps               = LaunchConfiguration('fps').perform(context)
    cam_x             = LaunchConfiguration('cam_x').perform(context)
    cam_y             = LaunchConfiguration('cam_y').perform(context)
    cam_z             = LaunchConfiguration('cam_z').perform(context)
    avoid_range_mm    = LaunchConfiguration('avoid_range_mm').perform(context)
    lateral_gain      = LaunchConfiguration('lateral_gain').perform(context)

    if not my_offset:
        my_offset = [0.0, 0.0]
    if not neighbors:
        neighbors = ['tb3_1' if robot_id == 'tb3_0' else 'tb3_0']
    if not neighbor_offsets:
        neighbor_offsets = [0.0, 0.0] * len(neighbors)

    suffix = f'_{robot_id}'

    actions = [
        # Lifecycle serial bridge → OpenCR. Self-transitions to active in main().
        Node(
            package='swerve_formation',
            executable='conveyor_base_node',
            name='conveyor_base_node' + suffix,
            parameters=[{
                'robot_id': robot_id,
                'usb_port': usb_port,
                'baud_rate': 115200,
            }],
            output='screen',
        ),

        # Pure-FF formation controller. enable_consensus stays False
        # (default) so this node runs without an EKF / SLAM stack and
        # just does the rigid-body feedforward from /virtual_center/cmd_vel.
        Node(
            package='swerve_formation',
            executable='laplacian_formation_node',
            name='laplacian_formation_node' + suffix,
            parameters=[{
                'robot_id': robot_id,
                'neighbors': neighbors,
                'my_offset': my_offset,
                'neighbor_offsets': neighbor_offsets,
                'enable_consensus': False,
            }],
            output='screen',
        ),
    ]

    if is_leader:
        # Camera + static TF, exactly as oak_camera.launch.py defines.
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(PathJoinSubstitution([
                    FindPackageShare('swerve_bringup'), 'launch',
                    'oak_camera.launch.py',
                ])),
                launch_arguments={
                    'robot_id': robot_id,
                    'cam_x':    cam_x,
                    'cam_y':    cam_y,
                    'cam_z':    cam_z,
                    'fps':      fps,
                }.items(),
            )
        )

        # Obstacle avoidance subscribes to the leader's depth and emits
        # the modified /virtual_center/cmd_vel that the laplacians on
        # both robots consume.
        actions.append(
            Node(
                package='swerve_formation',
                executable='obstacle_avoidance_node',
                name='obstacle_avoidance_node' + suffix,
                parameters=[{
                    'leader_robot_id': robot_id,
                    'avoid_range_mm':  int(avoid_range_mm),
                    'lateral_gain':    float(lateral_gain),
                }],
                output='screen',
            )
        )

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_id',         default_value='tb3_0'),
        DeclareLaunchArgument(
            'is_leader',        default_value='false',
            description='If true, also launches the OAK camera + obstacle_avoidance.'
        ),
        DeclareLaunchArgument('usb_port',         default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('neighbors',        default_value=''),
        DeclareLaunchArgument('my_offset',        default_value='0.0,0.0'),
        DeclareLaunchArgument('neighbor_offsets', default_value=''),
        DeclareLaunchArgument('fps',              default_value='15'),
        # tb3_1 measured mount values from CLAUDE.md. RE-MEASURE for tb3_0.
        DeclareLaunchArgument('cam_x',            default_value='0.128'),
        DeclareLaunchArgument('cam_y',            default_value='0.000'),
        DeclareLaunchArgument('cam_z',            default_value='-0.0175'),
        # Tuning knobs (override at runtime to change avoidance behaviour).
        DeclareLaunchArgument(
            'avoid_range_mm',   default_value='1200',
            description='Start avoiding when closest depth in the swathe < this many mm.'
        ),
        DeclareLaunchArgument(
            'lateral_gain',     default_value='0.10',
            description='Peak lateral push velocity (m/s) at urgency=1 (obstacle 0 mm away).'
        ),
        OpaqueFunction(function=launch_setup),
    ])
