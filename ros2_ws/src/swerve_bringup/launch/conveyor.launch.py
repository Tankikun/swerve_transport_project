"""
conveyor.launch.py — per-Pi bringup for the SLAM-anchored demo.

Spawns the full robot stack on a single Pi:

  conveyor_base_node       serial bridge to OpenCR
  ekf_node                 fuses /odom + /imu + /slam/pose
  laplacian_formation_node feedforward + Laplacian consensus correction
  leader_election_node     bully election on /formation/heartbeat
  path_follower_node       leader-only; consumes /formation/path
  formation_size_node      leader-only; publishes /formation/footprint
  alignment_node           leader-only; depth-based pre-run spacing
  rtabmap_localization     RTAB-Map in localization-only mode
                           against the pre-built lab.db; publishes
                           map → {robot_id}_odom and feeds /slam/pose
                           through slam_pose_relay_node.

This branch (`interface/v6-fuckyea`) requires localization to work.
The no-SLAM fallback that lived on `interface/v5-final` (static
map→odom TF, fixed_pose bridge override, FIXED_INITIAL_POSE GUI
fallback) is intentionally absent here.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
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


def launch_setup(context, *args, **kwargs):
    robot_id          = LaunchConfiguration('robot_id').perform(context)
    usb_port          = LaunchConfiguration('usb_port').perform(context)
    k_gain            = float(LaunchConfiguration('k_gain').perform(context))
    neighbors         = _parse_neighbors(LaunchConfiguration('neighbors').perform(context))
    my_offset         = _parse_xy_list(LaunchConfiguration('my_offset').perform(context))
    neighbor_offsets  = _parse_xy_list(LaunchConfiguration('neighbor_offsets').perform(context))
    db_path           = LaunchConfiguration('db_path').perform(context)
    fps               = LaunchConfiguration('fps').perform(context)
    cam_x             = LaunchConfiguration('cam_x').perform(context)
    cam_y             = LaunchConfiguration('cam_y').perform(context)
    cam_z             = LaunchConfiguration('cam_z').perform(context)
    # Closed-loop formation correction. Each robot's
    # laplacian_formation_node already subscribes to its own AND its
    # neighbour's `/{rid}/ekf/odom`. With consensus on, the relative
    # pose error feeds back into the per-robot twist as a small
    # correction (k_gain · (actual_world − desired_world)), counteracting
    # any residual drift between the two robots' SLAM-anchored EKFs.
    # Falls back to pure feedforward when the neighbour pose is missing
    # or stale (single-robot test, or the other Pi hasn't booted).
    enable_consensus  = (LaunchConfiguration('enable_consensus').perform(context).lower()
                         in ('true', '1', 'yes', 'on'))

    if not my_offset:
        my_offset = [0.0, 0.0]
    if not neighbor_offsets:
        neighbor_offsets = [0.0, 0.0] * max(1, len(neighbors))
    if not neighbors:
        # Sane default — single-robot launches still work
        neighbors = ['tb3_1' if robot_id == 'tb3_0' else 'tb3_0']

    # Per-robot unique node names so two robots on the same ROS network
    # don't collide on `/laplacian_formation_node` etc., which corrupts
    # DDS discovery (subscribers fail to match publishers and ros2 node
    # list shows duplicates with a name-collision warning).
    suffix = f'_{robot_id}'

    return [
        # Graph Laplacian formation controller. With enable_consensus on,
        # this node closes the loop on inter-robot relative pose using
        # both robots' /ekf/odom (which is wheel odom + IMU + SLAM).
        Node(
            package='swerve_formation',
            executable='laplacian_formation_node',
            name='laplacian_formation_node' + suffix,
            parameters=[{
                'robot_id': robot_id,
                'k_gain': k_gain,
                'neighbors': neighbors,
                'my_offset': my_offset,
                'neighbor_offsets': neighbor_offsets,
                'enable_consensus': enable_consensus,
            }],
            output='screen',
        ),

        # Lifecycle serial bridge → OpenCR
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

        # EKF — wheel odom prediction + IMU gyro Z (slip-immune yaw rate)
        # in the prediction step + RTAB-Map /slam/pose correction. SLAM
        # is mandatory in this build; without it the filter drifts.
        Node(
            package='swerve_formation',
            executable='ekf_node',
            name='ekf_node' + suffix,
            parameters=[{
                'robot_id':    robot_id,
                'gyro_z_sign': LaunchConfiguration('gyro_z_sign'),
            }],
            output='screen',
        ),

        # Leader election — bully algorithm, scales to 3+ robots
        Node(
            package='swerve_formation',
            executable='leader_election_node',
            name='leader_election_node' + suffix,
            parameters=[{'robot_id': robot_id}],
            output='screen',
        ),

        # Path follower — only the elected leader drives /virtual_center/cmd_vel.
        # Subscribes to /formation/path (latched, transient_local) from
        # path_planner_node on the laptop; rotates to /goal_pose yaw at end.
        Node(
            package='swerve_formation',
            executable='path_follower_node',
            name='path_follower_node' + suffix,
            parameters=[{'robot_id': robot_id}],
            output='screen',
        ),

        # Formation size (leader-only) — publishes /formation/footprint Polygon
        # consumed by both path_planner_node and path_follower_node.
        Node(
            package='swerve_formation',
            executable='formation_size_node',
            name='formation_size_node' + suffix,
            parameters=[{'robot_id': robot_id}],
            output='screen',
        ),

        # Camera + RTAB-Map localization. Publishes:
        #   map → {robot_id}_odom TF (continuous correction)
        #   /{rid}/rtabmap/localization_pose → /{rid}/slam/pose (via relay)
        # The /slam/pose feed is what makes ekf_node world-frame anchored.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('swerve_bringup'), 'launch',
                'rtabmap_localization.launch.py',
            ])),
            launch_arguments={
                'robot_id':    robot_id,
                'usb_port':    usb_port,
                'db_path':     db_path,
                'fps':         fps,
                'cam_x':       cam_x,
                'cam_y':       cam_y,
                'cam_z':       cam_z,
                'enable_base': 'false',
                'enable_ekf':  'false',
            }.items(),
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_id',         default_value='tb3_0'),
        DeclareLaunchArgument('usb_port',         default_value='/dev/ttyACM0'),
        # k_gain — Laplacian consensus gain. 0.1 is the docstring-recommended
        # safe value (cm-scale positional error → mm/s velocity correction).
        # Crank up cautiously if the formation visibly drifts apart over
        # the demo run; back off to 0.05 if it jitters or oscillates.
        DeclareLaunchArgument('k_gain',           default_value='0.1'),
        DeclareLaunchArgument('neighbors',        default_value=''),
        DeclareLaunchArgument('my_offset',        default_value='0.0,0.0'),
        DeclareLaunchArgument('neighbor_offsets', default_value='0.0,0.0'),
        DeclareLaunchArgument('db_path',          default_value='~/maps/room.db',
                              description='RTAB-Map database built by rtabmap_mapping.launch.py.'),
        DeclareLaunchArgument('fps',              default_value='15',
                              description='Camera FPS passed to oak_camera.launch.py.'),
        DeclareLaunchArgument('cam_x',            default_value='0.10'),
        DeclareLaunchArgument('cam_y',            default_value='0.00'),
        DeclareLaunchArgument('cam_z',            default_value='0.15'),
        DeclareLaunchArgument('enable_consensus', default_value='false',
                              description=('When true, laplacian_formation_node '
                                           'closes the loop on inter-robot pose '
                                           'using both /ekf/odom feeds — small '
                                           'velocity corrections counteract '
                                           'residual drift over the run. Falls '
                                           'back to pure feedforward if either '
                                           'neighbour pose is missing.')),
        DeclareLaunchArgument('gyro_z_sign', default_value='1.0',
                              description=('Sign of the IMU gyro Z reading. '
                                           'Set to -1.0 if a bench yaw test '
                                           'shows ekf yaw decreasing under '
                                           'physical CCW rotation (depends on '
                                           'how the OpenCR is mounted).')),
        OpaqueFunction(function=launch_setup),
    ])
