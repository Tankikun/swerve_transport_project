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
    offset_init_mode  = LaunchConfiguration('offset_init_mode').perform(context)
    db_path           = LaunchConfiguration('db_path').perform(context)
    fps               = LaunchConfiguration('fps').perform(context)
    cam_x             = LaunchConfiguration('cam_x').perform(context)
    cam_y             = LaunchConfiguration('cam_y').perform(context)
    cam_z             = LaunchConfiguration('cam_z').perform(context)
    # `enable_slam:=false` skips the RTAB-Map / camera include and
    # disables the EKF's visual correction. Used for the GUI demo path
    # where localisation comes from wheel odometry + IMU only and the
    # operator seeds initial pose by clicking on the map. We still need
    # a `map → {robot_id}_odom` TF for ros_pose_bridge to look up; with
    # SLAM off, a static identity TF stands in.
    enable_slam       = (LaunchConfiguration('enable_slam').perform(context).lower()
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

    nodes = [
        # Graph Laplacian formation controller
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

        # EKF — fuses raw /odom + IMU (always) + SLAM pose (if enable_slam).
        # When `enable_slam:=false` the EKF ignores any stale /slam/pose
        # messages and stays on odom + IMU, anchored only by /initialpose
        # hints from the GUI.
        Node(
            package='swerve_formation',
            executable='ekf_node',
            name='ekf_node' + suffix,
            parameters=[{
                'robot_id': robot_id,
                'use_slam': enable_slam,
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

        # Navigation — only the elected leader drives /virtual_center/cmd_vel
        Node(
            package='swerve_formation',
            executable='navigation_node',
            name='navigation_node' + suffix,
            parameters=[{'robot_id': robot_id}],
            output='screen',
        ),

        # Formation size (leader-only) — computes bounding envelope
        Node(
            package='swerve_formation',
            executable='formation_size_node',
            name='formation_size_node' + suffix,
            parameters=[{'robot_id': robot_id}],
            output='screen',
        ),
    ]

    if enable_slam:
        # Camera + RTAB-Map localization (base and EKF already started above)
        nodes.append(IncludeLaunchDescription(
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
        ))
        # Pre-run alignment — leader coordinates depth-based spacing
        # correction. Needs the OAK-D, so it only runs with SLAM on.
        nodes.append(Node(
            package='swerve_formation',
            executable='alignment_node',
            name='alignment_node' + suffix,
            parameters=[{
                'robot_id': robot_id,
                'neighbors': neighbors,
                'offset_init_mode': offset_init_mode,
            }],
            output='screen',
        ))
    else:
        # Stand-in for RTAB-Map's `map → {robot_id}_odom` so ros_pose_bridge's
        # TF lookup completes. Identity offset works because the GUI's
        # "Set Initial Pose" tool sends map-frame coords directly to
        # /{robot_id}/initialpose, and the EKF treats those values as
        # its own (odom-frame) state. Map and odom are aliased.
        nodes.append(Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_map_to_odom' + suffix,
            arguments=[
                '--frame-id', 'map',
                '--child-frame-id', f'{robot_id}_odom',
                '--x', '0', '--y', '0', '--z', '0',
                '--yaw', '0', '--pitch', '0', '--roll', '0',
            ],
            output='screen',
        ))

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_id',         default_value='tb3_0'),
        DeclareLaunchArgument('usb_port',         default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('k_gain',           default_value='1.5'),
        DeclareLaunchArgument('neighbors',        default_value=''),
        DeclareLaunchArgument('my_offset',        default_value='0.0,0.0'),
        DeclareLaunchArgument('neighbor_offsets', default_value='0.0,0.0'),
        DeclareLaunchArgument('offset_init_mode', default_value='manual'),
        DeclareLaunchArgument('db_path',          default_value='~/maps/room.db',
                              description='RTAB-Map database built by rtabmap_mapping.launch.py.'),
        DeclareLaunchArgument('fps',              default_value='15',
                              description='Camera FPS passed to oak_camera.launch.py.'),
        DeclareLaunchArgument('cam_x',            default_value='0.10'),
        DeclareLaunchArgument('cam_y',            default_value='0.00'),
        DeclareLaunchArgument('cam_z',            default_value='0.15'),
        DeclareLaunchArgument('enable_slam',      default_value='true',
                              description=('When false, skip RTAB-Map / camera, '
                                           'add a static map→odom TF, and run '
                                           'EKF on odom+IMU only. Used for the '
                                           'GUI-anchored real-robot demo.')),
        OpaqueFunction(function=launch_setup),
    ])
