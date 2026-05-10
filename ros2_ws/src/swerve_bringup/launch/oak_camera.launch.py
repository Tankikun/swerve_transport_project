"""
oak_camera.launch.py
--------------------
Brings up just the OAK-D Lite camera publisher + a standard DepthAI TF
chain with an intermediate `oak-d-base-frame`.

Used standalone for camera-only smoke tests, and included by
rtabmap_mapping.launch.py / rtabmap_localization.launch.py.

What runs:
  depthai_ros_driver::Camera component (loaded into a
    ComposableNodeContainer) — RGB + aligned stereo depth.
    Configured by swerve_bringup/config/depthai_oak_d_lite.yaml.
    depthai_descriptions URDF (robot_state_publisher) — publishes the
        DepthAI camera frame chain rooted at `{robot_id}_oak-d-base-frame`.
    static_transform_publisher — base_link → oak-d-base-frame (mount TF)

Coordinate conventions:
  base_link            : x forward, y left,   z up    (REP-103 body)
  *_camera_optical     : x right,   y down,   z forward (REP-103 optical)

Camera mount offsets are per-robot. Measured values live in the
``_CAMERA_MOUNT`` table below — when a robot is added, measure it
and append a row. Pass cam_x/cam_y/cam_z launch args to override.

Launch args:
  robot_id            tb3_0 / tb3_1
    cam_x, cam_y, cam_z translation from base_link to oak-d-base-frame [m]
    cam_roll, cam_pitch, cam_yaw rotation from base_link to oak-d-base-frame [rad]
                                             (leave empty to use the measured value from
                                                _CAMERA_MOUNT keyed by robot_id)
  fps                 camera publish rate (default 15)
  rgb_size, depth_size  legacy launch args, currently ignored — resolution
                       is set in depthai_oak_d_lite.yaml. Declared so
                       existing callers that pass them don't error out.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


# Measured base_link → oak-d-base-frame mount pose.
# Translation: meters. Rotation: radians.
# Add a row when adding a robot; values come from physical measurement,
# not from CAD. See CLAUDE.md > "Camera Mount TF".
_CAMERA_MOUNT = {
    'tb3_0': (0.1107, 0.0000, 0.0974, 0.0, 0.0, 0.0),
    'tb3_1': (0.1107, 0.0000, 0.0974, 0.0, 0.0, 0.0),
}


def _resolve_cam(robot_id, cam_x, cam_y, cam_z, cam_roll, cam_pitch, cam_yaw):
    """Resolve mount xyz+rpy from launch args or the per-robot table."""
    if cam_x or cam_y or cam_z or cam_roll or cam_pitch or cam_yaw:
        # Any explicit value → use as-is. Missing components default to 0.
        return (
            cam_x or '0.0', cam_y or '0.0', cam_z or '0.0',
            cam_roll or '0.0', cam_pitch or '0.0', cam_yaw or '0.0',
        )
    if robot_id in _CAMERA_MOUNT:
        x, y, z, r, p, yw = _CAMERA_MOUNT[robot_id]
        return str(x), str(y), str(z), str(r), str(p), str(yw)
    raise RuntimeError(
        f'No measured camera mount for robot_id={robot_id!r}. Either pass '
        f'cam_x:= cam_y:= cam_z:= cam_roll:= cam_pitch:= cam_yaw:= launch args, '
        f'or add the measurement to '
        f'_CAMERA_MOUNT in oak_camera.launch.py.'
    )


def launch_setup(context, *args, **kwargs):
    robot_id   = LaunchConfiguration('robot_id').perform(context)
    cam_x_raw  = LaunchConfiguration('cam_x').perform(context)
    cam_y_raw  = LaunchConfiguration('cam_y').perform(context)
    cam_z_raw  = LaunchConfiguration('cam_z').perform(context)
    cam_roll_raw  = LaunchConfiguration('cam_roll').perform(context)
    cam_pitch_raw = LaunchConfiguration('cam_pitch').perform(context)
    cam_yaw_raw   = LaunchConfiguration('cam_yaw').perform(context)
    fps        = LaunchConfiguration('fps').perform(context)
    suffix     = f'_{robot_id}'

    cam_x, cam_y, cam_z, cam_roll, cam_pitch, cam_yaw = _resolve_cam(
        robot_id,
        cam_x_raw, cam_y_raw, cam_z_raw,
        cam_roll_raw, cam_pitch_raw, cam_yaw_raw,
    )

    oak_name       = f'{robot_id}_oak'
    oak_base_frame = f'{robot_id}_oak-d-base-frame'

    yaml_path = os.path.join(
        get_package_share_directory('swerve_bringup'),
        'config', 'depthai_oak_d_lite.yaml',
    )

    urdf_launch_path = os.path.join(
        get_package_share_directory('depthai_descriptions'),
        'launch', 'urdf_launch.py',
    )

    return [
        # ── TF via DepthAI URDF ─────────────────────────────────────────
        # IMPORTANT: In depthai_descriptions, `tf_prefix` is passed into the
        # xacro as `camera_name` (a substring used to form frame names like
        # '<camera_name>_rgb_camera_optical_frame'), not a global "prefix all
        # frames" mechanism. depthai_ros_driver uses the ROS node name in the
        # same way when it fills Image.header.frame_id.
        #
        # We therefore set `tf_prefix == base_frame == oak_name` so that:
        #   - the URDF publishes `tb3_0_oak_rgb_camera_optical_frame`, and
        #   - the driver publishes image headers with the same frame_id.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(urdf_launch_path),
            launch_arguments={
                'camera_model':    'OAK-D-LITE',
                'tf_prefix':       oak_name,
                'base_frame':      oak_name,
                'parent_frame':    oak_base_frame,
                # Keep parent→base identity; the physical mount is published
                # by the static_transform_publisher below.
                'cam_pos_x':       '0.0',
                'cam_pos_y':       '0.0',
                'cam_pos_z':       '0.0',
                'cam_roll':        '0.0',
                'cam_pitch':       '0.0',
                'cam_yaw':         '0.0',
                'use_composition': 'false',
                'use_base_descr':  'false',
                'rs_compat':       'false',
            }.items(),
        ),

        # ── OAK-D camera publisher (depthai_ros_driver) ──────────────────
        # The driver builds image header frame_ids from node->get_name() (see
        # depthai_ros_driver sensor_helpers.cpp::tfPrefix), so we encode the
        # robot prefix in the node name (`{robot_id}_oak`) instead of via a
        # parameter — there is no `i_tf_prefix` parameter in v2.12.x.
        # i_publish_tf_from_calibration is off because TF is provided by the
        # DepthAI URDF above + our mount TF below; enabling calibration TF
        # would create two publishers on the same edge.
        # use_intra_process_comms=True puts the driver on the IPC publisher
        # branch (img_pub.cpp), which avoids image_transport plugin
        # auto-loading and the parallel `<topic>/compressed` siblings that
        # otherwise bypass our remappings.
        ComposableNodeContainer(
            name='oak_container' + suffix,
            namespace=robot_id,
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='depthai_ros_driver',
                    plugin='depthai_ros_driver::Camera',
                    name=oak_name,
                    namespace=f'{robot_id}/camera',
                    parameters=[yaml_path, {
                        'camera.i_publish_tf_from_calibration': False,
                        'rgb.i_fps': float(fps),
                        'stereo.i_fps': float(fps),
                    }],
                    extra_arguments=[{'use_intra_process_comms': True}],
                    # depthai_ros_driver publishes under
                    #   <ns>/<node_name>/<sensor>/image_raw — i.e.
                    #   /{robot_id}/camera/{robot_id}_oak/rgb/image_raw
                    #   /{robot_id}/camera/{robot_id}_oak/stereo/image_raw
                    # Strip the '{robot_id}_oak/' segment and rename
                    # 'stereo' → 'depth' so downstream subscribers (rtabmap,
                    # ai_camera_node, alignment_node) see the schema
                    # documented in CLAUDE.md.
                    remappings=[
                        (f'/{robot_id}/camera/{robot_id}_oak/rgb/image_raw',
                         f'/{robot_id}/camera/rgb/image_raw'),
                        (f'/{robot_id}/camera/{robot_id}_oak/rgb/camera_info',
                         f'/{robot_id}/camera/rgb/camera_info'),
                        (f'/{robot_id}/camera/{robot_id}_oak/stereo/image_raw',
                         f'/{robot_id}/camera/depth/image_raw'),
                        (f'/{robot_id}/camera/{robot_id}_oak/stereo/camera_info',
                         f'/{robot_id}/camera/depth/camera_info'),
                    ],
                ),
            ],
            output='screen',
        ),

        # ── Static TF: base_link → oak-d-base-frame (mount) ──────────────
        # TODO: Measure `{robot_id}_base_link` → `{robot_id}_oak-d-base-frame`
        # on the physical robot and update `_CAMERA_MOUNT` (or pass cam_*
        # launch args). See CLAUDE.md > "Camera Mount TF".
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='oak_mount_tf_static' + suffix,
            arguments=[
                '--x', cam_x, '--y', cam_y, '--z', cam_z,
                '--roll', cam_roll, '--pitch', cam_pitch, '--yaw', cam_yaw,
                '--frame-id', f'{robot_id}_base_link',
                '--child-frame-id', oak_base_frame,
            ],
            output='screen',
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_id', default_value='tb3_0',
            description='Robot ID / namespace (tb3_0, tb3_1, ...)'),
        DeclareLaunchArgument(
            'cam_x', default_value='',
            description='Camera mount X offset from base_link to oak-d-base-frame [m]. '
                        'Leave empty to use the measured value from '
                        '_CAMERA_MOUNT keyed by robot_id.'),
        DeclareLaunchArgument(
            'cam_y', default_value='',
            description='Camera mount Y offset from base_link to oak-d-base-frame [m]. Leave empty '
                        'to use _CAMERA_MOUNT.'),
        DeclareLaunchArgument(
            'cam_z', default_value='',
            description='Camera mount Z offset from base_link to oak-d-base-frame [m]. Leave empty '
                        'to use _CAMERA_MOUNT.'),
        DeclareLaunchArgument(
            'cam_roll', default_value='',
            description='Camera mount roll from base_link to oak-d-base-frame [rad]. Leave empty '
                        'to use _CAMERA_MOUNT.'),
        DeclareLaunchArgument(
            'cam_pitch', default_value='',
            description='Camera mount pitch from base_link to oak-d-base-frame [rad]. Leave empty '
                        'to use _CAMERA_MOUNT.'),
        DeclareLaunchArgument(
            'cam_yaw', default_value='',
            description='Camera mount yaw from base_link to oak-d-base-frame [rad]. Leave empty '
                        'to use _CAMERA_MOUNT.'),
        DeclareLaunchArgument(
            'fps', default_value='15',
            description='Camera publish rate (Hz). 15 is OAK-D Lite USB2 sweet spot.'),
        DeclareLaunchArgument(
            'rgb_size', default_value='960x540',
            description='RGB resolution (WxH).'),
        DeclareLaunchArgument(
            'depth_size', default_value='960x540',
            description='Depth resolution (WxH). Width MUST be a multiple of 16.'),
        OpaqueFunction(function=launch_setup),
    ])
