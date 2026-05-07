"""
oak_camera.launch.py
--------------------
Brings up just the OAK-D Lite camera publisher + the camera→base_link
static TF transform. Used standalone for camera-only smoke tests, and
included by rtabmap_mapping.launch.py / rtabmap_localization.launch.py.

What runs:
  depthai_ros_driver::Camera component (loaded into a
    ComposableNodeContainer) — RGB + aligned stereo depth.
    Configured by swerve_bringup/config/depthai_oak_d_lite.yaml.
  static_transform_publisher — base_link → camera optical frame

Coordinate conventions:
  base_link            : x forward, y left,   z up    (REP-103 body)
  *_camera_optical     : x right,   y down,   z forward (REP-103 optical)

The default mount offset assumes the OAK-D is bolted on the chassis
top plate at the robot centre, lens forward, ~10 cm forward of the
rotation centre and ~15 cm above the floor. **Re-measure for your
exact mount and override via launch args before mapping** —
RTAB-Map's pose accuracy depends on this transform being correct.

Launch args:
  robot_id            tb3_0 / tb3_1
  cam_x, cam_y, cam_z translation from base_link to optical frame [m]
  fps                 camera publish rate (default 15)
  rgb_size, depth_size  legacy launch args, currently ignored — resolution
                       is set in depthai_oak_d_lite.yaml. Declared so
                       existing callers that pass them don't error out.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def launch_setup(context, *args, **kwargs):
    robot_id   = LaunchConfiguration('robot_id').perform(context)
    cam_x      = LaunchConfiguration('cam_x').perform(context)
    cam_y      = LaunchConfiguration('cam_y').perform(context)
    cam_z      = LaunchConfiguration('cam_z').perform(context)
    fps        = LaunchConfiguration('fps').perform(context)
    suffix     = f'_{robot_id}'

    optical_frame = f'{robot_id}_oak_rgb_camera_optical_frame'

    yaml_path = os.path.join(
        get_package_share_directory('swerve_bringup'),
        'config', 'depthai_oak_d_lite.yaml',
    )

    return [
        # ── OAK-D camera publisher (depthai_ros_driver) ──────────────────
        # i_tf_prefix matches the static TF child frame below.
        # i_publish_tf_from_calibration is off because we publish the
        # measured base→camera transform ourselves; two publishers on the
        # same edge would clobber each other.
        ComposableNodeContainer(
            name='oak_container' + suffix,
            namespace=robot_id,
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='depthai_ros_driver',
                    plugin='depthai_ros_driver::Camera',
                    name='oak',
                    namespace=f'{robot_id}/camera',
                    parameters=[yaml_path, {
                        'camera.i_tf_prefix': f'{robot_id}_oak',
                        'camera.i_publish_tf_from_calibration': False,
                        'rgb.i_fps': float(fps),
                        'stereo.i_fps': float(fps),
                    }],
                    # depthai_ros_driver publishes under
                    #   <ns>/<node_name>/<sensor>/image_raw — i.e.
                    #   /{robot_id}/camera/oak/rgb/image_raw
                    #   /{robot_id}/camera/oak/stereo/image_raw
                    # Strip the 'oak/' segment and rename 'stereo' → 'depth'
                    # so downstream subscribers (rtabmap, ai_camera_node,
                    # alignment_node) see the schema documented in CLAUDE.md.
                    remappings=[
                        (f'/{robot_id}/camera/oak/rgb/image_raw',
                         f'/{robot_id}/camera/rgb/image_raw'),
                        (f'/{robot_id}/camera/oak/rgb/camera_info',
                         f'/{robot_id}/camera/rgb/camera_info'),
                        (f'/{robot_id}/camera/oak/stereo/image_raw',
                         f'/{robot_id}/camera/depth/image_raw'),
                        (f'/{robot_id}/camera/oak/stereo/camera_info',
                         f'/{robot_id}/camera/depth/camera_info'),
                    ],
                ),
            ],
            output='screen',
        ),

        # ── Static TF: base_link → camera optical frame ──────────────────
        # ROS optical frame from body frame:
        #   roll  = -π/2  (rotate the camera's +Z (forward) onto body +X)
        #   pitch =  0
        #   yaw   = -π/2  (rotate so optical +X points to body -Y)
        # This is the standard "no-tilt forward-facing camera" rotation.
        # Re-measure if the OAK-D is tilted up/down on the mount.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='oak_tf_static' + suffix,
            arguments=[
                '--x', cam_x, '--y', cam_y, '--z', cam_z,
                '--roll', '-1.5707963', '--pitch', '0', '--yaw', '-1.5707963',
                '--frame-id', f'{robot_id}_base_link',
                '--child-frame-id', optical_frame,
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
            'cam_x', default_value='0.10',
            description='Camera optical frame X offset from base_link [m] '
                        '(default 0.10 = 10 cm forward of rotation centre).'),
        DeclareLaunchArgument(
            'cam_y', default_value='0.00',
            description='Camera Y offset from base_link [m].'),
        DeclareLaunchArgument(
            'cam_z', default_value='0.15',
            description='Camera Z offset from base_link [m] '
                        '(default 0.15 = 15 cm above floor).'),
        DeclareLaunchArgument(
            'fps', default_value='15',
            description='Camera publish rate (Hz). 15 is OAK-D Lite USB2 sweet spot.'),
        DeclareLaunchArgument(
            'rgb_size', default_value='640x400',
            description='RGB resolution (WxH).'),
        DeclareLaunchArgument(
            'depth_size', default_value='640x400',
            description='Depth resolution (WxH). Width MUST be a multiple of 16.'),
        OpaqueFunction(function=launch_setup),
    ])
