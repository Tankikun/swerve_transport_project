"""
oak_camera.launch.py
--------------------
Brings up just the OAK-D Lite camera publisher + the camera→base_link
static TF transform. Used standalone for camera-only smoke tests, and
included by rtabmap_mapping.launch.py / rtabmap_localization.launch.py.

What runs:
  oak_camera_node       — depthai SDK 3.x publisher (custom, not the
                           depthai_ros_driver apt package which the
                           lab apt mirror cannot install reliably)
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
  rgb_size, depth_size  resolution as 'WxH' (depth must be / 16 wide)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    robot_id   = LaunchConfiguration('robot_id').perform(context)
    cam_x      = LaunchConfiguration('cam_x').perform(context)
    cam_y      = LaunchConfiguration('cam_y').perform(context)
    cam_z      = LaunchConfiguration('cam_z').perform(context)
    fps        = LaunchConfiguration('fps').perform(context)
    rgb_size   = LaunchConfiguration('rgb_size').perform(context)
    depth_size = LaunchConfiguration('depth_size').perform(context)
    suffix     = f'_{robot_id}'

    optical_frame = f'{robot_id}_oak_rgb_camera_optical_frame'

    return [
        # ── OAK-D camera publisher ───────────────────────────────────────
        Node(
            package='swerve_formation',
            executable='oak_camera_node',
            name='oak_camera_node' + suffix,
            parameters=[{
                'robot_id':         robot_id,
                'fps':              int(float(fps)),
                'rgb_size':         rgb_size,
                'stereo_size':      depth_size,
                'use_stereo_align': True,
            }],
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
