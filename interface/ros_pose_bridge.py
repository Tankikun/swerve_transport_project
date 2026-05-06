"""
ros_pose_bridge.py — Stream the robot's live pose from ROS 2 to the Flask GUI.

The interface/ web UI shows a small "is localization working RIGHT NOW" badge
plus a moving icon on the 2D and 3D maps. This standalone rclpy node feeds
both — it looks up the canonical map -> base_link transform at 10 Hz and
POSTs the result to server.py's /pose endpoint.

It ALSO carries the GUI's "Set Initial Pose" hint downstream: each tick it
polls server.py's `/set_initial_pose` (GET); when the seq increments past
`last_seen_seq`, the bridge publishes one `PoseWithCovarianceStamped` to
`/initialpose` for RTAB-Map to seed its current estimate. This is the
exact same topic AMCL / RViz "2D Pose Estimate" use, so RTAB-Map converges
in 1-2 sec instead of doing a slow global re-localization search.

How it works
------------
- TF lookup, NOT a topic subscription.
  RTAB-Map publishes the `map -> {robot_id}_odom -> {robot_id}_base_link`
  chain. The end of that chain is the only honest answer to "where is the
  robot in the map frame?" — it interpolates between visual matches via
  the odometry, so the 2D/3D marker glides instead of jumping at 1 Hz.
- The actual /slam/pose topic is also subscribed to, but ONLY to track
  when the LAST visual match occurred. That timestamp drives the GUI's
  green/orange badge: "fresh visual match" vs "dead-reckoning past 5 s".
- POST is best-effort. If Flask isn't up the node logs (throttled) and
  keeps polling — no rclpy crash.

Topics / TF
-----------
- TF:  `map -> {robot_id}_base_link`        (lookup at 10 Hz, lookup_timeout 0.5 s)
- Sub: `/{robot_id}/slam/pose`              (PoseStamped — last-match timer only)
- Pub: `/initialpose`                       (PoseWithCovarianceStamped — GUI hint
                                             republished for RTAB-Map; global
                                             namespace because the rtabmap node
                                             does not remap initialpose)
- HTTP: POST `{server_url}`                 (default http://localhost:5002/pose)
- HTTP: GET  `{initial_pose_url}`           (default http://localhost:5002/set_initial_pose)

Run
---
    # In a terminal with ROS sourced:
    source ~/swerve_transport_project/ros2_ws/install/setup.bash
    python3 interface/ros_pose_bridge.py --ros-args -p robot_id:=tb3_1

    # Or override server URL:
    python3 interface/ros_pose_bridge.py --ros-args \
        -p robot_id:=tb3_1 \
        -p server_url:=http://192.168.1.42:5002/pose \
        -p initial_pose_url:=http://192.168.1.42:5002/set_initial_pose

POST body schema (POST -> {server_url})
---------------------------------------
    {
      "robot_id":           "tb3_1",
      "localized":          true,
      "x":                  1.234,         # m, ROS map frame
      "y":                  -0.567,        # m, ROS map frame
      "yaw_rad":            0.785,         # rad, CCW from +x
      "yaw_deg":            45.0,
      "frame":              "map",
      "last_match_age_sec": 0.8,           # s since last /slam/pose msg
      "wall_clock_iso":     "2026-05-04T15:30:42.123456+00:00"
    }
When TF is not yet available the same shape is POSTed with `localized: false`
and x / y / yaw_* / last_match_age_sec set to null — the GUI uses this to
turn the status pill red.
"""

import math
from datetime import datetime, timezone

import rclpy
import requests
from rclpy.duration import Duration
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped


def quat_to_yaw(q):
    """Quaternion -> yaw (rotation about ROS Z axis), radians."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class PoseBridge(Node):
    def __init__(self):
        super().__init__('ros_pose_bridge')

        self.declare_parameter('robot_id',         'tb3_1')
        self.declare_parameter('server_url',       'http://localhost:5002/pose')
        self.declare_parameter('initial_pose_url', 'http://localhost:5002/set_initial_pose')
        self.declare_parameter('rate_hz',          10.0)

        self._robot_id         = str(self.get_parameter('robot_id').value)
        self._server_url       = str(self.get_parameter('server_url').value)
        self._initial_pose_url = str(self.get_parameter('initial_pose_url').value)
        rate                   = float(self.get_parameter('rate_hz').value)

        self._base_frame  = f'{self._robot_id}_base_link'
        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # Subscribe to /slam/pose only to learn when the last visual match
        # happened — used as a freshness signal for the GUI's badge.
        self._slam_sub = self.create_subscription(
            PoseStamped, f'/{self._robot_id}/slam/pose',
            self._slam_cb, 10)
        self._last_slam_pose_t = None  # ROS clock seconds, or None

        # Initial-pose republisher. The rtabmap node in our launch runs at the
        # GLOBAL namespace and subscribes to the un-remapped `/initialpose`
        # (same as RViz's "2D Pose Estimate"), so we publish there — NOT to
        # `/{robot_id}/initialpose`.
        self._init_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)
        self._last_seen_init_seq = 0   # only act on strictly higher seqs

        # Throttle the "TF not ready yet" warning to once per 5 s.
        self._last_warn_t = 0.0

        self._timer = self.create_timer(1.0 / rate, self._tick)

        self.get_logger().info(
            f'ros_pose_bridge: robot_id={self._robot_id} '
            f'-> {self._server_url} @ {rate:.1f} Hz '
            f'(initial-pose poll: {self._initial_pose_url})')

    # ----------------------------------------------------------- callbacks

    def _slam_cb(self, msg: PoseStamped):
        # We don't care about msg contents — only that a fresh visual match
        # arrived. Stamp it on the ROS clock.
        self._last_slam_pose_t = self.get_clock().now().nanoseconds * 1e-9

    def _tick(self):
        now_ros = self.get_clock().now().nanoseconds * 1e-9
        last_match_age = (now_ros - self._last_slam_pose_t
                          if self._last_slam_pose_t is not None else None)

        body = {
            'robot_id':           self._robot_id,
            'localized':          False,
            'x':                  None,
            'y':                  None,
            'yaw_rad':            None,
            'yaw_deg':            None,
            'frame':              'map',
            'last_match_age_sec': last_match_age,
            'wall_clock_iso':     datetime.now(timezone.utc).isoformat(),
        }

        try:
            t = self._tf_buffer.lookup_transform(
                'map', self._base_frame, rclpy.time.Time(),
                timeout=Duration(seconds=0.5))
            yaw = quat_to_yaw(t.transform.rotation)
            body.update({
                'localized': True,
                'x':         float(t.transform.translation.x),
                'y':         float(t.transform.translation.y),
                'yaw_rad':   float(yaw),
                'yaw_deg':   float(math.degrees(yaw)),
            })
        except (tf2_ros.LookupException,
                tf2_ros.ExtrapolationException,
                tf2_ros.ConnectivityException):
            if (now_ros - self._last_warn_t) > 5.0:
                self.get_logger().warn(
                    f'TF map -> {self._base_frame} not available; '
                    f'robot not localized yet')
                self._last_warn_t = now_ros

        try:
            requests.post(self._server_url, json=body, timeout=0.2)
        except requests.exceptions.RequestException:
            # Flask not up, network blip, etc. — stay quiet, keep polling.
            pass

        # ---- Initial-pose hint pickup ----
        # Poll the GUI's pending hint; on a fresh seq, republish to
        # /initialpose so RTAB-Map seeds its current estimate at the
        # user-clicked location.
        self._poll_initial_pose()

    # ---------------------------------------------------- initial-pose helper

    def _poll_initial_pose(self):
        """If server has a new /set_initial_pose hint, publish to /initialpose once."""
        try:
            resp = requests.get(self._initial_pose_url, timeout=0.2)
        except requests.exceptions.RequestException:
            # Server unreachable — same best-effort policy as the POST path.
            return
        if resp.status_code != 200:
            return
        try:
            data = resp.json()
        except ValueError:
            return
        if not data.get('available'):
            return
        seq = int(data.get('seq', 0))
        if seq <= self._last_seen_init_seq:
            return  # already published this hint

        x       = float(data.get('x',       0.0))
        y       = float(data.get('y',       0.0))
        yaw     = float(data.get('yaw_rad', 0.0))
        frame   = str(data.get('frame', 'map'))

        msg = PoseWithCovarianceStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = frame
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0
        # Yaw-only quaternion (rotation about +z).
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        # 6x6 row-major covariance, same defaults RViz's "2D Pose Estimate"
        # uses: 0.5 m std on x and y, 15 deg (~0.262 rad) std on yaw.
        # Diagonal entries: var_x=0.25, var_y=0.25, var_yaw≈0.069.
        cov = [0.0] * 36
        cov[0]  = 0.25   # x  x
        cov[7]  = 0.25   # y  y
        cov[35] = 0.069  # yaw yaw
        msg.pose.covariance = cov

        self._init_pub.publish(msg)
        self._last_seen_init_seq = seq
        self.get_logger().info(
            f'Published initial pose: x={x:.2f} y={y:.2f} '
            f'yaw={math.degrees(yaw):.0f}deg, seq={seq}')


def main(args=None):
    rclpy.init(args=args)
    node = PoseBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
