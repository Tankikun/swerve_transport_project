"""
ros_pose_bridge.py — Stream the robot's live pose from ROS 2 to the Flask GUI.

The interface/ web UI shows a small "is localization working RIGHT NOW" badge
plus a moving icon on the 2D and 3D maps. This standalone rclpy node feeds
both — it looks up the canonical map -> base_link transform at 10 Hz and
POSTs the result to server.py's /pose endpoint.

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
- TF: `map -> {robot_id}_base_link`        (lookup at 10 Hz, lookup_timeout 0.5 s)
- Sub: `/{robot_id}/slam/pose`             (PoseStamped — last-match timer only)
- HTTP: POST `{server_url}` (default http://localhost:5002/pose)

Run
---
    # In a terminal with ROS sourced:
    source ~/swerve_transport_project/ros2_ws/install/setup.bash
    python3 interface/ros_pose_bridge.py --ros-args -p robot_id:=tb3_1

    # Or override server URL:
    python3 interface/ros_pose_bridge.py --ros-args \
        -p robot_id:=tb3_1 \
        -p server_url:=http://192.168.1.42:5002/pose

POST body schema
----------------
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
from geometry_msgs.msg import PoseStamped


def quat_to_yaw(q):
    """Quaternion -> yaw (rotation about ROS Z axis), radians."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class PoseBridge(Node):
    def __init__(self):
        super().__init__('ros_pose_bridge')

        self.declare_parameter('robot_id',  'tb3_1')
        self.declare_parameter('server_url', 'http://localhost:5002/pose')
        self.declare_parameter('rate_hz',    10.0)

        self._robot_id   = str(self.get_parameter('robot_id').value)
        self._server_url = str(self.get_parameter('server_url').value)
        rate             = float(self.get_parameter('rate_hz').value)

        self._base_frame  = f'{self._robot_id}_base_link'
        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # Subscribe to /slam/pose only to learn when the last visual match
        # happened — used as a freshness signal for the GUI's badge.
        self._slam_sub = self.create_subscription(
            PoseStamped, f'/{self._robot_id}/slam/pose',
            self._slam_cb, 10)
        self._last_slam_pose_t = None  # ROS clock seconds, or None

        # Throttle the "TF not ready yet" warning to once per 5 s.
        self._last_warn_t = 0.0

        self._timer = self.create_timer(1.0 / rate, self._tick)

        self.get_logger().info(
            f'ros_pose_bridge: robot_id={self._robot_id} '
            f'-> {self._server_url} @ {rate:.1f} Hz')

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
