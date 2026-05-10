"""
ros_pose_bridge.py — Stream the robot's live pose from ROS 2 to the Flask GUI.

The interface/ web UI shows a small "is localization working RIGHT NOW" badge
plus a moving icon on the 2D and 3D maps. This standalone rclpy node feeds
both — at 10 Hz it picks a pose source (TF or EKF topic, see below) and
POSTs the result to server.py's /pose endpoint.

It ALSO carries the GUI's "Set Initial Pose" hint downstream: each tick it
polls server.py's `/set_initial_pose` (GET); when the seq increments past
`last_seen_seq`, the bridge publishes one `PoseWithCovarianceStamped` to
`/initialpose` for RTAB-Map AND `/{robot_id}/initialpose` for ekf_node.

Pose sources
------------
Two modes, switched by the `use_ekf_topic` parameter:

* `use_ekf_topic:=false`  (default — SLAM-anchored runs)
  TF lookup `map -> {robot_id}_base_link`. RTAB-Map publishes
  `map -> {robot_id}_odom`, and `conveyor_base_node` publishes
  `{robot_id}_odom -> {robot_id}_base_link`. The end of that chain is
  the canonical answer when SLAM is the world-frame anchor.

* `use_ekf_topic:=true`   (no-SLAM / GUI-anchored runs)
  Subscribe to `/{robot_id}/ekf/odom` directly and republish its values
  as if they were `map`-frame coordinates. The EKF state is set by the
  GUI's "Set Initial Pose" hints (via `/{robot_id}/initialpose`) and
  evolves with wheel odom + IMU. Use this when the
  `enable_slam:=false` bringup is in effect — the static
  `map → {robot_id}_odom = identity` TF added by that bringup makes
  the abstract "EKF frame" equal to the GUI's map frame numerically.

In either mode the `/slam/pose` subscription remains, used only to
stamp `last_match_age_sec` for the GUI's badge.

POST is best-effort. If Flask isn't up the node logs (throttled) and
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
import queue
import threading
import time
from datetime import datetime, timezone

import rclpy
import requests
from rclpy.duration import Duration
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry


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
        # When true, take the GUI pose feed straight from /{robot_id}/ekf/odom
        # instead of the TF chain. See module docstring "Pose sources".
        self.declare_parameter('use_ekf_topic',    False)

        self._robot_id         = str(self.get_parameter('robot_id').value)
        self._server_url       = str(self.get_parameter('server_url').value)
        self._initial_pose_url = str(self.get_parameter('initial_pose_url').value)
        rate                   = float(self.get_parameter('rate_hz').value)
        self._use_ekf_topic    = bool(self.get_parameter('use_ekf_topic').value)

        self._base_frame  = f'{self._robot_id}_base_link'
        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # Subscribe to /slam/pose only to learn when the last visual match
        # happened — used as a freshness signal for the GUI's badge.
        self._slam_sub = self.create_subscription(
            PoseStamped, f'/{self._robot_id}/slam/pose',
            self._slam_cb, 10)
        self._last_slam_pose_t = None  # ROS clock seconds, or None

        # Cached EKF pose for use_ekf_topic mode. Filled by _ekf_cb each
        # time a new /ekf/odom message arrives; the tick reads the latest.
        self._ekf_pose: tuple[float, float, float] | None = None  # (x, y, yaw)
        self._ekf_pose_t: float | None = None
        if self._use_ekf_topic:
            self._ekf_sub = self.create_subscription(
                Odometry, f'/{self._robot_id}/ekf/odom',
                self._ekf_cb, 10)

        # Initial-pose republisher. RTAB-Map subscribes to the un-remapped
        # global `/initialpose` (same as RViz's "2D Pose Estimate"). The
        # per-robot ekf_node subscribes to `/{robot_id}/initialpose` so it
        # can hard-reset when SLAM is unavailable. Publish to BOTH so the
        # IMU-only fallback path stays in sync with the visual one.
        self._init_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)
        self._init_pub_robot = self.create_publisher(
            PoseWithCovarianceStamped,
            f'/{self._robot_id}/initialpose', 10)
        self._last_seen_init_seq = 0   # only act on strictly higher seqs
        # On bridge restart we DON'T want to act on a hint queued before we
        # came up — the user may already have driven away from there or
        # localization may already have converged. First poll syncs to the
        # server's current seq so subsequent ticks only react to NEW hints.
        self._init_seq_synced    = False

        # Throttle the "TF not ready yet" warning to once per 5 s.
        self._last_warn_t = 0.0

        # HTTP work runs in a background thread so a slow Flask response
        # never starves the rclpy timer/subscription callbacks. The queue
        # holds POST bodies; a sentinel `None` is the worker shutdown signal.
        self._http_q = queue.Queue(maxsize=4)
        self._http_thread = threading.Thread(
            target=self._http_worker, name='pose-http', daemon=True)
        self._http_thread.start()

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

    def _ekf_cb(self, msg: Odometry):
        """Cache the latest /ekf/odom pose for `use_ekf_topic` mode."""
        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)
        yaw = quat_to_yaw(msg.pose.pose.orientation)
        self._ekf_pose = (x, y, yaw)
        self._ekf_pose_t = time.time()

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

        if self._use_ekf_topic:
            # Pose source: /{robot_id}/ekf/odom topic. Use cached value if
            # < 1 s old (EKF runs at ~33 Hz so this is always fresh once
            # the stack is up).
            wall_now = time.time()
            if (self._ekf_pose is not None and self._ekf_pose_t is not None
                    and (wall_now - self._ekf_pose_t) < 1.0):
                x, y, yaw = self._ekf_pose
                body.update({
                    'localized': True,
                    'x':         x,
                    'y':         y,
                    'yaw_rad':   float(yaw),
                    'yaw_deg':   float(math.degrees(yaw)),
                })
            elif (now_ros - self._last_warn_t) > 5.0:
                self.get_logger().warn(
                    f'/{self._robot_id}/ekf/odom not received yet; '
                    f'robot not localized')
                self._last_warn_t = now_ros
        else:
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

        # Hand the POST body to the worker thread and return immediately.
        # If the queue is full (Flask is slow / dead), drop the oldest pose
        # rather than block — losing a single 100 ms-stale frame is fine.
        try:
            self._http_q.put_nowait(body)
        except queue.Full:
            try:
                self._http_q.get_nowait()
                self._http_q.put_nowait(body)
            except (queue.Empty, queue.Full):
                pass

    # --------------------------------------------------------- HTTP worker

    def _http_worker(self):
        """Pumps POSTs to /pose and polls /set_initial_pose on a background thread.

        Doing HTTP off the rclpy executor keeps subscription callbacks
        responsive even when Flask is slow or unreachable (timeouts up to
        0.5 s + 0.5 s would otherwise stall the timer for ~1 s).
        """
        while True:
            body = self._http_q.get()
            if body is None:           # shutdown sentinel
                return
            try:
                requests.post(self._server_url, json=body, timeout=0.5)
            except requests.exceptions.RequestException:
                pass
            try:
                resp = requests.get(self._initial_pose_url, timeout=0.5)
                self._handle_initial_pose_response(resp)
            except requests.exceptions.RequestException:
                pass

    # ---------------------------------------------------- initial-pose helper

    def _handle_initial_pose_response(self, resp):
        """If server has a new /set_initial_pose hint, publish to /initialpose once.

        Called from the HTTP worker thread. rclpy publishers are
        thread-safe; the only shared state we mutate is
        `_last_seen_init_seq` and `_init_seq_synced` which are only read
        and written from this same worker, so no lock needed.
        """
        if resp.status_code != 200:
            return
        try:
            data = resp.json()
        except ValueError:
            return
        if not data.get('available'):
            return
        seq = int(data.get('seq', 0))

        # First successful poll: sync to whatever the server already had.
        # Without this, a bridge crash + restart would replay the most
        # recent hint, yanking an already-converged localization.
        if not self._init_seq_synced:
            self._last_seen_init_seq = seq
            self._init_seq_synced    = True
            self.get_logger().info(
                f'Initial-pose seq synced to server (seq={seq}); '
                f'will only act on strictly NEW hints from here.')
            return

        if seq <= self._last_seen_init_seq:
            return  # already published this hint

        x     = float(data.get('x',       0.0))
        y     = float(data.get('y',       0.0))
        yaw   = float(data.get('yaw_rad', 0.0))
        frame = str(data.get('frame', 'map'))

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
        self._init_pub_robot.publish(msg)
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