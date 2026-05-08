"""
obstacle_avoidance_node.py
--------------------------
Markerless, mapless obstacle avoidance for the two-robot transport demo.

Sits between the operator (or `goal_driver_node`) and the proven
feedforward `laplacian_formation_node`. Modifies a single Twist
on the way through:

    /virtual_center/cmd_vel/raw ──┐
                                  ├──► obstacle_avoidance ──► /virtual_center/cmd_vel
    /{leader}/camera/depth/image_raw ┘   (modified for obstacles)

The downstream `laplacian_formation_node` runs on every robot in pure
feedforward mode (consensus off — the rigid-body kinematics is exact
at any instant given a single virtual-centre twist), so as long as both
robots receive the same `/virtual_center/cmd_vel`, they sweep around
an obstacle in formation, holding the payload between them.

Why this approach (vs ArUco / SLAM):
  * No printed markers, no map .db, no EKF init wait. Single sensor,
    single closed loop.
  * The formation already works — see `feature/two-robot-test-seven`
    HANDOFF_TO_TAN.md §4. We only add the obstacle term.
  * Depth processing is cheap on the Pi (one slice + masked min per
    frame), well below the camera publish rate.

Algorithm (extracted into pure functions for testability — see
`tools/test_obstacle_logic.py`):
  1. `find_closest_in_swathe`: crop the depth image to a horizontal
     forward swathe (40–65 % of image height by default), mask invalid
     depths, return (closest_mm, col_u) where col_u ∈ [-1, +1] is the
     normalised column of the closest pixel (sign matches camera
     optical +x).
  2. `compute_avoidance`: given (closest_mm, col_u) and the operator's
     raw Twist + params, return the modified Twist plus a
     human-readable state string. If no obstacle within `avoid_range_mm`,
     pass through unchanged.

Coordinate convention:
  +y = ROS body left. The lateral push sign assumes the camera optical
  +x (image column right) maps to body -y (body right). For a forward-
  facing camera mounted with the standard ROS optical-from-body
  rotation (roll = -π/2, yaw = -π/2, used in `oak_camera.launch.py`),
  this is correct: an obstacle on the right side of the image is on
  the robot's right; pushing in +y moves the formation left, away
  from it.

Parameters (declared and tunable at runtime via ros2 param):
  leader_robot_id   str   robot whose camera we use     (default tb3_1)
  swathe_top_frac   float vertical band top  (0.0 = top of image)
  swathe_bot_frac   float vertical band bot  (1.0 = bottom of image)
  min_valid_mm      int   reject depths below this      (default 250)
  max_valid_mm      int   reject depths above this      (default 4000)
  avoid_range_mm    int   start avoiding within this    (default 1200)
  lateral_gain      float m/s of +y push at urgency=1   (default 0.10)
  speed_scale_floor float min linear x scale at urgency=1 (default 0.4)
  pub_rate_hz       float how often we re-publish       (default 20)
  depth_stale_s     float depth watchdog                (default 0.7)
"""

from __future__ import annotations

import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, String

# Pure-numpy math lives in obstacle_avoidance_lib so it can be unit-tested
# without ROS — see tools/test_obstacle_logic.py.
from swerve_formation.obstacle_avoidance_lib import (
    AvoidanceParams,
    compute_avoidance,
    find_closest_in_swathe,
)


# ─── ROS message helper ─────────────────────────────────────────────────────


def depth_image_to_uint16_mm(msg: Image):
    """
    Convert an Image message to a uint16 mm depth array.

    Accepts the two encodings depthai_ros_driver may produce:
      * '16UC1'  → uint16 millimetres (the standard, current driver)
      * '32FC1'  → float32 metres (some older builds)

    Returns None if the encoding is not recognised — the caller treats
    that as a depth-stale frame.
    """
    try:
        h, w = msg.height, msg.width
        if msg.encoding == '16UC1':
            return np.frombuffer(msg.data, dtype=np.uint16).reshape(h, w)
        if msg.encoding == '32FC1':
            arr = np.frombuffer(msg.data, dtype=np.float32).reshape(h, w)
            return (arr * 1000.0).clip(0, 65535).astype(np.uint16)
    except Exception:
        return None
    return None


# ─── Node ───────────────────────────────────────────────────────────────────


class ObstacleAvoidanceNode(Node):

    def __init__(self) -> None:
        super().__init__('obstacle_avoidance_node')

        self.declare_parameter('leader_robot_id',   'tb3_1')
        self.declare_parameter('swathe_top_frac',   0.40)
        self.declare_parameter('swathe_bot_frac',   0.65)
        self.declare_parameter('min_valid_mm',      250)
        self.declare_parameter('max_valid_mm',      4000)
        self.declare_parameter('avoid_range_mm',    1200)
        self.declare_parameter('lateral_gain',      0.10)
        self.declare_parameter('speed_scale_floor', 0.40)
        self.declare_parameter('pub_rate_hz',       20.0)
        self.declare_parameter('depth_stale_s',     0.7)

        self._leader_id     = str(self.get_parameter('leader_robot_id').value)
        self._pub_rate_hz   = float(self.get_parameter('pub_rate_hz').value)
        self._depth_stale_s = float(self.get_parameter('depth_stale_s').value)

        self._params = AvoidanceParams(
            swathe_top_frac   = float(self.get_parameter('swathe_top_frac').value),
            swathe_bot_frac   = float(self.get_parameter('swathe_bot_frac').value),
            min_valid_mm      = int(self.get_parameter('min_valid_mm').value),
            max_valid_mm      = int(self.get_parameter('max_valid_mm').value),
            avoid_range_mm    = int(self.get_parameter('avoid_range_mm').value),
            lateral_gain      = float(self.get_parameter('lateral_gain').value),
            speed_scale_floor = float(self.get_parameter('speed_scale_floor').value),
        )

        self._lock          = threading.Lock()
        self._raw_twist     = Twist()
        self._closest_mm    = None
        self._closest_col_u = 0.0
        self._depth_t       = 0.0

        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=2,
        )

        self.create_subscription(
            Twist, '/virtual_center/cmd_vel/raw',
            self._raw_cmd_cb, 10,
        )
        self.create_subscription(
            Image, f'/{self._leader_id}/camera/depth/image_raw',
            self._depth_cb, sensor_qos,
        )

        self._cmd_pub = self.create_publisher(
            Twist, '/virtual_center/cmd_vel', 10,
        )
        self._closest_pub = self.create_publisher(
            Float32, '/obstacle_avoidance/closest_m', 5,
        )
        self._state_pub = self.create_publisher(
            String, '/obstacle_avoidance/state', 5,
        )

        self.create_timer(1.0 / self._pub_rate_hz, self._pub_loop)

        self.get_logger().info(
            f'obstacle_avoidance_node ready  leader={self._leader_id}  '
            f'avoid_range={self._params.avoid_range_mm}mm  '
            f'lateral_gain={self._params.lateral_gain:.2f} m/s  '
            f'pub_rate={self._pub_rate_hz:.0f} Hz'
        )

    def _raw_cmd_cb(self, msg: Twist) -> None:
        with self._lock:
            self._raw_twist = msg

    def _depth_cb(self, msg: Image) -> None:
        arr = depth_image_to_uint16_mm(msg)
        if arr is None:
            return
        closest_mm, col_u = find_closest_in_swathe(arr, self._params)
        with self._lock:
            self._closest_mm    = closest_mm
            self._closest_col_u = col_u
            self._depth_t       = time.time()

    def _pub_loop(self) -> None:
        with self._lock:
            raw = self._raw_twist
            closest_mm = self._closest_mm
            col_u      = self._closest_col_u
            depth_age  = time.time() - self._depth_t if self._depth_t > 0 else 1e9

        depth_fresh = depth_age < self._depth_stale_s
        if not depth_fresh:
            out_x, out_y, out_wz = (
                float(raw.linear.x), float(raw.linear.y), float(raw.angular.z)
            )
            state = 'depth-stale'
        else:
            out_x, out_y, out_wz, state = compute_avoidance(
                closest_mm, col_u,
                float(raw.linear.x), float(raw.linear.y), float(raw.angular.z),
                self._params,
            )

        out = Twist()
        out.linear.x  = out_x
        out.linear.y  = out_y
        out.angular.z = out_wz
        self._cmd_pub.publish(out)

        if closest_mm is not None and depth_fresh:
            m = Float32()
            m.data = float(closest_mm) / 1000.0
            self._closest_pub.publish(m)
        s = String()
        s.data = state
        self._state_pub.publish(s)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ObstacleAvoidanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
