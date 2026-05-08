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

Algorithm (~20 lines of math, repeated at depth-frame rate):
  1. Crop the depth image to a horizontal forward "swathe" (40–60 % of
     image height). This skips floor and ceiling without needing TF
     transforms.
  2. Mask invalid depths (< MIN_VALID_MM, > MAX_VALID_MM).
  3. Find the closest valid pixel inside the swathe; remember its
     (column, depth_mm).
  4. If closest_mm > AVOID_RANGE_MM → no obstacle near. Pass the raw
     Twist through unchanged.
  5. Otherwise compute a lateral push:
        u_horiz = (col - W/2) / (W/2)        ∈ [-1, +1]
        push_dir = -sign(u_horiz)            (away from obstacle col)
        urgency  = 1 - clamp(closest_mm / AVOID_RANGE_MM)
        v_y_add  = LATERAL_GAIN * push_dir * urgency
        scale    = 1 - 0.6 * urgency         (slow down when close)
     Final Twist:
        linear.x  = raw.linear.x  * scale
        linear.y  = raw.linear.y  + v_y_add
        angular.z = raw.angular.z * scale     (don't fight rotation,
                                               just attenuate)
  6. Watchdog: if no depth frame in DEPTH_STALE_S, pass through raw
     (the formation is then operator-only).

Coordinate convention:
  +y = ROS body left. The lateral push sign assumes the camera optical
  +x (image column right) maps to body -y (body right). For a forward-
  facing camera mounted with the standard ROS optical-from-body
  rotation (roll = -π/2, yaw = -π/2, used in `oak_camera.launch.py`),
  this is correct: an obstacle on the right side of the image is on the
  robot's right; pushing in +y moves the formation left, away from it.

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

import math
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, String


def _depth_image_to_uint16_mm(msg: Image) -> np.ndarray | None:
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
            arr = np.frombuffer(msg.data, dtype=np.uint16).reshape(h, w)
            return arr
        if msg.encoding == '32FC1':
            arr = np.frombuffer(msg.data, dtype=np.float32).reshape(h, w)
            mm = (arr * 1000.0).clip(0, 65535).astype(np.uint16)
            return mm
    except Exception:
        return None
    return None


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

        self._leader_id        = str(self.get_parameter('leader_robot_id').value)
        self._swathe_top_frac  = float(self.get_parameter('swathe_top_frac').value)
        self._swathe_bot_frac  = float(self.get_parameter('swathe_bot_frac').value)
        self._min_valid_mm     = int(self.get_parameter('min_valid_mm').value)
        self._max_valid_mm     = int(self.get_parameter('max_valid_mm').value)
        self._avoid_range_mm   = int(self.get_parameter('avoid_range_mm').value)
        self._lateral_gain     = float(self.get_parameter('lateral_gain').value)
        self._scale_floor      = float(self.get_parameter('speed_scale_floor').value)
        self._pub_rate_hz      = float(self.get_parameter('pub_rate_hz').value)
        self._depth_stale_s    = float(self.get_parameter('depth_stale_s').value)

        # Cached state — operator twist + most recent obstacle reading.
        self._lock          = threading.Lock()
        self._raw_twist     = Twist()        # zero by default → safe
        self._raw_t         = 0.0
        self._closest_mm    = None           # None → no valid obstacle
        self._closest_col_u = 0.0            # normalised column ∈ [-1, +1]
        self._depth_t       = 0.0

        # Sensor data is best-effort; commands stay reliable so we don't
        # silently drop them under WiFi loss.
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
        # Debug: closest depth (m) and a one-line state summary so the
        # operator can sanity-check from `ros2 topic echo`.
        self._closest_pub = self.create_publisher(
            Float32, '/obstacle_avoidance/closest_m', 5,
        )
        self._state_pub = self.create_publisher(
            String, '/obstacle_avoidance/state', 5,
        )

        self.create_timer(1.0 / self._pub_rate_hz, self._pub_loop)

        self.get_logger().info(
            f'obstacle_avoidance_node ready  leader={self._leader_id}  '
            f'avoid_range={self._avoid_range_mm}mm  '
            f'lateral_gain={self._lateral_gain:.2f} m/s  '
            f'pub_rate={self._pub_rate_hz:.0f} Hz'
        )

    # ── callbacks ─────────────────────────────────────────────────────────

    def _raw_cmd_cb(self, msg: Twist) -> None:
        with self._lock:
            self._raw_twist = msg
            self._raw_t = time.time()

    def _depth_cb(self, msg: Image) -> None:
        arr = _depth_image_to_uint16_mm(msg)
        if arr is None:
            return
        h, w = arr.shape
        if h < 4 or w < 4:
            return

        top = int(self._swathe_top_frac * h)
        bot = int(self._swathe_bot_frac * h)
        if bot <= top + 1:
            return
        swathe = arr[top:bot, :]

        # Mask invalid depths (zeros from the driver, plus our range gate).
        valid = (swathe >= self._min_valid_mm) & (swathe <= self._max_valid_mm)
        if not valid.any():
            with self._lock:
                self._closest_mm = None
                self._depth_t    = time.time()
            return

        # Argmin over valid pixels only.
        masked = np.where(valid, swathe, np.uint16(self._max_valid_mm + 1))
        flat_idx = int(masked.argmin())
        row = flat_idx // w
        col = flat_idx %  w
        depth = int(masked[row, col])

        # Normalised column ∈ [-1, +1]; sign matches camera optical +x
        # (image right). +1 = obstacle on the right side of the image.
        col_u = (col - (w / 2.0)) / max(w / 2.0, 1.0)

        with self._lock:
            self._closest_mm    = depth
            self._closest_col_u = float(col_u)
            self._depth_t       = time.time()

    # ── publish loop ──────────────────────────────────────────────────────

    def _pub_loop(self) -> None:
        with self._lock:
            raw = self._raw_twist
            closest_mm = self._closest_mm
            col_u      = self._closest_col_u
            depth_age  = time.time() - self._depth_t if self._depth_t > 0 else 1e9

        out = Twist()
        out.linear.x  = float(raw.linear.x)
        out.linear.y  = float(raw.linear.y)
        out.angular.z = float(raw.angular.z)
        state = 'pass-through'

        depth_fresh = depth_age < self._depth_stale_s
        if not depth_fresh:
            state = 'depth-stale'
        elif closest_mm is None:
            state = 'no-valid-depth'
        elif closest_mm >= self._avoid_range_mm:
            state = f'clear  closest={closest_mm/1000.0:.2f}m'
        else:
            urgency = 1.0 - (float(closest_mm) / float(self._avoid_range_mm))
            urgency = max(0.0, min(1.0, urgency))
            push_dir = -math.copysign(1.0, col_u) if abs(col_u) > 1e-3 else 1.0
            v_y_add = self._lateral_gain * push_dir * urgency
            scale = 1.0 - (1.0 - self._scale_floor) * urgency
            out.linear.x  = float(raw.linear.x) * scale
            out.linear.y  = float(raw.linear.y) + v_y_add
            out.angular.z = float(raw.angular.z) * scale
            state = (
                f'AVOID  closest={closest_mm/1000.0:.2f}m  '
                f'col_u={col_u:+.2f}  push_y={v_y_add:+.2f}  '
                f'scale={scale:.2f}'
            )

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
