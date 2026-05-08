"""
fake_depth_publisher_node.py
----------------------------
Publishes synthetic 16UC1 depth images on
`/{leader_robot_id}/camera/depth/image_raw` so the operator can dry-run
`obstacle_avoidance_node` on the laptop **without any robots**.

Use this to verify, before going to the lab:
  * obstacle_avoidance subscribes correctly to your topic name.
  * Lateral push direction is what you expect (push goes AWAY from
    the obstacle column).
  * The state topic flips between 'clear' and 'AVOID' as the obstacle
    column changes.

Scenarios (selected via the `scenario` parameter):
  * 'clear'           — uniform wall at 3000 mm. Should always be clear.
  * 'box_centre'      — wall at 3500 mm + 600 mm box dead-centre. Pushes
                        +y (left) by convention (col_u rounds to +1 px
                        right of centre).
  * 'box_left'        — box in left third of image. Pushes -y (right).
  * 'box_right'       — box in right third of image. Pushes +y (left).
  * 'sweep'           — box that sweeps left→right→left every ~6 s. Use
                        this to watch the avoidance flip sign in real
                        time on `/obstacle_avoidance/state`.

This node owns the `tb3_X/camera` namespace it publishes to. Do NOT
run it on a Pi that's actually streaming depth from the OAK-D — the
real publisher and this fake one would race.

Recommended laptop dry-run (in two terminals):

  T1: ros2 run swerve_formation fake_depth_publisher_node \\
        --ros-args -p leader_robot_id:=tb3_1 -p scenario:=sweep

  T2: ros2 run swerve_formation obstacle_avoidance_node \\
        --ros-args -p leader_robot_id:=tb3_1 -p depth_stale_s:=2.0

  T3: ros2 topic pub -r 5 /virtual_center/cmd_vel/raw \\
        geometry_msgs/Twist '{linear: {x: 0.10, y: 0.0}, angular: {z: 0.0}}'

  T4: ros2 topic echo /virtual_center/cmd_vel
      ros2 topic echo /obstacle_avoidance/state

In T2 you should see the closest_m drop and col_u flip sign as the
sweep box moves across the image. In T4 you should see linear.y
oscillate between +lateral_gain*urgency and -lateral_gain*urgency,
with linear.x attenuated.
"""

from __future__ import annotations

import math

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header


def _now_header(node: Node, frame_id: str) -> Header:
    h = Header()
    h.stamp = node.get_clock().now().to_msg()
    h.frame_id = frame_id
    return h


def _far_wall(h: int, w: int, mm: int = 3000) -> np.ndarray:
    return np.full((h, w), mm, dtype=np.uint16)


def _wall_with_box(h: int, w: int,
                   box_mm: int, col_lo: int, col_hi: int,
                   wall_mm: int = 3500) -> np.ndarray:
    arr = np.full((h, w), wall_mm, dtype=np.uint16)
    arr[:, col_lo:col_hi] = box_mm
    return arr


class FakeDepthPublisherNode(Node):

    def __init__(self) -> None:
        super().__init__('fake_depth_publisher_node')

        self.declare_parameter('leader_robot_id', 'tb3_1')
        self.declare_parameter('scenario',        'sweep')
        self.declare_parameter('width',           640)
        self.declare_parameter('height',          400)
        self.declare_parameter('publish_hz',      15.0)
        self.declare_parameter('box_distance_mm', 700)
        self.declare_parameter('sweep_period_s',  6.0)

        leader = str(self.get_parameter('leader_robot_id').value)
        self._scenario = str(self.get_parameter('scenario').value)
        self._w        = int(self.get_parameter('width').value)
        self._h        = int(self.get_parameter('height').value)
        self._box_mm   = int(self.get_parameter('box_distance_mm').value)
        self._sweep_T  = max(0.5, float(self.get_parameter('sweep_period_s').value))

        topic = f'/{leader}/camera/depth/image_raw'
        self._frame_id = f'{leader}_oak_rgb_camera_optical_frame'
        self._pub = self.create_publisher(Image, topic, 10)

        rate = float(self.get_parameter('publish_hz').value)
        self._t0 = self.get_clock().now().nanoseconds / 1e9
        self.create_timer(1.0 / rate, self._tick)

        self.get_logger().info(
            f'fake_depth_publisher: scenario={self._scenario}  '
            f'topic={topic}  size={self._w}x{self._h}  rate={rate:.0f}Hz'
        )

    # ── frame factory ─────────────────────────────────────────────────────

    def _make_frame(self) -> np.ndarray:
        w, h = self._w, self._h
        s = self._scenario
        if s == 'clear':
            return _far_wall(h, w, mm=3000)
        if s == 'box_centre':
            mid = w // 2
            return _wall_with_box(h, w, self._box_mm, mid - 40, mid + 40)
        if s == 'box_left':
            return _wall_with_box(h, w, self._box_mm,
                                  int(0.10 * w), int(0.30 * w))
        if s == 'box_right':
            return _wall_with_box(h, w, self._box_mm,
                                  int(0.70 * w), int(0.90 * w))
        if s == 'sweep':
            t = (self.get_clock().now().nanoseconds / 1e9) - self._t0
            phase = (t % self._sweep_T) / self._sweep_T   # 0..1
            # Triangle wave in [-0.5, +0.5] over the period
            tri = 2 * abs(phase - 0.5) - 0.5
            box_centre_norm = 0.50 + 0.30 * math.copysign(1, -tri) * abs(tri) * 2
            cc = int(box_centre_norm * w)
            half = max(20, w // 16)
            lo = max(0, cc - half)
            hi = min(w, cc + half)
            return _wall_with_box(h, w, self._box_mm, lo, hi)
        # Unknown scenario → far wall, log once.
        self.get_logger().warn(
            f'unknown scenario={s!r}; falling back to "clear"',
            throttle_duration_sec=10.0,
        )
        return _far_wall(h, w, mm=3000)

    def _tick(self) -> None:
        arr = self._make_frame()
        msg = Image()
        msg.header   = _now_header(self, self._frame_id)
        msg.height   = arr.shape[0]
        msg.width    = arr.shape[1]
        msg.encoding = '16UC1'
        msg.is_bigendian = 0
        msg.step     = arr.shape[1] * 2
        msg.data     = arr.tobytes()
        self._pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FakeDepthPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
