"""
alignment_node.py
-----------------
Computes and locks in formation offsets from robot initial positions.

Two modes (offset_init_mode parameter):
  'odom'   — reads EKF poses of self + all neighbors, computes offsets so that
             the virtual center sits at the centroid of all robot positions.
             Fires once, then publishes offsets periodically forever.
  'manual' — uses the provided my_offset / neighbor_offsets directly.

Publishes to /formation/offsets (PoseArray):
  poses[0]  = this robot's offset from VC  [x, y]
  poses[1+] = each neighbor's offset        [x, y] (same order as 'neighbors' param)

The laplacian_formation_node subscribes to /formation/offsets and updates
its internal offsets when this publishes — so the formation adapts to
wherever the robots were physically placed.

Also exposes /{robot_id}/reset_odom (std_srvs/Trigger) — sends 'R\\n' to
the OpenCR firmware to zero the odometry.
"""

import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, PoseArray
from std_srvs.srv import Trigger
from std_msgs.msg import String


def _parse_float_list(value) -> list[float]:
    """Accept list or comma-separated string."""
    if isinstance(value, str):
        return [float(x.strip()) for x in value.split(',')]
    return [float(x) for x in value]


class AlignmentNode(Node):

    def __init__(self):
        super().__init__('alignment_node')

        self.declare_parameter('robot_id',         'tb3_0')
        self.declare_parameter('neighbors',        ['tb3_1'])
        self.declare_parameter('my_offset',        [0.0, 0.3])
        self.declare_parameter('neighbor_offsets', [0.0, -0.3])
        self.declare_parameter('offset_init_mode', 'manual')
        self.declare_parameter('init_timeout',     10.0)

        self._robot_id   = self.get_parameter('robot_id').value
        self._neighbors  = list(self.get_parameter('neighbors').value)
        self._mode       = self.get_parameter('offset_init_mode').value
        self._timeout    = float(self.get_parameter('init_timeout').value)

        self._my_off    = _parse_float_list(self.get_parameter('my_offset').value)
        self._nb_off_flat = _parse_float_list(self.get_parameter('neighbor_offsets').value)

        # State for odom mode
        self._my_pos:   list[float] | None          = None
        self._nb_poses: dict[str, list[float]]       = {}
        self._locked   = False
        self._locked_my_off   : list[float] = []
        self._locked_nb_off   : list[float] = []

        # Subscriptions (always subscribe — needed for odom mode; harmless in manual)
        self.create_subscription(
            Odometry, f'/{self._robot_id}/ekf/odom', self._own_odom_cb, 10
        )
        for nb in self._neighbors:
            self.create_subscription(
                Odometry, f'/{nb}/ekf/odom',
                lambda m, n=nb: self._nb_odom_cb(m, n), 10
            )

        # Publisher: formation offsets → laplacian_formation_node
        self._offsets_pub = self.create_publisher(PoseArray, '/formation/offsets', 10)

        # Service: reset odometry
        self._reset_serial_pub = self.create_publisher(
            String, f'/{self._robot_id}/serial_cmd', 10
        )
        self.create_service(Trigger, f'/{self._robot_id}/reset_odom', self._reset_odom_cb)

        if self._mode == 'manual':
            self._lock_and_start(self._my_off, self._nb_off_flat)
        else:
            self._init_start = self.get_clock().now()
            self.create_timer(0.5, self._check_odom_init)

        self.get_logger().info(
            f'alignment_node ready ({self._robot_id}) mode={self._mode} '
            f'my_offset={self._my_off}'
        )

    # ── Odom callbacks ────────────────────────────────────────────────────────

    def _own_odom_cb(self, msg: Odometry):
        self._my_pos = [msg.pose.pose.position.x, msg.pose.pose.position.y]

    def _nb_odom_cb(self, msg: Odometry, nb_id: str):
        self._nb_poses[nb_id] = [
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
        ]

    # ── Odom-mode initialisation ──────────────────────────────────────────────

    def _check_odom_init(self):
        if self._locked:
            return

        elapsed = (self.get_clock().now() - self._init_start).nanoseconds * 1e-9
        if elapsed > self._timeout:
            self.get_logger().error(
                'Offset init timeout — no valid EKF pose received. '
                'Falling back to manual offsets.'
            )
            self._lock_and_start(self._my_off, self._nb_off_flat)
            return

        if self._my_pos is None:
            return
        if not all(nb in self._nb_poses for nb in self._neighbors):
            return

        # Compute virtual centre = centroid of all robot positions
        all_pos = [self._my_pos] + [self._nb_poses[nb] for nb in self._neighbors]
        n = len(all_pos)
        vc = [sum(p[i] for p in all_pos) / n for i in range(2)]

        my_off = [self._my_pos[i] - vc[i] for i in range(2)]
        nb_off_flat: list[float] = []
        for nb in self._neighbors:
            nb_pos = self._nb_poses[nb]
            nb_off_flat += [nb_pos[i] - vc[i] for i in range(2)]

        self.get_logger().info(
            f'Offset init from odometry complete. '
            f'my_offset={[round(v,3) for v in my_off]}'
        )
        self._lock_and_start(my_off, nb_off_flat)

    # ── Lock offsets and start publishing ─────────────────────────────────────

    def _lock_and_start(self, my_off: list[float], nb_off_flat: list[float]):
        self._locked          = True
        self._locked_my_off   = my_off
        self._locked_nb_off   = nb_off_flat
        self.create_timer(1.0, self._publish_offsets)
        self._publish_offsets()

    def _publish_offsets(self):
        pa = PoseArray()
        pa.header.stamp    = self.get_clock().now().to_msg()
        pa.header.frame_id = 'odom'

        p0 = Pose()
        p0.position.x = float(self._locked_my_off[0])
        p0.position.y = float(self._locked_my_off[1]) if len(self._locked_my_off) > 1 else 0.0
        p0.orientation.w = 1.0
        pa.poses.append(p0)

        nb = self._locked_nb_off
        for i in range(0, len(nb), 2):
            p = Pose()
            p.position.x = float(nb[i])
            p.position.y = float(nb[i + 1]) if i + 1 < len(nb) else 0.0
            p.orientation.w = 1.0
            pa.poses.append(p)

        self._offsets_pub.publish(pa)

    # ── Reset odom service ────────────────────────────────────────────────────

    def _reset_odom_cb(self, request, response):
        """Send 'R' to the serial bridge which forwards it to OpenCR."""
        msg = String()
        msg.data = 'R'
        self._reset_serial_pub.publish(msg)
        response.success = True
        response.message = 'Odom reset command sent to OpenCR.'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = AlignmentNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
