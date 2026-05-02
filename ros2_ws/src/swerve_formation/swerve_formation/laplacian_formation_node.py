"""
laplacian_formation_node.py
---------------------------
Pure-feedforward rigid-body formation controller, runs on every robot.

Each tick this node receives the operator's twist for the (imaginary)
virtual centre on /virtual_center/cmd_vel and produces THIS robot's
body-frame twist using a closed-form rigid-body transform:

    v_robot_x  = vc_vx - vc_wz * my_offset.y
    v_robot_y  = vc_vy + vc_wz * my_offset.x
    v_robot_wz = vc_wz

There is NO pose feedback. We deliberately avoid pulling toward an
EKF-derived "desired position" because the two robots have separate
odometry frames (no SLAM, no shared world), so any cross-robot error
term is meaningless and produces phantom commands at startup.

This means the formation can drift over time (wheel slip, latency)
and there is no consensus correction. The instantaneous kinematics is
correct: at any moment all robots' commanded body twists are exactly
what a rigid body rotating around the virtual centre would dictate.

Saturation handling
-------------------
A formation-wide scaling factor caps the maximum per-wheel linear
speed across ALL robots in the formation to MAX_WHEEL_LINEAR (just
below the firmware's 0.198 m/s clamp). When the would-be outer-wheel
speed exceeds the cap, every robot's twist is scaled by the same
factor — the rigid shape is preserved, the formation just moves
slower. This is the asymmetric "outer wheel goes faster" behaviour
needed for object transport.
"""

import math

import numpy as np
import rclpy
from geometry_msgs.msg import Pose, PoseArray, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


# Worst-case wheel offset from the chassis centre. With
# HALF_WHEELBASE = HALF_TRACK_WIDTH = 0.15 m the wheels sit at
# sqrt(0.15**2 + 0.15**2) ≈ 0.212 m from centre.
CHASSIS_HALF_DIAGONAL = math.sqrt(0.15 ** 2 + 0.15 ** 2)

# Per-wheel linear speed cap. Firmware clamps at MAX_DRIVE_SPEED ≈
# 0.198 m/s; we leave a small margin for rounding and any small
# correction term added later.
MAX_WHEEL_LINEAR = 0.18


class LaplacianFormationController(Node):

    DT = 0.05  # 20 Hz control loop

    def __init__(self):
        super().__init__('laplacian_formation_node')

        self.declare_parameter('robot_id', 'tb3_0')
        self.declare_parameter('neighbors', ['tb3_1'])
        self.declare_parameter('my_offset', [0.0, 0.0])
        self.declare_parameter('neighbor_offsets', [0.0, 0.0])
        self.declare_parameter('k_gain', 0.0)  # legacy, unused (no consensus)

        self.robot_id  = self.get_parameter('robot_id').value
        self.neighbors = list(self.get_parameter('neighbors').value)

        self.my_offset = np.array(
            self.get_parameter('my_offset').value, dtype=float
        )
        nb_off = np.array(
            self.get_parameter('neighbor_offsets').value, dtype=float
        ).reshape(-1, 2)
        self.neighbor_offsets = {n: nb_off[i] for i, n in enumerate(self.neighbors)}

        # Last operator command for the virtual centre. Body-frame.
        self.virtual_vel     = np.zeros(2)
        self.virtual_angular = 0.0

        # Formation-state publisher carries last-known pose of self +
        # neighbours so formation_size_node has something to consume.
        # We update these from EKF if it is available, but the control
        # law does NOT depend on them.
        self.my_pose = np.zeros(2)
        self.neighbor_poses = {n: np.zeros(2) for n in self.neighbors}

        # Subscriptions
        self.create_subscription(
            Twist, '/virtual_center/cmd_vel', self._virtual_cmd_cb, 10
        )
        self.create_subscription(
            Odometry, f'/{self.robot_id}/ekf/odom', self._odom_cb, 10
        )
        for neighbor in self.neighbors:
            self.create_subscription(
                Odometry,
                f'/{neighbor}/ekf/odom',
                lambda msg, n=neighbor: self._neighbor_cb(msg, n),
                10,
            )

        # Publishers
        self._cmd_pub = self.create_publisher(
            Twist, f'/{self.robot_id}/cmd_vel', 10
        )
        self._state_pub = self.create_publisher(
            PoseArray, '/formation/state', 10
        )

        self.create_timer(self.DT, self._control_loop)

        self.get_logger().info(
            f'laplacian_formation_node ready: id={self.robot_id} '
            f'my_offset={self.my_offset.tolist()} '
            f'neighbors={self.neighbors}'
        )

    # ── Subscription callbacks ────────────────────────────────────────────────

    def _virtual_cmd_cb(self, msg: Twist):
        self.virtual_vel[0]  = float(msg.linear.x)
        self.virtual_vel[1]  = float(msg.linear.y)
        self.virtual_angular = float(msg.angular.z)

    def _odom_cb(self, msg: Odometry):
        self.my_pose[0] = msg.pose.pose.position.x
        self.my_pose[1] = msg.pose.pose.position.y

    def _neighbor_cb(self, msg: Odometry, neighbor_id: str):
        self.neighbor_poses[neighbor_id][0] = msg.pose.pose.position.x
        self.neighbor_poses[neighbor_id][1] = msg.pose.pose.position.y

    # ── Control loop (20 Hz) ──────────────────────────────────────────────────

    def _control_loop(self):
        vc_x  = float(self.virtual_vel[0])
        vc_y  = float(self.virtual_vel[1])
        vc_wz = float(self.virtual_angular)

        # ── Local saturation scaling for this robot and configured neighbors ──
        # Compute the worst-case per-wheel linear speed across this
        # robot plus the robots listed in `self.neighbors`, then pick a
        # single scale factor for that set. This preserves rigid-shape
        # scaling only if `self.neighbors` covers every robot that is
        # expected to share the same saturation decision.
        all_offsets = [(float(self.my_offset[0]), float(self.my_offset[1]))]
        for n in self.neighbors:
            nb = self.neighbor_offsets[n]
            all_offsets.append((float(nb[0]), float(nb[1])))

        max_worst = 0.0
        for (rx_i, ry_i) in all_offsets:
            bx = vc_x - vc_wz * ry_i
            by = vc_y + vc_wz * rx_i
            worst = math.hypot(bx, by) + abs(vc_wz) * CHASSIS_HALF_DIAGONAL
            if worst > max_worst:
                max_worst = worst

        if max_worst > MAX_WHEEL_LINEAR and max_worst > 1e-6:
            scale = MAX_WHEEL_LINEAR / max_worst
        else:
            scale = 1.0

        # ── Rigid-body feedforward for THIS robot ──────────────────────
        rx, ry = float(self.my_offset[0]), float(self.my_offset[1])
        ff_x  = (vc_x - vc_wz * ry) * scale
        ff_y  = (vc_y + vc_wz * rx) * scale
        ff_wz = vc_wz * scale

        cmd = Twist()
        cmd.linear.x  = ff_x
        cmd.linear.y  = ff_y
        cmd.angular.z = ff_wz
        self._cmd_pub.publish(cmd)

        # ── Publish formation state for formation_size_node ────────────
        pa = PoseArray()
        pa.header.stamp    = self.get_clock().now().to_msg()
        pa.header.frame_id = 'odom'
        p0 = Pose()
        p0.position.x = float(self.my_pose[0])
        p0.position.y = float(self.my_pose[1])
        p0.orientation.w = 1.0
        pa.poses.append(p0)
        for n in self.neighbors:
            p = Pose()
            p.position.x = float(self.neighbor_poses[n][0])
            p.position.y = float(self.neighbor_poses[n][1])
            p.orientation.w = 1.0
            pa.poses.append(p)
        self._state_pub.publish(pa)


def main(args=None):
    rclpy.init(args=args)
    node = LaplacianFormationController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
