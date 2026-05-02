"""
laplacian_formation_node.py
---------------------------
Hybrid rigid-body feedforward + peer-based drift correction. Runs on
every robot.

  cmd_body = rigid_body_feedforward(virtual_center_twist, my_offset)
           + R(-my_heading) * peer_drift_correction
  scale all of the above formation-wide so the worst-case per-wheel
  speed across the formation stays under MAX_WHEEL_LINEAR.

Rigid-body feedforward (instantaneous):
    v_robot_x  = vc_vx - vc_wz * my_offset.y
    v_robot_y  = vc_vy + vc_wz * my_offset.x
    v_robot_wz = vc_wz

Peer-based drift correction (closed loop):
    For each peer p, expected position of THIS robot relative to p in the
    rotated formation body frame is (my_offset - p.offset). In MY world
    frame (which == my odom frame, since we reset at startup), that's
    R(my_heading) * (my_offset - p.offset).

    Peer's actual position in MY world frame is its initial offset from
    me (known from launch params) PLUS its own odom delta:
        peer_in_my_world = (p.offset_initial - my_offset_initial) + p.odom

    The expected MY position becomes:
        my_expected = peer_in_my_world + R(my_heading) * (my_offset - p.offset)

    World-frame correction toward where I should be:
        err_world = my_expected - my_pose
        correction_world += K_DRIFT * err_world / num_peers

    Then we rotate the summed correction into MY body frame and add it
    to the feedforward.

This is safe because the OpenCR firmware now provides REAL encoder-based
odometry (not the previous commanded-value dead-reckoning). Pose feedback
that lies → unstable closed loop → robots wander; pose feedback that
matches reality → consensus actually pulls the formation tight.

Assumptions
-----------
1. At reset_odom, both robots' EKF frames are reset to (0,0,0).
2. Both robots are physically placed at the launch-param offsets and
   facing the SAME direction (so initial headings agree). The controller
   uses launch-param offsets as the source of truth for initial
   inter-robot geometry.
3. The robots remain a (mostly) rigid body — peer_heading ≈ my_heading.
   We don't subscribe to peer heading; we use my own heading to rotate
   the offset vectors. This is a small approximation that holds well
   for symmetric two-robot formations.

If those assumptions break, fall back to K_DRIFT = 0 (pure feedforward,
last-night's verified behaviour).
"""

import math

import numpy as np
import rclpy
from geometry_msgs.msg import Pose, PoseArray, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


# Worst-case wheel offset from the chassis centre.
CHASSIS_HALF_DIAGONAL = math.sqrt(0.15 ** 2 + 0.15 ** 2)

# Per-wheel linear-speed cap. Firmware clamps at 0.198 m/s; leave margin
# for the consensus correction to add a bit on top without saturating
# silently inside the IK.
MAX_WHEEL_LINEAR = 0.18


def _yaw_from_quat(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


class LaplacianFormationController(Node):

    DT = 0.05  # 20 Hz control loop

    def __init__(self):
        super().__init__('laplacian_formation_node')

        self.declare_parameter('robot_id', 'tb3_0')
        self.declare_parameter('neighbors', ['tb3_1'])
        self.declare_parameter('my_offset', [0.0, 0.0])
        self.declare_parameter('neighbor_offsets', [0.0, 0.0])
        # Peer-pull gain. Modest by default — encoder odom is reliable
        # but not perfect, and overshooting consensus pulls produce
        # visible jitter in 2-robot formations.
        self.declare_parameter('k_gain', 0.8)

        self.robot_id  = self.get_parameter('robot_id').value
        self.neighbors = list(self.get_parameter('neighbors').value)
        self.k_drift   = float(self.get_parameter('k_gain').value)

        self.my_offset = np.array(
            self.get_parameter('my_offset').value, dtype=float
        )
        nb_off = np.array(
            self.get_parameter('neighbor_offsets').value, dtype=float
        ).reshape(-1, 2)
        self.neighbor_offsets = {n: nb_off[i] for i, n in enumerate(self.neighbors)}

        # Last operator command for the virtual centre.
        self.virtual_vel     = np.zeros(2)
        self.virtual_angular = 0.0

        # EKF-derived state. Used for the drift-correction term and for
        # publishing /formation/state.
        self.my_pose       = np.zeros(2)
        self.my_heading    = 0.0
        self.my_pose_received = False
        self.neighbor_poses    = {n: np.zeros(2)         for n in self.neighbors}
        self.neighbor_received = {n: False               for n in self.neighbors}

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
            f'neighbors={self.neighbors} '
            f'k_drift={self.k_drift}'
        )

    # ── Subscription callbacks ────────────────────────────────────────────────

    def _virtual_cmd_cb(self, msg: Twist):
        self.virtual_vel[0]  = float(msg.linear.x)
        self.virtual_vel[1]  = float(msg.linear.y)
        self.virtual_angular = float(msg.angular.z)

    def _odom_cb(self, msg: Odometry):
        self.my_pose[0] = msg.pose.pose.position.x
        self.my_pose[1] = msg.pose.pose.position.y
        self.my_heading = _yaw_from_quat(msg.pose.pose.orientation)
        self.my_pose_received = True

    def _neighbor_cb(self, msg: Odometry, neighbor_id: str):
        self.neighbor_poses[neighbor_id][0] = msg.pose.pose.position.x
        self.neighbor_poses[neighbor_id][1] = msg.pose.pose.position.y
        self.neighbor_received[neighbor_id] = True

    # ── Control loop (20 Hz) ──────────────────────────────────────────────────

    def _control_loop(self):
        vc_x  = float(self.virtual_vel[0])
        vc_y  = float(self.virtual_vel[1])
        vc_wz = float(self.virtual_angular)

        # ── Step 1: rigid-body feedforward for THIS robot ────────────────
        rx, ry = float(self.my_offset[0]), float(self.my_offset[1])
        ff_x_body  = vc_x - vc_wz * ry
        ff_y_body  = vc_y + vc_wz * rx
        ff_wz_body = vc_wz

        # ── Step 2: peer-based drift correction ─────────────────────────
        # Only enabled once we have valid pose data for ourselves AND
        # every neighbour. Until then the term is exactly zero, so we
        # degrade gracefully to last-night's verified pure-FF behaviour.
        corr_world = np.zeros(2)
        if self.my_pose_received and all(self.neighbor_received.values()) \
                                 and self.k_drift > 1e-6:
            c, s = math.cos(self.my_heading), math.sin(self.my_heading)
            R = np.array([[c, -s], [s, c]])

            n_peers = max(1, len(self.neighbors))
            for peer in self.neighbors:
                p_off    = self.neighbor_offsets[peer].astype(float)
                p_pose   = self.neighbor_poses[peer].astype(float)

                # Peer's initial position in MY world frame is just the
                # difference of the launch-param offsets (we placed the
                # robots there).
                peer_initial_in_my_world = p_off - self.my_offset
                # Peer's current position in MY world frame: assumes both
                # odom frames started axis-aligned (robots facing the
                # same direction at reset).
                peer_now_in_my_world = peer_initial_in_my_world + p_pose

                # Where I SHOULD be, given peer is where it is and the
                # formation is rigid (rotated by my heading):
                my_expected = peer_now_in_my_world + R @ (self.my_offset - p_off)

                err = my_expected - self.my_pose          # world-frame error
                corr_world += (self.k_drift / n_peers) * err

            # Rotate the summed correction from world frame into MY body
            # frame so it can be added to the body-frame feedforward.
            corr_body_x =  c * corr_world[0] + s * corr_world[1]
            corr_body_y = -s * corr_world[0] + c * corr_world[1]
        else:
            corr_body_x = 0.0
            corr_body_y = 0.0

        cmd_x_body = ff_x_body + corr_body_x
        cmd_y_body = ff_y_body + corr_body_y
        cmd_wz_body = ff_wz_body

        # ── Step 3: formation-wide saturation scaling ───────────────────
        # Compute the worst-case per-wheel linear speed across every
        # robot in the formation — using the FEEDFORWARD twist (not the
        # corrected one), since each peer computes its own correction
        # locally and we don't try to predict it. The formation stays
        # synchronised because all peers see the same vc_* command.
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
            cmd_x_body  *= scale
            cmd_y_body  *= scale
            cmd_wz_body *= scale

        # Final per-axis safety clamp on the corrected command — protects
        # against a runaway correction term if the assumptions above ever
        # break (e.g. peer pose stops updating mid-run).
        if abs(cmd_x_body) > MAX_WHEEL_LINEAR:
            cmd_x_body = math.copysign(MAX_WHEEL_LINEAR, cmd_x_body)
        if abs(cmd_y_body) > MAX_WHEEL_LINEAR:
            cmd_y_body = math.copysign(MAX_WHEEL_LINEAR, cmd_y_body)

        cmd = Twist()
        cmd.linear.x  = cmd_x_body
        cmd.linear.y  = cmd_y_body
        cmd.angular.z = cmd_wz_body
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
