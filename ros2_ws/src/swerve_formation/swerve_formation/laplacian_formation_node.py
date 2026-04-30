"""
laplacian_formation_node.py
---------------------------
Decentralized Laplacian formation controller — runs on every robot.

Implements the formation control law from formation_path_planning.ipynb:

  v_i_world = v_vc
            + K_LEADER * (p_desired_from_vc - p_i)
            + sum over peers j: K_PEER * (p_desired_from_peer_j - p_i)

Where:
  p_desired_from_vc       = vc_pos + R(vc_heading) @ own_offset
  p_desired_from_peer_j   = peer_j_pos + R(vc_heading) @ (own_offset - peer_j_offset)

The output is converted to BODY frame before publishing /{robot_id}/cmd_vel,
matching the convention expected by conveyor_base_node and the OpenCR firmware.

Virtual Center convention
─────────────────────────
The "Virtual Center" (VC) is the geometric reference point of the formation.
For a 2-robot rigid payload, VC is typically the midpoint between robots.

In this implementation we treat the LEADER's pose as the VC pose (vc_odom_topic
defaults to /tb3_0/ekf/odom). This is a simplification — the navigation_node
plans paths from the leader's position. To use a true centroid, publish a
custom /virtual_center/odom topic and point vc_odom_topic at it.

With "leader = VC", the offsets must be set as follows for a SEPARATION gap:
  Leader   own_offset = (0, 0)
  Follower own_offset = (-SEPARATION, 0)
  Both    peer_offset = the OTHER robot's own_offset

Subscribes
  {vc_odom_topic}                  Odometry — VC pose+heading
  /virtual_center/cmd_vel          Twist    — VC velocity (from navigation_node)
  /{robot_id}/ekf/odom             Odometry — own EKF-fused pose
  /{peer_id}/ekf/odom              Odometry — each peer's pose

Publishes
  /{robot_id}/cmd_vel              Twist    — body-frame velocity for this robot
  /formation/state                 PoseArray — own + neighbours, for formation_size_node
"""

import math

import numpy as np
import rclpy
from geometry_msgs.msg import Pose, PoseArray, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


def _yaw_from_quat(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def _rot2d(theta: float) -> np.ndarray:
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[c, -s], [s, c]])


class LaplacianFormationController(Node):

    DT = 0.05  # 20 Hz control loop

    def __init__(self):
        super().__init__('laplacian_formation_node')

        # ── Parameters (backward-compatible with conveyor.launch.py) ─────────
        self.declare_parameter('robot_id',         'tb3_0')
        self.declare_parameter('neighbors',        ['tb3_1'])
        self.declare_parameter('my_offset',        [0.0, 0.0])     # this robot's offset from VC
        self.declare_parameter('neighbor_offsets', [-0.8, 0.0])    # flat [n0x, n0y, n1x, n1y, ...]

        # New parameters (notebook-style separate gains)
        self.declare_parameter('K_leader',         5.0)            # pull toward VC
        self.declare_parameter('K_peer',           0.5)            # pull toward peer
        self.declare_parameter('k_gain',           1.5)            # legacy single gain (used as fallback)
        self.declare_parameter('V_MAX',            0.18)           # m/s, just below motor limit
        self.declare_parameter('vc_odom_topic',    '/tb3_0/ekf/odom')
        self.declare_parameter('vc_cmd_vel_topic', '/virtual_center/cmd_vel')

        self._robot_id    = self.get_parameter('robot_id').value
        self._neighbors   = list(self.get_parameter('neighbors').value)
        self._own_off     = np.array(self.get_parameter('my_offset').value, dtype=float)

        nb_off = np.array(self.get_parameter('neighbor_offsets').value, dtype=float).reshape(-1, 2)
        self._peer_off    = {n: nb_off[i] for i, n in enumerate(self._neighbors)}

        self._K_leader    = float(self.get_parameter('K_leader').value)
        self._K_peer      = float(self.get_parameter('K_peer').value)
        self._V_MAX       = float(self.get_parameter('V_MAX').value)

        # ── State ────────────────────────────────────────────────────────────
        self._vc_pose: np.ndarray | None  = None    # [x, y, theta]
        self._vc_vel  = np.zeros(2)                  # world frame
        self._vc_omega = 0.0
        self._own_pose: np.ndarray | None = None     # [x, y, theta]
        self._peer_poses: dict[str, np.ndarray] = {}  # {peer_id: [x, y, theta]}

        # ── Subscriptions ────────────────────────────────────────────────────
        vc_odom_topic = self.get_parameter('vc_odom_topic').value
        vc_vel_topic  = self.get_parameter('vc_cmd_vel_topic').value

        self.create_subscription(Odometry, vc_odom_topic, self._vc_odom_cb, 10)
        self.create_subscription(Twist,    vc_vel_topic,  self._vc_vel_cb,  10)
        self.create_subscription(
            Odometry, f'/{self._robot_id}/ekf/odom', self._own_pose_cb, 10
        )
        for peer in self._neighbors:
            self.create_subscription(
                Odometry, f'/{peer}/ekf/odom',
                lambda msg, p=peer: self._peer_pose_cb(msg, p), 10
            )

        # ── Publishers ───────────────────────────────────────────────────────
        self._cmd_pub   = self.create_publisher(Twist,     f'/{self._robot_id}/cmd_vel', 10)
        self._state_pub = self.create_publisher(PoseArray, '/formation/state',           10)

        # Dynamic offset updates from alignment_node
        # poses[0]=own offset, poses[1+]=neighbor offsets (same order as neighbors param)
        self.create_subscription(PoseArray, '/formation/offsets', self._offsets_cb, 10)

        self.create_timer(self.DT, self._control_loop)

        self.get_logger().info(
            f'laplacian_formation_node ready: id={self._robot_id} '
            f'own_offset={self._own_off.tolist()} '
            f'peers={self._neighbors} '
            f'K_leader={self._K_leader} K_peer={self._K_peer} V_MAX={self._V_MAX}'
        )

    # ── Subscription callbacks ────────────────────────────────────────────────

    def _offsets_cb(self, msg: PoseArray):
        """Dynamic offset update from alignment_node (overrides launch params)."""
        if len(msg.poses) < 1:
            return
        self._own_off = np.array([msg.poses[0].position.x, msg.poses[0].position.y])
        for i, nb in enumerate(self._neighbors):
            if i + 1 < len(msg.poses):
                self._peer_off[nb] = np.array([
                    msg.poses[i + 1].position.x,
                    msg.poses[i + 1].position.y,
                ])

    def _vc_odom_cb(self, msg: Odometry):
        h = _yaw_from_quat(msg.pose.pose.orientation)
        self._vc_pose = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            h,
        ])

    def _vc_vel_cb(self, msg: Twist):
        # navigation_node publishes vc_vel in BODY frame (relative to leader).
        # When leader heading == vc heading (rigid 2-robot formation), this
        # equals the world-frame VC velocity.
        self._vc_vel  = np.array([msg.linear.x, msg.linear.y])
        self._vc_omega = float(msg.angular.z)

    def _own_pose_cb(self, msg: Odometry):
        h = _yaw_from_quat(msg.pose.pose.orientation)
        self._own_pose = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            h,
        ])

    def _peer_pose_cb(self, msg: Odometry, peer_id: str):
        h = _yaw_from_quat(msg.pose.pose.orientation)
        self._peer_poses[peer_id] = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            h,
        ])

    # ── Control loop (20 Hz) ──────────────────────────────────────────────────

    def _control_loop(self):
        if self._own_pose is None or self._vc_pose is None:
            return

        own_pos = self._own_pose[:2]
        vc_pos  = self._vc_pose[:2]
        vc_h    = self._vc_pose[2]
        R       = _rot2d(vc_h)

        # ── Leader correction term: toward own desired position from VC ──────
        p_from_vc = vc_pos + R @ self._own_off
        v_world   = self._vc_vel.copy()
        v_world  += self._K_leader * (p_from_vc - own_pos)

        # ── Peer correction terms: toward own desired position from each peer
        for peer_id, peer_off in self._peer_off.items():
            if peer_id not in self._peer_poses:
                continue
            peer_pos    = self._peer_poses[peer_id][:2]
            p_from_peer = peer_pos + R @ (self._own_off - peer_off)
            v_world    += self._K_peer * (p_from_peer - own_pos)

        # ── Cap world-frame speed ────────────────────────────────────────────
        speed = float(np.linalg.norm(v_world))
        if speed > self._V_MAX:
            v_world *= self._V_MAX / speed

        # ── Convert world frame → body frame (holonomic robot) ───────────────
        own_h = self._own_pose[2]
        c, s  = math.cos(own_h), math.sin(own_h)
        vx_body =  c * v_world[0] + s * v_world[1]
        vy_body = -s * v_world[0] + c * v_world[1]

        cmd = Twist()
        cmd.linear.x  = float(vx_body)
        cmd.linear.y  = float(vy_body)
        cmd.angular.z = float(self._vc_omega)
        self._cmd_pub.publish(cmd)

        # ── Publish formation state for formation_size_node ─────────────────
        pa = PoseArray()
        pa.header.stamp    = self.get_clock().now().to_msg()
        pa.header.frame_id = 'odom'
        p0 = Pose()
        p0.position.x = float(own_pos[0])
        p0.position.y = float(own_pos[1])
        p0.orientation.w = 1.0
        pa.poses.append(p0)
        for peer in self._neighbors:
            if peer not in self._peer_poses:
                continue
            p = Pose()
            p.position.x = float(self._peer_poses[peer][0])
            p.position.y = float(self._peer_poses[peer][1])
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
