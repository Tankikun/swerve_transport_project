"""
laplacian_formation_node.py
---------------------------
Rigid-body formation controller with optional Laplacian consensus
correction. Runs on every robot.

Each tick this node receives the operator's twist for the (imaginary)
virtual centre on /virtual_center/cmd_vel and produces THIS robot's
body-frame twist using a closed-form rigid-body transform:

    v_robot_x  = vc_vx - vc_wz * my_offset.y
    v_robot_y  = vc_vy + vc_wz * my_offset.x
    v_robot_wz = vc_wz

This is the **feedforward** layer. By itself it produces an exactly
rigid-body motion at any instant — no inter-robot pose feedback is
required to track the virtual centre's commanded twist.

Pose feedback (consensus correction)
------------------------------------
RTAB-Map runs in localisation-only mode on the robots
(`rtabmap_localization.launch.py` /
`rtabmap_laptop_localization.launch.py`) and feeds visual poses into
each robot's `ekf_node` via `slam_pose_relay_node`, so a shared
world frame IS now available — provided every robot loaded the same
.db (see those launch files' headers).

Even so, this controller's pose-feedback term is **OFF by default**
(`enable_consensus=False`). Reasons:

  1. Global re-localisation in RTAB-Map can take 5-30 s on startup.
     During that window the EKF is on pure dead-reckoning and a
     consensus term would amplify the wheel-only drift into phantom
     formation-correction commands.
  2. We have not yet hardware-validated shared-map operation
     (multiple robots localising against an identical .db). Until
     that's confirmed, treating each robot's pose as an absolute
     world-frame value is risky.

Once shared-map operation is validated, set `enable_consensus:=true`
on the launch line. The control law then becomes (per neighbour):

    desired_world = R(formation_theta) @ (my_offset - neighbour_offset)
    actual_world  = my_pose - neighbour_pose
    error         = actual_world - desired_world
    correction   -= k_gain * error                     # accumulated

    ff += correction        (added before saturation scale)

`formation_theta` is the circular mean of every robot's yaw — i.e.
the orientation of the formation in the world frame — so the
desired offset rotates with the formation as it turns.

Readiness gating ensures the correction only fires once every robot
has published at least one pose AND the most recent pose for each
neighbour is < 1 s old. If any neighbour goes stale the controller
falls back to pure feedforward and warns at most once per 5 s.

Saturation handling
-------------------
A formation-wide scaling factor caps the maximum per-wheel linear
speed across ALL robots in the formation to MAX_WHEEL_LINEAR (just
below the firmware's 0.198 m/s clamp). When the would-be outer-wheel
speed exceeds the cap, every robot's twist is scaled by the same
factor — the rigid shape is preserved, the formation just moves
slower. This is the asymmetric "outer wheel goes faster" behaviour
needed for object transport. The consensus correction (when active)
is added BEFORE this scale so it shares the same speed envelope.
"""

import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Pose, PoseArray, PoseStamped, Twist
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

# How long without an EKF pose update we tolerate before falling back
# to pure feedforward and warning. Matches the "all updated within 1 s"
# requirement in the consensus spec.
POSE_STALE_S = 1.0


def _yaw_from_quat(q) -> float:
    """Standard ROS quaternion → yaw."""
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def _circular_mean(angles) -> float:
    """Mean of yaw angles, robust across the ±π wrap."""
    sx = sum(math.sin(a) for a in angles)
    cx = sum(math.cos(a) for a in angles)
    return math.atan2(sx, cx)


class LaplacianFormationController(Node):

    DT = 0.05  # 20 Hz control loop

    def __init__(self):
        super().__init__('laplacian_formation_node')

        self.declare_parameter('robot_id', 'tb3_0')
        self.declare_parameter('neighbors', ['tb3_1'])
        self.declare_parameter('my_offset', [0.0, 0.0])
        self.declare_parameter('neighbor_offsets', [0.0, 0.0])
        # Consensus correction gain (Laplacian formulation). Active only
        # when `enable_consensus` is True. Default 0.1 is small enough to
        # produce gentle corrections (cm-scale positional error → mm/s
        # velocity contribution); raise carefully — an aggressive gain
        # combined with a noisy SLAM pose will jitter the formation.
        self.declare_parameter('k_gain', 0.1)
        # Master switch for the pose-feedback term. Off by default — see
        # the module docstring for rationale.
        self.declare_parameter('enable_consensus', False)

        self.robot_id  = self.get_parameter('robot_id').value
        self.neighbors = list(self.get_parameter('neighbors').value)
        self.k_gain    = float(self.get_parameter('k_gain').value)
        self.enable_consensus = bool(self.get_parameter('enable_consensus').value)

        my_offset_param = np.array(
            self.get_parameter('my_offset').value, dtype=float
        )
        if my_offset_param.size != 2:
            msg = (
                f"Invalid 'my_offset' parameter for {self.robot_id}: "
                f'expected exactly 2 values [x, y], got '
                f'{my_offset_param.size}: {my_offset_param.tolist()}'
            )
            self.get_logger().error(msg)
            raise ValueError(msg)
        self.my_offset = my_offset_param

        neighbor_offsets_param = np.array(
            self.get_parameter('neighbor_offsets').value, dtype=float
        )
        expected_neighbor_offset_values = 2 * len(self.neighbors)
        if neighbor_offsets_param.size != expected_neighbor_offset_values:
            msg = (
                f"Invalid 'neighbor_offsets' parameter for {self.robot_id}: "
                f'expected {expected_neighbor_offset_values} values '
                f'(2 per neighbor for {len(self.neighbors)} neighbors), got '
                f'{neighbor_offsets_param.size}: '
                f'{neighbor_offsets_param.tolist()}'
            )
            self.get_logger().error(msg)
            raise ValueError(msg)

        nb_off = neighbor_offsets_param.reshape(-1, 2)
        self.neighbor_offsets = {n: nb_off[i] for i, n in enumerate(self.neighbors)}

        # Last operator command for the virtual centre. Body-frame.
        self.virtual_vel     = np.zeros(2)
        self.virtual_angular = 0.0

        # Last-known EKF pose of self + neighbours. Three components:
        # [x, y, theta], world frame (`map` once SLAM is up; `odom` until
        # then). Always published in /formation/state for
        # formation_size_node. Only consumed by the control law when
        # `enable_consensus` is True (see _control_loop).
        self.my_pose = np.zeros(3)
        self.neighbor_poses = {n: np.zeros(3) for n in self.neighbors}

        # Per-pose update timestamps used by the consensus readiness gate.
        # `None` means "never received" — distinct from "received with
        # value (0,0,0) at startup".
        self._my_pose_t: float | None = None
        self._neighbor_pose_t: dict[str, float] = {}

        # SLAM localization flags. Consensus is suppressed until every
        # participant has received at least one SLAM fix — before that the
        # EKF poses are in separate, incompatible odom frames and any
        # relative-pose correction would be meaningless noise.
        self._self_slam_ready = False
        self._neighbor_slam_ready: dict[str, bool] = {n: False for n in self.neighbors}
        self._consensus_ever_active = False

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
        self.create_subscription(
            PoseStamped, f'/{self.robot_id}/slam/pose', self._slam_self_cb, 10
        )
        for neighbor in self.neighbors:
            self.create_subscription(
                PoseStamped,
                f'/{neighbor}/slam/pose',
                lambda msg, n=neighbor: self._slam_neighbor_cb(msg, n),
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
            f'enable_consensus={self.enable_consensus} '
            f'k_gain={self.k_gain:.3f}'
        )

    # ── Subscription callbacks ────────────────────────────────────────────────

    def _virtual_cmd_cb(self, msg: Twist):
        self.virtual_vel[0]  = float(msg.linear.x)
        self.virtual_vel[1]  = float(msg.linear.y)
        self.virtual_angular = float(msg.angular.z)

    def _odom_cb(self, msg: Odometry):
        self.my_pose[0] = msg.pose.pose.position.x
        self.my_pose[1] = msg.pose.pose.position.y
        self.my_pose[2] = _yaw_from_quat(msg.pose.pose.orientation)
        self._my_pose_t = time.time()

    def _neighbor_cb(self, msg: Odometry, neighbor_id: str):
        self.neighbor_poses[neighbor_id][0] = msg.pose.pose.position.x
        self.neighbor_poses[neighbor_id][1] = msg.pose.pose.position.y
        self.neighbor_poses[neighbor_id][2] = _yaw_from_quat(msg.pose.pose.orientation)
        self._neighbor_pose_t[neighbor_id] = time.time()

    def _slam_self_cb(self, msg: PoseStamped):
        if not self._self_slam_ready:
            self._self_slam_ready = True
            self.get_logger().info(
                f'[LOCALIZED] {self.robot_id} SLAM pose received — '
                f'self is map-anchored'
            )

    def _slam_neighbor_cb(self, msg: PoseStamped, neighbor_id: str):
        if not self._neighbor_slam_ready[neighbor_id]:
            self._neighbor_slam_ready[neighbor_id] = True
            self.get_logger().info(
                f'[LOCALIZED] {neighbor_id} SLAM pose received — '
                f'neighbor is map-anchored'
            )

    # ── Consensus readiness gate ──────────────────────────────────────────────

    def _consensus_ready(self) -> bool:
        """
        Returns True iff every pose required for the consensus correction
        has been received, is fresh, AND all participants have confirmed
        SLAM localization (i.e. their EKF is map-frame anchored, not
        dead-reckoning in a robot-local odom frame).

        - "SLAM ready": at least one /{robot_id}/slam/pose received.
          Before this, EKF positions are in separate odom frames and
          inter-robot relative poses are meaningless.
        - "Received": _my_pose_t / _neighbor_pose_t entry is not None.
          Distinct from value (0,0,0) which is a legitimate startup pose.
        - "Fresh":    last update is within POSE_STALE_S seconds.

        Startup absence (no entry at all) is silent — that's normal in
        the seconds before the first EKF message arrives. Only an
        already-seen-then-vanished neighbour produces the warning.
        """
        slam_missing = (
            ([self.robot_id] if not self._self_slam_ready else [])
            + [n for n in self.neighbors if not self._neighbor_slam_ready[n]]
        )
        if slam_missing:
            self.get_logger().warn(
                '[consensus] waiting for SLAM localization on: '
                + ', '.join(slam_missing),
                throttle_duration_sec=10.0,
            )
            return False

        if self._my_pose_t is None:
            return False
        now = time.time()
        if (now - self._my_pose_t) > POSE_STALE_S:
            self.get_logger().warn(
                f'[consensus] my own pose is stale '
                f'({now - self._my_pose_t:.1f}s) — falling back to pure '
                f'feedforward.',
                throttle_duration_sec=5.0,
            )
            return False
        for n in self.neighbors:
            t = self._neighbor_pose_t.get(n)
            if t is None:
                # never received → still in startup, silent
                return False
            if (now - t) > POSE_STALE_S:
                self.get_logger().warn(
                    f'[consensus] neighbour {n} pose stale '
                    f'({now - t:.1f}s) — falling back to pure feedforward.',
                    throttle_duration_sec=5.0,
                )
                return False
        return True

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
        # NOTE: this estimate uses only the rigid-body command; it does
        # NOT account for the consensus correction added below. With
        # k_gain=0.1 the correction is mm/s and the bound is fine; for
        # large k_gain, the post-scale velocity could exceed
        # MAX_WHEEL_LINEAR by a small margin (still inside firmware's
        # 0.198 m/s clamp).
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

        # ── Rigid-body feedforward (raw, pre-scale) ──────────────────
        rx, ry = float(self.my_offset[0]), float(self.my_offset[1])
        ff_x_raw = vc_x - vc_wz * ry
        ff_y_raw = vc_y + vc_wz * rx

        # ── Consensus correction (optional, opt-in) ───────────────────
        # Standard Laplacian formulation, lifted into the formation
        # frame:
        #   desired_world = R(formation_theta) · (my_offset - neighbor_offset)
        #   actual_world  = my_pose - neighbor_pose
        #   correction   -= k_gain · sum_n (actual_world - desired_world)
        # `formation_theta` is the circular mean of every robot's yaw,
        # so the desired offset rotates with the formation as it turns.
        # See module docstring for why this is OFF by default.
        if self.enable_consensus and self._consensus_ready():
            if not self._consensus_ever_active:
                self._consensus_ever_active = True
                self.get_logger().info(
                    '[consensus] All robots SLAM-localized — '
                    'pose-feedback correction is now active.'
                )
            yaws = [float(self.my_pose[2])] + [
                float(self.neighbor_poses[n][2]) for n in self.neighbors
            ]
            formation_theta = _circular_mean(yaws)
            c, s = math.cos(formation_theta), math.sin(formation_theta)
            R = np.array([[c, -s], [s, c]])

            correction = np.zeros(2)
            my_xy = self.my_pose[:2]
            for n in self.neighbors:
                desired_world = R @ (self.my_offset - self.neighbor_offsets[n])
                actual_world  = my_xy - self.neighbor_poses[n][:2]
                correction   -= self.k_gain * (actual_world - desired_world)

            ff_x_raw += float(correction[0])
            ff_y_raw += float(correction[1])

        # ── Apply formation-wide saturation scale ────────────────────
        ff_x  = ff_x_raw * scale
        ff_y  = ff_y_raw * scale
        ff_wz = vc_wz * scale   # angular component is never modified by consensus

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
