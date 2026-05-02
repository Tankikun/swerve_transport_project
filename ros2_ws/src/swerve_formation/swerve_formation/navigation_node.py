"""
navigation_node.py
------------------
Runs on every robot; ONLY the elected leader activates its control loop.
The leader drives /virtual_center/cmd_vel; follower robots track via
laplacian_formation_node.

Path planning: Hybrid APF (global avoidance) + velocity ramp (local smoothing).
This is taken directly from the formation_path_planning notebook — see that
notebook for derivations and visualisations.

Subscriptions
  /formation/leader          std_msgs/String   — robot_id of current leader
  /{robot_id}/ekf/odom       nav_msgs/Odometry — authoritative pose (EKF output)
  /navigation/goal           geometry_msgs/Twist  — single goal (linear.x/y = x/y, angular.z = θ)
  /navigation/waypoints      geometry_msgs/PoseArray — ordered waypoint sequence
  /navigation/obstacles      geometry_msgs/PoseArray — obstacle centres (x, y, radius in z)
  /formation/footprint_radius std_msgs/Float32  — half-width of formation for obstacle inflation

Publications
  /virtual_center/cmd_vel    geometry_msgs/Twist — velocity for the whole formation
  /navigation/status         std_msgs/String     — "IDLE" | "NAVIGATING" | "REACHED"
"""

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseArray
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Float32


def _wrap(a: float) -> float:
    """Wrap angle to [-π, π]."""
    return float((a + np.pi) % (2 * np.pi) - np.pi)


def _rot2d(h: float) -> np.ndarray:
    c, s = np.cos(h), np.sin(h)
    return np.array([[c, -s], [s, c]])


class NavigationNode(Node):
    """
    APF + velocity-ramp navigation for the elected formation leader.

    APF parameters (from formation_path_planning.ipynb):
      K_ATT  — conic attraction gain  (constant-magnitude pull toward goal)
      K_REP  — Khatib repulsion gain
      D_REP  — obstacle influence radius [m]
      SAFETY — extra clearance added to obstacle radius = formation half-width

    Velocity ramp (prevents snap / overshoot):
      ACC_MAX   — max linear acceleration  [m/s²]
      ALPHA_MAX — max angular acceleration [rad/s²]
    """

    # ── APF parameters (tuned in notebook, safe for 3D-printed joints) ───────
    K_ATT     = 1.0    # attractive gain
    K_REP     = 0.8    # repulsive gain
    D_REP     = 1.2    # obstacle influence radius [m]

    # ── Speed limits (XL430 physical max ≈ 0.20 m/s robot speed) ─────────────
    MAX_LINEAR  = 0.18   # m/s   (kept just below motor limit for safety)
    MAX_ANGULAR = 0.45   # rad/s
    SLOW_RADIUS = 0.40   # m     — start slowing down when this close to goal

    # ── Velocity ramp (from notebook simulation loop) ─────────────────────────
    ACC_MAX   = 0.15   # m/s²
    ALPHA_MAX = 0.25   # rad/s²

    # ── Heading control ───────────────────────────────────────────────────────
    K_HEADING = 3.0    # gain for aligning heading with velocity direction

    # ── Goal tolerance (default; overridable via ROS parameter `goal_tolerance`)
    # Sim default 0.05 m is fine; on real hardware bump to ~0.15 m to absorb
    # wheel slip and EKF drift (otherwise REACHED never fires).
    GOAL_TOL = 0.05    # m

    # ── Default formation safety margin (overridden by /formation/footprint_radius)
    DEFAULT_SAFETY = 0.45   # m  (≈ robot half-width + payload margin)

    # ── Control loop period ───────────────────────────────────────────────────
    DT = 0.05   # s  (20 Hz)

    def __init__(self):
        super().__init__('navigation_node')
        self.declare_parameter('robot_id', 'tb3_0')
        self.declare_parameter('goal_tolerance', self.GOAL_TOL)
        self._robot_id  = self.get_parameter('robot_id').value
        self._goal_tol  = float(self.get_parameter('goal_tolerance').value)

        # State
        self._is_leader  = False
        self._pose       = np.zeros(3)          # [x, y, theta]  from EKF
        self._waypoints: list[np.ndarray] = []  # queue of [x, y, theta]
        self._current_wp: np.ndarray | None = None

        # Obstacles: list of (cx, cy, radius) tuples
        self._obstacles: list[tuple[float, float, float]] = []
        self._safety = self.DEFAULT_SAFETY   # updated by /formation/footprint_radius

        # Velocity state (for ramp smoothing)
        self._vel_actual   = np.zeros(2)   # [vx, vy] in world frame
        self._omega_actual = 0.0

        # ── Subscriptions ────────────────────────────────────────────────────
        self.create_subscription(
            String, '/formation/leader', self._leader_cb, 10
        )
        self.create_subscription(
            Odometry, f'/{self._robot_id}/ekf/odom', self._pose_cb, 10
        )
        self.create_subscription(
            Twist, '/navigation/goal', self._goal_cb, 10
        )
        self.create_subscription(
            PoseArray, '/navigation/waypoints', self._waypoints_cb, 10
        )
        # Obstacles from SLAM / manual: pose.position.x/y = centre, pose.position.z = radius
        self.create_subscription(
            PoseArray, '/navigation/obstacles', self._obstacles_cb, 10
        )
        # Formation footprint from formation_size_node → inflates obstacle exclusion zones
        self.create_subscription(
            Float32, '/formation/footprint_radius', self._footprint_cb, 10
        )

        # ── Publishers ───────────────────────────────────────────────────────
        self._cmd_pub    = self.create_publisher(Twist, '/virtual_center/cmd_vel', 10)
        self._status_pub = self.create_publisher(String, '/navigation/status', 10)

        self.create_timer(self.DT, self._control_loop)
        self.get_logger().info(
            f'navigation_node ready ({self._robot_id}) — APF + velocity ramp '
            f'goal_tolerance={self._goal_tol:.3f} m'
        )

    # ── Subscription callbacks ────────────────────────────────────────────────

    def _leader_cb(self, msg: String):
        was_leader  = self._is_leader
        self._is_leader = (msg.data == self._robot_id)
        if self._is_leader and not was_leader:
            self.get_logger().info('Became leader — navigation active.')
            self._reset_ramp()
        elif not self._is_leader and was_leader:
            self.get_logger().info('Lost leadership — halting.')
            self._publish_stop()

    def _pose_cb(self, msg: Odometry):
        self._pose[0] = msg.pose.pose.position.x
        self._pose[1] = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self._pose[2] = np.arctan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

    def _goal_cb(self, msg: Twist):
        """Single goal via Twist: linear.x/y = target x/y, angular.z = target θ."""
        self._waypoints.clear()
        self._current_wp = np.array([msg.linear.x, msg.linear.y, msg.angular.z])
        self._reset_ramp()
        self.get_logger().info(
            f'New goal: x={msg.linear.x:.2f} y={msg.linear.y:.2f} θ={msg.angular.z:.2f}'
        )

    def _waypoints_cb(self, msg: PoseArray):
        """Ordered waypoint sequence — each pose: position x/y = target, yaw = θ."""
        new_wps = []
        for p in msg.poses:
            q = p.orientation
            yaw = np.arctan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            )
            new_wps.append(np.array([p.position.x, p.position.y, yaw]))
        if not new_wps:
            return
        self._waypoints    = new_wps[1:]
        self._current_wp   = new_wps[0]
        self._reset_ramp()
        self.get_logger().info(f'Waypoint sequence loaded: {len(new_wps)} points.')

    def _obstacles_cb(self, msg: PoseArray):
        """Obstacle list: pose.position.x/y = centre, z = radius."""
        self._obstacles = [
            (p.position.x, p.position.y, max(p.position.z, 0.05))
            for p in msg.poses
        ]

    def _footprint_cb(self, msg: Float32):
        """Formation half-width from formation_size_node — inflates obstacle zones."""
        self._safety = max(float(msg.data), 0.1)

    # ── APF planner (from formation_path_planning.ipynb) ─────────────────────

    def _apf_velocity(self) -> tuple[np.ndarray, float]:
        """
        Compute desired (v_world [2], omega) for the virtual centre using APF.

        Layer 1 — APF (global, obstacle-aware):
          f_att  = K_ATT * (goal - pos) / ||goal - pos||   ← conic: constant magnitude
          f_rep  = K_REP * (1/d - 1/D_REP) / d² * n̂       ← Khatib, inflated radius

        Layer 2 — velocity ramp (local smoothing, applied in control loop).

        Returns (v_des [2], omega_des) in world frame.
        Caller applies the ramp before publishing.
        """
        pos  = self._pose[:2]
        goal = self._current_wp[:2]

        to_goal = goal - pos
        d_goal  = np.linalg.norm(to_goal)

        if d_goal < self._goal_tol:
            return np.zeros(2), 0.0

        # ── Attractive force (conic: unit vector, constant magnitude) ─────
        f_att = self.K_ATT * to_goal / d_goal

        # ── Repulsive forces (Khatib, safety-inflated obstacle radius) ────
        f_rep = np.zeros(2)
        for (ox, oy, r) in self._obstacles:
            r_eff = r + self._safety
            diff  = pos - np.array([ox, oy])
            d_raw = np.linalg.norm(diff)
            d     = max(d_raw - r_eff, 0.01)   # clear-distance to inflated surface
            if d < self.D_REP:
                n_hat  = diff / max(d_raw, 1e-6)
                mag    = self.K_REP * (1.0/d - 1.0/self.D_REP) / (d ** 2)
                f_rep += mag * n_hat

        f_total = f_att + f_rep
        f_mag   = np.linalg.norm(f_total)

        if f_mag < 1e-6:
            return np.zeros(2), 0.0

        # Speed: scale by force magnitude up to MAX_LINEAR, slow near goal
        speed = min(self.MAX_LINEAR, f_mag)
        if d_goal < self.SLOW_RADIUS:
            speed *= d_goal / self.SLOW_RADIUS

        v_des = (speed / f_mag) * f_total   # world-frame velocity

        # Heading: align with velocity direction when moving fast enough
        if np.linalg.norm(v_des) > 0.05:
            desired_heading = np.arctan2(v_des[1], v_des[0])
        else:
            desired_heading = self._current_wp[2]   # use goal heading when nearly stopped

        omega_des = np.clip(
            self.K_HEADING * _wrap(desired_heading - self._pose[2]),
            -self.MAX_ANGULAR, self.MAX_ANGULAR
        )

        return v_des, omega_des

    # ── Velocity ramp (from notebook simulation loop) ────────────────────────

    def _apply_ramp(self, v_des: np.ndarray, omega_des: float):
        """
        Smoothly ramp velocity toward v_des / omega_des.
        Limits: ACC_MAX [m/s²], ALPHA_MAX [rad/s²].
        """
        dv = v_des - self._vel_actual
        dv_mag = np.linalg.norm(dv)
        max_dv = self.ACC_MAX * self.DT
        if dv_mag > max_dv:
            self._vel_actual = self._vel_actual + (max_dv / dv_mag) * dv
        else:
            self._vel_actual = v_des.copy()

        domega = np.clip(
            omega_des - self._omega_actual,
            -self.ALPHA_MAX * self.DT,
             self.ALPHA_MAX * self.DT
        )
        self._omega_actual += domega

    # ── Control loop (20 Hz) ─────────────────────────────────────────────────

    def _control_loop(self):
        if not self._is_leader or self._current_wp is None:
            return

        # Check if current waypoint is reached
        pos = self._pose[:2]
        if np.linalg.norm(self._current_wp[:2] - pos) < self._goal_tol:
            if self._waypoints:
                self._current_wp = self._waypoints.pop(0)
                self.get_logger().info(
                    f'Waypoint reached — moving to next ({len(self._waypoints)} remaining).'
                )
                self._reset_ramp()
            else:
                self.get_logger().info('Final goal reached — stopping.')
                self._current_wp = None
                self._publish_stop()
                self._publish_status('REACHED')
                return

        # APF → desired world-frame velocity
        v_des, omega_des = self._apf_velocity()

        # Velocity ramp
        self._apply_ramp(v_des, omega_des)

        # World frame → body frame (holonomic robot)
        th = self._pose[2]
        vx =  self._vel_actual[0] * np.cos(th) + self._vel_actual[1] * np.sin(th)
        vy = -self._vel_actual[0] * np.sin(th) + self._vel_actual[1] * np.cos(th)

        cmd = Twist()
        cmd.linear.x  = float(np.clip(vx, -self.MAX_LINEAR,  self.MAX_LINEAR))
        cmd.linear.y  = float(np.clip(vy, -self.MAX_LINEAR,  self.MAX_LINEAR))
        cmd.angular.z = float(np.clip(self._omega_actual, -self.MAX_ANGULAR, self.MAX_ANGULAR))
        self._cmd_pub.publish(cmd)
        self._publish_status('NAVIGATING')

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _reset_ramp(self):
        self._vel_actual   = np.zeros(2)
        self._omega_actual = 0.0

    def _publish_stop(self):
        self._cmd_pub.publish(Twist())
        self._reset_ramp()

    def _publish_status(self, s: str):
        msg = String()
        msg.data = s
        self._status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
