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


def _pure_pursuit_target(path, robot_xy, lookahead):
    """Pure-pursuit lookahead carrot (Coulter 1992).

    Finds a point `lookahead` meters ahead of the robot's closest
    projection onto the polyline `path`. Returns (tx, ty).

    Pure-pursuit treats the path as a continuous curve rather than a
    sequence of discrete waypoints, so motion is smooth regardless of
    waypoint density. This kills the ballistic line-to-line motion that
    happens when conic attraction targets one waypoint at a time.

    Args:
        path:       list of (x, y) tuples, ordered start → goal
        robot_xy:   current (x, y)
        lookahead:  meters ahead on the path to aim at

    Returns:
        (tx, ty) world-frame target. Falls back to path[-1] when the
        lookahead walks past the end of the path.
    """
    if len(path) < 2:
        return path[-1]
    rx, ry = float(robot_xy[0]), float(robot_xy[1])

    # Step A — find closest projection onto the polyline
    best_seg, best_t, best_d2 = 0, 0.0, float("inf")
    for i in range(len(path) - 1):
        ax, ay = float(path[i][0]),     float(path[i][1])
        bx, by = float(path[i + 1][0]), float(path[i + 1][1])
        dx, dy = bx - ax, by - ay
        seg_len2 = dx * dx + dy * dy
        if seg_len2 < 1e-9:
            t = 0.0
        else:
            t = ((rx - ax) * dx + (ry - ay) * dy) / seg_len2
            t = max(0.0, min(1.0, t))
        px, py = ax + t * dx, ay + t * dy
        d2 = (rx - px) ** 2 + (ry - py) ** 2
        if d2 < best_d2:
            best_d2, best_seg, best_t = d2, i, t

    # Step B — walk forward `lookahead` meters from (best_seg, best_t)
    remaining = float(lookahead)
    seg, t = best_seg, best_t
    while seg < len(path) - 1:
        ax, ay = float(path[seg][0]),     float(path[seg][1])
        bx, by = float(path[seg + 1][0]), float(path[seg + 1][1])
        dx, dy = bx - ax, by - ay
        seg_len = float(np.hypot(dx, dy))
        rem_in_seg = (1.0 - t) * seg_len
        if remaining <= rem_in_seg:
            new_t = t + remaining / seg_len if seg_len > 1e-9 else 0.0
            return (ax + new_t * dx, ay + new_t * dy)
        remaining -= rem_in_seg
        seg += 1
        t = 0.0
    return (float(path[-1][0]), float(path[-1][1]))


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

    # ── Pure-pursuit follower (Coulter 1992) ─────────────────────────────────
    LOOKAHEAD = 0.20    # m   — distance ahead on the polyline to aim at.
                        # Smooths through corners and pulls the robot back to
                        # the line if it drifts. Should be >= GOAL_TOL_INTERMEDIATE
                        # and roughly equal to the planner's target_spacing
                        # so the carrot stays on a real segment.

    # ── Goal tolerance (two-phase arrival: XY first, then in-place rotation) ─
    GOAL_TOL              = 0.05    # m   — FINAL XY arrival tolerance
    GOAL_TOL_INTERMEDIATE = 0.20    # m   — intermediate-waypoint pop tolerance.
                                    # apf_refine_path produces ~5–30 cm spaced
                                    # waypoints; 0.05 m tol pops each in 1 tick
                                    # and the planner's curve collapses into
                                    # ballistic point-to-point hops. 0.20 m
                                    # lets the controller actually trace the
                                    # polyline.
    YAW_TOL  = 0.05    # rad — yaw arrival tolerance (~3°)

    # ── Default formation safety margin (overridden by /formation/footprint_radius)
    DEFAULT_SAFETY = 0.45   # m  (≈ robot half-width + payload margin)

    # ── Control loop period ───────────────────────────────────────────────────
    DT = 0.05   # s  (20 Hz)

    def __init__(self):
        super().__init__('navigation_node')
        self.declare_parameter('robot_id', 'tb3_0')
        self._robot_id = self.get_parameter('robot_id').value

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

        # Two-phase arrival state: 'DRIVE' (chasing waypoint XY) →
        # 'ROTATE' (XY reached, spinning to goal yaw in place) → 'DONE'.
        self._phase = 'DRIVE'

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
            f'navigation_node ready ({self._robot_id}) — APF + velocity ramp'
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
        self._phase = 'DRIVE'
        self.get_logger().info(
            f'New goal: x={msg.linear.x:.2f} y={msg.linear.y:.2f} θ={msg.angular.z:.2f}'
        )

    def _waypoints_cb(self, msg: PoseArray):
        """Ordered waypoint sequence — each pose: position x/y = target, yaw = θ.

        The laptop-side planner runs A* + an APF refinement pass: waypoints
        are densified near obstacles and pre-shifted away from walls, so by
        the time they arrive here the line is already smooth and safe. The
        on-board APF in _apf_velocity() acts as a local safety layer in
        case sensor data picks up something the map missed.
        """
        new_wps = []
        for p in msg.poses:
            q = p.orientation
            yaw = np.arctan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            )
            new_wps.append(np.array([p.position.x, p.position.y, yaw]))
        if not new_wps:
            self.get_logger().warn('Empty waypoint sequence — ignoring.')
            return
        self._waypoints    = new_wps[1:]
        self._current_wp   = new_wps[0]
        self._reset_ramp()
        self._phase = 'DRIVE'

        # Distance to first / last waypoint and total path length, for
        # quick sanity-checking on the Pi terminal.
        first = new_wps[0][:2]; last = new_wps[-1][:2]
        d_to_first = float(np.linalg.norm(first - self._pose[:2]))
        total = 0.0
        for i in range(len(new_wps) - 1):
            total += float(np.linalg.norm(new_wps[i + 1][:2] - new_wps[i][:2]))
        self.get_logger().info(
            f'Plan received: {len(new_wps)} waypoints, '
            f'total {total:.2f} m. '
            f'First: ({first[0]:.2f}, {first[1]:.2f})  '
            f'Last: ({last[0]:.2f}, {last[1]:.2f}, '
            f'{np.rad2deg(new_wps[-1][2]):.0f}°). '
            f'd_to_first = {d_to_first:.2f} m.'
        )

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
        Compute desired (v_world [2], omega) for the virtual centre.

        Pure-pursuit (Coulter 1992) + Khatib repulsion + velocity ramp.

        Layer 1 — pure-pursuit lookahead:
          target = a point LOOKAHEAD m ahead on the polyline {current_wp, *_waypoints}
          f_att  = K_ATT * unit(target - pos)
          This treats the path as a continuous curve, so densification
          density doesn't cause line-to-line ballistic hops.

        Layer 2 — Khatib repulsion (only against /navigation/obstacles
                  — runtime dynamic objects, NOT the static map walls
                  which the planner already inflated around).

        Layer 3 — velocity ramp (applied by _apply_ramp in the control loop).

        Returns (v_des [2], omega_des) in world frame.
        """
        pos = self._pose[:2]

        # ── Build the polyline ahead of the robot ─────────────────────────
        # [current_wp, then each remaining waypoint]. We do NOT prepend pos
        # itself; pure_pursuit_target's projection step handles where the
        # robot is on this polyline.
        polyline: list[tuple[float, float]] = []
        if self._current_wp is not None:
            polyline.append((float(self._current_wp[0]),
                             float(self._current_wp[1])))
        for wp in self._waypoints:
            polyline.append((float(wp[0]), float(wp[1])))
        if len(polyline) == 0:
            return np.zeros(2), 0.0

        # ── Distance to the FINAL goal (used for slowdown only) ───────────
        final_xy = np.array(polyline[-1])
        d_final  = float(np.linalg.norm(final_xy - pos))
        if d_final < self.GOAL_TOL and len(self._waypoints) == 0:
            return np.zeros(2), 0.0

        # ── Pure-pursuit lookahead carrot ─────────────────────────────────
        target = _pure_pursuit_target(polyline, pos, self.LOOKAHEAD)
        to_target = np.array(target) - pos
        d_target  = float(np.linalg.norm(to_target))
        if d_target < 1e-6:
            return np.zeros(2), 0.0

        # ── Attractive force toward the carrot ────────────────────────────
        f_att = self.K_ATT * to_target / d_target

        # ── Khatib repulsion from runtime obstacles ───────────────────────
        f_rep = np.zeros(2)
        for (ox, oy, r) in self._obstacles:
            r_eff = r + self._safety
            diff  = pos - np.array([ox, oy])
            d_raw = float(np.linalg.norm(diff))
            d     = max(d_raw - r_eff, 0.01)
            if d < self.D_REP:
                n_hat  = diff / max(d_raw, 1e-6)
                mag    = self.K_REP * (1.0 / d - 1.0 / self.D_REP) / (d ** 2)
                f_rep += mag * n_hat

        f_total = f_att + f_rep
        f_mag   = float(np.linalg.norm(f_total))
        if f_mag < 1e-6:
            return np.zeros(2), 0.0

        # ── Speed: scale by force magnitude; slow only at the FINAL goal ──
        # SLOW_RADIUS gating to the final waypoint avoids stop-go-stop-go
        # at every densified intermediate point.
        speed = min(self.MAX_LINEAR, f_mag)
        is_final_wp = (len(self._waypoints) == 0)
        if is_final_wp and d_final < self.SLOW_RADIUS:
            speed *= d_final / self.SLOW_RADIUS

        v_des = (speed / f_mag) * f_total

        # ── Heading: aim at motion direction when fast, goal yaw when slow.
        if float(np.linalg.norm(v_des)) > 0.05:
            desired_heading = float(np.arctan2(v_des[1], v_des[0]))
        else:
            desired_heading = float(self._current_wp[2])

        omega_des = float(np.clip(
            self.K_HEADING * _wrap(desired_heading - self._pose[2]),
            -self.MAX_ANGULAR, self.MAX_ANGULAR,
        ))

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

        pos = self._pose[:2]
        d_xy = np.linalg.norm(self._current_wp[:2] - pos)

        # ── Waypoint advance / phase transition (only while DRIVE-ing) ─────
        # Loose tolerance for intermediate waypoints (so the controller
        # actually tracks the planner's curve), tight tolerance for the
        # final waypoint (so XY arrival is precise).
        wp_tol = self.GOAL_TOL_INTERMEDIATE if self._waypoints else self.GOAL_TOL
        if self._phase == 'DRIVE' and d_xy < wp_tol:
            if self._waypoints:
                # More waypoints to follow — advance immediately, no rotate.
                self._current_wp = self._waypoints.pop(0)
                self.get_logger().info(
                    f'Waypoint reached — moving to next '
                    f'({len(self._waypoints)} remaining).'
                )
                self._reset_ramp()
            else:
                # FINAL waypoint XY reached — switch to in-place rotation
                # so the robot ends at the exact yaw the user requested.
                self._phase = 'ROTATE'
                self._vel_actual = np.zeros(2)
                self.get_logger().info(
                    f'Reached XY ({pos[0]:.2f}, {pos[1]:.2f}) — '
                    f'rotating to {np.rad2deg(self._current_wp[2]):.0f}°.'
                )

        # ── ROTATE phase: zero linear velocity, spin to goal yaw ───────────
        if self._phase == 'ROTATE':
            yaw_err = _wrap(self._current_wp[2] - self._pose[2])
            if abs(yaw_err) < self.YAW_TOL:
                self.get_logger().info(
                    f'✓ ARRIVED — pose ({pos[0]:.2f}, {pos[1]:.2f}, '
                    f'{np.rad2deg(self._pose[2]):.0f}°) aligned at goal.'
                )
                self._current_wp = None
                self._phase = 'DONE'
                self._publish_stop()
                self._publish_status('REACHED')
                return

            omega_des = float(np.clip(
                self.K_HEADING * yaw_err,
                -self.MAX_ANGULAR, self.MAX_ANGULAR
            ))
            # Use the same ramp on angular velocity, but force linear to zero.
            self._apply_ramp(np.zeros(2), omega_des)
            cmd = Twist()
            cmd.linear.x  = 0.0
            cmd.linear.y  = 0.0
            cmd.angular.z = float(np.clip(self._omega_actual,
                                          -self.MAX_ANGULAR, self.MAX_ANGULAR))
            self._cmd_pub.publish(cmd)
            self._publish_status('ROTATING')
            return

        # ── DRIVE phase: existing APF + ramp + body-frame transform ────────
        v_des, omega_des = self._apf_velocity()
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
