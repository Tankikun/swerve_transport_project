"""
path_follower_node.py
---------------------
Waypoint follower for the elected formation leader.

Inputs come from the laptop UI as two one-shot messages:
  1. After localization:  /formation/offset    — PoseArray (frame_id "map")
                                                  with one pose per robot, where
                                                  pose.position.(x, y) is that
                                                  robot's WORLD-frame offset from
                                                  the virtual centre. This node
                                                  reads the entry at the
                                                  `formation_index` parameter and
                                                  uses it directly to compute the
                                                  virtual centre as
                                                  centre = leader_xy − offset.
  2. After goal click:    /navigation/waypoints — ordered waypoint list
                                                  (planned with A* + APF).

The follower drives /virtual_center/cmd_vel toward each waypoint in turn,
advancing when the virtual centre is within tolerance of the current target,
and declares success when the final waypoint is reached.

This node does NOT plan paths and does NOT do obstacle avoidance — that is the
laptop planner's responsibility. Its only job is to follow a list of dots.

Subscriptions
  /formation/leader        std_msgs/String         — robot_id of current leader
  /{robot_id}/ekf/odom     nav_msgs/Odometry       — authoritative pose (EKF output)
  /navigation/waypoints    geometry_msgs/PoseArray — ordered waypoint list (in map frame)
  /formation/offset        geometry_msgs/PoseArray — per-robot offsets from virtual
                                                     centre, in WORLD/map frame.
                                                     Read poses[formation_index].position

Publications
  /virtual_center/cmd_vel  geometry_msgs/Twist     — velocity for the formation
  /path_follower/status    std_msgs/String         — "IDLE" | "FOLLOWING" | "REACHED"
"""

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseArray
from nav_msgs.msg import Odometry
from std_msgs.msg import String


def _wrap(a: float) -> float:
    """Wrap an angle to [-pi, pi]."""
    return float((a + np.pi) % (2 * np.pi) - np.pi)


def _yaw_from_quat(q) -> float:
    """Extract yaw from a geometry_msgs/Quaternion (assumes near-flat ground)."""
    return float(np.arctan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    ))


class PathFollowerNode(Node):
    """Pure waypoint follower — no planning, no obstacle avoidance."""

    # ── Speed limits ─────────────────────────────────────────────────────────
    MAX_LINEAR  = 0.18    # m/s   (just below XL430 motor limit)
    MAX_ANGULAR = 0.45    # rad/s

    # ── Approach behaviour ──────────────────────────────────────────────────
    WP_TOL_INTERMEDIATE = 0.15  # m — advance to next dot when this close
    WP_TOL_FINAL        = 0.05  # m — tighter tolerance for the last dot
    SLOW_RADIUS         = 0.30  # m — start slowing on final approach

    # ── Control gains ───────────────────────────────────────────────────────
    K_LIN     = 1.5   # position error → desired speed
    K_HEADING = 3.0   # heading error  → angular velocity

    # ── Velocity ramp (prevents jerk between waypoints) ─────────────────────
    ACC_MAX   = 0.15  # m/s²
    ALPHA_MAX = 0.25  # rad/s²

    # ── Loop period ─────────────────────────────────────────────────────────
    DT = 0.05  # s   (20 Hz)

    def __init__(self):
        super().__init__('path_follower_node')
        self.declare_parameter('robot_id', 'tb3_1')
        self.declare_parameter('formation_index', 0)
        self._robot_id        = self.get_parameter('robot_id').value
        self._formation_index = int(self.get_parameter('formation_index').value)

        # ── State ────────────────────────────────────────────────────────────
        self._is_leader  = False
        self._pose       = np.zeros(3)           # [x, y, theta] from EKF, in map frame
        self._waypoints: list[np.ndarray] = []   # remaining dots after current
        self._current_wp: np.ndarray | None = None

        # Velocity ramp state (kept in WORLD frame; rotated to body before publish)
        self._vel_actual   = np.zeros(2)
        self._omega_actual = 0.0

        # Offset of THIS robot from the virtual centre, in the WORLD/map frame
        # [m]. Sent once by the UI after localization. Used directly to compute
        # the virtual centre as centre = leader_xy − offset.
        self._offset_world: np.ndarray | None = None
        self._warned_no_offset = False

        # ── Subscriptions ────────────────────────────────────────────────────
        self.create_subscription(String,   '/formation/leader',
                                 self._leader_cb, 10)
        self.create_subscription(Odometry, f'/{self._robot_id}/ekf/odom',
                                 self._pose_cb, 10)
        self.create_subscription(PoseArray, '/navigation/waypoints',
                                 self._waypoints_cb, 10)
        self.create_subscription(PoseArray, '/formation/offset',
                                 self._offset_cb, 10)

        # ── Publishers ───────────────────────────────────────────────────────
        self._cmd_pub    = self.create_publisher(Twist,  '/virtual_center/cmd_vel', 10)
        self._status_pub = self.create_publisher(String, '/path_follower/status', 10)

        # ── Control loop @ 20 Hz ─────────────────────────────────────────────
        self.create_timer(self.DT, self._control_loop)

        self.get_logger().info(
            f'path_follower_node ready ({self._robot_id}) — waypoint follower'
        )

    # ── Callbacks ────────────────────────────────────────────────────────────

    def _leader_cb(self, msg: String):
        """Activate the control loop only when this robot is the elected leader."""
        was_leader = self._is_leader
        self._is_leader = (msg.data == self._robot_id)
        if self._is_leader and not was_leader:
            self.get_logger().info('Became leader — follower active.')
            self._reset_ramp()
        elif not self._is_leader and was_leader:
            self.get_logger().info('Lost leadership — halting.')
            self._publish_stop()

    def _pose_cb(self, msg: Odometry):
        """Cache the latest EKF pose."""
        self._pose[0] = msg.pose.pose.position.x
        self._pose[1] = msg.pose.pose.position.y
        self._pose[2] = _yaw_from_quat(msg.pose.pose.orientation)

    def _waypoints_cb(self, msg: PoseArray):
        """Replace the current waypoint list with a new one from the planner."""
        if not msg.poses:
            self.get_logger().warn('Empty waypoint list — ignoring.')
            return
        wps = [
            np.array([p.position.x, p.position.y, _yaw_from_quat(p.orientation)])
            for p in msg.poses
        ]
        self._current_wp = wps[0]
        self._waypoints  = wps[1:]
        self._reset_ramp()
        self.get_logger().info(f'Received {len(wps)} waypoints.')

    def _offset_cb(self, msg: PoseArray):
        """
        Cache this robot's WORLD-frame offset from the virtual centre.

        Message contract: PoseArray in the map frame, one pose per robot.
        We pick poses[formation_index] and store its (x, y) directly — no
        rotation. The virtual centre is then simply leader_xy − offset_world.
        """
        if self._formation_index >= len(msg.poses):
            self.get_logger().warn(
                f'Offset PoseArray has {len(msg.poses)} entries; '
                f'formation_index={self._formation_index} is out of range — ignoring.'
            )
            return

        p = msg.poses[self._formation_index].position
        offset = np.array([float(p.x), float(p.y)])

        first = self._offset_world is None
        self._offset_world = offset

        if first:
            self.get_logger().info(
                f'Offset received (idx={self._formation_index}): '
                f'({offset[0]:.3f}, {offset[1]:.3f}) m in map frame.'
            )

    # ── Control loop ─────────────────────────────────────────────────────────

    def _control_loop(self):
        if not self._is_leader or self._current_wp is None:
            return

        pos       = self._virtual_center_xy()
        target_xy = self._current_wp[:2]
        dist      = float(np.linalg.norm(target_xy - pos))

        is_final = (len(self._waypoints) == 0)
        tol      = self.WP_TOL_FINAL if is_final else self.WP_TOL_INTERMEDIATE

        # 1. Reached the current waypoint?
        if dist < tol:
            if is_final:
                self.get_logger().info('Final waypoint reached — stopping.')
                self._current_wp = None
                self._publish_stop()
                self._publish_status('REACHED')
                return
            self._current_wp = self._waypoints.pop(0)
            self.get_logger().info(
                f'Waypoint reached — advancing ({len(self._waypoints)} remaining).'
            )
            return  # don't reset ramp: keeps motion smooth across waypoints

        # 2. Desired world-frame velocity (P controller toward the target).
        direction = (target_xy - pos) / max(dist, 1e-6)
        speed     = self.K_LIN * dist
        if is_final and dist < self.SLOW_RADIUS:
            speed = min(speed, self.MAX_LINEAR * (dist / self.SLOW_RADIUS))
        speed = min(speed, self.MAX_LINEAR)
        v_des = speed * direction

        # 3. Desired heading: face along motion when moving; otherwise hold the
        #    leader's current heading. We don't trust waypoint yaw because the
        #    laptop planner may not fill it in — defaulting to 0 would make the
        #    robot snap to face world east.
        if speed > 0.05:
            desired_heading = float(np.arctan2(v_des[1], v_des[0]))
        else:
            desired_heading = float(self._pose[2])
        omega_des = float(np.clip(
            self.K_HEADING * _wrap(desired_heading - self._pose[2]),
            -self.MAX_ANGULAR, self.MAX_ANGULAR,
        ))

        # 4. Smooth via velocity ramp.
        self._apply_ramp(v_des, omega_des)

        # 5. World → body frame (holonomic / swerve base).
        th = self._pose[2]
        vx =  self._vel_actual[0] * np.cos(th) + self._vel_actual[1] * np.sin(th)
        vy = -self._vel_actual[0] * np.sin(th) + self._vel_actual[1] * np.cos(th)

        # 6. Publish.
        cmd = Twist()
        cmd.linear.x  = float(np.clip(vx, -self.MAX_LINEAR, self.MAX_LINEAR))
        cmd.linear.y  = float(np.clip(vy, -self.MAX_LINEAR, self.MAX_LINEAR))
        cmd.angular.z = float(np.clip(self._omega_actual, -self.MAX_ANGULAR, self.MAX_ANGULAR))
        self._cmd_pub.publish(cmd)
        self._publish_status('FOLLOWING')

    # ── Helpers ──────────────────────────────────────────────────────────────

    def _virtual_center_xy(self) -> np.ndarray:
        """
        Compute the virtual centre's (x, y) in the world/map frame.

            centre = leader_xy − offset_world

        Both terms live in the map frame, so it's a plain subtraction — no
        rotation. If the offset hasn't been received yet, fall back to the
        leader's raw pose and warn once so testing isn't blocked.
        """
        if self._offset_world is None:
            if not self._warned_no_offset:
                self.get_logger().warn(
                    'No offset yet — using leader pose as virtual centre.'
                )
                self._warned_no_offset = True
            return self._pose[:2].copy()

        return self._pose[:2] - self._offset_world

    def _apply_ramp(self, v_des: np.ndarray, omega_des: float):
        """Move actual velocity toward desired by at most ACC_MAX*DT (and ALPHA_MAX*DT)."""
        dv     = v_des - self._vel_actual
        dv_mag = float(np.linalg.norm(dv))
        max_dv = self.ACC_MAX * self.DT
        if dv_mag > max_dv:
            self._vel_actual = self._vel_actual + (max_dv / dv_mag) * dv
        else:
            self._vel_actual = v_des.copy()

        domega = float(np.clip(
            omega_des - self._omega_actual,
            -self.ALPHA_MAX * self.DT,
             self.ALPHA_MAX * self.DT,
        ))
        self._omega_actual += domega

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
    node = PathFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
