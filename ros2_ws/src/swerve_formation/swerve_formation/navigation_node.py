import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np


class NavigationNode(Node):
    """
    Runs on every robot; only the elected leader activates its P-controller.
    Reads /navigation/goal (Twist: linear.x/y = target x/y, angular.z = target heading)
    and drives /virtual_center/cmd_vel to move the whole formation.
    """

    MAX_LINEAR = 0.3    # m/s
    MAX_ANGULAR = 0.5   # rad/s
    KP_XY = 0.8
    KP_THETA = 1.2
    GOAL_TOL = 0.05     # metres

    def __init__(self):
        super().__init__('navigation_node')
        self.declare_parameter('robot_id', 'tb3_0')
        self._robot_id = self.get_parameter('robot_id').value

        self._is_leader = False
        self._goal: np.ndarray | None = None   # [x, y, theta]
        self._pose = np.zeros(3)               # [x, y, theta]

        self.create_subscription(String, '/formation/leader', self._leader_cb, 10)
        self.create_subscription(
            Odometry, f'/{self._robot_id}/ekf/odom', self._pose_cb, 10
        )
        # Goal format: linear.x/y → target position, angular.z → target heading
        self.create_subscription(Twist, '/navigation/goal', self._goal_cb, 10)

        self._cmd_pub = self.create_publisher(Twist, '/virtual_center/cmd_vel', 10)
        self.create_timer(0.05, self._control_loop)
        self.get_logger().info(f'Navigation node ready for {self._robot_id}')

    def _leader_cb(self, msg: String):
        was_leader = self._is_leader
        self._is_leader = (msg.data == self._robot_id)
        if self._is_leader and not was_leader:
            self.get_logger().info('Became leader — navigation control active')
        elif not self._is_leader and was_leader:
            self.get_logger().info('Lost leadership — halting navigation output')
            self._cmd_pub.publish(Twist())

    def _pose_cb(self, msg: Odometry):
        self._pose[0] = msg.pose.pose.position.x
        self._pose[1] = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._pose[2] = np.arctan2(siny, cosy)

    def _goal_cb(self, msg: Twist):
        self._goal = np.array([msg.linear.x, msg.linear.y, msg.angular.z])

    def _control_loop(self):
        if not self._is_leader or self._goal is None:
            return

        dx = self._goal[0] - self._pose[0]
        dy = self._goal[1] - self._pose[1]
        if np.hypot(dx, dy) < self.GOAL_TOL:
            self._cmd_pub.publish(Twist())
            return

        th = self._pose[2]
        # World-frame velocity → body frame (holonomic)
        vx_w = np.clip(self.KP_XY * dx, -self.MAX_LINEAR, self.MAX_LINEAR)
        vy_w = np.clip(self.KP_XY * dy, -self.MAX_LINEAR, self.MAX_LINEAR)
        vx =  vx_w * np.cos(th) + vy_w * np.sin(th)
        vy = -vx_w * np.sin(th) + vy_w * np.cos(th)

        dtheta = self._goal[2] - self._pose[2]
        dtheta = np.arctan2(np.sin(dtheta), np.cos(dtheta))
        omega = np.clip(self.KP_THETA * dtheta, -self.MAX_ANGULAR, self.MAX_ANGULAR)

        cmd = Twist()
        cmd.linear.x, cmd.linear.y, cmd.angular.z = float(vx), float(vy), float(omega)
        self._cmd_pub.publish(cmd)


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
