import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseArray, Pose
from nav_msgs.msg import Odometry
import numpy as np


class LaplacianFormationController(Node):
    def __init__(self):
        super().__init__('laplacian_formation_node')

        self.declare_parameter('robot_id', 'tb3_0')
        self.declare_parameter('neighbors', ['tb3_1'])
        self.declare_parameter('k_gain', 1.5)
        self.declare_parameter('robot_index', 0)
        # This robot's [x, y] offset from the virtual center
        self.declare_parameter('my_offset', [0.0, 0.5])
        # Flat list [n0_x, n0_y, n1_x, n1_y, ...] in the same order as 'neighbors'
        self.declare_parameter('neighbor_offsets', [0.0, -0.5])

        self.robot_id = self.get_parameter('robot_id').value
        self.neighbors = self.get_parameter('neighbors').value
        self.k_gain = self.get_parameter('k_gain').value

        self.my_desired = np.array(self.get_parameter('my_offset').value, dtype=float)
        nb_off = np.array(self.get_parameter('neighbor_offsets').value, dtype=float).reshape(-1, 2)
        self.neighbor_desired = {n: nb_off[i] for i, n in enumerate(self.neighbors)}

        self.my_pose = np.zeros(2)
        self.neighbor_poses = {n: np.zeros(2) for n in self.neighbors}
        self.virtual_vel = np.zeros(2)
        self.virtual_angular = 0.0

        # Consume EKF-fused pose — never raw /odom
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
            Twist, '/virtual_center/cmd_vel', self._virtual_cmd_cb, 10
        )
        self.create_subscription(
            PoseArray, '/formation/offsets', self._offsets_cb, 10
        )

        self._cmd_pub = self.create_publisher(Twist, f'/{self.robot_id}/cmd_vel', 10)
        self._state_pub = self.create_publisher(PoseArray, '/formation/state', 10)

        self.create_timer(0.05, self._control_loop)
        self.get_logger().info(f'LaplacianFormationController ready for {self.robot_id}')

    def _odom_cb(self, msg: Odometry):
        self.my_pose[0] = msg.pose.pose.position.x
        self.my_pose[1] = msg.pose.pose.position.y

    def _neighbor_cb(self, msg: Odometry, neighbor_id: str):
        self.neighbor_poses[neighbor_id][0] = msg.pose.pose.position.x
        self.neighbor_poses[neighbor_id][1] = msg.pose.pose.position.y

    def _virtual_cmd_cb(self, msg: Twist):
        self.virtual_vel[0] = msg.linear.x
        self.virtual_vel[1] = msg.linear.y
        self.virtual_angular = msg.angular.z

    def _offsets_cb(self, msg: PoseArray):
        robot_index = self.get_parameter('robot_index').value
        if robot_index < len(msg.poses):
            p = msg.poses[robot_index]
            self.my_desired = np.array([p.position.x, p.position.y])
        remaining = [msg.poses[i] for i in range(len(msg.poses)) if i != robot_index]
        for i, n in enumerate(self.neighbors):
            if i < len(remaining):
                p = remaining[i]
                self.neighbor_desired[n] = np.array([p.position.x, p.position.y])

    def _control_loop(self):
        consensus = np.zeros(2)
        for neighbor in self.neighbors:
            actual_diff = self.my_pose - self.neighbor_poses[neighbor]
            desired_diff = self.my_desired - self.neighbor_desired[neighbor]
            consensus -= self.k_gain * (actual_diff - desired_diff)

        cmd = Twist()
        cmd.linear.x = float(self.virtual_vel[0] + consensus[0])
        cmd.linear.y = float(self.virtual_vel[1] + consensus[1])
        cmd.angular.z = self.virtual_angular
        self._cmd_pub.publish(cmd)

        # Publish known formation poses (self + all neighbors) for formation_size_node
        pa = PoseArray()
        pa.header.stamp = self.get_clock().now().to_msg()
        pa.header.frame_id = 'odom'
        p0 = Pose()
        p0.position.x, p0.position.y = float(self.my_pose[0]), float(self.my_pose[1])
        p0.orientation.w = 1.0
        pa.poses = [p0]
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
