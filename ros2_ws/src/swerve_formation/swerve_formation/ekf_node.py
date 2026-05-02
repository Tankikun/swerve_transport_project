import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import numpy as np


class EKFNode(Node):
    """
    Fuses raw wheel odometry (prediction) with SLAM pose (correction).
    State: [x, y, theta]. Only this node reads raw /{robot_id}/odom.
    Publishes authoritative pose on /{robot_id}/ekf/odom.
    """

    def __init__(self):
        super().__init__('ekf_node')
        self.declare_parameter('robot_id', 'tb3_0')
        robot_id = self.get_parameter('robot_id').value
        # Per-robot TF frame ids — must match what conveyor_base_node and the
        # static_transform_publisher in oak_camera.launch.py use, otherwise
        # rtabmap can't connect odom → base_link → camera_optical.
        self._odom_frame_id = f'{robot_id}_odom'
        self._base_frame_id = f'{robot_id}_base_link'

        self._mu = np.zeros(3)           # [x, y, theta]
        self._sigma = np.eye(3) * 0.1
        self._Q = np.diag([0.01, 0.01, 0.005])   # process noise
        self._R = np.diag([0.05, 0.05, 0.02])    # SLAM observation noise
        self._last_t: float | None = None

        # Only this node reads raw /odom
        self.create_subscription(Odometry, f'/{robot_id}/odom', self._odom_cb, 10)
        self.create_subscription(PoseStamped, f'/{robot_id}/slam/pose', self._slam_cb, 10)
        self._pub = self.create_publisher(Odometry, f'/{robot_id}/ekf/odom', 10)
        self.get_logger().info(f'EKF node ready for {robot_id}')

    # ------------------------------------------------------------------
    # Prediction step: integrate holonomic wheel odometry
    # ------------------------------------------------------------------
    def _odom_cb(self, msg: Odometry):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self._last_t is None:
            self._last_t = now
            return
        dt = now - self._last_t
        self._last_t = now

        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        omega = msg.twist.twist.angular.z
        th = self._mu[2]

        G = np.eye(3)
        G[0, 2] = (-vx * np.sin(th) - vy * np.cos(th)) * dt
        G[1, 2] = ( vx * np.cos(th) - vy * np.sin(th)) * dt

        self._mu[0] += (vx * np.cos(th) - vy * np.sin(th)) * dt
        self._mu[1] += (vx * np.sin(th) + vy * np.cos(th)) * dt
        self._mu[2] += omega * dt

        self._sigma = G @ self._sigma @ G.T + self._Q
        self._publish()

    # ------------------------------------------------------------------
    # Correction step: SLAM pose update
    # ------------------------------------------------------------------
    def _slam_cb(self, msg: PoseStamped):
        q = msg.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        slam_theta = np.arctan2(siny, cosy)

        z = np.array([msg.pose.position.x, msg.pose.position.y, slam_theta])
        H = np.eye(3)
        S = H @ self._sigma @ H.T + self._R
        K = self._sigma @ H.T @ np.linalg.inv(S)
        innov = z - self._mu
        innov[2] = np.arctan2(np.sin(innov[2]), np.cos(innov[2]))
        self._mu = self._mu + K @ innov
        self._sigma = (np.eye(3) - K @ H) @ self._sigma
        self._publish()

    def _publish(self):
        out = Odometry()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self._odom_frame_id
        out.child_frame_id  = self._base_frame_id
        out.pose.pose.position.x = float(self._mu[0])
        out.pose.pose.position.y = float(self._mu[1])
        out.pose.pose.orientation.z = float(np.sin(self._mu[2] / 2.0))
        out.pose.pose.orientation.w = float(np.cos(self._mu[2] / 2.0))
        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = EKFNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
