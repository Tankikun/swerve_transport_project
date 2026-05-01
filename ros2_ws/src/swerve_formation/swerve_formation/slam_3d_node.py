import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


class SLAM3DNode(Node):
    """
    3D SLAM interface node (exposed as console script '3d_slam_node').

    In production: replace this file with a launch-file integration of rtabmap_ros.
    rtabmap_ros subscribes to /{robot_id}/camera/depth and /{robot_id}/odom
    internally and publishes its corrected pose — wire that output to
    /{robot_id}/slam/pose so ekf_node can consume it.

    This stub publishes a static zero pose so the EKF starts up cleanly
    without waiting for real SLAM data.
    """

    def __init__(self):
        super().__init__('slam_3d_node')
        self.declare_parameter('robot_id', 'tb3_0')
        robot_id = self.get_parameter('robot_id').value

        self._pub = self.create_publisher(PoseStamped, f'/{robot_id}/slam/pose', 10)
        # 1 Hz stub publish — EKF ignores it once real SLAM comes online
        self.create_timer(1.0, self._publish_stub)
        self.get_logger().warn(
            'slam_3d_node is in stub mode — integrate rtabmap_ros for production SLAM'
        )

    def _publish_stub(self):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.pose.orientation.w = 1.0   # identity quaternion
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SLAM3DNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
