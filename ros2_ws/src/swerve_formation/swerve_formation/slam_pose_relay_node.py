"""
slam_pose_relay_node.py
-----------------------
Glue between RTAB-Map's localization output and `ekf_node`'s SLAM
correction step.

RTAB-Map publishes `/rtabmap/localization_pose` as
`geometry_msgs/PoseWithCovarianceStamped`. `ekf_node` (line 28 of
ekf_node.py) subscribes to `/{robot_id}/slam/pose` as
`geometry_msgs/PoseStamped`. Different message types, can't just
remap — this node converts and forwards.

Covariance is dropped on the floor: ekf_node uses a fixed
observation noise matrix `R = diag(0.05, 0.05, 0.02)` (see
ekf_node.py line 23). If you later want adaptive covariance,
upgrade `ekf_node` first to consume PoseWithCovarianceStamped
directly and remove this relay.

Parameters:
  in_topic   string   default: /rtabmap/localization_pose
  out_topic  string   default: /tb3_0/slam/pose

Frame_id is preserved as-is. RTAB-Map publishes in the `map` frame
by default — that's what `ekf_node` already assumes.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped


class SlamPoseRelay(Node):
    def __init__(self):
        super().__init__('slam_pose_relay')
        self.declare_parameter('in_topic',  '/rtabmap/localization_pose')
        self.declare_parameter('out_topic', '/tb3_0/slam/pose')
        in_topic  = str(self.get_parameter('in_topic').value)
        out_topic = str(self.get_parameter('out_topic').value)

        self._pub = self.create_publisher(PoseStamped, out_topic, 10)
        self.create_subscription(
            PoseWithCovarianceStamped, in_topic, self._cb, 10
        )
        self.get_logger().info(
            f'slam_pose_relay: {in_topic} (PoseWithCovarianceStamped) '
            f'-> {out_topic} (PoseStamped)'
        )

    def _cb(self, msg: PoseWithCovarianceStamped) -> None:
        out = PoseStamped()
        out.header = msg.header
        out.pose   = msg.pose.pose
        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = SlamPoseRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
