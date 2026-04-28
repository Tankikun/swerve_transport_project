"""
static_leader_publisher.py
--------------------------
Simulation helper: publishes a fixed robot ID to /formation/leader at 1 Hz.
Replaces leader_election_node in sim so navigation_node activates immediately.

Usage (via launch file):
  ros2 run swerve_formation static_leader_publisher --ros-args -p leader_id:=tb3_0
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class StaticLeaderPublisher(Node):
    def __init__(self):
        super().__init__('static_leader_publisher')
        self.declare_parameter('leader_id', 'tb3_0')
        self._leader = self.get_parameter('leader_id').value
        self._pub = self.create_publisher(String, '/formation/leader', 10)
        self.create_timer(1.0, self._publish)
        self.get_logger().info(f'Static leader: {self._leader}')

    def _publish(self):
        msg = String()
        msg.data = self._leader
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = StaticLeaderPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
