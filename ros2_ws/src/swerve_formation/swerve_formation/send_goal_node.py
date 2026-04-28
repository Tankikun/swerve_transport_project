"""
send_goal_node.py
-----------------
Simulation helper: publishes a single navigation goal and exits.
Sends to /navigation/goal as geometry_msgs/Twist:
  linear.x  = target x (m)
  linear.y  = target y (m)
  angular.z = target heading (rad, default 0)

Usage:
  ros2 run swerve_formation send_goal_node --ros-args -p x:=4.5 -p y:=3.5
  ros2 run swerve_formation send_goal_node --ros-args -p x:=4.5 -p y:=3.5 -p theta:=1.57

Obstacles (optional — publish separately, stays until overwritten):
  ros2 topic pub --once /navigation/obstacles geometry_msgs/msg/PoseArray \\
    '{poses: [{position: {x: 2.2, y: 1.8, z: 0.35}},
               {position: {x: 3.2, y: 2.8, z: 0.30}}]}'

Watch status:
  ros2 topic echo /navigation/status
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class SendGoalNode(Node):
    def __init__(self):
        super().__init__('send_goal_node')
        self.declare_parameter('x',     0.0)
        self.declare_parameter('y',     0.0)
        self.declare_parameter('theta', 0.0)

        x     = float(self.get_parameter('x').value)
        y     = float(self.get_parameter('y').value)
        theta = float(self.get_parameter('theta').value)

        pub = self.create_publisher(Twist, '/navigation/goal', 10)

        # Give navigation_node a moment to start
        self.create_timer(0.5, lambda: self._send(pub, x, y, theta))

    def _send(self, pub, x, y, theta):
        msg = Twist()
        msg.linear.x  = x
        msg.linear.y  = y
        msg.angular.z = theta
        pub.publish(msg)
        self.get_logger().info(f'Goal sent: x={x:.2f} y={y:.2f} θ={theta:.2f} rad')
        raise SystemExit


def main(args=None):
    rclpy.init(args=args)
    node = SendGoalNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
