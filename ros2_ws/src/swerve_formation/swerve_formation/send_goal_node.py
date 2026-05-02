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

        # Use TRANSIENT_LOCAL so a late-discovering subscriber (cross-machine
        # FastDDS unicast can take 1-2 s) still receives the message.
        from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        pub = self.create_publisher(Twist, '/navigation/goal', qos)

        # Wait for cross-machine discovery handshake. 0.5 s was too short
        # for laptop→Pi sends (sub_count remained 0 → message dropped).
        # Poll for at least one matched subscriber, with a 5 s ceiling, then
        # publish. With TRANSIENT_LOCAL above this is doubly safe.
        self._pub = pub
        self._x, self._y, self._theta = x, y, theta
        self._waited = 0.0
        self.create_timer(0.5, self._tick)

    def _tick(self):
        sub_count = self._pub.get_subscription_count()
        self._waited += 0.5
        if sub_count > 0 or self._waited >= 5.0:
            msg = Twist()
            msg.linear.x  = self._x
            msg.linear.y  = self._y
            msg.angular.z = self._theta
            self._pub.publish(msg)
            self.get_logger().info(
                f'Goal sent: x={self._x:.2f} y={self._y:.2f} '
                f'θ={self._theta:.2f} rad  (matched_subs={sub_count}, '
                f'waited={self._waited:.1f}s)'
            )
            # Linger briefly so TRANSIENT_LOCAL has time to deliver.
            self.create_timer(0.5, lambda: (_ for _ in ()).throw(SystemExit))


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
