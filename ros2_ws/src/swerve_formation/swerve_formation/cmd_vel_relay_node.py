"""
cmd_vel_relay_node.py
---------------------
Minimal Twist relay: subscribes to one topic, republishes byte-for-byte
to another. Used in the Tier-1 single-robot navigation launch to copy
/virtual_center/cmd_vel directly to /{robot_id}/cmd_vel — bypassing
laplacian_formation_node when there is only one robot and no formation
math is needed.

This duplicates a tiny piece of `topic_tools relay`, which we cannot
apt-install on the Pi because the lab apt mirror is unreliable.

Parameters
  in_topic   string  source topic   (default /virtual_center/cmd_vel)
  out_topic  string  destination    (default /tb3_0/cmd_vel)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CmdVelRelay(Node):
    def __init__(self):
        super().__init__('cmd_vel_relay')
        self.declare_parameter('in_topic',  '/virtual_center/cmd_vel')
        self.declare_parameter('out_topic', '/tb3_0/cmd_vel')
        in_topic  = self.get_parameter('in_topic').value
        out_topic = self.get_parameter('out_topic').value

        self._pub = self.create_publisher(Twist, out_topic, 10)
        self.create_subscription(Twist, in_topic, self._cb, 10)
        self.get_logger().info(f'cmd_vel_relay: {in_topic} -> {out_topic}')

    def _cb(self, msg: Twist):
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
