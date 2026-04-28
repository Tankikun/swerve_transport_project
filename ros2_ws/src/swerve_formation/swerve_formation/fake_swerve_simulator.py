"""
fake_swerve_simulator.py
------------------------
Simulates a holonomic swerve robot for testing without hardware.

Fixes vs original:
  - Integrates heading (angular.z was previously ignored)
  - cmd_vel is in BODY frame → rotated to world frame before integrating
  - Publishes to BOTH /{robot_id}/odom AND /{robot_id}/ekf/odom
    (navigation_node reads ekf/odom; other nodes read odom)
  - Full quaternion in odometry (not just orientation.w = 1)
  - Optional: subscribe to /virtual_center/cmd_vel instead of /{robot_id}/cmd_vel
    (useful for testing navigation_node in isolation, without laplacian node)

Parameters:
  robot_id               str    'tb3_0'
  start_x                float   0.0
  start_y                float   0.0
  start_theta            float   0.0   (radians)
  use_virtual_center     bool    False  (subscribe to /virtual_center/cmd_vel)
"""

import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker


class FakeSwerveRobot(Node):
    """
    Holonomic swerve robot simulator at 50 Hz.

    cmd_vel convention (matches OpenCR firmware):
      linear.x  = vx in BODY frame  [m/s]
      linear.y  = vy in BODY frame  [m/s]
      angular.z = omega             [rad/s]

    World-frame integration:
      ẋ =  vx·cos(θ) - vy·sin(θ)
      ẏ =  vx·sin(θ) + vy·cos(θ)
      θ̇ =  ω
    """

    def __init__(self):
        super().__init__('fake_swerve_robot')

        self.declare_parameter('robot_id',           'tb3_0')
        self.declare_parameter('start_x',             0.0)
        self.declare_parameter('start_y',             0.0)
        self.declare_parameter('start_theta',          0.0)
        self.declare_parameter('use_virtual_center',  False)

        self._robot_id = self.get_parameter('robot_id').value
        self._x        = float(self.get_parameter('start_x').value)
        self._y        = float(self.get_parameter('start_y').value)
        self._theta    = float(self.get_parameter('start_theta').value)
        use_vc         = self.get_parameter('use_virtual_center').value

        self._vx    = 0.0
        self._vy    = 0.0
        self._omega = 0.0
        self._last  = time.time()

        # Subscribe to cmd_vel — either body-level or virtual center
        cmd_topic = ('/virtual_center/cmd_vel' if use_vc
                     else f'/{self._robot_id}/cmd_vel')
        self.create_subscription(Twist, cmd_topic, self._cmd_cb, 10)
        self.get_logger().info(f'[{self._robot_id}] subscribing to {cmd_topic}')

        # Publish odometry to two topics:
        #   /odom     — for laplacian_formation_node / general use
        #   /ekf/odom — for navigation_node (which trusts only EKF output)
        #   In simulation the EKF is bypassed; fake_swerve IS the "perfect EKF"
        self._odom_pub     = self.create_publisher(
            Odometry, f'/{self._robot_id}/odom',     10
        )
        self._ekf_odom_pub = self.create_publisher(
            Odometry, f'/{self._robot_id}/ekf/odom', 10
        )
        self._marker_pub   = self.create_publisher(
            Marker, f'/{self._robot_id}/marker', 10
        )

        self.create_timer(0.02, self._update)   # 50 Hz physics
        self.get_logger().info(
            f'FakeSwerveRobot [{self._robot_id}] ready at '
            f'({self._x:.2f}, {self._y:.2f}, {math.degrees(self._theta):.1f}°)'
        )

    def _cmd_cb(self, msg: Twist):
        self._vx    = msg.linear.x
        self._vy    = msg.linear.y
        self._omega = msg.angular.z

    def _update(self):
        now = time.time()
        dt  = min(now - self._last, 0.1)   # clamp large dt (e.g. after pause)
        self._last = now

        # Rotate body-frame velocity to world frame
        c  = math.cos(self._theta)
        s  = math.sin(self._theta)
        wx = c * self._vx - s * self._vy
        wy = s * self._vx + c * self._vy

        # Integrate
        self._x     += wx * dt
        self._y     += wy * dt
        self._theta += self._omega * dt

        # Normalise heading to [-π, π]
        self._theta = (self._theta + math.pi) % (2 * math.pi) - math.pi

        self._publish_odom()
        self._publish_marker()

    def _publish_odom(self):
        now  = self.get_clock().now().to_msg()
        half = self._theta / 2.0
        qz   = math.sin(half)
        qw   = math.cos(half)

        odom = Odometry()
        odom.header.stamp       = now
        odom.header.frame_id    = 'odom'
        odom.child_frame_id     = self._robot_id

        odom.pose.pose.position.x    = self._x
        odom.pose.pose.position.y    = self._y
        odom.pose.pose.position.z    = 0.0
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x  = self._vx
        odom.twist.twist.linear.y  = self._vy
        odom.twist.twist.angular.z = self._omega

        self._odom_pub.publish(odom)
        self._ekf_odom_pub.publish(odom)   # pass-through: sim IS the perfect EKF

    def _publish_marker(self):
        m = Marker()
        m.header.stamp    = self.get_clock().now().to_msg()
        m.header.frame_id = 'odom'
        m.ns              = self._robot_id
        m.id              = 0
        m.type            = Marker.CYLINDER
        m.action          = Marker.ADD
        m.pose.position.x = self._x
        m.pose.position.y = self._y
        m.pose.position.z = 0.05
        m.pose.orientation.w = 1.0
        m.scale.x = 0.28
        m.scale.y = 0.28
        m.scale.z = 0.10
        m.color.a = 1.0
        if self._robot_id == 'tb3_0':
            m.color.b = 1.0   # blue
        else:
            m.color.r = 1.0   # red
        self._marker_pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = FakeSwerveRobot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
