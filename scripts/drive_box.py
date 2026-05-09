#!/usr/bin/env python3
"""
drive_box.py — Drive a square box pattern on a swerve robot for mapping.

Used by the feature/map-to-localize runbook. Drives:

    forward 0.5 m  ->  pivot 360°  ->  right (strafe) 0.5 m  ->  pivot 360°
 -> back 0.5 m     ->  pivot 360°  ->  left  (strafe) 0.5 m  ->  pivot 360°

Total ≈ 2.5 minutes at the default speeds. Holonomic — uses linear.x,
linear.y, and angular.z together since this is a swerve chassis. The
OpenCR firmware's 5-second cmd_vel watchdog will stop the wheels if
this script dies mid-pattern.

The four pivot-360° segments at every corner are the critical ingredient
for map quality: they sample each station from many yaw angles, giving
RTAB-Map's loop-closure detector a chance to match the same physical
spot from different headings.

Run with ROS sourced and ROS_DOMAIN_ID set:

    python3 drive_box.py --ros-args -p robot_id:=tb3_0 -p lin_v:=0.10 -p ang_v:=0.20
"""

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


def make_twist(vx=0.0, vy=0.0, wz=0.0):
    t = Twist()
    t.linear.x = float(vx)
    t.linear.y = float(vy)
    t.angular.z = float(wz)
    return t


class BoxDriver(Node):
    def __init__(self):
        super().__init__('box_driver')
        self.declare_parameter('robot_id', 'tb3_0')
        self.declare_parameter('lin_v', 0.10)        # m/s for translation
        self.declare_parameter('ang_v', 0.20)        # rad/s for pivots
        self.declare_parameter('leg_dist', 0.50)     # m per straight leg
        self.declare_parameter('settle_sec', 1.0)    # pause between segments

        rid = str(self.get_parameter('robot_id').value)
        self.lin_v = float(self.get_parameter('lin_v').value)
        self.ang_v = float(self.get_parameter('ang_v').value)
        self.leg_dist = float(self.get_parameter('leg_dist').value)
        self.settle = float(self.get_parameter('settle_sec').value)

        self.pub = self.create_publisher(Twist, f'/{rid}/cmd_vel', 10)
        self.get_logger().info(
            f'BoxDriver: robot_id={rid}, lin_v={self.lin_v}, ang_v={self.ang_v}, '
            f'leg={self.leg_dist} m')

    def hold(self, twist, duration_sec, label):
        """Publish twist at 20 Hz for duration_sec, then a single zero-twist."""
        self.get_logger().info(
            f'  {label}  duration={duration_sec:.2f}s  '
            f'vx={twist.linear.x:+.3f} vy={twist.linear.y:+.3f} '
            f'wz={twist.angular.z:+.3f}')
        t_end = time.time() + duration_sec
        while time.time() < t_end and rclpy.ok():
            self.pub.publish(twist)
            time.sleep(0.05)
        self.pub.publish(make_twist())  # zero between segments

    def run(self):
        # Give DDS time to discover the cmd_vel subscriber.
        self.get_logger().info('Waiting 2 s for subscriber discovery...')
        time.sleep(2.0)

        leg_t = self.leg_dist / self.lin_v             # ≈ 5 s at default
        turn_t = (2 * math.pi) / self.ang_v            # ≈ 31 s at default

        sequence = [
            ('forward 0.5 m',  make_twist(vx=+self.lin_v),  leg_t),
            ('pivot  360° CCW', make_twist(wz=+self.ang_v), turn_t),
            ('right   0.5 m',  make_twist(vy=-self.lin_v),  leg_t),
            ('pivot  360° CCW', make_twist(wz=+self.ang_v), turn_t),
            ('back    0.5 m',  make_twist(vx=-self.lin_v),  leg_t),
            ('pivot  360° CCW', make_twist(wz=+self.ang_v), turn_t),
            ('left    0.5 m',  make_twist(vy=+self.lin_v),  leg_t),
            ('pivot  360° CCW', make_twist(wz=+self.ang_v), turn_t),
        ]

        total = sum(d for *_, d in sequence) + len(sequence) * self.settle
        self.get_logger().info(
            f'Total planned drive time: {total:.1f}s ({len(sequence)} segments).')

        t0 = time.time()
        try:
            for i, (label, twist, dur) in enumerate(sequence, 1):
                self.get_logger().info(
                    f'[{i}/{len(sequence)}]  T+{time.time()-t0:6.1f}s  starting {label}')
                self.hold(twist, dur, label)
                time.sleep(self.settle)
                self.pub.publish(make_twist())
        except KeyboardInterrupt:
            self.get_logger().warn('Interrupted — sending hard stop.')
        finally:
            for _ in range(20):
                self.pub.publish(make_twist())
                time.sleep(0.05)
            self.get_logger().info(
                f'Done. Total elapsed: {time.time()-t0:.1f}s')


def main():
    rclpy.init()
    node = BoxDriver()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
