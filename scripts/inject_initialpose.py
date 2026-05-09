#!/usr/bin/env python3
"""
inject_initialpose.py — Manually publish a /initialpose hint to RTAB-Map.

Normally the GUI's 📍 Set Initial Pose button does this for you (the
browser writes to server.py's /set_initial_pose mailbox; the bridge
republishes to /initialpose). This script is a CLI fallback for:

  - Sanity-checking the pipeline without the browser
  - Headless / SSH testing on the Pi
  - Automated test harnesses

Run with ROS sourced:

    python3 inject_initialpose.py --ros-args \\
        -p x:=0.0 -p y:=0.0 -p yaw_deg:=0.0

The default covariance matches RViz's "2D Pose Estimate":
σ_x = σ_y = 0.5 m, σ_yaw = 15°. RTAB-Map will use this to seed its
search and typically converges within 1-2 frames at < 30 mm σ.

The /initialpose topic is published in the GLOBAL namespace because
the rtabmap node does NOT remap it (same convention as RViz / AMCL).
"""

import math
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node


class InitialPoseInjector(Node):
    def __init__(self):
        super().__init__('initialpose_injector')
        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('yaw_deg', 0.0)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('repeat', 5)
        self.declare_parameter('repeat_period_sec', 0.2)

        self.x = float(self.get_parameter('x').value)
        self.y = float(self.get_parameter('y').value)
        self.yaw = math.radians(float(self.get_parameter('yaw_deg').value))
        self.frame = str(self.get_parameter('frame_id').value)
        self.repeat = int(self.get_parameter('repeat').value)
        self.repeat_period = float(self.get_parameter('repeat_period_sec').value)

        self.pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)

    def run(self):
        # DDS discovery
        time.sleep(2.0)

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = self.frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(self.yaw / 2.0)

        # 6×6 row-major covariance — RViz "2D Pose Estimate" defaults.
        cov = [0.0] * 36
        cov[0]  = 0.25     # var(x)  → σ = 0.5 m
        cov[7]  = 0.25     # var(y)
        cov[35] = 0.069    # var(yaw) → σ ≈ 15°
        msg.pose.covariance = cov

        self.get_logger().info(
            f'Publishing /initialpose: frame={self.frame} '
            f'x={self.x:+.3f} y={self.y:+.3f} yaw={math.degrees(self.yaw):+.1f}° '
            f'(σ_xy=0.5 m, σ_yaw=15°)  ×{self.repeat}')

        for _ in range(self.repeat):
            self.pub.publish(msg)
            time.sleep(self.repeat_period)
        self.get_logger().info('Done.')


def main():
    rclpy.init()
    node = InitialPoseInjector()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
