#!/usr/bin/env python3
"""
nav_watch.py
------------
Single-screen live view of navigation state for a given robot. Useful
during Tier-1/2 hardware tests where you want pose, status, current
goal, and the commanded virtual-centre twist all in one place without
juggling four `ros2 topic echo` windows.

Usage:
  python3 tools/nav_watch.py --robot tb3_0
  python3 tools/nav_watch.py --robot tb3_1 --rate 5

Reads (no publications, purely diagnostic):
  /{robot}/ekf/odom            — authoritative pose
  /{robot}/cmd_vel             — twist actually sent to OpenCR
  /virtual_center/cmd_vel      — twist nav_node is asking the formation
                                  to execute
  /navigation/status           — IDLE | NAVIGATING | REACHED
  /navigation/goal             — last goal received

Stop with Ctrl+C.
"""

import argparse
import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String


def yaw_from_quat(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


class NavWatch(Node):
    def __init__(self, robot: str, rate_hz: float):
        super().__init__('nav_watch')
        self._robot = robot
        self._pose = None         # (x, y, yaw)
        self._cmd_robot = None    # Twist
        self._cmd_vc = None       # Twist
        self._status = '—'
        self._goal = None         # (x, y, theta)
        self._t_status = 0.0

        self.create_subscription(
            Odometry, f'/{robot}/ekf/odom', self._odom_cb, 10)
        self.create_subscription(
            Twist, f'/{robot}/cmd_vel', self._cmd_robot_cb, 10)
        self.create_subscription(
            Twist, '/virtual_center/cmd_vel', self._cmd_vc_cb, 10)
        self.create_subscription(
            String, '/navigation/status', self._status_cb, 10)
        self.create_subscription(
            Twist, '/navigation/goal', self._goal_cb, 10)

        self.create_timer(1.0 / rate_hz, self._render)

    def _odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        self._pose = (p.x, p.y, yaw_from_quat(msg.pose.pose.orientation))

    def _cmd_robot_cb(self, msg: Twist):
        self._cmd_robot = msg

    def _cmd_vc_cb(self, msg: Twist):
        self._cmd_vc = msg

    def _status_cb(self, msg: String):
        self._status = msg.data
        self._t_status = time.time()

    def _goal_cb(self, msg: Twist):
        self._goal = (msg.linear.x, msg.linear.y, msg.angular.z)

    def _render(self):
        # ANSI: clear screen + move cursor home
        print('\033[2J\033[H', end='')
        print(f'=== nav_watch  [{self._robot}]  '
              f'{time.strftime("%H:%M:%S")} ===')

        if self._pose is None:
            print('pose:    (waiting for /ekf/odom)')
        else:
            x, y, th = self._pose
            print(f'pose:    x={x:+.3f}  y={y:+.3f}  '
                  f'theta={math.degrees(th):+6.1f}°')

        if self._goal is None:
            print('goal:    (none received)')
        else:
            gx, gy, gth = self._goal
            if self._pose is not None:
                d = math.hypot(gx - self._pose[0], gy - self._pose[1])
                print(f'goal:    x={gx:+.3f}  y={gy:+.3f}  '
                      f'theta={math.degrees(gth):+6.1f}°   '
                      f'dist={d:.3f} m')
            else:
                print(f'goal:    x={gx:+.3f}  y={gy:+.3f}  '
                      f'theta={math.degrees(gth):+6.1f}°')

        age = time.time() - self._t_status if self._t_status else None
        age_str = f'{age:4.1f}s ago' if age is not None else 'never'
        print(f'status:  {self._status:>11}   ({age_str})')

        if self._cmd_vc:
            v = self._cmd_vc
            print(f'/vc cmd: vx={v.linear.x:+.3f}  vy={v.linear.y:+.3f}  '
                  f'wz={v.angular.z:+.3f}')
        else:
            print('/vc cmd: (none)')

        if self._cmd_robot:
            v = self._cmd_robot
            print(f'/robot:  vx={v.linear.x:+.3f}  vy={v.linear.y:+.3f}  '
                  f'wz={v.angular.z:+.3f}')
        else:
            print('/robot:  (none)')


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--robot', default='tb3_0',
                   help='Robot ID / namespace (default: tb3_0)')
    p.add_argument('--rate', type=float, default=4.0,
                   help='Render rate in Hz (default: 4)')
    args = p.parse_args()

    rclpy.init()
    node = NavWatch(args.robot, args.rate)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
