#!/usr/bin/env python3
"""
teleop_keyboard_node.py
-----------------------
Holonomic keyboard teleop for TurtleBot3 Conveyor.
Publishes geometry_msgs/Twist on cmd_vel at 10 Hz continuously.

Key layout:
  q  w  e   ->  front-left / forward / front-right
  a  s  d   ->  left / STOP / right
  z  x  c   ->  back-left / backward / back-right
  r=Turn CW   t=Turn CCW
  m=speed up  n=speed down
  Ctrl+C -> quit

Architecture:
  [this node] --/robot1/cmd_vel--> [serial_bridge_node] --serial--> [OpenCR]

FIX: Uses SingleThreadedExecutor + spin_once() in the same thread as the
keyboard loop. No background threads — eliminates GIL contention that caused
DDS to drop messages with the old threading approach.
"""

import sys, os, math, tty, termios, select
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import Twist

DIAG = 1.0 / math.sqrt(2)

KEY_MAP = {
    'w': ( 1.0,   0.0,  0.0),   # forward
    'x': (-1.0,   0.0,  0.0),   # backward
    'a': ( 0.0,   1.0,  0.0),   # strafe left  (ROS +y = left)
    'd': ( 0.0,  -1.0,  0.0),   # strafe right
    'q': ( DIAG,  DIAG, 0.0),   # front-left 45
    'e': ( DIAG, -DIAG, 0.0),   # front-right 45
    'z': (-DIAG,  DIAG, 0.0),   # back-left 45
    'c': (-DIAG, -DIAG, 0.0),   # back-right 45
    'r': ( 0.0,   0.0, -1.0),   # turn CW  (ROS -z = CW)
    't': ( 0.0,   0.0,  1.0),   # turn CCW
    's': ( 0.0,   0.0,  0.0),   # stop
    ' ': ( 0.0,   0.0,  0.0),   # stop
}
KEY_LABEL = {
    'w': 'Forward',       'x': 'Backward',
    'a': 'Left',          'd': 'Right',
    'q': 'Front-Left',    'e': 'Front-Right',
    'z': 'Back-Left',     'c': 'Back-Right',
    'r': 'Turn CW',       't': 'Turn CCW',
    's': 'STOP',          ' ': 'STOP',
}

SPEED_DEFAULT = 0.30   # m/s
SPEED_STEP    = 0.05
SPEED_MIN     = 0.05
SPEED_MAX     = 0.60
TURN_SPEED    = 0.80   # rad/s
CR            = chr(13)


def make_twist(vx, vy, wz, speed):
    t = Twist()
    if abs(wz) > 0.01:
        t.angular.z = wz * TURN_SPEED
    else:
        t.linear.x = vx * speed
        t.linear.y = vy * speed
    return t


def print_status(fd, label, speed):
    line = '  [' + label + ']  speed=' + ('%.2f' % speed) + ' m/s'
    os.write(fd, (CR + ' ' * 60 + CR + line).encode())


def main(args=None):
    rclpy.init(args=args)
    node = Node('teleop_keyboard_node')

    # Relative topic name — ROS namespace makes it /robot1/cmd_vel automatically
    pub = node.create_publisher(Twist, 'cmd_vel', 10)

    speed = SPEED_DEFAULT
    current_twist = [Twist()]   # list so timer_cb closure always sees latest

    def timer_cb():
        pub.publish(current_twist[0])

    # 10 Hz keep-alive: bridge watchdog never fires during normal operation
    node.create_timer(0.1, timer_cb)

    # SingleThreadedExecutor in the SAME thread as keyboard loop
    # No background thread -> no GIL contention -> reliable DDS delivery
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    fd  = sys.stdin.fileno()
    old = termios.tcgetattr(fd)

    print('TurtleBot3 Conveyor Teleop  (Ctrl+C to quit)')
    print('  q  w  e   |  front-left / forward / front-right')
    print('  a  s  d   |  left / STOP / right')
    print('  z  x  c   |  back-left / backward / back-right')
    print('  r=Turn CW   t=Turn CCW   m=faster   n=slower')
    print('Speed: %.2f m/s' % speed)
    sys.stdout.flush()

    try:
        tty.setraw(fd)
        while rclpy.ok():
            # Non-blocking keyboard check (10 ms window)
            rlist, _, _ = select.select([fd], [], [], 0.01)
            if rlist:
                ch = os.read(fd, 1).decode('utf-8', errors='ignore')
                if not ch:
                    pass
                elif ch == '\x03':          # Ctrl+C -> quit
                    break
                elif ch in KEY_MAP:
                    vx, vy, wz = KEY_MAP[ch]
                    current_twist[0] = make_twist(vx, vy, wz, speed)
                    pub.publish(current_twist[0])   # immediate send on key press
                    print_status(fd, KEY_LABEL.get(ch, ch), speed)
                elif ch == 'm':
                    speed = min(speed + SPEED_STEP, SPEED_MAX)
                    print_status(fd, 'Speed UP -> %.2f' % speed, speed)
                elif ch == 'n':
                    speed = max(speed - SPEED_STEP, SPEED_MIN)
                    print_status(fd, 'Speed DOWN -> %.2f' % speed, speed)

            # Process ROS2 timer + any incoming messages — returns immediately
            executor.spin_once(timeout_sec=0.0)

    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        current_twist[0] = Twist()          # zero twist = stop
        pub.publish(current_twist[0])
        executor.spin_once(timeout_sec=0.1) # flush the stop command
        print('\nTeleop stopped.')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
