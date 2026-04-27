#!/usr/bin/env python3
"""
odom_watch.py — Live odometry display for TurtleBot3 Conveyor
Usage:  python3 ~/odom_watch.py [namespace]
        odom            (via alias)
        odom /robot2    (different namespace)

Press Ctrl+C to exit.
"""

import sys, math, rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

# ── ANSI helpers ──────────────────────────────────────────────────────────────
CLEAR  = '\033[2J\033[H'   # clear screen, cursor home
BOLD   = '\033[1m'
DIM    = '\033[2m'
RESET  = '\033[0m'
GREEN  = '\033[92m'
CYAN   = '\033[96m'
YELLOW = '\033[93m'
RED    = '\033[91m'
WHITE  = '\033[97m'

def color_val(v, threshold=0.01):
    """Green if moving, white if still."""
    return GREEN if abs(v) > threshold else WHITE

def heading_arrow(deg):
    """Return a unicode arrow for the heading direction."""
    arrows = ['↑','↗','→','↘','↓','↙','←','↖']
    idx = round(deg / 45) % 8
    return arrows[idx]

def draw_compass(deg, size=7):
    """Draw a tiny ASCII compass rose with the heading marked."""
    cx = cy = size // 2
    rad = math.radians(deg)
    nx = round(cx + (cx - 0.5) * math.sin(rad))
    ny = round(cy - (cy - 0.5) * math.cos(rad))
    grid = [['·'] * size for _ in range(size)]
    # Mark center
    grid[cy][cx] = '+'
    # Mark heading tip (clamp to grid)
    nx = max(0, min(size - 1, nx))
    ny = max(0, min(size - 1, ny))
    grid[ny][nx] = '●'
    # Draw path line (simplified — just mark intermediate)
    steps = max(abs(nx - cx), abs(ny - cy))
    if steps > 0:
        for s in range(1, steps):
            px = round(cx + (nx - cx) * s / steps)
            py = round(cy + (ny - cy) * s / steps)
            if grid[py][px] == '·':
                grid[py][px] = '─' if abs(nx - cx) > abs(ny - cy) else '│'
    lines = ['  ' + ' '.join(row) for row in grid]
    lines[0] = lines[0].replace('·', ' ', 1)
    return '\n'.join(lines)

def bar(v, max_v=0.5, width=20):
    """Simple velocity bar: negative=left, positive=right, center=zero."""
    filled = int(abs(v) / max_v * (width // 2))
    filled = min(filled, width // 2)
    if v >= 0:
        left  = ' ' * (width // 2)
        right = '█' * filled + '░' * (width // 2 - filled)
    else:
        left  = '░' * (width // 2 - filled) + '█' * filled
        right = ' ' * (width // 2)
    return f'[{left}|{right}]'


class OdomWatcher(Node):

    def __init__(self, ns):
        super().__init__('odom_watcher')
        topic = f'{ns}/odom' if ns else '/robot1/odom'
        self.sub = self.create_subscription(Odometry, topic, self._cb, 10)
        self._msg_count = 0
        self._topic = topic
        print(CLEAR, end='', flush=True)

    def _cb(self, msg: Odometry):
        self._msg_count += 1
        p   = msg.pose.pose.position
        q   = msg.pose.pose.orientation
        t   = msg.twist.twist
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
        deg = math.degrees(yaw)
        vx, vy, wz = t.linear.x, t.linear.y, t.angular.z

        speed = math.hypot(vx, vy)
        moving = speed > 0.01 or abs(wz) > 0.01

        arrow = heading_arrow(deg)
        compass = draw_compass(deg)
        state_str = f'{GREEN}MOVING{RESET}' if moving else f'{DIM}stopped{RESET}'

        out = []
        out.append(CLEAR)
        out.append(f'{BOLD}{CYAN}══ TurtleBot3 Conveyor — Odometry ══{RESET}  {DIM}topic: {self._topic}  msgs: {self._msg_count}{RESET}')
        out.append('')

        # Position block
        out.append(f'{BOLD}  POSITION{RESET}')
        out.append(f'    {YELLOW}X{RESET}  {color_val(vx)}{p.x:+8.4f} m{RESET}')
        out.append(f'    {YELLOW}Y{RESET}  {color_val(vy)}{p.y:+8.4f} m{RESET}')
        out.append(f'    {YELLOW}θ{RESET}  {color_val(wz)}{deg:+8.2f} °{RESET}   {WHITE}{arrow}{RESET}  {state_str}')
        out.append('')

        # Velocity block
        out.append(f'{BOLD}  VELOCITY{RESET}')
        out.append(f'    {YELLOW}vx{RESET}  {bar(vx)}  {color_val(vx)}{vx:+.4f} m/s{RESET}')
        out.append(f'    {YELLOW}vy{RESET}  {bar(vy)}  {color_val(vy)}{vy:+.4f} m/s{RESET}')
        out.append(f'    {YELLOW}ωz{RESET}  {bar(wz, 2.0)}  {color_val(wz)}{wz:+.4f} rad/s{RESET}')
        out.append(f'    {YELLOW}|v|{RESET} {GREEN if speed>0.01 else WHITE}{speed:.4f} m/s{RESET}')
        out.append('')

        # Compass block
        out.append(f'{BOLD}  HEADING{RESET}')
        out.append(compass)
        out.append(f'  {DIM}↑ = robot +X (forward){RESET}')
        out.append('')

        out.append(f'  {DIM}Ctrl+C to exit  │  send "R\\n" to OpenCR to reset odometry{RESET}')

        print('\n'.join(out), end='', flush=True)


def main():
    ns = sys.argv[1] if len(sys.argv) > 1 else '/robot1'
    # Strip trailing slash
    ns = ns.rstrip('/')
    rclpy.init()
    node = OdomWatcher(ns)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        print(f'\n{RESET}')
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
