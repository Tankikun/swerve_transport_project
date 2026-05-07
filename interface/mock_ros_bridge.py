"""
mock_ros_bridge.py
------------------
Mac-side stand-in for ros2 + rosbridge_websocket. Lets you test the entire
web UI -> navigation flow on your laptop, with no ROS installed.

What it does:
  * Speaks just enough of the rosbridge JSON protocol that roslibjs is happy.
    Supports: advertise / unadvertise / publish / subscribe / unsubscribe.
  * Loads map.json -> extracts obstacle blobs -> publishes /navigation/obstacles.
  * Simulates a robot at (0, 0) that moves toward incoming /navigation/goal
    using the SAME APF + velocity-ramp logic as navigation_node.py
    (so what you see on the screen matches what the real robot will do).
  * Publishes /tb3_0/ekf/odom continuously (so the UI can draw a marker).
  * Publishes /navigation/status: IDLE / NAVIGATING / REACHED.

Wire layout (Mac, all on localhost):

    browser :5002 --HTTP-->  server.py     (serves index.html, map.json)
    browser :9090 --WS-->    mock_ros_bridge.py  (this script)
                                 |  pub /navigation/obstacles
                                 |  pub /tb3_0/ekf/odom (live)
                                 |  pub /navigation/status
                                 |
                                 v
                        UI's roslibjs subscriptions

When you go from Mac mock -> real Pi, just swap this script for
`ros2 run rosbridge_server rosbridge_websocket`. The browser code does not
change — same WS endpoint, same messages.

Run:
    /usr/bin/python3 mock_ros_bridge.py --map map.json --port 9090
"""

import argparse
import asyncio
import json
import math
import time
from pathlib import Path

import numpy as np
import websockets

from map_to_obstacles import extract_obstacles


# ── Robot simulation parameters (mirror navigation_node.py) ──────────────────
MAX_LINEAR  = 0.18    # m/s
MAX_ANGULAR = 0.45    # rad/s
ACC_MAX     = 0.15    # m/s²
ALPHA_MAX   = 0.25    # rad/s²
GOAL_TOL    = 0.08    # m
SLOW_RADIUS = 0.40    # m
DT          = 0.05    # s  (20 Hz, matches navigation_node)

# APF
K_ATT  = 1.0
K_REP  = 0.8
D_REP  = 1.2
SAFETY = 0.45    # formation half-width


# ── Mini rosbridge server ────────────────────────────────────────────────────

class Hub:
    """Tracks connected clients and which topics each subscribes to."""

    def __init__(self):
        # ws -> set of topic names
        self.subs: dict = {}
        # cached "last value" per topic (sent on subscribe so latecomers see it)
        self.last: dict = {}

    def add(self, ws):
        self.subs[ws] = set()

    def remove(self, ws):
        self.subs.pop(ws, None)

    def subscribe(self, ws, topic):
        self.subs.setdefault(ws, set()).add(topic)

    def unsubscribe(self, ws, topic):
        self.subs.get(ws, set()).discard(topic)

    async def publish(self, topic, msg, store_last=True):
        """Push a message to every client subscribed to this topic."""
        if store_last:
            self.last[topic] = msg
        payload = json.dumps({'op': 'publish', 'topic': topic, 'msg': msg})
        dead = []
        for ws, topics in self.subs.items():
            if topic in topics:
                try:
                    await ws.send(payload)
                except Exception:
                    dead.append(ws)
        for ws in dead:
            self.remove(ws)


# ── APF + ramp simulation (mirrors navigation_node.py) ───────────────────────

class RobotSim:
    """Simple holonomic-robot sim with APF planner + velocity ramp."""

    def __init__(self, obstacles):
        # State: pose [x, y, theta] in WORLD frame (matches /odom convention)
        self.pose  = np.zeros(3, dtype=float)
        self.vel   = np.zeros(2, dtype=float)
        self.omega = 0.0
        self.goal  = None     # np.array([x, y, theta]) or None
        self.obstacles = obstacles   # list of (cx, cy, radius)
        self.status = 'IDLE'

    # World-frame APF velocity (same equations as navigation_node._apf_velocity)
    def _apf_velocity(self):
        if self.goal is None:
            return np.zeros(2), 0.0

        pos     = self.pose[:2]
        to_goal = self.goal[:2] - pos
        d_goal  = np.linalg.norm(to_goal)

        if d_goal < GOAL_TOL:
            return np.zeros(2), 0.0

        f_att = K_ATT * to_goal / d_goal

        f_rep = np.zeros(2)
        for (ox, oy, r) in self.obstacles:
            r_eff = r + SAFETY
            diff  = pos - np.array([ox, oy])
            d_raw = np.linalg.norm(diff)
            d     = max(d_raw - r_eff, 0.01)
            if d < D_REP:
                n_hat  = diff / max(d_raw, 1e-6)
                mag    = K_REP * (1.0 / d - 1.0 / D_REP) / (d ** 2)
                f_rep += mag * n_hat

        f_total = f_att + f_rep
        f_mag   = float(np.linalg.norm(f_total))
        if f_mag < 1e-6:
            return np.zeros(2), 0.0

        speed = min(MAX_LINEAR, f_mag)
        if d_goal < SLOW_RADIUS:
            speed *= d_goal / SLOW_RADIUS
        v_des = (speed / f_mag) * f_total

        if np.linalg.norm(v_des) > 0.05:
            desired_heading = math.atan2(v_des[1], v_des[0])
        else:
            desired_heading = self.goal[2]
        heading_err = (desired_heading - self.pose[2] + math.pi) % (2 * math.pi) - math.pi
        omega_des   = max(-MAX_ANGULAR, min(MAX_ANGULAR, 3.0 * heading_err))
        return v_des, omega_des

    def step(self, dt: float):
        v_des, omega_des = self._apf_velocity()

        # Ramp
        dv = v_des - self.vel
        max_dv = ACC_MAX * dt
        if np.linalg.norm(dv) > max_dv:
            self.vel = self.vel + (max_dv / np.linalg.norm(dv)) * dv
        else:
            self.vel = v_des.copy()

        domega = max(-ALPHA_MAX * dt, min(ALPHA_MAX * dt, omega_des - self.omega))
        self.omega += domega

        # Integrate
        self.pose[0] += self.vel[0] * dt
        self.pose[1] += self.vel[1] * dt
        self.pose[2] += self.omega * dt

        # Goal-reached check
        if self.goal is not None:
            dist = np.linalg.norm(self.goal[:2] - self.pose[:2])
            if dist < GOAL_TOL:
                self.status = 'REACHED'
                self.goal   = None
                self.vel    = np.zeros(2)
                self.omega  = 0.0
            else:
                self.status = 'NAVIGATING'
        else:
            if self.status != 'REACHED':
                self.status = 'IDLE'

    # ── ROS-style messages ───────────────────────────────────────────────────

    def odom_msg(self):
        """Build a nav_msgs/Odometry message (subset roslibjs cares about)."""
        x, y, th = float(self.pose[0]), float(self.pose[1]), float(self.pose[2])
        # yaw -> quaternion (Z-axis rotation)
        qz = math.sin(th / 2.0)
        qw = math.cos(th / 2.0)
        return {
            'header': {'frame_id': 'odom',
                       'stamp': {'sec': int(time.time()), 'nanosec': 0}},
            'child_frame_id': 'base_link',
            'pose': {
                'pose': {
                    'position':    {'x': x, 'y': y, 'z': 0.0},
                    'orientation': {'x': 0.0, 'y': 0.0, 'z': qz, 'w': qw},
                },
                'covariance': [0.0] * 36,
            },
            'twist': {
                'twist': {
                    'linear':  {'x': float(self.vel[0]),
                                'y': float(self.vel[1]), 'z': 0.0},
                    'angular': {'x': 0.0, 'y': 0.0, 'z': float(self.omega)},
                },
                'covariance': [0.0] * 36,
            },
        }

    def status_msg(self):
        return {'data': self.status}


def obstacles_to_posearray(obstacles):
    """Convert (x, z, r) blobs to a geometry_msgs/PoseArray.
    Convention used by navigation_node: pose.position.x = x, .y = z, .z = radius
    (since the floor plane is X-Z in our cleaned cloud)."""
    poses = []
    for (x, z, r) in obstacles:
        poses.append({
            'position':    {'x': float(x), 'y': float(z), 'z': float(r)},
            'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0},
        })
    return {
        'header': {'frame_id': 'map',
                   'stamp': {'sec': int(time.time()), 'nanosec': 0}},
        'poses': poses,
    }


# ── WebSocket handler ────────────────────────────────────────────────────────

async def ws_handler(ws, hub: Hub, sim: RobotSim, obstacles_msg: dict):
    print(f'[ws] client connected: {ws.remote_address}')
    hub.add(ws)
    try:
        async for raw in ws:
            try:
                m = json.loads(raw)
            except json.JSONDecodeError:
                continue
            op = m.get('op')

            if op == 'subscribe':
                topic = m['topic']
                hub.subscribe(ws, topic)
                # Send latched obstacles immediately on subscribe
                if topic == '/navigation/obstacles':
                    await ws.send(json.dumps({
                        'op': 'publish', 'topic': topic, 'msg': obstacles_msg,
                    }))
                # Send last known message for any cached topic
                elif topic in hub.last:
                    await ws.send(json.dumps({
                        'op': 'publish', 'topic': topic, 'msg': hub.last[topic],
                    }))
                print(f'[ws] subscribed: {topic}')

            elif op == 'unsubscribe':
                hub.unsubscribe(ws, m.get('topic', ''))

            elif op == 'advertise':
                # Browser saying "I'll publish on this topic" — no-op for us
                pass

            elif op == 'unadvertise':
                pass

            elif op == 'publish':
                topic = m.get('topic')
                msg   = m.get('msg', {})
                if topic == '/navigation/goal':
                    # Twist convention: linear.x/y = goal x/y, angular.z = goal yaw
                    gx = float(msg.get('linear',  {}).get('x', 0.0))
                    gy = float(msg.get('linear',  {}).get('y', 0.0))
                    gth= float(msg.get('angular', {}).get('z', 0.0))
                    sim.goal   = np.array([gx, gy, gth])
                    sim.status = 'NAVIGATING'
                    print(f'[goal] x={gx:+.2f}  y={gy:+.2f}  θ={gth:+.2f}')
                # Other publishes: silently accepted
            else:
                print(f'[ws] unknown op: {op}')
    except websockets.ConnectionClosed:
        pass
    finally:
        hub.remove(ws)
        print(f'[ws] client disconnected: {ws.remote_address}')


# ── Main loops ───────────────────────────────────────────────────────────────

async def physics_loop(hub: Hub, sim: RobotSim):
    """20 Hz simulation: step the robot, publish odom + status."""
    last_status = None
    while True:
        sim.step(DT)
        await hub.publish('/tb3_0/ekf/odom', sim.odom_msg(), store_last=True)
        if sim.status != last_status:
            await hub.publish('/navigation/status', sim.status_msg(), store_last=True)
            last_status = sim.status
        await asyncio.sleep(DT)


async def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--map',  default='map.json')
    ap.add_argument('--port', type=int, default=9090)
    ap.add_argument('--max-blob-cells', type=int, default=None,
                    help='Skip blobs larger than this (None = no cap; '
                         'exclude_largest already handles walls)')
    args = ap.parse_args()

    map_path = Path(args.map)
    if not map_path.exists():
        raise SystemExit(f'map.json not found: {map_path}')

    obstacles = extract_obstacles(args.map,
                                  min_blob_cells=3,
                                  max_blob_cells=args.max_blob_cells,
                                  inflate_radius=0.0,
                                  exclude_largest=True,
                                  sample_wall_step=6)
    obstacles_msg = obstacles_to_posearray(obstacles)
    print(f'[init] loaded {len(obstacles)} obstacles from {args.map}')

    hub = Hub()
    sim = RobotSim(obstacles)

    async def handler(ws):
        await ws_handler(ws, hub, sim, obstacles_msg)

    # Start sim loop and websocket server
    asyncio.create_task(physics_loop(hub, sim))
    print(f'[ws] listening on ws://localhost:{args.port}')
    async with websockets.serve(handler, 'localhost', args.port):
        await asyncio.Future()   # run forever


if __name__ == '__main__':
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print('\nbye')
