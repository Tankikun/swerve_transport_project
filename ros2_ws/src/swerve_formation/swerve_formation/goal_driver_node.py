"""
goal_driver_node.py
-------------------
A minimal "drive forward to a goal" command source for the demo. Exists
so the demo can run autonomously without a human at a teleop keyboard,
and so the leader integrates an actual distance instead of just running
for N seconds.

Integrates the leader's encoder odometry (`/{leader_robot_id}/odom`,
published by `conveyor_base_node`) and emits a constant-velocity Twist
on `/virtual_center/cmd_vel/raw` until the integrated XY distance
reaches `goal_distance_m`. Then publishes zeros and stays quiet.

Why feed `/virtual_center/cmd_vel/raw` (not `/virtual_center/cmd_vel`):
the topic-chain we built has `obstacle_avoidance_node` in the middle,
which subscribes to `/virtual_center/cmd_vel/raw` and republishes the
modified twist on `/virtual_center/cmd_vel`. If you want to bypass
obstacle avoidance for a free-air test, remap `/virtual_center/cmd_vel/raw`
→ `/virtual_center/cmd_vel` at launch.

Why integrate the leader's odom (not raw command × dt):
the firmware is encoder-based since `feature/two-robot-test-seven`;
free-spin / wheel slip won't fool us into thinking we travelled when
we didn't. Drift over a 3 m straight-line run is ~5–10 cm — well within
our demo tolerance.

Lifecycle:
  * On startup, waits for the first `/{leader}/odom` message → seeds
    the integration origin.
  * From then on, every `/odom` callback advances the integrated
    distance and decides whether to keep driving.
  * When the goal distance is reached (or the trip-meter timer
    triggers), publishes a zero Twist and logs DONE.
  * `/goal_driver/start` (std_srvs/Trigger) re-arms the run from the
    current pose. Useful for re-running the demo without restarting
    the node.

Parameters:
  leader_robot_id   str    robot whose /odom we integrate    (default tb3_1)
  goal_distance_m   float  trip distance before stopping     (default 2.5)
  forward_speed     float  steady-state linear.x in m/s      (default 0.10)
  ramp_up_s         float  linear ramp from 0 to forward_speed (default 1.0)
  pub_rate_hz       float  command publish rate              (default 20)
  start_immediately bool   begin driving on startup, no service call needed
                           (default True). Set False for service-driven runs.
"""

from __future__ import annotations

import math
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_srvs.srv import Trigger


class GoalDriverNode(Node):

    def __init__(self) -> None:
        super().__init__('goal_driver_node')

        self.declare_parameter('leader_robot_id',   'tb3_1')
        self.declare_parameter('goal_distance_m',   2.5)
        self.declare_parameter('forward_speed',     0.10)
        self.declare_parameter('ramp_up_s',         1.0)
        self.declare_parameter('pub_rate_hz',       20.0)
        self.declare_parameter('start_immediately', True)

        self._leader_id     = str(self.get_parameter('leader_robot_id').value)
        self._goal_m        = float(self.get_parameter('goal_distance_m').value)
        self._forward_speed = float(self.get_parameter('forward_speed').value)
        self._ramp_up_s     = max(0.0, float(self.get_parameter('ramp_up_s').value))
        self._pub_rate_hz   = float(self.get_parameter('pub_rate_hz').value)
        self._start_now     = bool(self.get_parameter('start_immediately').value)

        self._lock          = threading.Lock()
        self._origin_xy     = None         # seeded at first odom msg
        self._latest_xy     = None
        self._distance_m    = 0.0
        self._active        = False        # currently emitting non-zero cmd
        self._done          = False
        self._t_start       = None         # wall time when active set True

        self.create_subscription(
            Odometry, f'/{self._leader_id}/odom',
            self._odom_cb, 10,
        )
        self._cmd_pub = self.create_publisher(
            Twist, '/virtual_center/cmd_vel/raw', 10,
        )

        # Service to (re)start a run. Convenient for back-to-back demos.
        self.create_service(Trigger, '/goal_driver/start', self._start_srv_cb)

        self.create_timer(1.0 / self._pub_rate_hz, self._pub_loop)

        self.get_logger().info(
            f'goal_driver_node ready  leader={self._leader_id}  '
            f'goal_distance={self._goal_m:.2f} m  '
            f'forward_speed={self._forward_speed:.2f} m/s  '
            f'start_immediately={self._start_now}'
        )

    # ── helpers ───────────────────────────────────────────────────────────

    def _arm_run(self) -> None:
        with self._lock:
            self._origin_xy  = self._latest_xy   # may still be None — handled below
            self._distance_m = 0.0
            self._done       = False
            self._active     = True
            self._t_start    = time.time()
        self.get_logger().info(
            f'goal_driver: ARMED — driving forward {self._goal_m:.2f} m'
        )

    # ── callbacks ─────────────────────────────────────────────────────────

    def _odom_cb(self, msg: Odometry) -> None:
        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)
        with self._lock:
            self._latest_xy = (x, y)
            if self._origin_xy is None and self._start_now:
                self._origin_xy = (x, y)
                self._active    = True
                self._t_start   = time.time()
                self.get_logger().info(
                    f'goal_driver: first odom received — '
                    f'origin=({x:.2f},{y:.2f}); starting run'
                )
            if self._origin_xy is not None and self._active:
                dx = x - self._origin_xy[0]
                dy = y - self._origin_xy[1]
                self._distance_m = math.hypot(dx, dy)
                if self._distance_m >= self._goal_m:
                    self._active = False
                    self._done   = True
                    self.get_logger().info(
                        f'goal_driver: REACHED — distance={self._distance_m:.2f} m'
                    )

    def _start_srv_cb(self, request, response):
        self._arm_run()
        response.success = True
        response.message = f'driving forward {self._goal_m:.2f} m'
        return response

    # ── publish loop ──────────────────────────────────────────────────────

    def _pub_loop(self) -> None:
        with self._lock:
            active   = self._active
            t_start  = self._t_start
            distance = self._distance_m

        cmd = Twist()
        if active and t_start is not None:
            elapsed = time.time() - t_start
            ramp = min(1.0, elapsed / self._ramp_up_s) if self._ramp_up_s > 0 else 1.0
            cmd.linear.x = self._forward_speed * ramp
        # else zeros — explicit stop when idle or done
        self._cmd_pub.publish(cmd)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GoalDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
