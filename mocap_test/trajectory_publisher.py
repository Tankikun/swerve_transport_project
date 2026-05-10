#!/usr/bin/env python3
"""
trajectory_publisher.py
-----------------------
Drives a 2-robot swerve formation along a closed-bottom upside-down-U
loop for MoCap accuracy benchmarking. Publishes Twist messages on
/virtual_center/cmd_vel; laplacian_formation_node on each robot
translates that into per-robot /{robot_id}/cmd_vel.

Trajectory (5 legs, all body-frame relative to the virtual centre).
Bounding box of the virtual-centre path is 2 m x 2 m.

    A:  1 --> 2   strafe forward 2.0 m
    B:  2 --> 3   arc CCW 180 deg, radius 1.0 m
    C:  3 --> 4   strafe forward 2.0 m
    D:  4 --> 5   pivot +90 deg (rotation only)
    E:  5 --> 1   strafe forward 2.0 m   (closes the loop)

World-frame layout (start at origin facing world +Y / "north"):

    1 = ( 0, 0)  heading  90 deg
    2 = ( 0, 2)  heading  90 deg
    3 = (-2, 2)  heading 270 deg   (after CCW arc, R=1.0)
    4 = (-2, 0)  heading 270 deg
    5 = (-2, 0)  heading   0 deg   (after +90 deg pivot)

Speeds chosen so the outer robot of a 0.9 m-wide formation stays
inside MAX_WHEEL_LINEAR=0.18 m/s during the arc. Strafe and pivot
have margin to spare.

Run:
    ros2 run rclpy ...   # (this script is standalone, see __main__)

    python3 trajectory_publisher.py            # full loop, default speeds
    python3 trajectory_publisher.py --slow     # half-speed for cleaner data

The script does NOT subscribe to anything. It is open-loop — relies on
laplacian_formation_node to do the formation kinematics and on the
operator to start the robots in the right pose. MoCap is the ground
truth; this script's job is just to drive the trajectory reproducibly.
"""

import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


# ── Trajectory geometry (2 m x 2 m bounding box for the VC path) ─────
LEG_LENGTH_M  = 2.0     # straight legs A, C, E
ARC_RADIUS_M  = 1.0     # arc B (R = arc_vx / arc_omega; see below)
PIVOT_ANGLE_RAD = math.pi / 2.0   # leg D

# ── Default speeds (body-frame, virtual centre) ──────────────────────
# Chosen so the outer robot of the largest tested formation (D=0.9 m)
# stays inside MAX_WHEEL_LINEAR=0.18 m/s during the arc, so the
# laplacian saturation scaler never engages:
#   v_robot_center  = vx + omega * (D/2)       (worst-case offset)
#   v_wheel_max     = v_robot_center + omega * 0.212   (chassis half-diag)
#   For D=0.9, vx=0.10, omega=0.10:
#       v_robot_center = 0.10 + 0.045 = 0.145 m/s
#       v_wheel_max    = 0.145 + 0.0212 = 0.166 m/s   ✓ under 0.18
# arc_vx / arc_omega is exactly ARC_RADIUS_M, so the arc traces a
# circular path (not elliptical).
DEFAULT_STRAFE_VX   = 0.10    # legs A, C, E (no rotation, no constraint)
DEFAULT_ARC_VX      = 0.10    # leg B (combined with omega)
DEFAULT_ARC_OMEGA   = 0.10    # leg B (=> arc duration = pi/0.10 ≈ 31.42 s)
DEFAULT_PIVOT_OMEGA = 0.20    # leg D (no translation)

# Pause between legs so checkpoints are visible in the bag as flat
# zero-velocity sections. 1 s is enough to mark the transition without
# stretching the run.
CHECKPOINT_PAUSE_S = 1.0

# Control loop rate at which we publish the constant Twist.
PUB_HZ = 20.0


def make_twist(vx: float = 0.0, vy: float = 0.0, wz: float = 0.0) -> Twist:
    t = Twist()
    t.linear.x = float(vx)
    t.linear.y = float(vy)
    t.angular.z = float(wz)
    return t


class TrajectoryPublisher(Node):

    def __init__(self, args):
        super().__init__('mocap_trajectory_publisher')
        self._pub = self.create_publisher(Twist, '/virtual_center/cmd_vel', 10)
        self._args = args
        self._stop_twist = make_twist()

        # Wait briefly so subscribers are connected before the first leg.
        # rclpy doesn't expose a clean wait-for-subscriber, so we sleep.
        self.get_logger().info('Waiting 1.0 s for /virtual_center/cmd_vel subscribers...')
        time.sleep(1.0)

    # ──────────────────────────────────────────────────────────────────
    # Primitive: hold a constant Twist for a given duration, publishing
    # at PUB_HZ. Cleanly publishes a final zero twist when done so the
    # robots brake at the checkpoint.
    # ──────────────────────────────────────────────────────────────────
    def _hold(self, twist: Twist, duration_s: float, leg_name: str):
        self.get_logger().info(
            f'[{leg_name}] vx={twist.linear.x:+.3f} vy={twist.linear.y:+.3f} '
            f'wz={twist.angular.z:+.3f} for {duration_s:.2f} s'
        )
        period = 1.0 / PUB_HZ
        n_ticks = int(round(duration_s * PUB_HZ))
        for _ in range(n_ticks):
            if not rclpy.ok():
                return
            self._pub.publish(twist)
            time.sleep(period)
        # Brake on every leg boundary so each checkpoint is a clean stop.
        for _ in range(int(CHECKPOINT_PAUSE_S * PUB_HZ)):
            if not rclpy.ok():
                return
            self._pub.publish(self._stop_twist)
            time.sleep(period)

    # ──────────────────────────────────────────────────────────────────
    # The trajectory itself.
    # ──────────────────────────────────────────────────────────────────
    def run(self):
        scale = 0.5 if self._args.slow else 1.0
        strafe_vx   = DEFAULT_STRAFE_VX  * scale
        arc_vx      = DEFAULT_ARC_VX     * scale
        arc_omega   = DEFAULT_ARC_OMEGA  * scale
        pivot_omega = DEFAULT_PIVOT_OMEGA * scale

        # Leg A: 1 -> 2  forward LEG_LENGTH at strafe_vx
        leg_a_t = LEG_LENGTH_M / strafe_vx
        self._hold(make_twist(vx=strafe_vx), leg_a_t, 'A: 1->2 strafe')

        # Leg B: 2 -> 3  arc CCW 180 deg
        # tangential speed = arc_vx, omega = arc_omega
        # arc length = pi * R must equal arc_vx * t
        # consistency check: omega = arc_vx / R
        expected_omega = arc_vx / ARC_RADIUS_M
        if abs(expected_omega - arc_omega) > 1e-3:
            self.get_logger().warn(
                f'Arc kinematics mismatch: arc_vx/R={expected_omega:.3f} '
                f'vs arc_omega={arc_omega:.3f}. Using arc_omega; arc shape '
                f'will be elliptical, not circular at R={ARC_RADIUS_M}.'
            )
        leg_b_t = math.pi / arc_omega   # half-revolution at arc_omega
        self._hold(make_twist(vx=arc_vx, wz=arc_omega), leg_b_t, 'B: 2->3 arc CCW 180')

        # Leg C: 3 -> 4  forward LEG_LENGTH (robot is now facing -X world)
        leg_c_t = LEG_LENGTH_M / strafe_vx
        self._hold(make_twist(vx=strafe_vx), leg_c_t, 'C: 3->4 strafe')

        # Leg D: 4 -> 5  pivot +90 deg in place
        leg_d_t = PIVOT_ANGLE_RAD / pivot_omega
        self._hold(make_twist(wz=pivot_omega), leg_d_t, 'D: 4->5 pivot +90')

        # Leg E: 5 -> 1  forward LEG_LENGTH (closing leg)
        leg_e_t = LEG_LENGTH_M / strafe_vx
        self._hold(make_twist(vx=strafe_vx), leg_e_t, 'E: 5->1 strafe (close)')

        # Final stop — publish zeros for a couple of seconds so the bag
        # ends with a flat tail (helps trajectory-error tools).
        self._hold(self._stop_twist, 0.0, 'STOP')
        self.get_logger().info('Trajectory complete.')


def parse_args(argv):
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--slow', action='store_true',
                   help='Run at half-speed (cleaner data, longer test).')
    return p.parse_args(argv)


def main(argv=None):
    args = parse_args(sys.argv[1:] if argv is None else argv)
    rclpy.init(args=None)
    node = TrajectoryPublisher(args)
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted; publishing stop twist.')
        node._pub.publish(make_twist())
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
