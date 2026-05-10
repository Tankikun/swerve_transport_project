"""
path_planner_node.py
====================
Laptop-side path planner. ONE-SHOT per goal: subscribe to the user's goal,
both robot poses, and the formation footprint; plan a path for the *virtual
center* of the formation; publish it on a latched topic so the leader's
follower (and the dormant follower copies on the other robots) can consume
it natively.

Per Tankhun's architecture spec (swerve-transport architecture reference):

  Subscriptions
    /goal_pose                 geometry_msgs/PoseStamped     (Flask UI publishes once on submit)
    /formation/footprint       geometry_msgs/Polygon         (formation_size_node, on change)
    /tb3_0/pose                geometry_msgs/PoseStamped     (ekf_node @ tb3_0, 20 Hz)
    /tb3_1/pose                geometry_msgs/PoseStamped     (ekf_node @ tb3_1, 20 Hz)

  Publication
    /formation/path            geometry_msgs/PoseArray       (LATCHED via transient_local)

  Map
    Loaded from disk at startup (default: map.json next to this node, or
    --map-path parameter). map.json is the preprocessed grid produced by
    db_to_map_json.py from the lab.db RTAB-Map mapping run.

The path is for the virtual center C = (P0 + P1) / 2. The Polygon footprint
is converted to a bounding-circle radius (max distance from centroid to any
vertex) — this is the v1 simplification; a true polygon-aware planner is
follow-up work, see NAVIGATION_PIPELINE.md.

This node imports `compute_plan` from `astar_planner.py` (in the
`interface/` folder of this repo). The algorithm itself is unchanged from
the Mac-side dev sandbox; only the I/O is wrapped here.
"""

import json
import math
import os
import sys
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from geometry_msgs.msg import (Point, Polygon, Pose, PoseArray,
                               PoseStamped, Quaternion)
from std_msgs.msg import Header

# ── Import the planner module from interface/ ────────────────────────────
# The algorithm lives in interface/astar_planner.py (synced from the
# Mac-side dev repo). We add interface/ to sys.path at startup.
_THIS = Path(__file__).resolve()
_REPO_ROOT = _THIS.parents[4]  # ros2_ws/src/swerve_formation/swerve_formation/<here>
_INTERFACE = _REPO_ROOT / 'interface'
if str(_INTERFACE) not in sys.path:
    sys.path.insert(0, str(_INTERFACE))
from astar_planner import compute_plan  # noqa: E402


# ── Quaternion <-> yaw helpers (yaw is rotation about Z) ─────────────────
def yaw_to_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


def quat_to_yaw(q: Quaternion) -> float:
    # Standard ROS yaw extraction from quaternion (z-axis rotation only).
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def polygon_to_radius(poly: Polygon) -> float:
    """Convert a Polygon footprint to a bounding-circle radius.

    Uses max distance from centroid to any vertex. Conservative — any
    point inside the polygon is within `radius` of the centroid.

    Returns 0.0 if the polygon is empty or has fewer than 1 vertex.
    """
    pts = poly.points
    if not pts:
        return 0.0
    cx = sum(p.x for p in pts) / len(pts)
    cy = sum(p.y for p in pts) / len(pts)
    return max(math.hypot(p.x - cx, p.y - cy) for p in pts)


class PathPlannerNode(Node):
    """One-shot path planner for the formation virtual center."""

    DEFAULT_FORMATION_RADIUS = 0.45     # m — used until /formation/footprint arrives
    DEFAULT_TARGET_SPACING   = 0.15     # m — arc-length resample default
    DEFAULT_YAW_POLICY       = 'free'   # crab-walk; the formation rotates only at endpoints

    def __init__(self):
        super().__init__('path_planner_node')

        # ── Parameters ────────────────────────────────────────────────
        # map_path: where map.json lives. Default: <repo>/interface/map.json.
        default_map = str(_INTERFACE / 'map.json')
        self.declare_parameter('map_path', default_map)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('yaw_policy', self.DEFAULT_YAW_POLICY)
        self.declare_parameter('target_spacing', self.DEFAULT_TARGET_SPACING)

        map_path = str(self.get_parameter('map_path').value)
        self._frame_id = str(self.get_parameter('frame_id').value)
        self._yaw_policy = str(self.get_parameter('yaw_policy').value)
        self._target_spacing = float(self.get_parameter('target_spacing').value)

        # ── Load map once at startup ──────────────────────────────────
        if not os.path.exists(map_path):
            self.get_logger().error(
                f"map.json not found at '{map_path}'. Set the map_path "
                f"parameter or place map.json next to interface/. Aborting.")
            raise FileNotFoundError(map_path)
        with open(map_path) as f:
            self._map_data = json.load(f)
        meta = self._map_data['metadata']
        self.get_logger().info(
            f"Map loaded: {meta['grid_width']}×{meta['grid_height']} cells "
            f"@ {meta['resolution']:.3f} m, "
            f"X[{meta['min_x']:.2f},{meta['max_x']:.2f}] "
            f"Y[{meta['min_y']:.2f},{meta['max_y']:.2f}]")

        # ── State ─────────────────────────────────────────────────────
        self._pose0 = None   # (x, y, yaw) for tb3_0
        self._pose1 = None   # (x, y, yaw) for tb3_1
        self._goal  = None   # (x, y, yaw) goal for the virtual center
        self._formation_radius = self.DEFAULT_FORMATION_RADIUS
        self._seq = 0        # incremented per published plan, surfaced in metadata

        # ── Publishers (latched / transient_local for /formation/path) ──
        latched_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._path_pub = self.create_publisher(
            PoseArray, '/formation/path', latched_qos)

        # ── Subscriptions ─────────────────────────────────────────────
        self.create_subscription(PoseStamped, '/goal_pose',
                                 self._goal_cb, 10)
        self.create_subscription(Polygon, '/formation/footprint',
                                 self._footprint_cb, 10)
        self.create_subscription(PoseStamped, '/tb3_0/pose',
                                 lambda m: self._pose_cb(0, m), 10)
        self.create_subscription(PoseStamped, '/tb3_1/pose',
                                 lambda m: self._pose_cb(1, m), 10)

        self.get_logger().info(
            f"path_planner_node ready. yaw_policy='{self._yaw_policy}', "
            f"target_spacing={self._target_spacing} m, "
            f"default formation_radius={self._formation_radius} m. "
            f"Waiting for /goal_pose, /tb3_0/pose, /tb3_1/pose.")

    # ── Callbacks ────────────────────────────────────────────────────

    def _pose_cb(self, robot_idx: int, msg: PoseStamped):
        x = msg.pose.position.x
        y = msg.pose.position.y
        yaw = quat_to_yaw(msg.pose.orientation)
        if robot_idx == 0:
            self._pose0 = (x, y, yaw)
        else:
            self._pose1 = (x, y, yaw)

    def _footprint_cb(self, msg: Polygon):
        new_r = polygon_to_radius(msg)
        if new_r <= 0.0:
            self.get_logger().warn(
                "/formation/footprint arrived empty; keeping current radius")
            return
        if abs(new_r - self._formation_radius) > 1e-3:
            self.get_logger().info(
                f"formation_radius: {self._formation_radius:.3f} → {new_r:.3f} m")
            self._formation_radius = new_r

    def _goal_cb(self, msg: PoseStamped):
        gx = msg.pose.position.x
        gy = msg.pose.position.y
        gyaw = quat_to_yaw(msg.pose.orientation)
        self._goal = (gx, gy, gyaw)
        self.get_logger().info(
            f"goal received: ({gx:.2f}, {gy:.2f}, yaw={math.degrees(gyaw):.0f}°)")
        self._try_plan()

    # ── Planning ─────────────────────────────────────────────────────

    def _try_plan(self):
        """Attempt to plan if everything we need is available."""
        if self._pose0 is None:
            self.get_logger().warn("no /tb3_0/pose yet; deferring plan")
            return
        if self._pose1 is None:
            self.get_logger().warn("no /tb3_1/pose yet; deferring plan")
            return
        if self._goal is None:
            return  # called from _goal_cb so this shouldn't fire

        # ── Virtual center C = (P0 + P1) / 2 (per spec) ──
        cx = 0.5 * (self._pose0[0] + self._pose1[0])
        cy = 0.5 * (self._pose0[1] + self._pose1[1])
        # Use the average of robot yaws as the start yaw of the virtual
        # center. The formation may not be perfectly aligned at start.
        start_yaw = math.atan2(
            math.sin(self._pose0[2]) + math.sin(self._pose1[2]),
            math.cos(self._pose0[2]) + math.cos(self._pose1[2]))

        gx, gy, goal_yaw = self._goal
        self.get_logger().info(
            f"planning: VC=({cx:.2f},{cy:.2f}) → goal=({gx:.2f},{gy:.2f}), "
            f"radius={self._formation_radius:.2f} m, "
            f"yaw_policy='{self._yaw_policy}'")

        try:
            plan = compute_plan(
                self._map_data,
                start=(cx, cy),
                goal=(gx, gy),
                formation_radius=self._formation_radius,
                yaw_policy=self._yaw_policy,
                start_yaw=start_yaw,
                goal_yaw=goal_yaw,
                target_spacing=self._target_spacing,
            )
        except (ValueError, RuntimeError) as e:
            self.get_logger().error(f"plan failed: {e}")
            return

        # ── Convert plan dict → PoseArray ────────────────────────────
        pa = self._plan_to_pose_array(plan)
        self._seq += 1
        self._path_pub.publish(pa)

        md = plan['metadata']
        self.get_logger().info(
            f"plan #{self._seq} published: {len(pa.poses)} poses, "
            f"vc_distance={md['vc_distance']:.2f} m, "
            f"radius_used={md['formation_radius']:.2f} m"
            + (" (REDUCED)" if md.get('radius_reduced') else "")
        )

    def _plan_to_pose_array(self, plan: dict) -> PoseArray:
        """Serialize compute_plan() output to geometry_msgs/PoseArray."""
        vc = plan['virtual_center']
        wps = vc['waypoints']      # [[x, y], ...]
        headings = vc['headings']  # [yaw, ...] same length

        pa = PoseArray()
        pa.header = Header()
        pa.header.frame_id = self._frame_id
        pa.header.stamp = self.get_clock().now().to_msg()

        # Waypoints + headings should always be same length; defensive:
        n = min(len(wps), len(headings))
        for i in range(n):
            x, y = wps[i]
            yaw = headings[i]
            p = Pose()
            p.position.x = float(x)
            p.position.y = float(y)
            p.position.z = 0.0
            p.orientation = yaw_to_quat(float(yaw))
            pa.poses.append(p)
        return pa


def main(args=None):
    rclpy.init(args=args)
    try:
        node = PathPlannerNode()
    except FileNotFoundError:
        rclpy.shutdown()
        sys.exit(1)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
