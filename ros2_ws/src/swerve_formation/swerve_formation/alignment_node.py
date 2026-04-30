import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Pose, PoseArray, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, String
from std_srvs.srv import Trigger


class AlignmentNode(Node):
    """
    Pre-run alignment node. Runs on every robot; only the elected leader
    coordinates the alignment sequence when its trigger_alignment service
    is called. Followers respond by measuring their own depth and publishing
    it so the leader can compute corrections for all robots.

    Sequence (leader only):
      1. Measure leader depth from OAK-D central ROI.
      2. Call /{neighbor}/trigger_alignment on each follower (they publish depth).
      3. Wait up to 3 s for all neighbor depth readings.
      4. Nudge every robot toward/away from the object to equalise depth.
      5. Compute final robot-to-robot spacing; publish PoseArray of offsets.
      6. Publish 'done' on /alignment/status.
    """

    def __init__(self):
        super().__init__('alignment_node')
        self.declare_parameter('robot_id', 'tb3_0')
        self.declare_parameter('neighbors', ['tb3_1'])
        self.declare_parameter('alignment_speed', 0.05)
        self.declare_parameter('depth_topic', 'depth')
        self.declare_parameter('offset_init_mode', 'manual')

        self._robot_id = self.get_parameter('robot_id').value
        self._neighbors = list(self.get_parameter('neighbors').value)
        self._alignment_speed = self.get_parameter('alignment_speed').value
        self._offset_init_mode = self.get_parameter('offset_init_mode').value

        self._is_leader = False
        self._my_pose = np.zeros(2)
        self._neighbor_poses = {n: np.zeros(2) for n in self._neighbors}

        self._depth_lock = threading.Lock()
        self._latest_depth: np.ndarray | None = None

        self._neighbor_depth_events = {n: threading.Event() for n in self._neighbors}
        self._neighbor_depths: dict[str, float] = {}

        # Subscriptions
        self.create_subscription(String, '/formation/leader', self._leader_cb, 10)
        self.create_subscription(
            Odometry, f'/{self._robot_id}/ekf/odom', self._odom_cb, 10
        )
        for neighbor in self._neighbors:
            self.create_subscription(
                Odometry,
                f'/{neighbor}/ekf/odom',
                lambda msg, n=neighbor: self._neighbor_odom_cb(msg, n),
                10,
            )
        self.create_subscription(
            Image,
            f'/{self._robot_id}/camera/depth',
            self._depth_cb,
            10,
        )
        for neighbor in self._neighbors:
            self.create_subscription(
                Float32,
                f'/{neighbor}/alignment/depth',
                lambda msg, n=neighbor: self._neighbor_depth_cb(msg, n),
                10,
            )

        # Publishers
        self._cmd_pub = self.create_publisher(Twist, f'/{self._robot_id}/cmd_vel', 10)
        self._status_pub = self.create_publisher(String, '/alignment/status', 10)
        self._offsets_pub = self.create_publisher(PoseArray, '/formation/offsets', 10)
        self._depth_pub = self.create_publisher(
            Float32, f'/{self._robot_id}/alignment/depth', 10
        )
        self._neighbor_cmd_pubs = {
            n: self.create_publisher(Twist, f'/{n}/cmd_vel', 10)
            for n in self._neighbors
        }

        # Service clients for triggering neighbor alignment
        self._neighbor_trigger_clients = {
            n: self.create_client(Trigger, f'/{n}/trigger_alignment')
            for n in self._neighbors
        }

        # Inbound service — starts sequence on leader, measures depth on follower
        self.create_service(
            Trigger, f'/{self._robot_id}/trigger_alignment', self._trigger_cb
        )

        self.get_logger().info(
            f'AlignmentNode ready for {self._robot_id} '
            f'(offset_init_mode={self._offset_init_mode})'
        )

        if self._offset_init_mode == 'odom':
            threading.Thread(target=self._odom_init_sequence, daemon=True).start()
        elif self._offset_init_mode == 'camera':
            threading.Thread(target=self._auto_camera_sequence, daemon=True).start()

    # ------------------------------------------------------------------
    # Subscription callbacks
    # ------------------------------------------------------------------

    def _leader_cb(self, msg: String):
        self._is_leader = (msg.data == self._robot_id)

    def _odom_cb(self, msg: Odometry):
        self._my_pose[0] = msg.pose.pose.position.x
        self._my_pose[1] = msg.pose.pose.position.y

    def _neighbor_odom_cb(self, msg: Odometry, neighbor_id: str):
        self._neighbor_poses[neighbor_id][0] = msg.pose.pose.position.x
        self._neighbor_poses[neighbor_id][1] = msg.pose.pose.position.y

    def _depth_cb(self, msg: Image):
        if msg.encoding != '16UC1':
            return
        arr = np.frombuffer(bytes(msg.data), dtype=np.uint16).reshape(
            msg.height, msg.width
        )
        with self._depth_lock:
            self._latest_depth = arr

    def _neighbor_depth_cb(self, msg: Float32, neighbor_id: str):
        self._neighbor_depths[neighbor_id] = msg.data
        self._neighbor_depth_events[neighbor_id].set()

    # ------------------------------------------------------------------
    # Trigger service handler
    # ------------------------------------------------------------------

    def _trigger_cb(self, request, response):
        if self._is_leader:
            threading.Thread(
                target=self._run_alignment_sequence, daemon=True
            ).start()
            response.success = True
            response.message = 'Alignment sequence started'
        else:
            # Follower: measure depth and publish for the leader to collect
            depth = self._measure_depth()
            if depth is None:
                response.success = False
                response.message = 'No depth data available'
                return response
            out = Float32()
            out.data = depth
            self._depth_pub.publish(out)
            response.success = True
            response.message = f'Depth measured: {depth:.3f} m'
        return response

    # ------------------------------------------------------------------
    # Depth measurement helper
    # ------------------------------------------------------------------

    def _measure_depth(self) -> float | None:
        with self._depth_lock:
            frame = self._latest_depth
        if frame is None:
            return None
        h, w = frame.shape
        roi = frame[h // 4: 3 * h // 4, w // 4: 3 * w // 4]
        valid = roi[roi > 0]
        if valid.size == 0:
            return None
        return float(np.median(valid)) / 1000.0

    # ------------------------------------------------------------------
    # Alignment sequence (runs in background thread on leader only)
    # ------------------------------------------------------------------

    def _run_alignment_sequence(self):
        self._publish_status('aligning')

        # Step 1 — measure leader depth
        my_depth = self._measure_depth()
        if my_depth is None:
            self.get_logger().warn('Alignment: no depth data for leader — aborting')
            self._publish_status('idle')
            return

        # Step 2 — reset neighbor events and fire service calls
        for n in self._neighbors:
            self._neighbor_depth_events[n].clear()
            self._neighbor_depths.pop(n, None)

        for n, cli in self._neighbor_trigger_clients.items():
            if cli.service_is_ready():
                cli.call_async(Trigger.Request())
            else:
                self.get_logger().warn(
                    f'Alignment: service /{n}/trigger_alignment not ready'
                )

        # Step 3 — wait up to 3 s for all neighbor depths
        deadline = time.time() + 3.0
        for n in self._neighbors:
            remaining = max(0.0, deadline - time.time())
            self._neighbor_depth_events[n].wait(timeout=remaining)

        missing = [n for n in self._neighbors if n not in self._neighbor_depths]
        if missing:
            self.get_logger().warn(
                f'Alignment: missing depth from {missing} — aborting'
            )
            self._publish_status('idle')
            return

        # Step 4 — compute per-robot corrections and apply nudges in parallel
        all_robots = [self._robot_id] + self._neighbors
        depth_map = {self._robot_id: my_depth}
        depth_map.update(self._neighbor_depths)
        mean_depth = float(np.mean([depth_map[r] for r in all_robots]))

        cmd_pubs = {self._robot_id: self._cmd_pub}
        cmd_pubs.update(self._neighbor_cmd_pubs)

        def nudge(robot_id: str, correction: float):
            duration = (
                abs(correction) / self._alignment_speed
                if self._alignment_speed > 0.0
                else 0.0
            )
            if duration < 0.001:
                return
            cmd = Twist()
            cmd.linear.x = self._alignment_speed * (1.0 if correction > 0 else -1.0)
            pub = cmd_pubs[robot_id]
            t0 = time.time()
            while time.time() - t0 < duration:
                pub.publish(cmd)
                time.sleep(0.05)
            pub.publish(Twist())

        nudge_threads = [
            threading.Thread(
                target=nudge,
                args=(r, mean_depth - depth_map[r]),
                daemon=True,
            )
            for r in all_robots
        ]
        for t in nudge_threads:
            t.start()
        for t in nudge_threads:
            t.join()

        # Step 5 — read final EKF poses and publish offset PoseArray
        time.sleep(0.2)
        positions = np.array(
            [self._my_pose.copy()]
            + [self._neighbor_poses[n].copy() for n in self._neighbors]
        )
        center = positions.mean(axis=0)

        pa = PoseArray()
        pa.header.stamp = self.get_clock().now().to_msg()
        pa.header.frame_id = 'odom'
        for pos in positions:
            p = Pose()
            offset = pos - center
            p.position.x = float(offset[0])
            p.position.y = float(offset[1])
            p.orientation.w = 1.0
            pa.poses.append(p)
        self._offsets_pub.publish(pa)

        # Step 6 — signal completion
        self._publish_status('done')
        self.get_logger().info('Alignment sequence complete')

    # ------------------------------------------------------------------
    # Automatic initialisation paths
    # ------------------------------------------------------------------

    def _odom_init_sequence(self):
        deadline = time.time() + 10.0
        while time.time() < deadline:
            poses_ready = (
                np.any(self._my_pose != 0)
                and all(np.any(self._neighbor_poses[n] != 0) for n in self._neighbors)
            )
            if poses_ready:
                break
            time.sleep(0.5)
        else:
            self.get_logger().warn(
                'Offset init (odom): timed out waiting for valid EKF poses — skipping'
            )
            return

        positions = np.array(
            [self._my_pose.copy()]
            + [self._neighbor_poses[n].copy() for n in self._neighbors]
        )
        center = positions.mean(axis=0)

        pa = PoseArray()
        pa.header.stamp = self.get_clock().now().to_msg()
        pa.header.frame_id = 'odom'
        offsets = []
        for pos in positions:
            p = Pose()
            offset = pos - center
            p.position.x = float(offset[0])
            p.position.y = float(offset[1])
            p.orientation.w = 1.0
            pa.poses.append(p)
            offsets.append(offset.tolist())
        self._offsets_pub.publish(pa)
        self._publish_status('done')
        self.get_logger().info(f'Offset init from odometry complete: {offsets}')

    def _auto_camera_sequence(self):
        deadline = time.time() + 10.0
        while time.time() < deadline:
            with self._depth_lock:
                ready = self._latest_depth is not None
            if ready:
                break
            time.sleep(0.5)
        else:
            self.get_logger().warn(
                'Offset init (camera): timed out waiting for depth frame — skipping'
            )
            return

        self._run_alignment_sequence()

    def _publish_status(self, status: str):
        msg = String()
        msg.data = status
        self._status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = AlignmentNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
