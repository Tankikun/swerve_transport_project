import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class LeaderElectionNode(Node):
    """
    Bully-inspired election: the active robot with the lowest priority value wins.
    Publishes its own presence on /formation/heartbeat and the current winner
    on /formation/leader. Scales to 3+ robots automatically.
    """

    HEARTBEAT_INTERVAL = 0.5   # seconds between heartbeat publishes
    PEER_TIMEOUT = 2.0         # seconds before a peer is considered dead

    def __init__(self):
        super().__init__('leader_election_node')
        self.declare_parameter('robot_id', 'tb3_0')
        # Lower priority value → higher rank (tb3_0=0 beats tb3_1=1, etc.)
        self.declare_parameter('priority', 0)

        self._robot_id = self.get_parameter('robot_id').value
        self._priority = self.get_parameter('priority').value

        # peer_id → (priority, last_seen_time)
        self._peers: dict[str, tuple[int, float]] = {}
        self._current_leader: str | None = None

        self._hb_pub = self.create_publisher(String, '/formation/heartbeat', 10)
        self._leader_pub = self.create_publisher(String, '/formation/leader', 10)
        self.create_subscription(String, '/formation/heartbeat', self._hb_cb, 10)

        self.create_timer(self.HEARTBEAT_INTERVAL, self._publish_heartbeat)
        self.create_timer(self.HEARTBEAT_INTERVAL, self._elect)
        self.get_logger().info(
            f'Leader election ready: {self._robot_id} (priority {self._priority})'
        )

    def _publish_heartbeat(self):
        msg = String()
        msg.data = f'{self._robot_id}:{self._priority}'
        self._hb_pub.publish(msg)

    def _hb_cb(self, msg: String):
        parts = msg.data.split(':')
        if len(parts) != 2:
            return
        peer_id, peer_prio_str = parts
        try:
            self._peers[peer_id] = (int(peer_prio_str), time.time())
        except ValueError:
            pass

    def _elect(self):
        now = time.time()
        # Drop peers whose heartbeats have expired
        self._peers = {
            pid: (prio, t)
            for pid, (prio, t) in self._peers.items()
            if now - t < self.PEER_TIMEOUT
        }

        # All candidates: active peers + self
        candidates = {self._robot_id: self._priority}
        for pid, (prio, _) in self._peers.items():
            candidates[pid] = prio

        leader = min(candidates, key=candidates.__getitem__)

        if leader != self._current_leader:
            self._current_leader = leader
            self.get_logger().info(f'Leader elected: {leader}')

        out = String()
        out.data = leader
        self._leader_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = LeaderElectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
