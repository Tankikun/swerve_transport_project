import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from geometry_msgs.msg import PoseArray, Polygon, Point32
import numpy as np


class FormationSizeNode(Node):
    """
    Leader-only node. Subscribes to /formation/state (robot poses) and the
    camera's estimated object size, computes the system bounding envelope,
    and publishes the footprint to /navigation/footprint for the nav stack.
    """

    def __init__(self):
        super().__init__('formation_size_node')
        self.declare_parameter('robot_id', 'tb3_0')
        self.declare_parameter('robot_radius', 0.2)   # metres

        self._robot_id = self.get_parameter('robot_id').value
        self._robot_radius = self.get_parameter('robot_radius').value

        self._is_leader = False
        self._formation_poses: list[tuple[float, float]] = []
        self._object_size = 0.0   # metres (from camera estimate)

        self.create_subscription(String, '/formation/leader', self._leader_cb, 10)
        self.create_subscription(PoseArray, '/formation/state', self._formation_cb, 10)
        self.create_subscription(
            Float32, f'/{self._robot_id}/camera/object_size', self._camera_cb, 10
        )
        self._footprint_pub = self.create_publisher(Polygon, '/navigation/footprint', 10)
        self.create_timer(0.5, self._update_footprint)
        self.get_logger().info(f'FormationSizeNode ready for {self._robot_id}')

    def _leader_cb(self, msg: String):
        self._is_leader = (msg.data == self._robot_id)

    def _formation_cb(self, msg: PoseArray):
        self._formation_poses = [(p.position.x, p.position.y) for p in msg.poses]

    def _camera_cb(self, msg: Float32):
        self._object_size = msg.data

    def _update_footprint(self):
        if not self._is_leader or not self._formation_poses:
            return

        pts = np.array(self._formation_poses)
        margin = self._robot_radius + self._object_size * 0.5
        x_min, y_min = pts.min(axis=0) - margin
        x_max, y_max = pts.max(axis=0) + margin

        poly = Polygon()
        for x, y in [(x_min, y_min), (x_max, y_min), (x_max, y_max), (x_min, y_max)]:
            pt = Point32()
            pt.x, pt.y = float(x), float(y)
            poly.points.append(pt)
        self._footprint_pub.publish(poly)


def main(args=None):
    rclpy.init(args=args)
    node = FormationSizeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
