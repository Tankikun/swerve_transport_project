import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
import numpy as np


class AICameraNode(Node):
    """
    Pure-subscriber object-size estimator.

    The OAK-D Lite is owned exclusively by depthai_ros_driver (loaded
    from oak_camera.launch.py). Only one host process may hold the
    device, so this node MUST NOT open its own depthai pipeline.

    Subscribes:
      /{robot_id}/camera/depth/image_raw  — sensor_msgs/Image (16UC1, mm)
    Publishes:
      /{robot_id}/camera/object_size      — std_msgs/Float32 (heuristic
                                            estimate, metres)
    """

    def __init__(self):
        super().__init__('ai_camera_node')
        self.declare_parameter('robot_id', 'tb3_0')
        robot_id = self.get_parameter('robot_id').value

        self._size_pub = self.create_publisher(
            Float32, f'/{robot_id}/camera/object_size', 10
        )
        # depthai_ros_driver publishes images with sensor_data QoS
        # (BEST_EFFORT). A RELIABLE subscription will silently fail to
        # match, so we use sensor_data here to be compatible.
        self._depth_sub = self.create_subscription(
            Image,
            f'/{robot_id}/camera/depth/image_raw',
            self._depth_cb,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            f'ai_camera_node subscribed to /{robot_id}/camera/depth/image_raw'
        )

    def _depth_cb(self, msg: Image):
        if msg.encoding != '16UC1':
            self.get_logger().warn_once(
                f'Unexpected depth encoding {msg.encoding!r}; expected 16UC1'
            )
            return
        depth_mm = np.frombuffer(msg.data, dtype=np.uint16).reshape(
            msg.height, msg.width
        )
        self._publish_object_size(depth_mm)

    def _publish_object_size(self, depth_mm: np.ndarray):
        h, w = depth_mm.shape
        roi = depth_mm[h // 4: 3 * h // 4, w // 4: 3 * w // 4]
        valid = roi[roi > 0]
        if valid.size == 0:
            return
        median_m = float(np.median(valid)) / 1000.0
        object_size = max(0.0, 1.0 - median_m / 2.0)
        msg = Float32()
        msg.data = object_size
        self._size_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = AICameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
