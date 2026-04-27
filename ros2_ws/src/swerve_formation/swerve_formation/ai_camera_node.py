import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
import numpy as np


class AICameraNode(Node):
    """
    OAK-D Lite RGB + stereo depth pipeline via DepthAI.
    Publishes:
      /{robot_id}/camera/rgb         — sensor_msgs/Image (bgr8)
      /{robot_id}/camera/depth       — sensor_msgs/Image (16UC1, mm)
      /{robot_id}/camera/object_size — std_msgs/Float32 (estimated diameter, metres)
    Requires: pip install depthai
    Falls back to stub mode (no publishes) when hardware is unavailable.
    """

    def __init__(self):
        super().__init__('ai_camera_node')
        self.declare_parameter('robot_id', 'tb3_0')
        self.declare_parameter('fps', 30)

        robot_id = self.get_parameter('robot_id').value
        fps = self.get_parameter('fps').value

        self._rgb_pub = self.create_publisher(Image, f'/{robot_id}/camera/rgb', 10)
        self._depth_pub = self.create_publisher(Image, f'/{robot_id}/camera/depth', 10)
        self._size_pub = self.create_publisher(Float32, f'/{robot_id}/camera/object_size', 10)

        self._rgb_q = None
        self._depth_q = None

        try:
            import depthai as dai
            pipeline = self._build_pipeline(dai, fps)
            self._device = dai.Device(pipeline)
            self._rgb_q = self._device.getOutputQueue('rgb', maxSize=4, blocking=False)
            self._depth_q = self._device.getOutputQueue('depth', maxSize=4, blocking=False)
            self.create_timer(1.0 / fps, self._capture)
            self.get_logger().info(f'OAK-D Lite pipeline started for {robot_id}')
        except Exception as e:
            self.get_logger().warn(
                f'DepthAI unavailable ({e}) — ai_camera_node running in stub mode'
            )

    @staticmethod
    def _build_pipeline(dai, fps: int):
        pipeline = dai.Pipeline()

        cam_rgb = pipeline.create(dai.node.ColorCamera)
        cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        cam_rgb.setFps(fps)

        mono_l = pipeline.create(dai.node.MonoCamera)
        mono_l.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        mono_l.setBoardSocket(dai.CameraBoardSocket.LEFT)

        mono_r = pipeline.create(dai.node.MonoCamera)
        mono_r.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        mono_r.setBoardSocket(dai.CameraBoardSocket.RIGHT)

        stereo = pipeline.create(dai.node.StereoDepth)
        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
        mono_l.out.link(stereo.left)
        mono_r.out.link(stereo.right)

        xout_rgb = pipeline.create(dai.node.XLinkOut)
        xout_rgb.setStreamName('rgb')
        cam_rgb.video.link(xout_rgb.input)

        xout_depth = pipeline.create(dai.node.XLinkOut)
        xout_depth.setStreamName('depth')
        stereo.depth.link(xout_depth.input)

        return pipeline

    def _capture(self):
        if self._rgb_q is None:
            return
        if rgb_frame := self._rgb_q.tryGet():
            self._rgb_pub.publish(self._to_image(rgb_frame.getCvFrame(), 'bgr8'))
        if depth_frame := self._depth_q.tryGet():
            depth_np = depth_frame.getFrame()
            self._depth_pub.publish(self._to_image(depth_np, '16UC1'))
            self._publish_object_size(depth_np)

    def _publish_object_size(self, depth_mm: np.ndarray):
        h, w = depth_mm.shape
        roi = depth_mm[h // 4: 3 * h // 4, w // 4: 3 * w // 4]
        valid = roi[roi > 0]
        if valid.size == 0:
            return
        # Rough heuristic: centre-depth → estimated object diameter
        median_m = float(np.median(valid)) / 1000.0
        object_size = max(0.0, 1.0 - median_m / 2.0)
        msg = Float32()
        msg.data = object_size
        self._size_pub.publish(msg)

    @staticmethod
    def _to_image(frame: np.ndarray, encoding: str) -> Image:
        msg = Image()
        msg.height, msg.width = frame.shape[:2]
        msg.encoding = encoding
        msg.step = int(frame.strides[0])
        msg.data = frame.tobytes()
        return msg


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
