"""
oak_camera_node.py
------------------
Minimal ROS 2 publisher for the Luxonis OAK-D Lite using the depthai
Python SDK directly. Drop-in replacement for the parts of
`depthai_ros_driver` we actually need for RTAB-Map visual SLAM.

Why hand-rolled instead of depthai_ros_driver: the lab apt mirror
corrupts large .deb downloads (proxy MITM at 192.168.2.1) so
`ros-humble-depthai-ros-driver` never installs cleanly. Pi already
has the depthai Python SDK working; this node is ~250 lines vs the
50 MB driver package.

Built for depthai SDK 3.x (the version on pi2 is 3.5.0). The 3.x API
differs significantly from 2.x — no XLinkOut nodes, output queues
are created directly from output objects, pipeline lifecycle uses
pipeline.start() / pipeline.isRunning().

Published topics (default; remappable via launch):
  /{robot_id}/camera/rgb/image_raw       sensor_msgs/Image   (BGR8)
  /{robot_id}/camera/rgb/camera_info     sensor_msgs/CameraInfo
  /{robot_id}/camera/depth/image_raw     sensor_msgs/Image   (16UC1, mm)
  /{robot_id}/camera/depth/camera_info   sensor_msgs/CameraInfo

Coordinate frame:
  - Header.frame_id = "{robot_id}_oak_rgb_camera_optical_frame"
  - The camera→base_link transform must be published separately
    (see static_transform_publisher in the launch file).

Tunable via ROS parameters:
  robot_id          str    namespace prefix              (default tb3_0)
  fps               int    publish rate per topic        (default 15)
  rgb_size          str    "WxH"                         (default 640x400)
  stereo_size       str    "WxH"                         (default 640x400)
  use_stereo_align  bool   align depth to RGB frame      (default True)
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass

import depthai as dai
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Header


@dataclass
class _Resolution:
    width: int
    height: int


def _parse_size(s: str, default: tuple[int, int]) -> _Resolution:
    try:
        w, h = s.lower().split('x')
        return _Resolution(int(w), int(h))
    except Exception:
        return _Resolution(*default)


def _try_set(obj, name: str, value):
    """Call obj.<name>(value) if the method exists. Returns True on success."""
    fn = getattr(obj, name, None)
    if fn is None:
        return False
    try:
        fn(value)
        return True
    except Exception:
        return False


class OakCameraNode(Node):
    def __init__(self):
        super().__init__('oak_camera_node')

        self.declare_parameter('robot_id',         'tb3_0')
        self.declare_parameter('fps',              15)
        self.declare_parameter('rgb_size',         '640x400')
        self.declare_parameter('stereo_size',      '640x400')
        self.declare_parameter('use_stereo_align', True)

        self._robot_id   = str(self.get_parameter('robot_id').value)
        self._fps        = int(self.get_parameter('fps').value)
        self._rgb_size   = _parse_size(str(self.get_parameter('rgb_size').value),    (640, 400))
        self._dpt_size   = _parse_size(str(self.get_parameter('stereo_size').value), (640, 400))
        self._align_dpt  = bool(self.get_parameter('use_stereo_align').value)

        ns      = self._robot_id
        rgb_ns  = f'/{ns}/camera/rgb'
        dpt_ns  = f'/{ns}/camera/depth'

        self._rgb_frame_id = f'{ns}_oak_rgb_camera_optical_frame'
        self._dpt_frame_id = (
            self._rgb_frame_id if self._align_dpt
            else f'{ns}_oak_right_camera_optical_frame'
        )

        # ── Publishers ───────────────────────────────────────────────────
        self._rgb_pub      = self.create_publisher(Image,      f'{rgb_ns}/image_raw',   10)
        self._rgb_info_pub = self.create_publisher(CameraInfo, f'{rgb_ns}/camera_info', 10)
        self._dpt_pub      = self.create_publisher(Image,      f'{dpt_ns}/image_raw',   10)
        self._dpt_info_pub = self.create_publisher(CameraInfo, f'{dpt_ns}/camera_info', 10)

        # ── depthai pipeline (3.x lifecycle) ─────────────────────────────
        # Pipeline + queues are owned by the worker thread. The worker calls
        # pipeline.start() and pipeline.stop() so we never touch the SDK
        # from two threads.
        self._stop_evt = threading.Event()
        self._worker   = threading.Thread(target=self._pipeline_loop, daemon=True)
        self._worker.start()

        self.get_logger().info(
            f'oak_camera_node ready ({self._robot_id}) — '
            f'rgb={self._rgb_size.width}x{self._rgb_size.height}@{self._fps}fps  '
            f'depth={self._dpt_size.width}x{self._dpt_size.height}  '
            f'aligned_to_rgb={self._align_dpt}  '
            f'depthai={dai.__version__}'
        )

    # ── frame → ROS message conversion ────────────────────────────────────

    def _now_header(self, frame_id: str) -> Header:
        h = Header()
        h.stamp = self.get_clock().now().to_msg()
        h.frame_id = frame_id
        return h

    def _camera_info(self, frame_id: str, K: np.ndarray, w: int, h: int) -> CameraInfo:
        ci = CameraInfo()
        ci.header = self._now_header(frame_id)
        ci.width  = w
        ci.height = h
        ci.distortion_model = 'plumb_bob'
        ci.d = [0.0] * 5
        ci.k = K.flatten().tolist()
        ci.r = [1.0, 0.0, 0.0,
                0.0, 1.0, 0.0,
                0.0, 0.0, 1.0]
        ci.p = [K[0, 0], 0.0,     K[0, 2], 0.0,
                0.0,     K[1, 1], K[1, 2], 0.0,
                0.0,     0.0,     1.0,     0.0]
        return ci

    def _publish_rgb(self, frame, K: np.ndarray) -> None:
        cv = frame.getCvFrame()
        msg = Image()
        msg.header   = self._now_header(self._rgb_frame_id)
        msg.height   = cv.shape[0]
        msg.width    = cv.shape[1]
        msg.encoding = 'bgr8'
        msg.is_bigendian = 0
        msg.step     = cv.shape[1] * 3
        msg.data     = cv.tobytes()
        self._rgb_pub.publish(msg)
        self._rgb_info_pub.publish(
            self._camera_info(self._rgb_frame_id, K,
                               self._rgb_size.width, self._rgb_size.height)
        )

    def _publish_depth(self, frame, K: np.ndarray) -> None:
        d = frame.getCvFrame()  # uint16, depth in mm (depthai default)
        msg = Image()
        msg.header   = self._now_header(self._dpt_frame_id)
        msg.height   = d.shape[0]
        msg.width    = d.shape[1]
        msg.encoding = '16UC1'
        msg.is_bigendian = 0
        msg.step     = d.shape[1] * 2
        msg.data     = d.tobytes()
        self._dpt_pub.publish(msg)
        self._dpt_info_pub.publish(
            self._camera_info(self._dpt_frame_id, K,
                               self._dpt_size.width, self._dpt_size.height)
        )

    # ── main worker thread: build + run pipeline ──────────────────────────

    def _pipeline_loop(self) -> None:
        try:
            with dai.Pipeline() as pipeline:
                # RGB centre camera
                cam_rgb = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
                rgb_out = cam_rgb.requestOutput(
                    (self._rgb_size.width, self._rgb_size.height),
                    dai.ImgFrame.Type.BGR888i,
                    fps=float(self._fps),
                )

                # Mono pair → stereo depth
                mono_l = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
                mono_r = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)
                l_out = mono_l.requestOutput(
                    (self._dpt_size.width, self._dpt_size.height),
                    dai.ImgFrame.Type.NV12, fps=float(self._fps),
                )
                r_out = mono_r.requestOutput(
                    (self._dpt_size.width, self._dpt_size.height),
                    dai.ImgFrame.Type.NV12, fps=float(self._fps),
                )

                stereo = pipeline.create(dai.node.StereoDepth)
                # SDK 3.x preset names changed; try whichever exists.
                for name in ('FAST_DENSITY', 'FAST_ACCURACY', 'DEFAULT', 'HIGH_DENSITY'):
                    preset = getattr(dai.node.StereoDepth.PresetMode, name, None)
                    if preset is not None:
                        try:
                            stereo.setDefaultProfilePreset(preset)
                            break
                        except Exception:
                            continue
                if self._align_dpt:
                    _try_set(stereo, 'setDepthAlign',
                             dai.CameraBoardSocket.CAM_A)
                # depthai 3.x requires explicit depth output size — must
                # be a multiple of 16 in width or stereo crashes the link.
                _try_set(stereo, 'setOutputSize',
                         (self._dpt_size.width, self._dpt_size.height))
                # If the above signature didn't take a tuple, try (w, h)
                if hasattr(stereo, 'setOutputSize'):
                    try:
                        stereo.setOutputSize(self._dpt_size.width,
                                             self._dpt_size.height)
                    except Exception:
                        pass
                l_out.link(stereo.left)
                r_out.link(stereo.right)

                # 3.x: queues are created directly from outputs, no
                # XLinkOut nodes.
                rgb_q   = rgb_out.createOutputQueue(maxSize=4, blocking=False)
                depth_q = stereo.depth.createOutputQueue(maxSize=4, blocking=False)

                pipeline.start()

                # Read calibration AFTER device exists (pipeline.start opens it).
                device = pipeline.getDefaultDevice()
                calib  = device.readCalibration()
                rgb_K  = np.array(calib.getCameraIntrinsics(
                    dai.CameraBoardSocket.CAM_A,
                    self._rgb_size.width, self._rgb_size.height,
                ), dtype=float)
                dpt_K = rgb_K.copy() if self._align_dpt else np.array(
                    calib.getCameraIntrinsics(
                        dai.CameraBoardSocket.CAM_C,
                        self._dpt_size.width, self._dpt_size.height,
                    ), dtype=float)

                self.get_logger().info(
                    f'pipeline running. device={device.getDeviceName()} '
                    f'rgb_K diag={rgb_K[0,0]:.1f},{rgb_K[1,1]:.1f}'
                )

                # Sleep between tryGet polls. Without it the loop pegs a
                # full CPU core hammering tryGet() and starves the depthai
                # USB transport thread (which needs the GIL to deliver
                # frames). Symptom on Pi 4: oak_camera_node says
                # "pipeline running" but never publishes a single frame,
                # at any fps. 1 ms keeps poll latency well under any
                # realistic frame interval (≥33 ms at 30 fps) while
                # dropping CPU from ~100 % to a few percent.
                poll_sleep = 0.001
                while not self._stop_evt.is_set() and pipeline.isRunning():
                    rgb_frame = rgb_q.tryGet()
                    if rgb_frame is not None:
                        try:
                            self._publish_rgb(rgb_frame, rgb_K)
                        except Exception as e:
                            self.get_logger().warning(f'rgb publish error: {e}')
                    dpt_frame = depth_q.tryGet()
                    if dpt_frame is not None:
                        try:
                            self._publish_depth(dpt_frame, dpt_K)
                        except Exception as e:
                            self.get_logger().warning(f'depth publish error: {e}')
                    time.sleep(poll_sleep)
        except Exception as e:
            self.get_logger().error(f'pipeline failed: {e}')

    def destroy_node(self):
        self._stop_evt.set()
        if self._worker is not None:
            self._worker.join(timeout=3.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = OakCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
