import math
import threading
import time

import rclpy
import serial
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.lifecycle import Node, State, TransitionCallbackReturn
from std_srvs.srv import Trigger


class ConveyorBaseNode(Node):
    """
    Lifecycle serial bridge: /{robot_id}/cmd_vel → OpenCR USB-CDC.
    Firmware expects "x_dot y_dot gamma_dot\n" at 115200 baud.
    Firmware sends "POSE x y theta vx vy wz\n" lines; published as /{robot_id}/odom.

    Lifecycle transitions:
      configure  — opens serial port, creates subscription + watchdog timer + odom publisher
      activate   — enables command forwarding, starts serial reader thread
      deactivate — zeroes motors, stops reader thread
      cleanup    — closes serial port, destroys ROS entities
      shutdown   — zeroes motors, closes serial port
    """

    WATCHDOG_S = 1.0

    def __init__(self):
        super().__init__('conveyor_base_node')
        self.declare_parameter('robot_id', 'tb3_0')
        self.declare_parameter('usb_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)

        self._ser: serial.Serial | None = None
        self._sub = None
        self._watchdog_timer = None
        self._active = False
        self._last_cmd_t = 0.0
        self._odom_pub = None
        self._reset_srv = None
        self._stop_event: threading.Event | None = None
        self._read_thread: threading.Thread | None = None

    # ------------------------------------------------------------------
    # Lifecycle callbacks
    # ------------------------------------------------------------------

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        robot_id = self.get_parameter('robot_id').value
        usb_port = self.get_parameter('usb_port').value
        baud_rate = self.get_parameter('baud_rate').value

        try:
            self._ser = serial.Serial(usb_port, baud_rate, timeout=0.1)
            self.get_logger().info(f'Serial {usb_port} @ {baud_rate} baud opened')
        except serial.SerialException as e:
            self.get_logger().error(f'Cannot open serial port: {e}')
            return TransitionCallbackReturn.FAILURE

        self._sub = self.create_subscription(
            Twist, f'/{robot_id}/cmd_vel', self._cmd_cb, 10
        )
        self._watchdog_timer = self.create_timer(0.1, self._watchdog_cb)
        self._last_cmd_t = time.time()

        self._odom_pub = self.create_publisher(Odometry, f'/{robot_id}/odom', 10)
        self._reset_srv = self.create_service(
            Trigger, f'/{robot_id}/reset_odom', self._reset_odom_cb
        )

        self.get_logger().info(f'ConveyorBaseNode configured for {robot_id}')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self._active = True
        self._stop_event = threading.Event()
        self._read_thread = threading.Thread(target=self._serial_reader, daemon=True)
        self._read_thread.start()
        self.get_logger().info('ConveyorBaseNode activated — forwarding commands to OpenCR')
        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self._active = False
        if self._stop_event:
            self._stop_event.set()
        if self._read_thread:
            self._read_thread.join(timeout=2.0)
            self._read_thread = None
        self._stop_event = None
        self._send(0.0, 0.0, 0.0)
        self.get_logger().info('ConveyorBaseNode deactivated — motors zeroed')
        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        if self._stop_event:
            self._stop_event.set()
        if self._read_thread:
            self._read_thread.join(timeout=2.0)
            self._read_thread = None
        self._stop_event = None
        self._send(0.0, 0.0, 0.0)
        self._close_serial()
        if self._sub:
            self.destroy_subscription(self._sub)
            self._sub = None
        if self._watchdog_timer:
            self.destroy_timer(self._watchdog_timer)
            self._watchdog_timer = None
        self.get_logger().info('ConveyorBaseNode cleaned up')
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self._send(0.0, 0.0, 0.0)
        self._close_serial()
        self.get_logger().info('ConveyorBaseNode shutdown')
        return TransitionCallbackReturn.SUCCESS

    # ------------------------------------------------------------------
    # Serial reader thread
    # ------------------------------------------------------------------

    def _serial_reader(self):
        while not self._stop_event.is_set():
            try:
                raw = self._ser.readline()
            except serial.SerialException as e:
                self.get_logger().warn(f'Serial read error: {e}')
                continue
            if not raw:
                continue
            try:
                line = raw.decode('ascii', errors='ignore').strip()
            except Exception:
                continue
            if not line.startswith('POSE'):
                continue
            parts = line.split()
            if len(parts) != 7:
                continue
            try:
                x, y, theta, vx, vy, wz = (float(p) for p in parts[1:])
            except ValueError:
                continue
            self._publish_odom(x, y, theta, vx, vy, wz)

    def _publish_odom(self, x: float, y: float, theta: float,
                      vx: float, vy: float, wz: float):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.z = math.sin(theta / 2.0)
        msg.pose.pose.orientation.w = math.cos(theta / 2.0)
        msg.twist.twist.linear.x = vx
        msg.twist.twist.linear.y = vy
        msg.twist.twist.angular.z = wz
        self._odom_pub.publish(msg)

    # ------------------------------------------------------------------
    # Reset odom service
    # ------------------------------------------------------------------

    def _reset_odom_cb(self, request, response):
        if not self._active or self._ser is None or not self._ser.is_open:
            response.success = False
            response.message = 'Node not active'
            return response
        try:
            self._ser.write(b'R\n')
        except serial.SerialException as e:
            self.get_logger().warn(f'Reset odom serial write failed: {e}')
            response.success = False
            response.message = f'Serial error: {e}'
            return response
        self._publish_odom(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        response.success = True
        response.message = 'Odometry reset'
        return response

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _cmd_cb(self, msg: Twist):
        self._last_cmd_t = time.time()
        if self._active:
            self._send(msg.linear.x, msg.linear.y, msg.angular.z)

    def _watchdog_cb(self):
        if self._active and time.time() - self._last_cmd_t > self.WATCHDOG_S:
            self._send(0.0, 0.0, 0.0)

    def _send(self, x_dot: float, y_dot: float, gamma_dot: float):
        if self._ser is None or not self._ser.is_open:
            return
        line = f'{x_dot:.4f} {y_dot:.4f} {gamma_dot:.4f}\n'
        try:
            self._ser.write(line.encode('ascii'))
        except serial.SerialException as e:
            self.get_logger().error(f'Serial write failed: {e}')

    def _close_serial(self):
        if self._ser and self._ser.is_open:
            self._ser.close()
        self._ser = None


def main(args=None):
    rclpy.init(args=args)
    node = ConveyorBaseNode()
    try:
        if node.trigger_configure() != TransitionCallbackReturn.SUCCESS:
            node.get_logger().fatal('configure failed — exiting')
            return
        node.trigger_activate()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.trigger_deactivate()
        node.trigger_cleanup()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
