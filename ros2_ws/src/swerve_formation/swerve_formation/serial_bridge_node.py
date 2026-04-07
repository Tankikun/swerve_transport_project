import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import time


class SerialBridgeNode(Node):
    """
    Bridges ROS 2 cmd_vel → OpenCR custom swerve firmware over USB-CDC serial.
    Expects firmware to parse: "x_dot y_dot gamma_dot\n"
    """
    def __init__(self):
        super().__init__('serial_bridge_node')

        self.declare_parameter('robot_id', 'robot_1')
        self.declare_parameter('usb_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)

        robot_id  = self.get_parameter('robot_id').value
        usb_port  = self.get_parameter('usb_port').value
        baud_rate = self.get_parameter('baud_rate').value

        # Open serial port to OpenCR
        try:
            self.ser = serial.Serial(usb_port, baud_rate, timeout=0.1)
            self.get_logger().info(f"Serial port {usb_port} opened at {baud_rate} baud")
        except serial.SerialException as e:
            self.get_logger().fatal(f"Cannot open serial port: {e}")
            raise SystemExit

        # Software watchdog: if no cmd received in 1s, send zero
        self.last_cmd_time = time.time()
        self.WATCHDOG_S = 1.0

        self.create_subscription(
            Twist,
            f'/{robot_id}/cmd_vel',
            self.cmd_callback,
            10
        )

        # Watchdog timer at 10 Hz
        self.create_timer(0.1, self.watchdog_check)
        self.get_logger().info(f"Serial bridge ready for {robot_id}")

    def cmd_callback(self, msg: Twist):
        self.last_cmd_time = time.time()
        self._send(msg.linear.x, msg.linear.y, msg.angular.z)

    def watchdog_check(self):
        if time.time() - self.last_cmd_time > self.WATCHDOG_S:
            self._send(0.0, 0.0, 0.0)

    def _send(self, x_dot: float, y_dot: float, gamma_dot: float):
        line = f"{x_dot:.4f} {y_dot:.4f} {gamma_dot:.4f}\n"
        try:
            self.ser.write(line.encode('ascii'))
        except serial.SerialException as e:
            self.get_logger().error(f"Serial write failed: {e}")

    def destroy_node(self):
        self._send(0.0, 0.0, 0.0)   # zero motors on shutdown
        self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SerialBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()