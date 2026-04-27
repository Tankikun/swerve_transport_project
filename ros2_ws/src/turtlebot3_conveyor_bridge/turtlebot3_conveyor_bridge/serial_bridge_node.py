#!/usr/bin/env python3
"""
serial_bridge_node.py
---------------------
Sends "x y wz\n" to OpenCR on each /cmd_vel.
Reads "POSE x y theta vx vy wz\n" from OpenCR (~3 Hz) and publishes:
  - nav_msgs/Odometry  on  /odom          (standard ROS2 odometry message)
  - tf2 transform      on  /tf            (odom → base_link)

Both are required for full ROS2 nav stack / RViz2 compatibility.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
import tf2_ros
import math
import serial
import time

WATCHDOG_SEC  = 5.0   # match firmware CMD_TIMEOUT_MS: hold last command for 5 s before stopping
WHEEL_RADIUS  = 0.033
HALF_L        = 0.15
HALF_W        = 0.15

# Module home-axis angles [FL, FR, RL, RR] in radians
THETA_AXIS = [3*math.pi/4, math.pi/4, -3*math.pi/4, -math.pi/4]
MOD_PX     = [ HALF_L,  HALF_L, -HALF_L, -HALF_L]
MOD_PY     = [ HALF_W, -HALF_W,  HALF_W, -HALF_W]

DXL_POS_TO_RAD  = (2.0 * math.pi) / 4096.0
DXL_VEL_TO_RADS = 0.229 * (2.0 * math.pi) / 60.0
STEER_CENTER    = 2048


def fk_body_velocity(joint_ticks, wheel_ticks):
    """FK from raw ticks — used only for old ODOM-format firmware."""
    ik_j = [joint_ticks[2], joint_ticks[3], joint_ticks[0], joint_ticks[1]]
    ik_w = [wheel_ticks[2], wheel_ticks[3], wheel_ticks[0], wheel_ticks[1]]
    delta = [(t - STEER_CENTER) * DXL_POS_TO_RAD for t in ik_j]

    def to_signed32(v):
        v = int(v) & 0xFFFFFFFF
        return v - 0x100000000 if v >= 0x80000000 else v

    omega = [to_signed32(w) * DXL_VEL_TO_RADS for w in ik_w]
    sum_vx = sum_vy = sum_n = sum_d = 0.0
    for i in range(4):
        d  = THETA_AXIS[i] + delta[i]
        bx = omega[i] * WHEEL_RADIUS * math.cos(d)
        by = omega[i] * WHEEL_RADIUS * math.sin(d)
        sum_vx += bx
        sum_vy += by
        sum_n  += -MOD_PY[i] * bx + MOD_PX[i] * by
        sum_d  += MOD_PX[i]**2 + MOD_PY[i]**2
    vx = sum_vx / 4.0
    vy = sum_vy / 4.0
    wz = sum_n / sum_d if sum_d > 1e-9 else 0.0
    return vx, vy, wz


class ConveyorSerialBridge(Node):

    def __init__(self):
        super().__init__('conveyor_serial_bridge')

        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baudrate').value

        try:
            self.ser = serial.Serial(
                port, baud,
                timeout=1.0,
                write_timeout=1.0,
                dsrdtr=False,
                rtscts=False,
            )
            self.get_logger().info('Opened %s @ %d baud.' % (port, baud))
            self.get_logger().info('Waiting 5 s for OpenCR boot + homing...')
            time.sleep(5.0)
            # Flush any POSE lines that buffered during the wait.
            # Without this, all ~167 buffered lines would be processed at once
            # causing a CPU spike right when the user tries to drive.
            self.ser.reset_input_buffer()
            self.get_logger().info('Serial ready — waiting for /cmd_vel')
        except serial.SerialException as exc:
            self.get_logger().fatal('Cannot open serial port: %s' % exc)
            raise

        self._last_rx_t = self.get_clock().now()
        self._line_buf  = ''

        # Odometry state — used when parsing old ODOM lines (backup firmware)
        self._odom_x     = 0.0
        self._odom_y     = 0.0
        self._odom_theta = 0.0
        self._odom_t     = self.get_clock().now()

        # Publisher: standard nav_msgs/Odometry
        self._odom_pub = self.create_publisher(Odometry, 'odom', 10)
        # TF broadcaster: odom → base_link (required by RViz2 / nav2)
        self._tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.create_subscription(Twist, 'cmd_vel', self._cmd_vel_cb, 10)
        self.create_timer(0.2,  self._watchdog_cb)
        # Serial read at 100 ms — same cadence as original backup bridge.
        # This ensures CPU load matches what worked before.
        self.create_timer(0.1,  self._read_serial_cb)

        self.get_logger().info('conveyor_serial_bridge ready. Publishing /odom.')

    # ── Serial reading ────────────────────────────────────────────────────────

    def _read_serial_cb(self):
        try:
            waiting = self.ser.in_waiting
            if waiting <= 0:
                return
            chunk = self.ser.read(min(waiting, 512)).decode('ascii', errors='replace')
            self._line_buf += chunk
            # Safety cap: if buffer grows huge (e.g. firmware flood), reset
            if len(self._line_buf) > 4096:
                self._line_buf = ''
                self.get_logger().warn('Serial line buffer overflow — cleared.')
                return
            while '\n' in self._line_buf:
                line, self._line_buf = self._line_buf.split('\n', 1)
                line = line.strip()
                if not line:
                    continue
                if line.startswith('POSE '):
                    self._handle_pose_line(line)
                elif line.startswith('ODOM '):
                    self._handle_odom_line(line)
                elif line.startswith('ODOM_RESET'):
                    self._odom_x = self._odom_y = self._odom_theta = 0.0
                    self.get_logger().info('Odometry reset.')
                elif line.startswith('OK') or line.startswith('[DBG]'):
                    self.get_logger().debug('OpenCR: ' + line)
                else:
                    self.get_logger().info('OpenCR: ' + line)
        except serial.SerialException as exc:
            self.get_logger().error('Serial read error: %s' % exc)

    def _handle_pose_line(self, line: str):
        """New firmware: 'POSE x y theta vx vy wz' — already integrated."""
        parts = line.split()
        if len(parts) != 7:
            return
        try:
            x, y, theta, vx, vy, wz = (float(p) for p in parts[1:])
        except ValueError:
            return
        self._publish_odom(x, y, theta, vx, vy, wz)

    def _handle_odom_line(self, line: str):
        """Old firmware: 'ODOM j:a,b,c,d w:e,f,g,h' — FK + integrate on RPi."""
        try:
            j_part = line.split('j:')[1].split(' ')[0]
            w_part = line.split('w:')[1]
            joint_rads = [float(v) for v in j_part.split(',')]
            wheel_rads = [float(v) for v in w_part.split(',')]
            if len(joint_rads) != 4 or len(wheel_rads) != 4:
                return
        except (IndexError, ValueError):
            return

        joint_ticks = [int(v / DXL_POS_TO_RAD) + STEER_CENTER for v in joint_rads]
        wheel_ticks = [int(v / DXL_VEL_TO_RADS) for v in wheel_rads]
        vx, vy, wz  = fk_body_velocity(joint_ticks, wheel_ticks)

        now = self.get_clock().now()
        dt  = (now - self._odom_t).nanoseconds * 1e-9
        self._odom_t = now
        if dt > 0.5:
            dt = 0.03
        c = math.cos(self._odom_theta)
        s = math.sin(self._odom_theta)
        self._odom_x     += (c * vx - s * vy) * dt
        self._odom_y     += (s * vx + c * vy) * dt
        self._odom_theta += wz * dt
        while self._odom_theta >  math.pi: self._odom_theta -= 2 * math.pi
        while self._odom_theta < -math.pi: self._odom_theta += 2 * math.pi
        self._publish_odom(self._odom_x, self._odom_y, self._odom_theta, vx, vy, wz)

    def _publish_odom(self, x, y, theta, vx, vy, wz):
        """
        Publish a fully-populated nav_msgs/Odometry and broadcast odom→base_link TF.

        Odometry covariance matrices are 6×6, row-major (36 elements).
        Indices for a 2D ground robot:
          [0]  = xx  (x position variance)
          [7]  = yy  (y position variance)
          [35] = θθ  (yaw variance)
          [21] = vx·vx, [28] = vy·vy, [35] already used for yaw,
                 index [35] in twist covariance = wz·wz
        """
        try:
            now = self.get_clock().now().to_msg()

            # Quaternion from yaw (rotation about Z only — 2D robot)
            half = theta / 2.0
            qx, qy = 0.0, 0.0
            qz = math.sin(half)
            qw = math.cos(half)

            # ── nav_msgs/Odometry ─────────────────────────────────────────────
            odom = Odometry()

            # Header
            odom.header.stamp    = now
            odom.header.frame_id = 'odom'          # fixed world frame
            odom.child_frame_id  = 'base_link'     # robot body frame

            # Pose (position + orientation in odom frame)
            odom.pose.pose.position.x    = x
            odom.pose.pose.position.y    = y
            odom.pose.pose.position.z    = 0.0
            odom.pose.pose.orientation.x = qx
            odom.pose.pose.orientation.y = qy
            odom.pose.pose.orientation.z = qz
            odom.pose.pose.orientation.w = qw
            # Pose covariance (diagonal, 2D robot — x, y, yaw only)
            odom.pose.covariance[0]  = 0.01   # σ²_xx
            odom.pose.covariance[7]  = 0.01   # σ²_yy
            odom.pose.covariance[14] = 1e9    # σ²_zz  (unused, set large)
            odom.pose.covariance[21] = 1e9    # σ²_roll (unused)
            odom.pose.covariance[28] = 1e9    # σ²_pitch (unused)
            odom.pose.covariance[35] = 0.05   # σ²_yaw

            # Twist (linear + angular velocity in base_link frame)
            odom.twist.twist.linear.x  = vx
            odom.twist.twist.linear.y  = vy
            odom.twist.twist.linear.z  = 0.0
            odom.twist.twist.angular.x = 0.0
            odom.twist.twist.angular.y = 0.0
            odom.twist.twist.angular.z = wz
            # Twist covariance
            odom.twist.covariance[0]  = 0.01   # σ²_vx
            odom.twist.covariance[7]  = 0.01   # σ²_vy
            odom.twist.covariance[14] = 1e9    # σ²_vz  (unused)
            odom.twist.covariance[21] = 1e9    # σ²_wx  (unused)
            odom.twist.covariance[28] = 1e9    # σ²_wy  (unused)
            odom.twist.covariance[35] = 0.05   # σ²_wz

            self._odom_pub.publish(odom)

            # ── TF: odom → base_link ─────────────────────────────────────────
            tf = TransformStamped()
            tf.header.stamp    = now
            tf.header.frame_id = 'odom'
            tf.child_frame_id  = 'base_link'
            tf.transform.translation.x = x
            tf.transform.translation.y = y
            tf.transform.translation.z = 0.0
            tf.transform.rotation.x    = qx
            tf.transform.rotation.y    = qy
            tf.transform.rotation.z    = qz
            tf.transform.rotation.w    = qw
            self._tf_broadcaster.sendTransform(tf)

        except Exception as exc:
            self.get_logger().error('publish_odom failed: %s' % exc)

    # ── cmd_vel / watchdog ────────────────────────────────────────────────────

    def _cmd_vel_cb(self, msg: Twist):
        self._last_rx_t = self.get_clock().now()
        self._send('%.3f %.3f %.3f\n' % (msg.linear.x, msg.linear.y, msg.angular.z))

    def _watchdog_cb(self):
        elapsed = (self.get_clock().now() - self._last_rx_t).nanoseconds * 1e-9
        if elapsed > WATCHDOG_SEC:
            self.get_logger().warn(
                'No cmd_vel for %.1f s — sending STOP.' % elapsed,
                throttle_duration_sec=2.0
            )
            self._send('0.000 0.000 0.000\n')

    def _send(self, cmd: str):
        try:
            self.ser.write(cmd.encode('ascii'))
            self.get_logger().info('-> OpenCR: ' + cmd.strip(),
                                   throttle_duration_sec=0.5)
        except serial.SerialException as exc:
            self.get_logger().error('Serial write failed: %s' % exc)

    def destroy_node(self):
        try:
            self.ser.write(b'0.000 0.000 0.000\n')
            self.ser.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ConveyorSerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
