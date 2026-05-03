"""
conveyor_base_node.py
---------------------
Lifecycle serial bridge: /{robot_id}/cmd_vel  →  OpenCR USB-CDC
                         OpenCR POSE line     →  /{robot_id}/odom  +  TF

Serial protocol (115200 baud):
  Send:    "x_dot y_dot gamma_dot\n"       (m/s, m/s, rad/s)
  Receive: "POSE x y theta vx vy wz\n"    (~3 Hz, firmware dead-reckoning)

Lifecycle transitions:
  configure  — opens serial port, creates sub/pub/timers
  activate   — enables command forwarding to hardware
  deactivate — zeroes motors, disables forwarding
  cleanup    — closes serial port, destroys ROS entities
  shutdown   — zeroes motors, closes serial port
"""

import math
import time

import rclpy
import serial
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.lifecycle import Node, State, TransitionCallbackReturn
from std_srvs.srv import Trigger
import tf2_ros

# ── FK constants (must match turtlebot3_conveyor.h) ──────────────────────────
_WHEEL_RADIUS    = 0.033
_HALF_L          = 0.15
_HALF_W          = 0.15
_THETA_AXIS      = [3*math.pi/4, math.pi/4, -3*math.pi/4, -math.pi/4]
_MOD_PX          = [ _HALF_L,  _HALF_L, -_HALF_L, -_HALF_L]
_MOD_PY          = [ _HALF_W, -_HALF_W,  _HALF_W, -_HALF_W]
_DXL_POS_TO_RAD  = (2.0 * math.pi) / 4096.0
_DXL_VEL_TO_RADS = 0.229 * (2.0 * math.pi) / 60.0
_STEER_CENTER    = 2048


# NOTE — Legacy firmware-rollback safety net.
# This function and `ConveyorBaseNode._handle_odom_legacy` below are
# only exercised when the OpenCR firmware sends pre-2024 "ODOM j:... w:..."
# lines. The current firmware emits "POSE x y theta vx vy wz" and is
# handled by `_handle_pose`, which makes this code dead in normal
# operation. Kept so we can roll the firmware back without losing odometry.
# TODO: if firmware rollback is no longer a concern, remove this function
# AND `ConveyorBaseNode._handle_odom_legacy` below AND its near-duplicate
# at `turtlebot3_conveyor_bridge/serial_bridge_node.py::fk_body_velocity`
# (line ~37) and the matching ODOM parser in the same file (line ~160) —
# the two should be deleted together to keep the bridge and base-node FK
# logic in lock-step.
def _fk_body_velocity(joint_rads, wheel_rads):
    """FK from old ODOM-format firmware: joint angles + wheel speeds → body vel."""
    joint_ticks = [int(v / _DXL_POS_TO_RAD) + _STEER_CENTER for v in joint_rads]
    wheel_ticks = [int(v / _DXL_VEL_TO_RADS) for v in wheel_rads]
    ik_j = [joint_ticks[2], joint_ticks[3], joint_ticks[0], joint_ticks[1]]
    ik_w = [wheel_ticks[2], wheel_ticks[3], wheel_ticks[0], wheel_ticks[1]]
    delta = [(t - _STEER_CENTER) * _DXL_POS_TO_RAD for t in ik_j]

    def s32(v):
        v = int(v) & 0xFFFFFFFF
        return v - 0x100000000 if v >= 0x80000000 else v

    omega = [s32(w) * _DXL_VEL_TO_RADS for w in ik_w]
    svx = svy = sn = sd = 0.0
    for i in range(4):
        d   = _THETA_AXIS[i] + delta[i]
        bx  = omega[i] * _WHEEL_RADIUS * math.cos(d)
        by  = omega[i] * _WHEEL_RADIUS * math.sin(d)
        svx += bx;  svy += by
        sn  += -_MOD_PY[i] * bx + _MOD_PX[i] * by
        sd  += _MOD_PX[i]**2 + _MOD_PY[i]**2
    return svx / 4.0, svy / 4.0, (sn / sd if sd > 1e-9 else 0.0)


class ConveyorBaseNode(Node):

    WATCHDOG_S  = 5.0   # hold last command for 5 s before zeroing
    BOOT_WAIT_S = 5.0   # wait for OpenCR homing after serial open

    def __init__(self):
        super().__init__('conveyor_base_node')
        self.declare_parameter('robot_id', 'tb3_0')
        self.declare_parameter('usb_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)

        self._ser: serial.Serial | None = None
        self._sub            = None
        self._odom_pub       = None
        self._tf_broadcaster = None
        self._read_timer     = None
        self._watchdog_timer = None
        self._active = False
        self._last_cmd_t = 0.0
        self._reset_srv = None
        # Per-robot frame IDs for the published Odometry msg + odom→base_link
        # TF. Set in on_configure once we know the robot_id parameter. Using
        # robot-prefixed frames lets multiple robots coexist in the same TF
        # tree without colliding (otherwise both would publish to plain
        # `odom`/`base_link` and rtabmap couldn't tell them apart).
        self._odom_frame_id = 'odom'
        self._base_frame_id = 'base_link'
        self._line_buf = ''
        self._odom_x = self._odom_y = self._odom_theta = 0.0

    # ------------------------------------------------------------------
    # Lifecycle callbacks
    # ------------------------------------------------------------------

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        robot_id  = self.get_parameter('robot_id').value
        usb_port  = self.get_parameter('usb_port').value
        baud_rate = self.get_parameter('baud_rate').value
        # Stamp the per-robot TF / Odometry frame IDs.
        self._odom_frame_id = f'{robot_id}_odom'
        self._base_frame_id = f'{robot_id}_base_link'

        try:
            self._ser = serial.Serial(
                usb_port, baud_rate,
                timeout=1.0, write_timeout=1.0,
                dsrdtr=False, rtscts=False,
            )
            self.get_logger().info(f'Serial {usb_port} @ {baud_rate} opened.')
            self.get_logger().info(f'Waiting {self.BOOT_WAIT_S}s for OpenCR boot + homing...')
            time.sleep(self.BOOT_WAIT_S)
            self._ser.reset_input_buffer()   # discard POSE lines buffered during boot
            self.get_logger().info('Serial ready.')
        except serial.SerialException as e:
            self.get_logger().error(f'Cannot open serial port: {e}')
            return TransitionCallbackReturn.FAILURE

        self._sub = self.create_subscription(
            Twist, f'/{robot_id}/cmd_vel', self._cmd_cb, 10
        )
        self._odom_pub       = self.create_publisher(Odometry, f'/{robot_id}/odom', 10)
        self._tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        # Read serial at 50 Hz (was 10 Hz). Firmware now emits POSE at
        # 33 Hz (ODOM_DIV=1, see turtlebot3_conveyor.ino), and EKF +
        # navigation + RTAB-Map all benefit from fresher pose timestamps:
        # the Odometry msg's stamp is set to "when Pi processed the line",
        # so faster polling → smaller stamp lag → cleaner downstream sync.
        # 50 Hz keeps serial buffer drained within 20 ms even at peak rate.
        self._read_timer     = self.create_timer(0.02, self._read_serial_cb)
        self._watchdog_timer = self.create_timer(0.2, self._watchdog_cb)

        self._last_cmd_t = time.time()

        self._reset_srv = self.create_service(
            Trigger, f'/{robot_id}/reset_odom', self._reset_odom_cb
        )

        self._odom_t     = time.time()
        self.get_logger().info(f'ConveyorBaseNode configured for /{robot_id}')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self._active = True
        self.get_logger().info('ConveyorBaseNode activated — forwarding commands to OpenCR')
        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self._active = False
        self._send(0.0, 0.0, 0.0)
        self.get_logger().info('Deactivated — motors zeroed.')
        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self._send(0.0, 0.0, 0.0)
        self._close_serial()
        for attr, destroy in [('_sub',            self.destroy_subscription),
                               ('_odom_pub',       self.destroy_publisher),
                               ('_read_timer',     self.destroy_timer),
                               ('_watchdog_timer', self.destroy_timer)]:
            obj = getattr(self, attr)
            if obj:
                try:    destroy(obj)
                except Exception: pass
                setattr(self, attr, None)
        self.get_logger().info('ConveyorBaseNode cleaned up.')
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self._send(0.0, 0.0, 0.0)
        self._close_serial()
        self.get_logger().info('ConveyorBaseNode shutdown.')
        return TransitionCallbackReturn.SUCCESS

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
        # Do NOT publish a synthetic (0, 0, 0) Odometry here. With multi-robot
        # operation, every robot publishing zero at the same time would make
        # downstream consumers (laplacian, navigation) momentarily think every
        # robot is at the same point. The next real `POSE …` line from
        # firmware (handled in `_handle_pose`) will propagate the reset
        # naturally; the legacy `ODOM_RESET` line, if firmware sends one,
        # zeros our local integration state in `_read_serial_cb`.
        response.success = True
        response.message = 'Odometry reset command sent to firmware'
        return response

    # ── Serial reading ────────────────────────────────────────────────────────

    def _read_serial_cb(self):
        if self._ser is None or not self._ser.is_open:
            return
        try:
            waiting = self._ser.in_waiting
            if not waiting:
                return
            chunk = self._ser.read(min(waiting, 512)).decode('ascii', errors='replace')
            self._line_buf += chunk
            if len(self._line_buf) > 4096:   # safety cap
                self._line_buf = ''
                return
            while '\n' in self._line_buf:
                line, self._line_buf = self._line_buf.split('\n', 1)
                line = line.strip()
                if not line:
                    continue
                if line.startswith('POSE '):
                    self._handle_pose(line)
                elif line.startswith('ODOM '):
                    self._handle_odom_legacy(line)
                elif line.startswith('ODOM_RESET'):
                    self._odom_x = self._odom_y = self._odom_theta = 0.0
                elif line.startswith('OK') or line.startswith('[DBG]'):
                    self.get_logger().debug('OpenCR: ' + line)
                else:
                    self.get_logger().info('OpenCR: ' + line)
        except serial.SerialException as e:
            self.get_logger().error(f'Serial read error: {e}')

    def _handle_pose(self, line: str):
        """Current firmware: 'POSE x y theta vx vy wz' — pre-integrated on OpenCR."""
        parts = line.split()
        if len(parts) != 7:
            return
        try:
            x, y, theta, vx, vy, wz = (float(p) for p in parts[1:])
        except ValueError:
            return
        self._publish_odom(x, y, theta, vx, vy, wz)

    # NOTE — Legacy firmware-rollback safety net.
    # Only invoked when the OpenCR firmware sends pre-2024 "ODOM j:... w:..."
    # lines. Current firmware uses "POSE …" handled by `_handle_pose`, so
    # this is dead in normal operation. See the TODO above
    # `_fk_body_velocity` at module top — the two functions and the
    # near-duplicate in `turtlebot3_conveyor_bridge/serial_bridge_node.py`
    # should be removed together if firmware rollback is no longer needed.
    def _handle_odom_legacy(self, line: str):
        """Old firmware fallback: 'ODOM j:... w:...' — integrate on RPi side."""
        try:
            j_part = line.split('j:')[1].split(' ')[0]
            w_part = line.split('w:')[1]
            jv = [float(v) for v in j_part.split(',')]
            wv = [float(v) for v in w_part.split(',')]
            if len(jv) != 4 or len(wv) != 4:
                return
        except (IndexError, ValueError):
            return
        vx, vy, wz = _fk_body_velocity(jv, wv)
        now = time.time()
        dt  = now - self._odom_t
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

    # ── Odometry publisher ────────────────────────────────────────────────────

    def _publish_odom(self, x: float, y: float, theta: float,
                      vx: float, vy: float, wz: float):
        if self._odom_pub is None:
            return
        try:
            now  = self.get_clock().now().to_msg()
            half = theta / 2.0
            qz   = math.sin(half)
            qw   = math.cos(half)

            # Standard nav_msgs/Odometry
            odom = Odometry()
            odom.header.stamp       = now
            odom.header.frame_id    = self._odom_frame_id
            odom.child_frame_id     = self._base_frame_id
            odom.pose.pose.position.x    = x
            odom.pose.pose.position.y    = y
            odom.pose.pose.position.z    = 0.0
            odom.pose.pose.orientation.x = 0.0
            odom.pose.pose.orientation.y = 0.0
            odom.pose.pose.orientation.z = qz
            odom.pose.pose.orientation.w = qw
            odom.pose.covariance[0]  = 0.01   # σ²_xx
            odom.pose.covariance[7]  = 0.01   # σ²_yy
            odom.pose.covariance[14] = 1e9    # σ²_zz  (unused — 2-D robot)
            odom.pose.covariance[21] = 1e9    # σ²_roll  (unused)
            odom.pose.covariance[28] = 1e9    # σ²_pitch (unused)
            odom.pose.covariance[35] = 0.05   # σ²_yaw
            odom.twist.twist.linear.x  = vx
            odom.twist.twist.linear.y  = vy
            odom.twist.twist.linear.z  = 0.0
            odom.twist.twist.angular.x = 0.0
            odom.twist.twist.angular.y = 0.0
            odom.twist.twist.angular.z = wz
            odom.twist.covariance[0]  = 0.01
            odom.twist.covariance[7]  = 0.01
            odom.twist.covariance[14] = 1e9
            odom.twist.covariance[21] = 1e9
            odom.twist.covariance[28] = 1e9
            odom.twist.covariance[35] = 0.05
            self._odom_pub.publish(odom)

            # TF: odom → base_link
            tf = TransformStamped()
            tf.header.stamp       = now
            tf.header.frame_id    = self._odom_frame_id
            tf.child_frame_id     = self._base_frame_id
            tf.transform.translation.x = x
            tf.transform.translation.y = y
            tf.transform.translation.z = 0.0
            tf.transform.rotation.x    = 0.0
            tf.transform.rotation.y    = 0.0
            tf.transform.rotation.z    = qz
            tf.transform.rotation.w    = qw
            self._tf_broadcaster.sendTransform(tf)

        except Exception as e:
            self.get_logger().error(f'publish_odom failed: {e}')

    # ── cmd_vel / watchdog ────────────────────────────────────────────────────

    def _cmd_cb(self, msg: Twist):
        self._last_cmd_t = time.time()
        if self._active:
            self._send(msg.linear.x, msg.linear.y, msg.angular.z)

    def _watchdog_cb(self):
        if self._active and (time.time() - self._last_cmd_t) > self.WATCHDOG_S:
            self.get_logger().warn(
                f'No cmd_vel for {self.WATCHDOG_S}s — sending STOP.',
                throttle_duration_sec=2.0,
            )
            self._send(0.0, 0.0, 0.0)

    def _send(self, x: float, y: float, gz: float):
        if self._ser is None or not self._ser.is_open:
            return
        try:
            self._ser.write(f'{x:.4f} {y:.4f} {gz:.4f}\n'.encode('ascii'))
        except serial.SerialException as e:
            self.get_logger().error(f'Serial write failed: {e}')

    def _close_serial(self):
        if self._ser and self._ser.is_open:
            try:
                self._ser.close()
            except Exception:
                pass
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
