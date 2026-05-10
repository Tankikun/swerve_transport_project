import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu
import numpy as np


# How long without an IMU sample before falling back to the wheel-derived
# yaw rate. The IMU runs at ~11 Hz from firmware, so 0.5 s is ~5 missed
# samples — anything longer means the IMU stream is genuinely down.
_IMU_STALE_S = 0.5

# Process-noise variance on theta when the gyro is active. Gyro per-axis
# noise is ~1e-4 rad²/s² (see _IMU_GYRO_VAR in conveyor_base_node.py); per
# integration step at ~33 Hz that's about 9e-8 rad². We round up to 5e-5
# to leave headroom for gyro bias drift over the timescales between SLAM
# corrections (~5 Hz). Compare to the wheel-derived value of 5e-3 — a
# 100× reduction in theta uncertainty growth.
_Q_THETA_GYRO = 5.0e-5
_Q_THETA_WHEEL = 5.0e-3


class EKFNode(Node):
    """
    Fuses raw wheel odometry (prediction) with SLAM pose (correction).
    State: [x, y, theta]. Only this node reads raw /{robot_id}/odom.
    Publishes authoritative pose on /{robot_id}/ekf/odom.

    The yaw rate used in the prediction step prefers the IMU gyro Z
    (slip-immune) over the wheel-derived omega whenever a fresh IMU sample
    is available. Wheel-derived omega is corrupted by mid-turn slip when
    the four steering modules are not yet aligned — that drift was the
    dominant cause of bad PnP priors at RTAB-Map loop-closure verification.
    """

    def __init__(self):
        super().__init__('ekf_node')
        self.declare_parameter('robot_id', 'tb3_0')
        # Sign of the gyro Z reading after firmware unit conversion. Set
        # to -1.0 via launch parameter if a bench yaw test (rotate the
        # chassis CCW; published ekf yaw should increase) shows the
        # opposite sign — the MPU-9250's mounted Z direction depends on
        # the OpenCR's physical orientation in the chassis.
        self.declare_parameter('gyro_z_sign', 1.0)

        robot_id = self.get_parameter('robot_id').value
        self._gyro_z_sign = float(self.get_parameter('gyro_z_sign').value)
        # Per-robot TF frame ids — must match what conveyor_base_node and the
        # static_transform_publisher in oak_camera.launch.py use, otherwise
        # rtabmap can't connect odom → base_link → camera_optical.
        self._odom_frame_id = f'{robot_id}_odom'
        self._base_frame_id = f'{robot_id}_base_link'

        self._mu = np.zeros(3)           # [x, y, theta]
        self._sigma = np.eye(3) * 0.1
        # Translation noise stays as before; theta entry is set per-step
        # depending on whether the gyro is fresh — see _odom_cb.
        self._Q = np.diag([0.01, 0.01, _Q_THETA_WHEEL])
        self._R = np.diag([0.05, 0.05, 0.02])    # SLAM observation noise
        self._last_t: float | None = None

        # Cached gyro Z rate from the most recent IMU message and the
        # wall-clock time it arrived. None until the first IMU is seen.
        self._gyro_z: float | None = None
        self._gyro_t: float | None = None
        self._imu_ever_seen = False

        # Only this node reads raw /odom
        self.create_subscription(Odometry, f'/{robot_id}/odom', self._odom_cb, 10)
        self.create_subscription(PoseStamped, f'/{robot_id}/slam/pose', self._slam_cb, 10)
        self.create_subscription(Imu, f'/{robot_id}/imu', self._imu_cb, 10)
        self._pub = self.create_publisher(Odometry, f'/{robot_id}/ekf/odom', 10)
        self.get_logger().info(
            f'EKF node ready for {robot_id} (gyro_z_sign={self._gyro_z_sign:+.1f})'
        )

    # ------------------------------------------------------------------
    # IMU cache: store latest gyro Z + receive time. The actual fusion
    # happens in _odom_cb so the prediction stays driven by the (faster)
    # odom stream and we don't need to interleave two prediction sources.
    # ------------------------------------------------------------------
    def _imu_cb(self, msg: Imu):
        self._gyro_z = self._gyro_z_sign * float(msg.angular_velocity.z)
        self._gyro_t = self.get_clock().now().nanoseconds * 1e-9
        if not self._imu_ever_seen:
            self._imu_ever_seen = True
            self.get_logger().info(
                'First IMU sample received — gyro fusion active.'
            )

    # ------------------------------------------------------------------
    # Prediction step: integrate holonomic wheel odometry
    # ------------------------------------------------------------------
    def _odom_cb(self, msg: Odometry):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self._last_t is None:
            self._last_t = now
            return
        dt = now - self._last_t
        self._last_t = now

        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y

        # Pick the yaw rate source. Prefer gyro when fresh; fall back to
        # wheel-derived omega otherwise (e.g. before the first IMU sample
        # arrives, or if the firmware IMU stream stalls). The Q[2,2]
        # entry tracks the source so the covariance reflects which sensor
        # the prediction trusted.
        if (self._gyro_z is not None
                and self._gyro_t is not None
                and (now - self._gyro_t) < _IMU_STALE_S):
            omega = self._gyro_z
            self._Q[2, 2] = _Q_THETA_GYRO
        else:
            omega = msg.twist.twist.angular.z
            self._Q[2, 2] = _Q_THETA_WHEEL
            if self._imu_ever_seen:
                self.get_logger().warn(
                    f'IMU stale ({now - (self._gyro_t or now):.2f}s) — '
                    'falling back to wheel-derived yaw rate.',
                    throttle_duration_sec=5.0,
                )

        th = self._mu[2]

        G = np.eye(3)
        G[0, 2] = (-vx * np.sin(th) - vy * np.cos(th)) * dt
        G[1, 2] = (vx * np.cos(th) - vy * np.sin(th)) * dt

        self._mu[0] += (vx * np.cos(th) - vy * np.sin(th)) * dt
        self._mu[1] += (vx * np.sin(th) + vy * np.cos(th)) * dt
        self._mu[2] += omega * dt

        self._sigma = G @ self._sigma @ G.T + self._Q
        self._publish()

    # ------------------------------------------------------------------
    # Correction step: SLAM pose update
    # ------------------------------------------------------------------
    def _slam_cb(self, msg: PoseStamped):
        q = msg.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        slam_theta = np.arctan2(siny, cosy)

        z = np.array([msg.pose.position.x, msg.pose.position.y, slam_theta])
        H = np.eye(3)
        S = H @ self._sigma @ H.T + self._R
        K = self._sigma @ H.T @ np.linalg.inv(S)
        innov = z - self._mu
        innov[2] = np.arctan2(np.sin(innov[2]), np.cos(innov[2]))
        self._mu = self._mu + K @ innov
        self._sigma = (np.eye(3) - K @ H) @ self._sigma
        self._publish()

    def _publish(self):
        out = Odometry()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self._odom_frame_id
        out.child_frame_id  = self._base_frame_id
        out.pose.pose.position.x = float(self._mu[0])
        out.pose.pose.position.y = float(self._mu[1])
        out.pose.pose.orientation.z = float(np.sin(self._mu[2] / 2.0))
        out.pose.pose.orientation.w = float(np.cos(self._mu[2] / 2.0))

        # Pose covariance is required by RTAB-Map's optimizer to weight
        # graph edges. With all-zero covariance, RTAB falls back to a
        # 1e-3 default that is far too tight on this swerve platform —
        # the optimizer then trusts wheel odom over visual loop closures
        # and rejects every proposed loop. Project the EKF's 3×3 sigma
        # (x, y, theta) into the 6×6 (x, y, z, roll, pitch, yaw) layout;
        # unused dimensions get 1e9 so consumers know not to fuse them.
        s = self._sigma
        cov = out.pose.covariance
        cov[0]  = float(s[0, 0]); cov[1]  = float(s[0, 1]); cov[5]  = float(s[0, 2])
        cov[6]  = float(s[1, 0]); cov[7]  = float(s[1, 1]); cov[11] = float(s[1, 2])
        cov[30] = float(s[2, 0]); cov[31] = float(s[2, 1]); cov[35] = float(s[2, 2])
        cov[14] = 1e9   # z   (unused — flat-floor robot)
        cov[21] = 1e9   # roll
        cov[28] = 1e9   # pitch

        # Twist covariance: per-step velocity uncertainty. Use the same
        # process-noise diagonal we already track in self._Q so RTAB sees
        # a consistent story between absolute pose growth and per-frame
        # increments. Q[2,2] swaps between gyro / wheel each step.
        Q = self._Q
        tcov = out.twist.covariance
        tcov[0]  = float(Q[0, 0])
        tcov[7]  = float(Q[1, 1])
        tcov[35] = float(Q[2, 2])
        tcov[14] = 1e9
        tcov[21] = 1e9
        tcov[28] = 1e9
        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = EKFNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
