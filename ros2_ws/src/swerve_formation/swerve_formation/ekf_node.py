import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import Imu
import numpy as np


class EKFNode(Node):
    """
    Fuses raw wheel odometry (prediction) with two correction sources:

      1. IMU yaw    — `/{robot_id}/imu`         (1-D θ observation)
      2. SLAM pose  — `/{robot_id}/slam/pose`   (full 3-D x,y,θ)

    The IMU is the primary correction now that visual localisation
    (RTAB-Map) is no longer guaranteed for the demo. SLAM, when present,
    refines x,y too and is kept as an opt-in fallback. State: [x, y, θ].

    Only this node reads raw /{robot_id}/odom.
    Publishes authoritative pose on /{robot_id}/ekf/odom.

    `/{robot_id}/initialpose` (PoseWithCovarianceStamped) hard-resets
    the state and covariance — the GUI's "Set Initial Pose" tool fans
    out to this topic so the EKF picks up the user-clicked pose even
    when no SLAM correction is ever produced.
    """

    def __init__(self):
        super().__init__('ekf_node')
        self.declare_parameter('robot_id', 'tb3_0')
        # Per-source observation-noise variances. IMU yaw is tighter than
        # SLAM yaw because consumer IMUs typically resolve heading to
        # ~1° once warm; SLAM yaw is matched-keyframe-quality.
        self.declare_parameter('imu_yaw_var', 0.005)
        self.declare_parameter('slam_var', [0.05, 0.05, 0.02])
        # When true, /slam/pose corrections are ignored. Set this for
        # demos where RTAB-Map will not be running and we don't want
        # spurious old-cached SLAM messages to perturb the filter.
        self.declare_parameter('use_slam', True)
        robot_id = self.get_parameter('robot_id').value
        # Per-robot TF frame ids — must match what conveyor_base_node and the
        # static_transform_publisher in oak_camera.launch.py use, otherwise
        # rtabmap can't connect odom → base_link → camera_optical.
        self._odom_frame_id = f'{robot_id}_odom'
        self._base_frame_id = f'{robot_id}_base_link'

        self._mu = np.zeros(3)           # [x, y, theta]
        self._sigma = np.eye(3) * 0.1
        self._Q = np.diag([0.01, 0.01, 0.005])   # process noise
        slam_var = list(self.get_parameter('slam_var').value)
        self._R_slam = np.diag(slam_var)         # SLAM observation noise
        self._R_imu = np.array([[float(self.get_parameter('imu_yaw_var').value)]])
        self._use_slam = bool(self.get_parameter('use_slam').value)
        self._last_t: float | None = None

        # IMU yaw is reported in the IMU's own (boot-time) frame; align
        # it to the EKF's world frame on the first valid sample.
        self._imu_yaw_offset: float | None = None

        # Only this node reads raw /odom
        self.create_subscription(Odometry, f'/{robot_id}/odom', self._odom_cb, 10)
        self.create_subscription(Imu, f'/{robot_id}/imu', self._imu_cb, 50)
        self.create_subscription(PoseStamped, f'/{robot_id}/slam/pose', self._slam_cb, 10)
        # Hard reset from GUI / external init — bypasses the Kalman update.
        self.create_subscription(
            PoseWithCovarianceStamped, f'/{robot_id}/initialpose',
            self._initialpose_cb, 10,
        )
        self._pub = self.create_publisher(Odometry, f'/{robot_id}/ekf/odom', 10)
        # path_planner_node (laptop) subscribes to /{robot_id}/pose to
        # compute the formation's virtual centre. We republish the same
        # EKF state as a PoseStamped on every publish — frame_id 'map'
        # because, with `enable_slam:=false`, conveyor.launch.py installs
        # a static map → {robot_id}_odom identity TF that aliases the
        # two frames, and the GUI's clicked goal is in 'map'.
        self._pose_pub = self.create_publisher(
            PoseStamped, f'/{robot_id}/pose', 10)
        self.get_logger().info(
            f'EKF ready for {robot_id} (use_slam={self._use_slam}, '
            f'imu_yaw_var={float(self._R_imu[0, 0]):.4f})'
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
        omega = msg.twist.twist.angular.z
        th = self._mu[2]

        G = np.eye(3)
        G[0, 2] = (-vx * np.sin(th) - vy * np.cos(th)) * dt
        G[1, 2] = ( vx * np.cos(th) - vy * np.sin(th)) * dt

        self._mu[0] += (vx * np.cos(th) - vy * np.sin(th)) * dt
        self._mu[1] += (vx * np.sin(th) + vy * np.cos(th)) * dt
        self._mu[2] += omega * dt

        self._sigma = G @ self._sigma @ G.T + self._Q
        self._publish()

    # ------------------------------------------------------------------
    # Correction step: IMU yaw (1-D observation on θ)
    # ------------------------------------------------------------------
    def _imu_cb(self, msg: Imu):
        # orientation_covariance[0] == -1 means the IMU is publishing
        # raw angular velocity / linear accel only — no AHRS-fused
        # quaternion, so there's no yaw to fuse here.
        if msg.orientation_covariance[0] < 0.0:
            return
        q = msg.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        imu_yaw = float(np.arctan2(siny, cosy))

        if self._imu_yaw_offset is None:
            # Snap the IMU's frame onto the EKF's current θ. After this
            # offset is fixed, drift in the IMU is corrected against
            # this anchor, not relearned every message.
            self._imu_yaw_offset = self._mu[2] - imu_yaw
            self.get_logger().info(
                f'[imu] aligned: imu_yaw={imu_yaw:+.3f} rad, '
                f'offset={self._imu_yaw_offset:+.3f} rad'
            )

        z_theta = imu_yaw + self._imu_yaw_offset
        H = np.array([[0.0, 0.0, 1.0]])           # observe θ only
        S = float(H @ self._sigma @ H.T + self._R_imu)
        K = (self._sigma @ H.T).flatten() / S      # 3-vector
        innov = float(np.arctan2(np.sin(z_theta - self._mu[2]),
                                 np.cos(z_theta - self._mu[2])))
        self._mu = self._mu + K * innov
        self._sigma = (np.eye(3) - np.outer(K, H.flatten())) @ self._sigma
        self._publish()

    # ------------------------------------------------------------------
    # Correction step: SLAM pose update (optional)
    # ------------------------------------------------------------------
    def _slam_cb(self, msg: PoseStamped):
        if not self._use_slam:
            return
        q = msg.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        slam_theta = np.arctan2(siny, cosy)

        z = np.array([msg.pose.position.x, msg.pose.position.y, slam_theta])
        H = np.eye(3)
        S = H @ self._sigma @ H.T + self._R_slam
        K = self._sigma @ H.T @ np.linalg.inv(S)
        innov = z - self._mu
        innov[2] = np.arctan2(np.sin(innov[2]), np.cos(innov[2]))
        self._mu = self._mu + K @ innov
        self._sigma = (np.eye(3) - K @ H) @ self._sigma
        # Re-anchor IMU offset against the post-SLAM yaw so the next
        # IMU sample doesn't unwind the visual correction.
        self._imu_yaw_offset = None
        self._publish()

    # ------------------------------------------------------------------
    # External hard reset (GUI initial-pose / re-localisation hint)
    # ------------------------------------------------------------------
    def _initialpose_cb(self, msg: PoseWithCovarianceStamped):
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        theta = float(np.arctan2(siny, cosy))
        self._mu = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            theta,
        ])
        self._sigma = np.eye(3) * 0.05
        # Force IMU realignment against the new world θ.
        self._imu_yaw_offset = None
        self.get_logger().info(
            f'[init] EKF state reset to '
            f'({self._mu[0]:+.2f}, {self._mu[1]:+.2f}, '
            f'{np.degrees(self._mu[2]):+.0f}°)'
        )
        self._publish()

    def _publish(self):
        stamp = self.get_clock().now().to_msg()
        out = Odometry()
        out.header.stamp = stamp
        out.header.frame_id = self._odom_frame_id
        out.child_frame_id  = self._base_frame_id
        out.pose.pose.position.x = float(self._mu[0])
        out.pose.pose.position.y = float(self._mu[1])
        out.pose.pose.orientation.z = float(np.sin(self._mu[2] / 2.0))
        out.pose.pose.orientation.w = float(np.cos(self._mu[2] / 2.0))
        self._pub.publish(out)
        # Also republish as PoseStamped in the map frame for the laptop
        # planner. Same x/y/yaw values; frame_id reflects the world frame
        # the GUI uses.
        ps = PoseStamped()
        ps.header.stamp    = stamp
        ps.header.frame_id = 'map'
        ps.pose.position.x = float(self._mu[0])
        ps.pose.position.y = float(self._mu[1])
        ps.pose.orientation.z = float(np.sin(self._mu[2] / 2.0))
        ps.pose.orientation.w = float(np.cos(self._mu[2] / 2.0))
        self._pose_pub.publish(ps)


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
