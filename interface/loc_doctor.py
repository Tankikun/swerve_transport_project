"""
loc_doctor.py — Read-only diagnostic monitor for RTAB-Map localization.

Subscribes to every topic that participates in the laptop GUI's "LOC: ..."
status and prints one block per second showing what each layer is actually
doing. Designed to answer the most annoying class of bug:

    "The pill says LOC: LIVE 0.47, -1.19 but it's stuck — when I move the
     robot to a different room the coordinate doesn't change."

The likely root cause for that specific symptom is that RTAB-Map has
never visually matched the saved map: it publishes an *identity* TF for
`map -> {robot_id}_odom` from startup (default behavior of
`rtabmap_slam`), so the bridge sees a valid `map -> base_link` chain
and reports `localized: true`, but the actual pose is just whatever
the wheel-odometry integrator says — frozen at the last commanded
location because picking up the robot doesn't move the encoders.

`loc_doctor` exposes that directly:

  - Per-topic Hz on every link in the chain (camera, depth, odom,
    ekf, rtabmap/info, slam/pose, localization_pose).
  - Live counts of `slam/pose` / `localization_pose` messages — these
    are 0 if RTAB-Map never matched.
  - The actual `map -> {rid}_odom` TF translation. If it's identity
    (0,0,0), RTAB-Map has never corrected wheel odom.
  - Most recent inliers/matches from `/{rid}/rtabmap/info`. If
    `inliers` stays below the threshold, matching is being attempted
    and *rejected* — different fix from "never attempted at all".
  - A one-line plain-English diagnosis derived from the above.

The script is read-only — pure subscriptions and TF lookups. Safe to
run concurrently with anything else on the robot.

Best run on **pi2** (no FastDDS network dependency). Can also run on
the laptop if you've confirmed peer discovery is healthy.

Outputs
-------
  - stdout : color-coded human-readable block, one per `print_period_sec`
  - jsonl  : one record per print, written to
             `~/loc_doctor_logs/loc_doctor_{robot_id}_{stamp}.jsonl`
             (disable with `-p no_log:=true` or override path with
             `-p log_dir:=...`)

Run
---
    # On pi2 (recommended):
    ssh pi2 "source ~/ros2_ws/install/setup.bash && \\
             python3 ~/swerve_transport_project/interface/loc_doctor.py \\
             --ros-args -p robot_id:=tb3_1"

    # Slower print rate, no log file:
    python3 loc_doctor.py --ros-args \\
        -p robot_id:=tb3_1 -p print_period_sec:=2.0 -p no_log:=true
"""

import json
import math
import os
import time
from collections import deque
from datetime import datetime

import rclpy
import tf2_ros
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

# rtabmap_msgs is shipped with rtabmap_ros, but if for some reason
# this script is run in an env without it sourced, fall back to
# counting Info messages without parsing them.
try:
    from rtabmap_msgs.msg import Info as RtabInfo
    HAVE_RTAB_INFO = True
except ImportError:                     # pragma: no cover
    RtabInfo = None
    HAVE_RTAB_INFO = False


# ── ANSI palette (off if stdout isn't a TTY, e.g. piped to a file) ────────
def _supports_color():
    return os.environ.get('NO_COLOR') is None and \
           hasattr(os.sys.stdout, 'isatty') and os.sys.stdout.isatty()


if _supports_color():
    GREEN, YELLOW, RED, DIM, BOLD, END = (
        '\033[92m', '\033[93m', '\033[91m', '\033[2m', '\033[1m', '\033[0m')
else:
    GREEN = YELLOW = RED = DIM = BOLD = END = ''


def quat_to_yaw(qx, qy, qz, qw):
    return math.atan2(2.0 * (qw * qz + qx * qy),
                      1.0 - 2.0 * (qy * qy + qz * qz))


class HzTracker:
    """Sliding-window Hz estimator. Keeps timestamps in a small ring buffer."""

    def __init__(self, window=20):
        self.times = deque(maxlen=window)
        self.count = 0

    def tick(self):
        self.times.append(time.time())
        self.count += 1

    def hz(self):
        if len(self.times) < 2:
            return 0.0
        span = self.times[-1] - self.times[0]
        if span <= 0:
            return 0.0
        return (len(self.times) - 1) / span

    def age_sec(self):
        if not self.times:
            return None
        return time.time() - self.times[-1]


class LocDoctor(Node):
    def __init__(self):
        super().__init__('loc_doctor')

        self.declare_parameter('robot_id',         'tb3_1')
        self.declare_parameter('print_period_sec', 1.0)
        self.declare_parameter('log_dir',
                               os.path.expanduser('~/loc_doctor_logs'))
        self.declare_parameter('no_log',           False)

        self.rid          = str(self.get_parameter('robot_id').value)
        self.print_period = float(self.get_parameter('print_period_sec').value)
        self.no_log       = bool(self.get_parameter('no_log').value)
        log_dir           = str(self.get_parameter('log_dir').value)

        # ── jsonl log file ────────────────────────────────────────────────
        self.log_file = None
        if not self.no_log:
            try:
                os.makedirs(log_dir, exist_ok=True)
                stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
                log_path = os.path.join(
                    log_dir, f'loc_doctor_{self.rid}_{stamp}.jsonl')
                self.log_file = open(log_path, 'w', buffering=1)  # line-buffered
                self.get_logger().info(f'JSONL log -> {log_path}')
            except OSError as e:
                self.get_logger().warn(f'Could not open log file: {e}')

        # ── Hz trackers ───────────────────────────────────────────────────
        self.hz_cam_rgb   = HzTracker()
        self.hz_cam_depth = HzTracker()
        self.hz_odom      = HzTracker()
        self.hz_ekf       = HzTracker()
        self.hz_info      = HzTracker()
        self.hz_slam_pose = HzTracker()
        self.hz_loc_pose  = HzTracker()

        # ── Last-seen state (for value display & change detection) ────────
        self.last_odom_pose = None       # (x, y, yaw_rad)
        self.last_ekf_pose  = None
        self.last_info      = {}         # parsed from rtabmap_msgs/Info

        # ── Subscriptions ─────────────────────────────────────────────────
        # Camera topics use sensor-data QoS (BEST_EFFORT, KEEP_LAST 5) — if
        # we subscribed with default reliability we'd see no messages from
        # the OAK driver.
        rid = self.rid
        self.create_subscription(
            Image, f'/{rid}/camera/rgb/image_raw',
            lambda m: self.hz_cam_rgb.tick(), qos_profile_sensor_data)
        self.create_subscription(
            Image, f'/{rid}/camera/depth/image_raw',
            lambda m: self.hz_cam_depth.tick(), qos_profile_sensor_data)
        self.create_subscription(
            Odometry, f'/{rid}/odom', self._odom_cb, 10)
        self.create_subscription(
            Odometry, f'/{rid}/ekf/odom', self._ekf_cb, 10)
        self.create_subscription(
            PoseStamped, f'/{rid}/slam/pose',
            lambda m: self.hz_slam_pose.tick(), 10)
        self.create_subscription(
            PoseWithCovarianceStamped,
            f'/{rid}/rtabmap/localization_pose',
            lambda m: self.hz_loc_pose.tick(), 10)
        if HAVE_RTAB_INFO:
            self.create_subscription(
                RtabInfo, f'/{rid}/rtabmap/info', self._info_cb, 10)

        # ── TF listener ───────────────────────────────────────────────────
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ── Print timer ───────────────────────────────────────────────────
        self.start_time = time.time()
        self.create_timer(self.print_period, self._tick)

        msg_extras = ('rtabmap_msgs available: yes'
                      if HAVE_RTAB_INFO
                      else f'{RED}rtabmap_msgs MISSING — info will be silent{END}')
        self.get_logger().info(
            f'loc_doctor watching /{rid}/* every {self.print_period:.1f}s '
            f'({msg_extras})')

    # ── callbacks ─────────────────────────────────────────────────────────
    def _odom_cb(self, msg: Odometry):
        self.hz_odom.tick()
        p = msg.pose.pose
        self.last_odom_pose = (
            p.position.x, p.position.y,
            quat_to_yaw(p.orientation.x, p.orientation.y,
                        p.orientation.z, p.orientation.w))

    def _ekf_cb(self, msg: Odometry):
        self.hz_ekf.tick()
        p = msg.pose.pose
        self.last_ekf_pose = (
            p.position.x, p.position.y,
            quat_to_yaw(p.orientation.x, p.orientation.y,
                        p.orientation.z, p.orientation.w))

    def _info_cb(self, msg):
        self.hz_info.tick()
        # rtabmap_msgs/Info ships a parallel list of stats keys/values
        # (used for plotting in rtabmap_viz). Pull the few we care about
        # for diagnosing match-quality issues.
        try:
            stats = dict(zip(msg.statsKeys, msg.statsValues))
        except Exception:
            stats = {}

        def _stat(*candidates):
            """Return the first matching key (rtabmap key names vary by version)."""
            for key in candidates:
                if key in stats:
                    return stats[key]
            return None

        self.last_info = {
            'ref_id':         msg.ref_id,
            'loop_id':        msg.loop_closure_id,
            'proximity_id':   msg.proximity_detection_id,
            # Loop closure visual stats (the actual match attempt)
            'loop_inliers':   _stat('Loop/Inliers/', 'Loop/VisualInliers/'),
            'loop_matches':   _stat('Loop/Matches/', 'Loop/VisualMatches/'),
            # Most-recent visual-registration attempt (per-frame)
            'reg_inliers':    _stat('RegistrationVis/Inliers/'),
            'reg_matches':    _stat('RegistrationVis/Matches/'),
        }

    # ── helpers ───────────────────────────────────────────────────────────
    def _lookup_tf(self, target, source):
        """Return (x, y, yaw_deg) or None."""
        try:
            t = self.tf_buffer.lookup_transform(
                target, source, rclpy.time.Time(),
                timeout=Duration(seconds=0.1))
            yaw = quat_to_yaw(t.transform.rotation.x, t.transform.rotation.y,
                              t.transform.rotation.z, t.transform.rotation.w)
            return (t.transform.translation.x,
                    t.transform.translation.y,
                    math.degrees(yaw))
        except (tf2_ros.LookupException,
                tf2_ros.ExtrapolationException,
                tf2_ros.ConnectivityException):
            return None

    @staticmethod
    def _is_identity(tf):
        if tf is None:
            return False
        x, y, yaw_deg = tf
        return abs(x) < 1e-4 and abs(y) < 1e-4 and abs(yaw_deg) < 1e-2

    # ── tick: collect → diagnose → print → log ────────────────────────────
    def _tick(self):
        rid = self.rid
        elapsed = time.time() - self.start_time

        snap = {
            'cam_rgb_hz':   self.hz_cam_rgb.hz(),
            'cam_depth_hz': self.hz_cam_depth.hz(),
            'odom_hz':      self.hz_odom.hz(),
            'ekf_hz':       self.hz_ekf.hz(),
            'info_hz':      self.hz_info.hz(),
            'slam_pose_hz': self.hz_slam_pose.hz(),
            'loc_pose_hz':  self.hz_loc_pose.hz(),
            'slam_pose_count': self.hz_slam_pose.count,
            'loc_pose_count':  self.hz_loc_pose.count,
            'tf_map_to_odom':       self._lookup_tf('map', f'{rid}_odom'),
            'tf_map_to_base_link':  self._lookup_tf('map', f'{rid}_base_link'),
            'tf_odom_to_base_link': self._lookup_tf(f'{rid}_odom',
                                                    f'{rid}_base_link'),
            'last_odom_pose': self.last_odom_pose,
            'last_ekf_pose':  self.last_ekf_pose,
            'rtabmap_info':   dict(self.last_info),
        }
        diagnosis = self._diagnose(snap)

        self._print_block(elapsed, snap, diagnosis)

        if self.log_file:
            record = {
                't': time.time(), 't_rel': elapsed,
                'diagnosis': diagnosis,
                **snap,
            }
            try:
                self.log_file.write(json.dumps(record, default=str) + '\n')
            except Exception as e:                          # pragma: no cover
                self.get_logger().warn(f'JSONL write failed: {e}')

    # ── diagnosis decision tree ──────────────────────────────────────────
    def _diagnose(self, s):
        """Return (severity, message) where severity in {'ok','warn','err'}."""
        cam_hz   = s['cam_rgb_hz']
        info_hz  = s['info_hz']
        info     = s['rtabmap_info']
        slam_n   = s['slam_pose_count']
        m2o      = s['tf_map_to_odom']

        if cam_hz < 1.0:
            return ('err',
                    'CAMERA DEAD — /camera/rgb/image_raw not publishing. '
                    'Check that oak_camera is up and i_usb_speed=SUPER negotiated.')

        if not HAVE_RTAB_INFO:
            return ('warn',
                    'rtabmap_msgs not installed in this env — cannot tell whether '
                    'matching is happening. Source rtabmap_ros and re-run.')

        if info_hz < 0.1:
            return ('err',
                    'RTAB-MAP NOT RUNNING (or starved of RGBD pairs). '
                    'Check `ros2 node list | grep rtabmap` and the topic remaps.')

        if slam_n == 0:
            inliers = info.get('loop_inliers') or info.get('reg_inliers') or 0
            try:
                inliers = int(inliers)
            except (TypeError, ValueError):
                inliers = 0

            if inliers == 0:
                return ('warn',
                        'NO MATCHES YET — RTAB-Map ran 0 inliers on its last attempt. '
                        'Likely scene lacks features, robot is in untrained area, or '
                        '.db on pi mismatches map.json on laptop. Try Set Initial '
                        'Pose + drive 10 cm.')
            else:
                return ('warn',
                        f'MATCHING ATTEMPTED but rejected ({inliers} inliers, '
                        'threshold likely 15). Lower Vis/MinInliers in '
                        'rtabmap_localization.yaml, or improve lighting / move '
                        'closer to a feature-rich wall.')

        if self._is_identity(m2o):
            return ('warn',
                    'TF map->odom is IDENTITY: rtabmap publishing the chain but '
                    'has not corrected wheel odom yet. GUI shows "LIVE" but the '
                    'coordinate is just whatever wheel odom integrated to.')

        return ('ok',
                f'localization producing slam/pose (n={slam_n}). '
                'GUI should show fresh LIVE pose that follows the robot.')

    # ── pretty print ─────────────────────────────────────────────────────
    @staticmethod
    def _fmt_hz(hz, ok_min):
        if hz < 0.1:
            return f'{RED}{hz:5.1f}Hz{END}'
        if hz < ok_min:
            return f'{YELLOW}{hz:5.1f}Hz{END}'
        return f'{GREEN}{hz:5.1f}Hz{END}'

    @staticmethod
    def _fmt_count(c):
        return f'{RED}NEVER{END}' if c == 0 else f'{GREEN}n={c}{END}'

    @staticmethod
    def _fmt_tf(tf, identity_warn=False):
        if tf is None:
            return f'{RED}-- (lookup failed){END}'
        x, y, yaw = tf
        s = f'x={x:+7.3f} y={y:+7.3f} yaw={yaw:+6.1f}°'
        if identity_warn and abs(x) < 1e-4 and abs(y) < 1e-4 and abs(yaw) < 1e-2:
            return f'{YELLOW}{s}  [IDENTITY]{END}'
        return s

    @staticmethod
    def _fmt_pose(p):
        if p is None:
            return f'{DIM}(no msgs){END}'
        x, y, yaw = p
        return f'x={x:+7.3f} y={y:+7.3f} yaw={math.degrees(yaw):+6.1f}°'

    def _print_block(self, elapsed, s, diag):
        sev, msg = diag
        ts = datetime.now().strftime('%H:%M:%S')
        rid = self.rid
        info = s['rtabmap_info']

        print(f'\n{BOLD}[loc_doctor {ts}  T+{elapsed:6.1f}s  /{rid}/]{END}')
        print(f'  camera   rgb={self._fmt_hz(s["cam_rgb_hz"], 10)}  '
              f'depth={self._fmt_hz(s["cam_depth_hz"], 10)}')
        print(f'  odom     wheel={self._fmt_hz(s["odom_hz"], 20)} '
              f'{self._fmt_pose(s["last_odom_pose"])}')
        print(f'  ekf      out  ={self._fmt_hz(s["ekf_hz"], 20)} '
              f'{self._fmt_pose(s["last_ekf_pose"])}')
        print(f'  rtabmap  info ={self._fmt_hz(s["info_hz"], 0.5)}  '
              f'inliers={info.get("loop_inliers") or info.get("reg_inliers") or "-"}  '
              f'matches={info.get("loop_matches") or info.get("reg_matches") or "-"}  '
              f'ref_id={info.get("ref_id", "-")}  '
              f'loop_id={info.get("loop_id", "-")}')
        print(f'  outputs  slam/pose={self._fmt_hz(s["slam_pose_hz"], 0.5)} '
              f'{self._fmt_count(s["slam_pose_count"])}  '
              f'loc_pose={self._fmt_hz(s["loc_pose_hz"], 0.5)} '
              f'{self._fmt_count(s["loc_pose_count"])}')
        print(f'  TF  map        -> {rid}_odom        {self._fmt_tf(s["tf_map_to_odom"], identity_warn=True)}')
        print(f'  TF  map        -> {rid}_base_link   {self._fmt_tf(s["tf_map_to_base_link"])}')
        print(f'  TF  {rid}_odom -> {rid}_base_link   {self._fmt_tf(s["tf_odom_to_base_link"])}')

        color = {'ok': GREEN, 'warn': YELLOW, 'err': RED}[sev]
        prefix = {'ok': '✓', 'warn': '⚠', 'err': '✗'}[sev]
        # Wrap long diagnosis at ~78 cols for readability.
        print(f'  {color}{prefix} {msg}{END}')


def main(args=None):
    rclpy.init(args=args)
    node = LocDoctor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        # Clean exit on Ctrl-C or `timeout`/SIGTERM. Without catching
        # ExternalShutdownException the user sees a noisy traceback
        # every time they stop the script.
        pass
    finally:
        if node.log_file:
            try:
                node.log_file.close()
            except Exception:
                pass
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
