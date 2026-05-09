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
  - Motion delta: did the robot move since last tick? Helps catch
    "I'm pressing teleop keys but nothing is happening" early.
  - Localization-pose covariance (last visual match's confidence).
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
  - log    : ANSI-stripped human-readable copy of stdout, written to
             `~/loc_doctor_logs/loc_doctor_{robot_id}_{stamp}.log`
             (so you can `less` / grep without parsing JSON)

Disable file output with `-p no_log:=true`. Override path with
`-p log_dir:=...`.

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
import re
import socket
import sys
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
           hasattr(sys.stdout, 'isatty') and sys.stdout.isatty()


if _supports_color():
    GREEN, YELLOW, RED, DIM, BOLD, CYAN, END = (
        '\033[92m', '\033[93m', '\033[91m', '\033[2m',
        '\033[1m', '\033[36m', '\033[0m')
else:
    GREEN = YELLOW = RED = DIM = BOLD = CYAN = END = ''

ANSI_RE = re.compile(r'\033\[[0-9;]*m')


def strip_ansi(s):
    return ANSI_RE.sub('', s)


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

        # ── log files (jsonl + plain-text) ────────────────────────────────
        self.jsonl_file    = None
        self.text_file     = None
        self.jsonl_path    = None
        self.text_path     = None
        if not self.no_log:
            try:
                os.makedirs(log_dir, exist_ok=True)
                stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
                base = os.path.join(log_dir, f'loc_doctor_{self.rid}_{stamp}')
                self.jsonl_path = base + '.jsonl'
                self.text_path  = base + '.log'
                # Line-buffered so a kill -9 still leaves the latest line.
                self.jsonl_file = open(self.jsonl_path, 'w', buffering=1)
                self.text_file  = open(self.text_path,  'w', buffering=1)
                self.get_logger().info(f'JSONL  -> {self.jsonl_path}')
                self.get_logger().info(f'TEXT   -> {self.text_path}')
            except OSError as e:
                self.get_logger().warn(f'Could not open log file(s): {e}')

        # ── Hz trackers ───────────────────────────────────────────────────
        self.hz_cam_rgb   = HzTracker()
        self.hz_cam_depth = HzTracker()
        self.hz_odom      = HzTracker()
        self.hz_ekf       = HzTracker()
        self.hz_info      = HzTracker()
        self.hz_slam_pose = HzTracker()
        self.hz_loc_pose  = HzTracker()

        # ── Last-seen state ───────────────────────────────────────────────
        self.last_odom_pose         = None  # (x, y, yaw_rad)  most recent msg
        self.last_ekf_pose          = None
        self.last_info              = {}    # parsed from rtabmap_msgs/Info
        self.last_loc_pose          = None  # (x, y, yaw_rad)  from /localization_pose
        self.last_loc_cov_diag      = None  # (var_x, var_y, var_yaw)
        # For motion delta: the values at the previous _tick() invocation.
        self._odom_pose_prev_tick   = None
        self._ekf_pose_prev_tick    = None
        self._tick_prev_t           = None

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
            self._loc_pose_cb, 10)
        if HAVE_RTAB_INFO:
            self.create_subscription(
                RtabInfo, f'/{rid}/rtabmap/info', self._info_cb, 10)

        # ── TF listener ───────────────────────────────────────────────────
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ── Session bookkeeping + print timer ─────────────────────────────
        self.start_time = time.time()
        self.tick_n     = 0
        self._print_session_header()
        self.create_timer(self.print_period, self._tick)

    # ── session-level prints ─────────────────────────────────────────────
    def _print_session_header(self):
        """Printed once at startup (and to text/jsonl logs as a header line).

        Captures the env this run is observing — useful when you scroll
        back through a log days later and need to remember WHICH robot,
        WHICH ROS_DOMAIN, WHICH peers file you were on.
        """
        env = {
            'ROS_DOMAIN_ID':   os.environ.get('ROS_DOMAIN_ID', '(unset)'),
            'RMW_IMPL':        os.environ.get('RMW_IMPLEMENTATION', '(default)'),
            'FASTRTPS_FILE':   os.environ.get(
                'FASTRTPS_DEFAULT_PROFILES_FILE', '(unset)'),
            'RMW_ZENOH_CONF':  os.environ.get('RMW_ZENOH_CONFIG_FILE', '(unset)'),
        }
        rtab_status = ('yes' if HAVE_RTAB_INFO
                       else 'MISSING (rtabmap_msgs not in PYTHONPATH)')

        bar = '═' * 78
        lines = [
            f'{CYAN}{BOLD}╔{bar}╗{END}',
            f'{CYAN}{BOLD}║ loc_doctor session{END}',
            f'{CYAN}║   robot_id           : {self.rid}',
            f'{CYAN}║   hostname           : {socket.gethostname()}',
            f'{CYAN}║   ROS_DOMAIN_ID      : {env["ROS_DOMAIN_ID"]}',
            f'{CYAN}║   RMW_IMPLEMENTATION : {env["RMW_IMPL"]}',
            f'{CYAN}║   FastDDS profiles   : {env["FASTRTPS_FILE"]}',
            f'{CYAN}║   rtabmap_msgs       : {rtab_status}',
            f'{CYAN}║   print period       : {self.print_period:.1f}s',
            f'{CYAN}║   jsonl log          : {self.jsonl_path or "(disabled)"}',
            f'{CYAN}║   text log           : {self.text_path  or "(disabled)"}',
            f'{CYAN}║   started            : {datetime.now().isoformat(timespec="seconds")}',
            f'{CYAN}{BOLD}╚{bar}╝{END}',
        ]
        for ln in lines:
            print(ln)
        # Mirror header into log files (without ANSI).
        if self.text_file:
            for ln in lines:
                self.text_file.write(strip_ansi(ln) + '\n')
        if self.jsonl_file:
            self.jsonl_file.write(json.dumps({
                'kind': 'session_start',
                't': time.time(),
                'robot_id': self.rid,
                'hostname': socket.gethostname(),
                'env': env,
                'rtabmap_msgs_available': HAVE_RTAB_INFO,
                'print_period_sec': self.print_period,
                'started_iso': datetime.now().isoformat(),
            }) + '\n')

    def _print_session_footer(self):
        """Printed at clean exit (Ctrl-C / SIGTERM). Summarizes what happened."""
        elapsed = time.time() - self.start_time
        bar = '─' * 78
        ever_localized = self.hz_slam_pose.count > 0
        status_blurb = (f'{GREEN}EVER LOCALIZED (slam/pose received {self.hz_slam_pose.count} '
                        f'msgs total){END}'
                        if ever_localized
                        else f'{RED}NEVER LOCALIZED — slam/pose stayed silent the whole run{END}')
        lines = [
            f'\n{BOLD}{bar}{END}',
            f'{BOLD}loc_doctor session ended after {elapsed:.1f}s ({self.tick_n} ticks){END}',
            f'  total camera frames     : rgb={self.hz_cam_rgb.count}  depth={self.hz_cam_depth.count}',
            f'  total odom / ekf msgs   : {self.hz_odom.count} / {self.hz_ekf.count}',
            f'  total rtabmap/info msgs : {self.hz_info.count}',
            f'  total slam/pose msgs    : {self.hz_slam_pose.count}',
            f'  total loc_pose msgs     : {self.hz_loc_pose.count}',
            f'  status                  : {status_blurb}',
        ]
        if self.jsonl_path:
            lines.append(f'  logs                    : {self.jsonl_path}')
            lines.append(f'                            {self.text_path}')
        lines.append(f'{BOLD}{bar}{END}')
        for ln in lines:
            print(ln)
        if self.text_file:
            for ln in lines:
                self.text_file.write(strip_ansi(ln) + '\n')
        if self.jsonl_file:
            self.jsonl_file.write(json.dumps({
                'kind': 'session_end',
                't': time.time(),
                'elapsed_sec': elapsed,
                'tick_count': self.tick_n,
                'cam_rgb_count': self.hz_cam_rgb.count,
                'cam_depth_count': self.hz_cam_depth.count,
                'odom_count': self.hz_odom.count,
                'ekf_count': self.hz_ekf.count,
                'info_count': self.hz_info.count,
                'slam_pose_count': self.hz_slam_pose.count,
                'loc_pose_count': self.hz_loc_pose.count,
                'ever_localized': ever_localized,
            }) + '\n')

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

    def _loc_pose_cb(self, msg: PoseWithCovarianceStamped):
        self.hz_loc_pose.tick()
        p = msg.pose.pose
        self.last_loc_pose = (
            p.position.x, p.position.y,
            quat_to_yaw(p.orientation.x, p.orientation.y,
                        p.orientation.z, p.orientation.w))
        # Covariance is row-major 6x6: indices 0,7,35 are var(x), var(y), var(yaw).
        # Note: msg.pose.covariance is a numpy array — `if cov` raises
        # "truth value of an array with more than one element is ambiguous",
        # so check length explicitly. Cast to float for clean JSON serialization.
        cov = msg.pose.covariance
        if cov is not None and len(cov) >= 36:
            self.last_loc_cov_diag = (float(cov[0]), float(cov[7]), float(cov[35]))

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

        # map_correction is the actual transform rtabmap proposes from
        # map -> odom this iteration. Useful to see even when slam/pose
        # hasn't published yet — it shows whether matching is happening
        # at all even without crossing the publish threshold.
        try:
            mc = msg.mapCorrection
            mc_x   = float(mc.translation.x)
            mc_y   = float(mc.translation.y)
            mc_yaw = math.degrees(quat_to_yaw(
                mc.rotation.x, mc.rotation.y, mc.rotation.z, mc.rotation.w))
        except Exception:
            mc_x = mc_y = mc_yaw = None

        self.last_info = {
            'ref_id':              msg.ref_id,
            'loop_id':             msg.loop_closure_id,
            'proximity_id':        msg.proximity_detection_id,
            'loop_inliers':        _stat('Loop/Inliers/', 'Loop/VisualInliers/'),
            'loop_matches':        _stat('Loop/Matches/', 'Loop/VisualMatches/'),
            'reg_inliers':         _stat('RegistrationVis/Inliers/'),
            'reg_matches':         _stat('RegistrationVis/Matches/'),
            'map_correction_x':    mc_x,
            'map_correction_y':    mc_y,
            'map_correction_yaw':  mc_yaw,
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

    @staticmethod
    def _motion_delta(curr, prev, dt):
        """Returns (dx, dy, dyaw_deg, vx_mps, vy_mps, vyaw_dps) or None."""
        if curr is None or prev is None or dt <= 0:
            return None
        dx   = curr[0] - prev[0]
        dy   = curr[1] - prev[1]
        dyaw = curr[2] - prev[2]
        # Wrap into [-pi, pi].
        while dyaw >  math.pi: dyaw -= 2 * math.pi
        while dyaw < -math.pi: dyaw += 2 * math.pi
        return (dx, dy, math.degrees(dyaw),
                dx / dt, dy / dt, math.degrees(dyaw) / dt)

    @staticmethod
    def _moving(delta, lin_thresh=0.005, ang_thresh=0.5):
        """delta from _motion_delta. Threshold defaults: 5 mm / 0.5 deg per tick."""
        if delta is None:
            return False
        dx, dy, dyaw_deg, *_ = delta
        return (math.hypot(dx, dy) > lin_thresh) or (abs(dyaw_deg) > ang_thresh)

    # ── tick: collect → diagnose → print → log ────────────────────────────
    def _tick(self):
        rid = self.rid
        now = time.time()
        elapsed = now - self.start_time
        self.tick_n += 1

        dt = (now - self._tick_prev_t) if self._tick_prev_t else self.print_period
        odom_delta = self._motion_delta(
            self.last_odom_pose, self._odom_pose_prev_tick, dt)
        ekf_delta  = self._motion_delta(
            self.last_ekf_pose,  self._ekf_pose_prev_tick,  dt)

        snap = {
            'cam_rgb_hz':        self.hz_cam_rgb.hz(),
            'cam_depth_hz':      self.hz_cam_depth.hz(),
            'odom_hz':           self.hz_odom.hz(),
            'ekf_hz':            self.hz_ekf.hz(),
            'info_hz':           self.hz_info.hz(),
            'slam_pose_hz':      self.hz_slam_pose.hz(),
            'loc_pose_hz':       self.hz_loc_pose.hz(),
            'slam_pose_count':   self.hz_slam_pose.count,
            'loc_pose_count':    self.hz_loc_pose.count,
            'tf_map_to_odom':       self._lookup_tf('map', f'{rid}_odom'),
            'tf_map_to_base_link':  self._lookup_tf('map', f'{rid}_base_link'),
            'tf_odom_to_base_link': self._lookup_tf(f'{rid}_odom',
                                                    f'{rid}_base_link'),
            'last_odom_pose':    self.last_odom_pose,
            'last_ekf_pose':     self.last_ekf_pose,
            'last_loc_pose':     self.last_loc_pose,
            'last_loc_cov_diag': self.last_loc_cov_diag,
            'odom_delta':        odom_delta,
            'ekf_delta':         ekf_delta,
            'odom_moving':       self._moving(odom_delta),
            'ekf_moving':        self._moving(ekf_delta),
            'rtabmap_info':      dict(self.last_info),
        }
        diagnosis = self._diagnose(snap)

        self._print_block(elapsed, snap, diagnosis)

        # Snapshot for next tick's delta.
        self._odom_pose_prev_tick = self.last_odom_pose
        self._ekf_pose_prev_tick  = self.last_ekf_pose
        self._tick_prev_t         = now

        if self.jsonl_file:
            try:
                self.jsonl_file.write(json.dumps({
                    'kind': 'tick', 't': now, 't_rel': elapsed,
                    'tick_n': self.tick_n,
                    'diagnosis': diagnosis,
                    **snap,
                }, default=str) + '\n')
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

    @staticmethod
    def _fmt_motion(delta, moving):
        if delta is None:
            return f'{DIM}(no delta yet){END}'
        dx, dy, dyaw_deg, vx, vy, vyaw_dps = delta
        tag = (f'{GREEN}MOVING{END}' if moving else f'{DIM}stationary{END}')
        return (f'{tag}  Δ=({dx:+.3f}m,{dy:+.3f}m,{dyaw_deg:+.1f}°) '
                f'v=({vx:+.2f},{vy:+.2f}) m/s, {vyaw_dps:+.1f}°/s')

    @staticmethod
    def _fmt_cov(cov):
        if cov is None:
            return f'{DIM}(no msgs){END}'
        var_x, var_y, var_yaw = cov
        sx, sy = math.sqrt(max(0, var_x)), math.sqrt(max(0, var_y))
        syaw = math.degrees(math.sqrt(max(0, var_yaw)))
        # RViz default for "2D Pose Estimate" is std=0.5 m / 15° → coloured
        # green if rtabmap thinks it's much tighter than the seed, yellow if
        # similar, red if >1 m std.
        worst = max(sx, sy)
        col = GREEN if worst < 0.1 else (YELLOW if worst < 0.5 else RED)
        return f'{col}σ_x={sx:.3f}m σ_y={sy:.3f}m σ_yaw={syaw:.2f}°{END}'

    @staticmethod
    def _fmt_map_correction(info):
        mx = info.get('map_correction_x')
        my = info.get('map_correction_y')
        myaw = info.get('map_correction_yaw')
        if mx is None:
            return f'{DIM}(no info msgs){END}'
        s = f'x={mx:+7.3f} y={my:+7.3f} yaw={myaw:+6.1f}°'
        if abs(mx) < 1e-4 and abs(my) < 1e-4 and abs(myaw) < 1e-2:
            return f'{YELLOW}{s}  [IDENTITY]{END}'
        return s

    def _print_block(self, elapsed, s, diag):
        """Print one block to stdout AND mirror into the .log file."""
        sev, msg = diag
        ts = datetime.now().strftime('%H:%M:%S')
        rid = self.rid
        info = s['rtabmap_info']

        lines = []
        lines.append(f'\n{BOLD}┌─[loc_doctor {ts}  T+{elapsed:6.1f}s  '
                     f'tick #{self.tick_n}  /{rid}/]{END}')

        # ── SENSORS ───────────────────────────────────────────────────
        lines.append(f'│ {BOLD}SENSORS{END}')
        lines.append(f'│   camera        rgb={self._fmt_hz(s["cam_rgb_hz"], 10)}  '
                     f'depth={self._fmt_hz(s["cam_depth_hz"], 10)}')
        lines.append(f'│   wheel odom    {self._fmt_hz(s["odom_hz"], 20)} '
                     f'{self._fmt_pose(s["last_odom_pose"])}')
        lines.append(f'│                 {self._fmt_motion(s["odom_delta"], s["odom_moving"])}')
        lines.append(f'│   ekf out       {self._fmt_hz(s["ekf_hz"], 20)} '
                     f'{self._fmt_pose(s["last_ekf_pose"])}')
        lines.append(f'│                 {self._fmt_motion(s["ekf_delta"], s["ekf_moving"])}')

        # ── PROCESSING ────────────────────────────────────────────────
        lines.append(f'│ {BOLD}PROCESSING (rtabmap){END}')
        lines.append(f'│   info rate     {self._fmt_hz(s["info_hz"], 0.5)}')
        lines.append(f'│   inliers       {info.get("loop_inliers") or info.get("reg_inliers") or "-"}'
                     f'  matches={info.get("loop_matches") or info.get("reg_matches") or "-"}'
                     f'  ref_id={info.get("ref_id", "-")}'
                     f'  loop_id={info.get("loop_id", "-")}')
        lines.append(f'│   map_correction {self._fmt_map_correction(info)}')

        # ── OUTPUTS ───────────────────────────────────────────────────
        lines.append(f'│ {BOLD}OUTPUTS{END}')
        lines.append(f'│   /slam/pose    {self._fmt_hz(s["slam_pose_hz"], 0.5)} '
                     f'{self._fmt_count(s["slam_pose_count"])}')
        lines.append(f'│   /loc_pose     {self._fmt_hz(s["loc_pose_hz"],  0.5)} '
                     f'{self._fmt_count(s["loc_pose_count"])}')
        lines.append(f'│   loc_pose σ    {self._fmt_cov(s["last_loc_cov_diag"])}')

        # ── TF ────────────────────────────────────────────────────────
        lines.append(f'│ {BOLD}TF (the chain the GUI follows){END}')
        lines.append(f'│   map        -> {rid}_odom         {self._fmt_tf(s["tf_map_to_odom"], identity_warn=True)}')
        lines.append(f'│   map        -> {rid}_base_link    {self._fmt_tf(s["tf_map_to_base_link"])}')
        lines.append(f'│   {rid}_odom -> {rid}_base_link    {self._fmt_tf(s["tf_odom_to_base_link"])}')

        # ── DIAGNOSIS ─────────────────────────────────────────────────
        color = {'ok': GREEN, 'warn': YELLOW, 'err': RED}[sev]
        prefix = {'ok': '✓ OK', 'warn': '⚠ WARN', 'err': '✗ ERROR'}[sev]
        lines.append(f'└ {color}{BOLD}{prefix}{END}{color}: {msg}{END}')

        for ln in lines:
            print(ln)
        if self.text_file:
            for ln in lines:
                self.text_file.write(strip_ansi(ln) + '\n')


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
        try:
            node._print_session_footer()
        except Exception:
            pass
        for f in (node.jsonl_file, node.text_file):
            if f:
                try:
                    f.close()
                except Exception:
                    pass
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
