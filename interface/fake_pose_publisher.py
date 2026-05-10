"""
fake_pose_publisher.py — Simulate ros_pose_bridge.py + RTAB-Map for GUI testing.

Bypasses ROS entirely. Watches the server's /set_initial_pose/<robot_id>
mailbox for hints from the GUI's "Set Initial Pose" tool, and a
configurable delay later (default 2 s — simulates RTAB-Map's
re-localization time) starts POSTing a "localized" pose for that robot
to /pose at 10 Hz. The GUI's LOC pill goes LIVE and the box frame
appears, exactly as if the real bridge + RTAB-Map were running.

The hint from the GUI is used purely as a TRIGGER. The POSE actually
published per robot is a fixed value baked into the script (default to
the centres of two robots resting side-by-side on the floor — see
--r1-pose / --r2-pose). This way you always know exactly what should
appear in the GUI regardless of where you clicked.

Fixed poses (defaults)
----------------------
  tb3_0 (R1): x = -0.75 m,  y = -1.20 m,  yaw = 84°
  tb3_1 (R2): x = -0.25 m,  y = -1.20 m,  yaw = 84°
  → 0.50 m apart, both facing +y-ish (84° from +X CCW).

Override either with --r1-pose X Y YAW_DEG / --r2-pose X Y YAW_DEG.
Pass --use-hint-coords to fall back to the legacy "publish wherever
the user clicked" behaviour.

Flow per robot
--------------
  GUI: click 📍 Set Initial Pose, pick R1, click on map, drag yaw, click.
       → POST /set_initial_pose/tb3_0 {x, y, yaw_rad}
  This script polls /set_initial_pose/tb3_0 every tick; when seq bumps
  it schedules `time.time() + delay_sec`. After that, every tick:
       POST /pose body={"robot_id": "tb3_0", "localized": true,
                         x: <fixed>, y: <fixed>, yaw_rad: <fixed>, ...}
  GUI's pill flips R1 → LIVE, R1 box appears at the FIXED coords
  (not where the user clicked).

Re-arms cleanly: every time the user sets a new initial pose for a
robot, that robot goes LOST for `delay_sec` seconds, then comes back
LIVE at the fixed location. Other robots are unaffected.

Usage
-----
    # Default — both robots, default fixed poses, 2 s delay
    python3 fake_pose_publisher.py

    # Custom poses
    python3 fake_pose_publisher.py --r1-pose -1.0 -1.5 0   --r2-pose -0.5 -1.5 0

    # Use whatever coords the user clicked instead of fixed ones
    python3 fake_pose_publisher.py --use-hint-coords

    # Faster simulated re-localization
    python3 fake_pose_publisher.py --delay-sec 0.5

Stop with Ctrl-C.
"""

import argparse
import math
import sys
import time
from datetime import datetime, timezone

try:
    import requests
except ImportError:
    sys.exit("error: install requests with `pip3 install requests`.")


class RobotSim:
    """One simulated robot: tracks the latest initial-pose hint and
    decides whether to post a "localized" pose this tick."""

    def __init__(self, robot_id, delay_sec, fixed_pose=None):
        # fixed_pose: {'x', 'y', 'yaw_rad'} — overrides the hint coords
        # when the robot becomes "localized". None → use hint coords
        # (legacy behaviour, --use-hint-coords).
        self.robot_id      = robot_id
        self.delay_sec     = delay_sec
        self.fixed_pose    = fixed_pose
        self.last_seen_seq = 0
        self.pose          = None        # what we'll publish; set on hint
        self.activate_at   = None        # wall-clock time to start publishing
        self.is_live       = False       # printed log gate

    def maybe_update_from_hint(self, snapshot):
        """Called each poll. snapshot is the JSON the server returned
        for /set_initial_pose/<robot_id>; may be {available: false}."""
        if not snapshot.get("available"):
            return
        seq = int(snapshot.get("seq", 0))
        if seq <= self.last_seen_seq:
            return
        # New hint. Reset, decide which pose to use, schedule activation.
        self.last_seen_seq = seq

        if self.fixed_pose is not None:
            self.pose = dict(self.fixed_pose)
            src = "FIXED"
        else:
            self.pose = {
                "x":       float(snapshot["x"]),
                "y":       float(snapshot["y"]),
                "yaw_rad": float(snapshot.get("yaw_rad", 0.0)),
            }
            src = "HINT"

        self.activate_at = time.time() + self.delay_sec
        self.is_live     = False
        print(f"[fake] {self.robot_id}: hint #{seq} received → publishing "
              f"{src} pose ({self.pose['x']:+.2f}, {self.pose['y']:+.2f}, "
              f"yaw={math.degrees(self.pose['yaw_rad']):+.0f}°) "
              f"in {self.delay_sec:.1f} s")

    def should_publish(self, now):
        return (self.activate_at is not None
                and self.pose is not None
                and now >= self.activate_at)

    def announce_live_once(self):
        if not self.is_live:
            self.is_live = True
            print(f"[fake] {self.robot_id}: now LIVE at "
                  f"({self.pose['x']:+.2f}, {self.pose['y']:+.2f}, "
                  f"yaw={math.degrees(self.pose['yaw_rad']):+.0f}°)")


def post_pose(pose_url, robot, last_match_age_sec=0.5):
    body = {
        "robot_id":           robot.robot_id,
        "localized":          True,
        "x":                  robot.pose["x"],
        "y":                  robot.pose["y"],
        "yaw_rad":            robot.pose["yaw_rad"],
        "yaw_deg":            math.degrees(robot.pose["yaw_rad"]),
        "frame":              "map",
        "last_match_age_sec": float(last_match_age_sec),
        "wall_clock_iso":     datetime.now(timezone.utc).isoformat(),
    }
    try:
        requests.post(pose_url, json=body, timeout=0.5)
    except requests.exceptions.RequestException as e:
        print(f"[!] POST {pose_url} for {robot.robot_id} failed: {e}")


def poll_hint(base_url, robot_id):
    try:
        r = requests.get(f"{base_url}/set_initial_pose/{robot_id}", timeout=0.5)
        if r.status_code == 200:
            return r.json()
    except requests.exceptions.RequestException:
        pass
    return None


def main():
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--server-base", default="http://localhost:5002",
                   help="Server base URL (default http://localhost:5002).")
    p.add_argument("--robots", nargs="+", default=["tb3_0", "tb3_1"],
                   help="Which robot ids to watch (default tb3_0 tb3_1). "
                        "First = R1, second = R2 for --r1-pose / --r2-pose.")
    p.add_argument("--delay-sec", type=float, default=2.0,
                   help="Seconds to wait after an initial-pose hint before "
                        "publishing 'localized: true'. Simulates RTAB-Map "
                        "re-localization time (default 2.0).")
    p.add_argument("--rate-hz", type=float, default=10.0,
                   help="POST cadence per active robot, Hz (default 10).")

    p.add_argument("--r1-pose", nargs=3, type=float,
                   default=[-0.75, -1.20, 84.0],
                   metavar=("X", "Y", "YAW_DEG"),
                   help="Fixed pose for the first robot in --robots "
                        "(default: -0.75 -1.20 84). X/Y in metres, yaw in "
                        "degrees CCW from +X.")
    p.add_argument("--r2-pose", nargs=3, type=float,
                   default=[-0.25, -1.20, 84.0],
                   metavar=("X", "Y", "YAW_DEG"),
                   help="Fixed pose for the second robot in --robots "
                        "(default: -0.25 -1.20 84).")
    p.add_argument("--use-hint-coords", action="store_true",
                   help="Publish at the hint coordinates (wherever the user "
                        "clicked) instead of the fixed --r1-pose / --r2-pose. "
                        "Useful when you want to drive the box around the map.")
    args = p.parse_args()

    base_url = args.server_base.rstrip("/")
    pose_url = f"{base_url}/pose"
    period   = 1.0 / args.rate_hz

    # Build the per-robot fixed-pose map. Position 0 in --robots maps to
    # --r1-pose, position 1 to --r2-pose. Robots beyond that get no fixed
    # pose (they fall back to hint coords) — adjust this script if you
    # add a third robot.
    fixed_poses_by_id = {}
    if not args.use_hint_coords:
        cli_poses = [args.r1_pose, args.r2_pose]
        for i, rid in enumerate(args.robots):
            if i < len(cli_poses):
                fp = cli_poses[i]
                fixed_poses_by_id[rid] = {
                    "x":       float(fp[0]),
                    "y":       float(fp[1]),
                    "yaw_rad": math.radians(float(fp[2])),
                }

    sims = {rid: RobotSim(rid, args.delay_sec, fixed_poses_by_id.get(rid))
            for rid in args.robots}

    print(f"[fake] watching: {', '.join(args.robots)}")
    print(f"[fake] hint queue : {base_url}/set_initial_pose/<robot_id>")
    print(f"[fake] pose post  : {pose_url}")
    print(f"[fake] delay      : {args.delay_sec:.2f} s after each hint")
    print(f"[fake] rate       : {args.rate_hz:.1f} Hz per active robot")
    if args.use_hint_coords:
        print(f"[fake] mode       : HINT coords (publishing wherever the user clicks)")
    else:
        print(f"[fake] mode       : FIXED coords (hint is just the trigger)")
        for rid, fp in fixed_poses_by_id.items():
            print(f"[fake]   {rid}: x={fp['x']:+.3f}  y={fp['y']:+.3f}  "
                  f"yaw={math.degrees(fp['yaw_rad']):+.1f}°")
    print(f"[fake] Use the GUI's '📍 Set Initial Pose' tool to drive me. "
          f"Ctrl-C to stop.\n")

    try:
        while True:
            now = time.time()

            # 1. Poll for new initial-pose hints (re-arm when seq bumps).
            for rid, sim in sims.items():
                snap = poll_hint(base_url, rid)
                if snap is not None:
                    sim.maybe_update_from_hint(snap)

            # 2. Publish for any robot whose delay has elapsed.
            for sim in sims.values():
                if sim.should_publish(now):
                    sim.announce_live_once()
                    post_pose(pose_url, sim)

            time.sleep(period)
    except KeyboardInterrupt:
        print("\n[fake] stopped.")


if __name__ == "__main__":
    main()
