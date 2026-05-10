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

--walk mode (formation animation)
---------------------------------
Pass `--walk` to also animate the two robots along the most recently
planned path once "Send Goal" has been clicked in the GUI. The script
polls `path_plan.json` for changes; when a new plan appears, it walks
the virtual centre along the dense waypoints at `--speed` (m/s) and
publishes each robot's pose as `vc_pose ⊕ offset_local` (rotated by
the segment heading). This is the cooperative-formation traversal the
demo wants to show on screen — a substitute for the real EKF + Laplacian
controller pipeline. Both robots stay in lockstep by construction, so
"object falling out of formation" is impossible in fake mode.

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
import json
import math
import os
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


class FormationAnimator:
    """Walk a virtual centre along the latest path_plan.json, then place
    each robot at `vc ⊕ R(vc_heading) · offset_local` so the formation
    moves in lockstep.

    The plan file (written by server.py's /plan handler) is polled by
    mtime — a new plan triggers a fresh walk from the start. Robots
    animate only after they're all "live" (have a pose); the GUI flow
    is set initial pose first, then send goal.
    """

    def __init__(self, plan_path, sims, speed_mps=0.18, ramp_time=0.5,
                 hold_at_goal=2.0):
        self.plan_path     = plan_path
        self.sims          = sims                  # rid → RobotSim
        self.speed         = float(speed_mps)
        self.ramp_time     = float(ramp_time)
        self.hold_at_goal  = float(hold_at_goal)

        # Snapshot the plan file's current mtime at startup so a stale
        # path_plan.json from a previous session does NOT auto-trigger an
        # animation. We only react to a *change* — i.e. a fresh Send Goal
        # in the current session. If the file doesn't exist yet, _mtime
        # stays None and the first plan written will be picked up.
        try:
            self._mtime: float | None = os.path.getmtime(plan_path)
            print(f"[walk] ignoring existing plan at {plan_path} "
                  f"(mtime snapshotted; click Send Goal to start a new walk)")
        except OSError:
            self._mtime = None
        self._wps: list = []          # virtual-centre waypoints [[x,y],...]
        self._cum: list = []          # cumulative arclength (m)
        self._total = 0.0
        self._offsets: dict = {}      # rid → [ox, oy] in vc body frame

        self._walk_start_t: float | None = None
        self._s = 0.0                 # arclength position (m)
        self._done = False
        self._goal_announced = False

    def _all_live(self):
        return all(sim.is_live and sim.pose is not None
                   for sim in self.sims.values())

    def maybe_reload(self):
        try:
            mt = os.path.getmtime(self.plan_path)
        except OSError:
            return
        if self._mtime is not None and mt == self._mtime:
            return
        try:
            with open(self.plan_path) as f:
                plan = json.load(f)
        except (OSError, json.JSONDecodeError):
            return

        vc  = plan.get("virtual_center") or {}
        wps = vc.get("waypoints") or []
        if len(wps) < 2:
            print("[walk] plan has < 2 waypoints — not animating")
            self._mtime = mt
            return

        cum = [0.0]
        for i in range(1, len(wps)):
            dx = wps[i][0] - wps[i - 1][0]
            dy = wps[i][1] - wps[i - 1][1]
            cum.append(cum[-1] + math.hypot(dx, dy))

        offsets = {}
        for r in plan.get("robots", []):
            offsets[r["name"]] = list(r["offset_local"])

        self._mtime    = mt
        self._wps      = wps
        self._cum      = cum
        self._total    = cum[-1]
        self._offsets  = offsets
        self._walk_start_t = None  # arm — actual start gated on _all_live()
        self._s        = 0.0
        self._done     = False
        self._goal_announced = False
        print(f"[walk] new plan loaded: {len(wps)} waypoints, "
              f"{self._total:.2f} m total, "
              f"~{self._total / max(self.speed, 1e-6):.1f} s @ {self.speed:.2f} m/s")

    def _vc_at(self, s):
        """Linear interpolation along arclength → (x, y, heading)."""
        # Locate segment.
        i = 0
        while i + 1 < len(self._cum) and self._cum[i + 1] < s:
            i += 1
        if i + 1 >= len(self._wps):
            i = len(self._wps) - 2
        x0, y0 = self._wps[i]
        x1, y1 = self._wps[i + 1]
        seg_len = max(self._cum[i + 1] - self._cum[i], 1e-9)
        t = max(0.0, min(1.0, (s - self._cum[i]) / seg_len))
        x = x0 + t * (x1 - x0)
        y = y0 + t * (y1 - y0)
        # Look-ahead heading: use the segment direction the VC is *on*
        # right now. With dense (5 cm) waypoints this is already smooth.
        yaw = math.atan2(y1 - y0, x1 - x0)
        return x, y, yaw

    def step(self, now):
        """Advance the virtual centre and write each sim's pose. Returns
        True iff this tick wrote new poses (caller still publishes them
        through the normal post_pose path)."""
        if not self._wps:
            return False
        if not self._all_live():
            return False
        if self._walk_start_t is None:
            self._walk_start_t = now
            print(f"[walk] starting traversal — robots: "
                  f"{', '.join(self.sims.keys())}")

        elapsed = now - self._walk_start_t
        # Trapezoidal arclength: ramp from 0 → speed over ramp_time, then
        # cruise. Closed-form so cadence jitter doesn't accumulate error.
        if elapsed <= self.ramp_time:
            self._s = 0.5 * self.speed * (elapsed ** 2) / max(self.ramp_time, 1e-6)
        else:
            self._s = (0.5 * self.speed * self.ramp_time
                       + self.speed * (elapsed - self.ramp_time))

        if self._s >= self._total:
            self._s = self._total
            if not self._goal_announced:
                self._goal_announced = True
                print(f"[walk] reached goal after {elapsed:.1f} s")
            travel_t = (0.5 * self.ramp_time
                        + (self._total - 0.5 * self.speed * self.ramp_time)
                        / max(self.speed, 1e-6))
            self._done = (elapsed - travel_t) > self.hold_at_goal

        vc_x, vc_y, vc_yaw = self._vc_at(self._s)
        c, s = math.cos(vc_yaw), math.sin(vc_yaw)
        for rid, sim in self.sims.items():
            if sim.pose is None:
                continue
            ox, oy = self._offsets.get(rid, [0.0, 0.0])
            sim.pose["x"]       = vc_x + c * ox - s * oy
            sim.pose["y"]       = vc_y + s * ox + c * oy
            sim.pose["yaw_rad"] = vc_yaw
        return True


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
    p.add_argument("--walk", action="store_true",
                   help="After 'Send Goal' in the GUI, animate both robots "
                        "along the planned path keeping formation. Reads "
                        "the latest plan from --plan-file.")
    p.add_argument("--plan-file",
                   default=os.path.join(os.path.dirname(__file__) or ".",
                                        "path_plan.json"),
                   help="Path to the planner's output JSON. Polled by mtime.")
    p.add_argument("--speed", type=float, default=0.18,
                   help="Cruise speed for --walk, m/s (default 0.18 — "
                        "matches MAX_WHEEL_LINEAR in laplacian_formation_node).")
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

    animator = (FormationAnimator(args.plan_file, sims, speed_mps=args.speed)
                if args.walk else None)

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
    if animator is not None:
        print(f"[fake] walk mode  : ON  ({args.plan_file}, "
              f"speed={args.speed:.2f} m/s)")
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

            # 2. Walk animator: pick up new plans, advance the formation.
            #    Mutates sim.pose in place; the publish step below sends
            #    the updated values out as if SLAM had observed them.
            if animator is not None:
                animator.maybe_reload()
                animator.step(now)

            # 3. Publish for any robot whose delay has elapsed.
            for sim in sims.values():
                if sim.should_publish(now):
                    sim.announce_live_once()
                    post_pose(pose_url, sim)

            time.sleep(period)
    except KeyboardInterrupt:
        print("\n[fake] stopped.")


if __name__ == "__main__":
    main()
