"""
sim_navigator.py
----------------
ROS-free stand-in for navigation_node.py.

Pretends to be the robot driving toward the goal pose: reads the planned
path, runs the same APF + velocity-ramp + slowdown math friend's
navigation_node.py uses, and POSTs its simulated pose to the Flask
server at 20 Hz. The browser's live-pose indicator moves as if a real
robot were running, so the whole pipeline (Set Pose → Goal → Plan Path
→ Robot Drives) can be tested before the ROS bridge is online.

When the real robot comes up, navigation_node.py takes over and this
script becomes obsolete (no rewrite needed — it shares the same math).

Workflow
--------
    Terminal 1:  python3 server.py --map map.json --port 5002
    Terminal 2:  python3 sim_navigator.py
    Browser:     http://localhost:5002/  (index.html)

    1. Click "Sim Pose" in the GUI to spawn the robot somewhere.
    2. Click in the 3D view to set a goal pose.
    3. Click "Plan Path" — A* writes path_plan.json.
    4. The simulator picks up the new plan and starts driving.
    5. Browser shows the robot envelope sliding along the yellow path.

Notes
-----
- Constants exactly mirror navigation_node.py so behaviour matches.
- Obstacle repulsion is sourced from the occupancy grid (g==2 cells)
  instead of /scan, but the math is identical.
- New plans (bumped seq) are picked up automatically — you can re-plan
  while the sim is running and it'll re-target.

Stop with Ctrl-C.
"""

import json
import math
import time

import numpy as np
import requests
from scipy.spatial import cKDTree

# ── Control constants — copied verbatim from navigation_node.py ──────────
# K_REP = 0: the planner already inflated the path away from every map
# obstacle (formation_radius cascade, 0.27–0.45 m clearance). Adding APF
# repulsion in the sim controller creates a local-minimum oscillation
# whenever the path runs near a wall — the goal-attraction and wall-
# repulsion balance to ~zero net force, the robot stalls and dithers.
# `navigation_node.py` on the real robot uses APF on `/navigation/obstacles`
# (external dynamic objects), NOT on the static map walls, so this matches
# its semantics. If you ever publish dynamic obstacles into the sim,
# raise K_REP back up — but use a list-of-points input, not the grid.
K_REP        = 0.00    # repulsive gain — disabled (see comment above)
D_REP        = 0.50    # obstacle influence radius [m]  (unused while K_REP=0)
MAX_LINEAR   = 0.18    # m/s
MAX_ANGULAR  = 0.45    # rad/s
SLOW_RADIUS  = 0.40    # m  — start ramping down within this distance
ACC_MAX      = 0.15    # m/s²
ALPHA_MAX    = 0.25    # rad/s²
DT           = 0.05    # s  (20 Hz control loop)
# Heading P-gain — must match navigation_node.K_HEADING. The previous
# implementation used `yaw_err / DT` (gain ≈ 20 rad/s per rad of error),
# which made the sim 7× more aggressive than the real robot and hid yaw
# overshoot bugs. The real navigation_node uses K_HEADING = 3.0.
K_HEADING    = 3.0     # rad/s per rad of yaw error

# ── Two-phase arrival ───────────────────────────────────────────────────
# Phase 1 (DRIVE):  pure-pursuit toward goal XY until within GOAL_TOL_POS.
# Phase 2 (ROTATE): in-place rotate until yaw_err < YAW_TOL.
GOAL_TOL_POS = 0.05    # m   — XY arrival tolerance
YAW_TOL      = 0.05    # rad — final yaw tolerance (~3°)

# ── Pure-pursuit follower ───────────────────────────────────────────────
# Lookahead distance: the controller steers toward a point this far ahead
# on the path from the closest projection of the robot, instead of chasing
# discrete waypoints. This naturally smooths through corners and pulls
# the robot back onto the line if it drifts (prevents the "slippery" feel).
LOOKAHEAD    = 0.20    # m

SERVER       = "http://localhost:5002"
PLAN_FILE    = "path_plan.json"
SIM_ROBOT_ID = "tb3_1_sim"


def get_json(path):
    try:
        r = requests.get(f"{SERVER}{path}", timeout=2.0)
        return r.json() if r.ok else None
    except requests.RequestException:
        return None


def post_pose(x, y, yaw):
    """Push the simulated pose so the browser's live-pose indicator moves."""
    payload = {
        "robot_id":  SIM_ROBOT_ID,
        "localized": True,
        "x":         float(x),
        "y":         float(y),
        "yaw_rad":   float(yaw),
        "yaw_deg":   float(math.degrees(yaw)),
        "frame":     "map",
        "from_sim":  True,
    }
    try:
        requests.post(f"{SERVER}/pose", json=payload, timeout=1.0)
    except requests.RequestException:
        pass  # server temporarily down, just keep moving


def load_plan_file():
    """Read path_plan.json from disk (server.py writes it there)."""
    try:
        with open(PLAN_FILE) as f:
            return json.load(f)
    except (FileNotFoundError, json.JSONDecodeError):
        return None


def pure_pursuit_target(path, robot_xy, lookahead):
    """Find a point `lookahead` meters ahead of the robot's closest
    projection onto the polyline `path`. Returns (tx, ty).

    This is the standard pure-pursuit algorithm: it always steers toward
    a point that is a fixed distance ahead on the path, so the robot
    naturally curves through waypoints and is pulled back to the line if
    it drifts off — instead of chasing discrete waypoints which causes
    heading snap and orbital motion."""
    if len(path) < 2:
        return path[-1]

    # Step A: find the closest point projection (best segment + parameter t)
    best_seg = 0
    best_t   = 0.0
    best_d2  = float("inf")
    rx, ry   = robot_xy
    for i in range(len(path) - 1):
        ax, ay = path[i]
        bx, by = path[i + 1]
        dx, dy = bx - ax, by - ay
        seg_len2 = dx * dx + dy * dy
        if seg_len2 < 1e-9:
            t = 0.0
        else:
            t = ((rx - ax) * dx + (ry - ay) * dy) / seg_len2
            t = max(0.0, min(1.0, t))
        px = ax + t * dx
        py = ay + t * dy
        d2 = (rx - px) ** 2 + (ry - py) ** 2
        if d2 < best_d2:
            best_d2  = d2
            best_seg = i
            best_t   = t

    # Step B: walk forward `lookahead` meters from (best_seg, best_t).
    remaining = lookahead
    seg, t    = best_seg, best_t
    while seg < len(path) - 1:
        ax, ay = path[seg]
        bx, by = path[seg + 1]
        dx, dy = bx - ax, by - ay
        seg_len = math.hypot(dx, dy)
        rem_in_seg = (1.0 - t) * seg_len
        if remaining <= rem_in_seg:
            new_t = t + remaining / seg_len if seg_len > 1e-9 else 0.0
            return (ax + new_t * dx, ay + new_t * dy)
        remaining -= rem_in_seg
        seg += 1
        t = 0.0
    # Walked off the end — aim at the goal.
    return path[-1]


def wrap_pi(a):
    """Wrap an angle to (-pi, pi]."""
    return (a + math.pi) % (2 * math.pi) - math.pi


def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


def main():
    print("[sim] connecting to server...")
    map_data = get_json("/map")
    if map_data is None:
        print(f"[sim] FATAL: cannot reach {SERVER}/map. Is server.py running?")
        return

    grid = np.asarray(map_data["grid"], dtype=np.int32)
    meta = map_data["metadata"]
    res, mn_x, mn_y = meta["resolution"], meta["min_x"], meta["min_y"]

    # Obstacle KD-tree: every g==2 cell becomes a point at its world center.
    obs_rows, obs_cols = np.where(grid == 2)
    obs_xy = np.column_stack([
        mn_x + (obs_cols + 0.5) * res,
        mn_y + (obs_rows + 0.5) * res,
    ]).astype(np.float32)
    obs_tree = cKDTree(obs_xy) if len(obs_xy) else None
    print(f"[sim] map loaded: grid {grid.shape}, "
          f"{len(obs_xy)} obstacle cells")

    # Initial pose: read whatever the GUI most recently posted.
    pose = get_json("/pose")
    if not pose or not pose.get("available") or not pose.get("localized"):
        print("[sim] FATAL: no localized pose in /pose. "
              "Click 'Sim Pose' or 'Set Initial Pose' in the GUI first.")
        return

    x   = float(pose["x"])
    y   = float(pose["y"])
    yaw = float(pose.get("yaw_rad", pose.get("yaw", 0.0)))
    # Holonomic state: (vx, vy) in WORLD frame, w = angular rate.
    # Translation direction is decoupled from yaw (true swerve crab).
    vx, vy, w = 0.0, 0.0, 0.0

    print(f"[sim] starting at ({x:+.2f}, {y:+.2f}, "
          f"{math.degrees(yaw):+.1f}°)")
    print("[sim] waiting for a plan... (click Plan Path in the GUI)")

    last_plan_seq  = -1
    arrived_at_seq = -1
    waypoints      = []
    goal_yaw       = 0.0      # radians, target heading at goal
    start_yaw_plan = 0.0      # yaw at the moment the plan was issued (for smooth_to_goal lerp)
    path_total_len = 0.0      # arc-length sum of `waypoints` at plan-receipt time
    yaw_policy     = "tangent"  # "tangent" | "free" | "smooth_to_goal"
    phase          = "IDLE"   # IDLE → DRIVE → ROTATE → DONE
    log_throttle   = 0

    try:
        while True:
            t0 = time.monotonic()

            # ── Pick up new plans on seq change ─────────────────────────
            plan = load_plan_file()
            if plan is not None:
                seq = plan.get("seq", 0)
                if seq != last_plan_seq:
                    last_plan_seq = seq
                    vc       = plan.get("virtual_center", {})
                    wp_list  = vc.get("waypoints", [])
                    plan_md  = plan.get("metadata", {})
                    dist_m   = plan_md.get("vc_distance")
                    snap_g   = plan_md.get("goal_snap_dist", 0.0)
                    snap_s   = plan_md.get("start_snap_dist", 0.0)
                    goal_yaw = float(plan_md.get("goal_yaw") or 0.0)
                    dist_s   = (f"{dist_m:.2f} m"
                                if isinstance(dist_m, (int, float)) else "n/a")
                    if wp_list:
                        cand_goal = (float(wp_list[-1][0]),
                                     float(wp_list[-1][1]))
                        d_to_goal = math.hypot(cand_goal[0] - x,
                                               cand_goal[1] - y)
                        if d_to_goal < GOAL_TOL_POS:
                            print(f"[sim] plan #{seq}: goal is only "
                                  f"{d_to_goal*100:.1f} cm from current "
                                  f"pose — robot WILL NOT MOVE.")
                            if snap_g > 0.05:
                                print(f"[sim]   reason: A* snapped your "
                                      f"goal {snap_g*100:.0f} cm "
                                      f"(inflation halo). Click further "
                                      f"from any wall.")
                            waypoints = []
                            phase     = "IDLE"
                        else:
                            waypoints = [(float(p[0]), float(p[1]))
                                         for p in wp_list]
                            # Cache total arc length for smooth_to_goal lerp
                            path_total_len = 0.0
                            for i in range(1, len(waypoints)):
                                path_total_len += math.hypot(
                                    waypoints[i][0] - waypoints[i - 1][0],
                                    waypoints[i][1] - waypoints[i - 1][1])
                            # Snapshot start yaw for smooth_to_goal lerp
                            start_yaw_plan = yaw
                            vx, vy, w = 0.0, 0.0, 0.0
                            phase = "DRIVE"
                            # Read yaw_policy from plan metadata
                            yaw_policy = str(plan_md.get("yaw_policy",
                                                         "tangent"))
                            print(f"[sim]   yaw_policy={yaw_policy}")
                            print(f"[sim] new plan #{seq}: "
                                  f"{len(waypoints)} waypoints, "
                                  f"distance {dist_s}, "
                                  f"goal_yaw={math.degrees(goal_yaw):+.0f}°")
                            if snap_s > 0.05 or snap_g > 0.05:
                                print(f"[sim]   ⚠ start snapped "
                                      f"{snap_s*100:.0f} cm, "
                                      f"goal snapped {snap_g*100:.0f} cm "
                                      f"(inflation halo)")
                    else:
                        print(f"[sim] plan #{seq}: empty path "
                              f"(unreachable goal). Robot will not move.")
                        waypoints = []
                        phase     = "IDLE"

            # ── IDLE: sit still, keep posting current pose ──────────────
            if phase == "IDLE" or not waypoints:
                post_pose(x, y, yaw)
                time.sleep(DT)
                continue

            gx, gy = waypoints[-1]
            d_goal = math.hypot(gx - x, gy - y)

            # ── PHASE TRANSITIONS ──────────────────────────────────────
            if phase == "DRIVE" and d_goal < GOAL_TOL_POS:
                if yaw_policy == "free":
                    # Skip ROTATE — yaw isn't enforced under "free".
                    if arrived_at_seq != last_plan_seq:
                        print(f"[sim] ✓ ARRIVED (free yaw) at "
                              f"({x:+.3f}, {y:+.3f}, "
                              f"{math.degrees(yaw):+.1f}°) for "
                              f"plan #{last_plan_seq}")
                        arrived_at_seq = last_plan_seq
                    phase = "DONE"
                    vx, vy, w = 0.0, 0.0, 0.0
                else:
                    print(f"[sim] ✓ reached XY ({x:+.3f}, {y:+.3f}) "
                          f"— now rotating to "
                          f"{math.degrees(goal_yaw):+.0f}°")
                    phase = "ROTATE"
                    vx, vy, w = 0.0, 0.0, 0.0   # full stop before rotating

            if phase == "ROTATE":
                yaw_err = wrap_pi(goal_yaw - yaw)
                if abs(yaw_err) < YAW_TOL:
                    if arrived_at_seq != last_plan_seq:
                        print(f"[sim] ✓ ARRIVED — pose "
                              f"({x:+.3f}, {y:+.3f}, "
                              f"{math.degrees(yaw):+.1f}°) "
                              f"for plan #{last_plan_seq}")
                        arrived_at_seq = last_plan_seq
                    phase = "DONE"
                    vx, vy, w = 0.0, 0.0, 0.0

            # ── DRIVE phase: HOLONOMIC pure-pursuit ─────────────────────
            # Output is (target_vx, target_vy, target_w) in WORLD frame.
            # The robot translates in any direction independently of its
            # yaw — true swerve crab. Yaw is steered toward the motion
            # direction (when moving fast) or the goal yaw (when slow);
            # this matches navigation_node._apf_velocity exactly.
            if phase == "DRIVE":
                # Lookahead point on the path
                tx, ty = pure_pursuit_target(waypoints, (x, y), LOOKAHEAD)

                # ── Attractive direction: world-frame unit vector ──────
                dx_t, dy_t = tx - x, ty - y
                d_t = math.hypot(dx_t, dy_t)
                if d_t > 1e-6:
                    f_att_x = dx_t / d_t
                    f_att_y = dy_t / d_t
                else:
                    f_att_x = f_att_y = 0.0

                # ── Repulsive force: Khatib, in world frame ────────────
                # The path is already inflated-clear by A*, so K_REP=0.10
                # is a safety net only.
                f_rep_x = f_rep_y = 0.0
                if obs_tree is not None and K_REP > 0:
                    for i in obs_tree.query_ball_point((x, y), D_REP):
                        ox, oy = obs_xy[i]
                        ddx, ddy = x - ox, y - oy
                        d_obs = math.hypot(ddx, ddy)
                        if d_obs < 1e-3:
                            continue
                        mag = K_REP * (1.0 / d_obs - 1.0 / D_REP) / (d_obs * d_obs)
                        f_rep_x += mag * ddx / d_obs
                        f_rep_y += mag * ddy / d_obs

                # ── Combine and pick speed ─────────────────────────────
                f_tot_x = f_att_x + f_rep_x
                f_tot_y = f_att_y + f_rep_y
                f_mag = math.hypot(f_tot_x, f_tot_y)

                if f_mag < 1e-6:
                    target_vx = target_vy = 0.0
                else:
                    speed = MAX_LINEAR
                    # Slow down only when approaching the FINAL goal
                    # (Tier 1: don't slow at every densified waypoint).
                    if d_goal < SLOW_RADIUS:
                        speed *= d_goal / SLOW_RADIUS
                    target_vx = (speed / f_mag) * f_tot_x
                    target_vy = (speed / f_mag) * f_tot_y

                # ── Yaw control: dispatch on yaw_policy ────────────────
                # "tangent": face direction of motion when fast, goal_yaw
                #            when slow. Matches navigation_node.py.
                # "free":    don't actively control yaw. Robot keeps
                #            whatever heading it currently has — useful
                #            for transport where payload orientation is
                #            decoupled from motion direction (true crab).
                # "smooth_to_goal": lerp from initial yaw to goal_yaw
                #            based on path progress. (Simple linear lerp.)
                v_mag = math.hypot(target_vx, target_vy)
                if yaw_policy == "free":
                    target_w = 0.0
                else:
                    if yaw_policy == "tangent":
                        if v_mag > 0.05:
                            desired_yaw = math.atan2(target_vy, target_vx)
                        else:
                            desired_yaw = goal_yaw
                    else:  # "smooth_to_goal"
                        # Lerp from `start_yaw_plan` (yaw at plan-issue time)
                        # to `goal_yaw` proportionally to arc-length progress
                        # along the planned path. Progress is approximated as
                        # 1 - d_goal/total_len so the yaw rotates smoothly
                        # over the duration of the trip rather than snapping.
                        if path_total_len > 1e-3:
                            progress = clamp(
                                1.0 - (d_goal / path_total_len), 0.0, 1.0)
                        else:
                            progress = 1.0
                        # Shortest-arc interpolation (handle ±π wrap)
                        delta = wrap_pi(goal_yaw - start_yaw_plan)
                        desired_yaw = wrap_pi(start_yaw_plan + delta * progress)
                    yaw_err = wrap_pi(desired_yaw - yaw)
                    # P-control with K_HEADING — must match navigation_node.
                    target_w = clamp(K_HEADING * yaw_err,
                                     -MAX_ANGULAR, MAX_ANGULAR)

            # ── ROTATE phase: spin in place to match goal yaw ──────────
            elif phase == "ROTATE":
                target_vx = target_vy = 0.0
                target_w  = clamp(K_HEADING * wrap_pi(goal_yaw - yaw),
                                  -MAX_ANGULAR, MAX_ANGULAR)

            # ── DONE phase: hold position ──────────────────────────────
            else:  # "DONE"
                target_vx, target_vy, target_w = 0.0, 0.0, 0.0

            # ── Acceleration / jerk limit ─────────────────────────────
            # Vector ramp on (vx, vy) — the limit is on the magnitude of
            # the velocity *change*, not per-axis. Matches navigation_node.
            dvx = target_vx - vx
            dvy = target_vy - vy
            dv_mag = math.hypot(dvx, dvy)
            max_dv = ACC_MAX * DT
            if dv_mag > max_dv:
                vx += (max_dv / dv_mag) * dvx
                vy += (max_dv / dv_mag) * dvy
            else:
                vx, vy = target_vx, target_vy
            w += clamp(target_w - w, -ALPHA_MAX * DT, ALPHA_MAX * DT)

            # ── Integrate HOLONOMIC kinematics (3-DoF: vx, vy world frame + w) ──
            # The robot can translate in ANY direction regardless of yaw.
            # NOT unicycle: x is NOT tied to cos(yaw), y is NOT tied to sin(yaw).
            x   += vx * DT
            y   += vy * DT
            yaw  = wrap_pi(yaw + w * DT)

            post_pose(x, y, yaw)

            # Throttled status line every ~1 s
            log_throttle += 1
            if log_throttle >= 20:
                log_throttle = 0
                yaw_err_deg = math.degrees(wrap_pi(goal_yaw - yaw))
                v_mag = math.hypot(vx, vy)
                print(f"[sim] {phase:6s} pos=({x:+.2f}, {y:+.2f}) "
                      f"v=({vx:+.2f},{vy:+.2f})|{v_mag:.2f} m/s  "
                      f"w={w:+.2f} rad/s  "
                      f"d_goal={d_goal:.2f} m  "
                      f"yaw_err={yaw_err_deg:+.0f}°")

            # Maintain real-time tick.
            elapsed = time.monotonic() - t0
            if elapsed < DT:
                time.sleep(DT - elapsed)

    except KeyboardInterrupt:
        print("\n[sim] stopped by user")


if __name__ == "__main__":
    main()
