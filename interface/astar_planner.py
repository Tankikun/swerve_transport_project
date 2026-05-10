"""
astar_planner.py
----------------
A* global path planner for the leader robot, with:

  1. Robot-footprint inflation of the occupancy grid
     (so the planner knows the conveyor is ~30 × 30 cm, not a point)
  2. 8-connected A* with Euclidean heuristic on the inflated grid
  3. APF (artificial potential field) gradient-descent smoothing —
     resamples the A* result to uniform spacing, then iteratively
     pulls each interior waypoint toward its neighbors (smoothing)
     and away from obstacles via the distance-transform gradient.
     The endpoints stay pinned. The result is a dense, U-shaped curve
     instead of the V the line-of-sight simplifier used to produce.
  4. Two-robot visualization: leader + follower at formation offset

Map format (friend's Z-up pipeline):
  grid[y_idx][x_idx] ∈ {0=unknown, 1=ground, 2=obstacle}
  cell -> world: x = min_x + col*res,  y = min_y + row*res

Usage:
  /usr/bin/python3 astar_planner.py
  /usr/bin/python3 astar_planner.py --start -4.5 -3.5 --goal -2.5 -1.0
  /usr/bin/python3 astar_planner.py --robot-radius 0.20 --formation 0.5
"""

import argparse
import heapq
import json
import math
from pathlib import Path

import numpy as np
from scipy import ndimage


# ────────────────────────────────────────────────────────────────────────────
# Coordinate conversions
# ────────────────────────────────────────────────────────────────────────────

def world_to_cell(x, y, meta):
    """World (x, y) -> grid cell (col, row). Returns (None, None) if out of bounds."""
    col = int((x - meta['min_x']) / meta['resolution'])
    row = int((y - meta['min_y']) / meta['resolution'])
    if 0 <= col < meta['grid_width'] and 0 <= row < meta['grid_height']:
        return col, row
    return None, None


def cell_to_world(col, row, meta):
    """Grid cell (col, row) -> world (x, y) (cell center)."""
    x = meta['min_x'] + (col + 0.5) * meta['resolution']
    y = meta['min_y'] + (row + 0.5) * meta['resolution']
    return x, y


# ────────────────────────────────────────────────────────────────────────────
# Obstacle inflation: dilate by robot bounding radius
# ────────────────────────────────────────────────────────────────────────────

def inflate_grid(grid, res, robot_radius):
    """Return a binary obstacle mask dilated by `robot_radius` meters.

    Only TRUE obstacle cells (g==2) are inflated. 'Unknown' cells (g==0) are
    NOT treated as blocked — there are unknown patches inside indoor rooms
    where the camera didn't see (under furniture, occluded floor) and we
    don't want to block planning through them.

    For outdoor / large-area maps, callers can additionally OR with (g==0)
    before passing the grid in."""
    g = np.asarray(grid, dtype=np.int32)
    blocked  = (g == 2)
    iters    = max(1, int(math.ceil(robot_radius / res)))
    inflated = ndimage.binary_dilation(blocked, iterations=iters)
    print(f"[inflate] radius={robot_radius:.2f} m  iterations={iters} cells "
          f"(cell={res:.2f} m)")
    print(f"[inflate] obstacle cells: {blocked.sum()} -> {inflated.sum()} "
          f"(grew by {inflated.sum() - blocked.sum()})")
    return inflated


# ────────────────────────────────────────────────────────────────────────────
# A* on a 2D occupancy grid
# ────────────────────────────────────────────────────────────────────────────

# 8-connected neighborhood + sqrt(2) cost for diagonals
NEIGHBORS = [
    (+1,  0, 1.0), (-1,  0, 1.0), ( 0, +1, 1.0), ( 0, -1, 1.0),
    (+1, +1, math.sqrt(2)), (+1, -1, math.sqrt(2)),
    (-1, +1, math.sqrt(2)), (-1, -1, math.sqrt(2)),
]


def astar(blocked, start, goal):
    """Standard A* with Euclidean heuristic on a binary blocked-mask grid."""
    H, W = blocked.shape
    sc, sr = start
    gc, gr = goal
    if blocked[sr, sc]:
        raise ValueError(f"start cell ({sc},{sr}) is blocked")
    if blocked[gr, gc]:
        raise ValueError(f"goal cell ({gc},{gr}) is blocked")

    def h(c, r):
        return math.hypot(c - gc, r - gr)

    open_heap = [(h(sc, sr), 0.0, (sc, sr))]
    came_from = {}
    g_score   = {(sc, sr): 0.0}
    closed    = set()

    while open_heap:
        f, g, (c, r) = heapq.heappop(open_heap)
        if (c, r) in closed:
            continue
        if (c, r) == (gc, gr):
            path = [(c, r)]
            while (c, r) in came_from:
                c, r = came_from[(c, r)]
                path.append((c, r))
            path.reverse()
            print(f"[astar] expanded {len(closed)} cells, path length {len(path)}")
            return path
        closed.add((c, r))
        for dc, dr, step in NEIGHBORS:
            nc, nr = c + dc, r + dr
            if not (0 <= nc < W and 0 <= nr < H):
                continue
            if blocked[nr, nc]:
                continue
            if dc != 0 and dr != 0:
                if blocked[r, nc] and blocked[nr, c]:
                    continue
            tentative = g + step
            if tentative < g_score.get((nc, nr), float('inf')):
                g_score[(nc, nr)] = tentative
                came_from[(nc, nr)] = (c, r)
                heapq.heappush(open_heap, (tentative + h(nc, nr), tentative, (nc, nr)))

    print(f"[astar] no path found (expanded {len(closed)} cells)")
    return None


# ────────────────────────────────────────────────────────────────────────────
# APF gradient-descent smoothing
# ────────────────────────────────────────────────────────────────────────────
#
# The A* result hugs the inflated obstacle boundary in 1-cell zigzags.
# The old line-of-sight simplifier collapsed it into 2–3 sharp waypoints
# (a V), which is hard for the formation controller to track and looks
# bad in the GUI. Here we instead:
#
#   1. resample the A* path to uniform spacing in world frame
#   2. precompute distance-to-nearest-obstacle via EDT
#   3. iterate per interior waypoint w_i:
#        smoothing = α · (½(w_{i-1} + w_{i+1}) − w_i)         // pull to midpoint
#        repulsion = β · (clearance − d) · ∇d̂   if d < clearance, capped
#        anchor    = γ · (a_i − w_i)                          // hold homotopy
#      then a hard reject if the step would land in a blocked cell.
#   4. stop when max per-step displacement < ε
#
# Endpoints (start, goal) stay pinned. The anchor term keeps the result
# in the same homotopy class as A* so the smoother can't tunnel through
# a wall. Per-iteration displacement is bounded ≤ smooth + ½·res + anchor,
# so divergence is impossible and convergence is monotone for sane gains.

def apf_smooth_path(path_cells, blocked, meta, formation_radius, *,
                    target_spacing=0.05,
                    smooth_w=0.50,
                    repel_w=0.05,
                    anchor_w=0.05,
                    max_iters=200,
                    conv_eps=5e-4):
    """Smooth an A*-found path with APF-style relaxation.

    `path_cells`: list of (col, row) from astar(). `blocked`: bool grid
    used to compute the obstacle distance field. `formation_radius` is
    the inflation radius (m) — repulsion targets a clearance of 0.75·R
    inside that buffer so the smoothed path drifts a little toward the
    middle of the corridor instead of riding the boundary.

    Returns a list of [x, y] in world metres, ~`target_spacing` apart.
    """
    res = float(meta['resolution'])
    if not path_cells:
        return []

    pts = np.array([cell_to_world(c, r, meta) for c, r in path_cells],
                   dtype=float)
    if len(pts) < 2:
        return [list(p) for p in pts]

    # ── 1. Resample to uniform spacing along arclength ────────────────
    seg_lens = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    cum      = np.concatenate([[0.0], np.cumsum(seg_lens)])
    total    = float(cum[-1])
    if total < target_spacing * 1.5:
        return [list(pts[0]), list(pts[-1])]

    n_samples = max(3, int(round(total / target_spacing)) + 1)
    s_new     = np.linspace(0.0, total, n_samples)
    new_pts          = np.zeros((n_samples, 2))
    new_pts[:, 0]    = np.interp(s_new, cum, pts[:, 0])
    new_pts[:, 1]    = np.interp(s_new, cum, pts[:, 1])
    anchors          = new_pts.copy()

    # ── 2. Distance-to-nearest-obstacle in metres ─────────────────────
    # distance_transform_edt(input) gives every non-zero pixel its
    # distance to the nearest zero pixel. Pass `~blocked` so every FREE
    # cell gets its distance to the nearest BLOCKED cell. Inside any
    # blocked cell d == 0.
    dt   = ndimage.distance_transform_edt(~blocked).astype(float) * res
    H, W = blocked.shape

    def sample_dt(x, y):
        cx = (x - meta['min_x']) / res - 0.5
        cy = (y - meta['min_y']) / res - 0.5
        c0 = max(0, min(W - 2, int(math.floor(cx))))
        r0 = max(0, min(H - 2, int(math.floor(cy))))
        fx = max(0.0, min(1.0, cx - c0))
        fy = max(0.0, min(1.0, cy - r0))
        v00 = dt[r0,     c0]
        v10 = dt[r0,     c0 + 1]
        v01 = dt[r0 + 1, c0]
        v11 = dt[r0 + 1, c0 + 1]
        return ((1 - fx) * (1 - fy) * v00 + fx * (1 - fy) * v10 +
                (1 - fx) * fy       * v01 + fx * fy       * v11)

    eps_h = res * 0.5

    def grad_dt(x, y):
        gx = (sample_dt(x + eps_h, y) - sample_dt(x - eps_h, y)) / (2 * eps_h)
        gy = (sample_dt(x, y + eps_h) - sample_dt(x, y - eps_h)) / (2 * eps_h)
        return gx, gy

    def cell_blocked(x, y):
        col = int((x - meta['min_x']) / res)
        row = int((y - meta['min_y']) / res)
        if 0 <= col < W and 0 <= row < H:
            return bool(blocked[row, col])
        return True   # off-grid is "blocked" — never step out of the map

    # Stay this far from any obstacle. A* already guarantees the input
    # cells satisfy d ≥ formation_radius (because the grid was inflated
    # by formation_radius), so the smoother only has to keep them there.
    clearance = 0.75 * float(formation_radius)
    max_repel_step = res                # hard cap, ≤ 1 cell per iteration

    # ── 3. Iterative relaxation ───────────────────────────────────────
    pts = new_pts.copy()
    last_disp = float('inf')
    for it in range(max_iters):
        prev = pts.copy()
        # Endpoints (i=0 and i=-1) stay pinned.
        for i in range(1, len(pts) - 1):
            x, y   = pts[i]
            xp, yp = pts[i - 1]
            xn, yn = pts[i + 1]

            sx = smooth_w * ((xp + xn) * 0.5 - x)
            sy = smooth_w * ((yp + yn) * 0.5 - y)

            d = sample_dt(x, y)
            if d < clearance:
                gx, gy = grad_dt(x, y)
                gn = math.hypot(gx, gy)
                if gn > 1e-9:
                    violation = clearance - max(d, 0.0)
                    mag = min(repel_w * violation, max_repel_step)
                    rx = mag * gx / gn
                    ry = mag * gy / gn
                else:
                    rx = ry = 0.0
            else:
                rx = ry = 0.0

            ax = anchor_w * (anchors[i, 0] - x)
            ay = anchor_w * (anchors[i, 1] - y)

            x_new = x + sx + rx + ax
            y_new = y + sy + ry + ay
            # Hard reject: never step into a blocked cell.
            if cell_blocked(x_new, y_new):
                continue
            pts[i, 0] = x_new
            pts[i, 1] = y_new

        last_disp = float(np.max(np.linalg.norm(pts - prev, axis=1)))
        if last_disp < conv_eps:
            print(f"[apf-smooth] converged after {it + 1} iters "
                  f"(max_disp={last_disp * 1000:.2f} mm)")
            break
    else:
        print(f"[apf-smooth] hit max_iters={max_iters} "
              f"(max_disp={last_disp * 1000:.2f} mm)")

    print(f"[apf-smooth] {len(path_cells)} A* cells -> "
          f"{len(pts)} smoothed waypoints (~{target_spacing*100:.0f} cm spacing)")
    return [list(p) for p in pts]


# ────────────────────────────────────────────────────────────────────────────
# Plan computation (callable from server.py)
# ────────────────────────────────────────────────────────────────────────────

def compute_plan(map_data: dict, start: tuple, goal: tuple,
                 formation_radius: float = 0.45,
                 robots=None) -> dict:
    """Plan ONE path for the virtual center; the formation moves together.

    Returns the plan dict that server.py serializes to path_plan.json.
    The GUI consumes `virtual_center.waypoints` (list of [x, y]) to draw
    dots/lines on the floor.
    """
    if robots is None:
        robots = [
            {"name": "leader",   "offset_local": [+0.25, 0.0], "color": "#00e5ff"},
            {"name": "follower", "offset_local": [-0.25, 0.0], "color": "#ff9933"},
        ]

    meta = map_data['metadata']
    grid = map_data['grid']
    res  = float(meta['resolution'])

    blocked = inflate_grid(grid, res, formation_radius)
    free       = ~blocked
    cc_labels, n_cc = ndimage.label(free)
    cc_sizes   = np.bincount(cc_labels.ravel())
    cc_sizes[0] = 0
    main_cc    = int(np.argmax(cc_sizes))
    main_mask  = (cc_labels == main_cc)

    def snap_to_main(c, r):
        if main_mask[r, c]:
            return c, r
        for radius in range(1, max(blocked.shape)):
            for dc in range(-radius, radius + 1):
                for dr in (-radius, radius):
                    nc, nr = c + dc, r + dr
                    if (0 <= nc < blocked.shape[1] and 0 <= nr < blocked.shape[0]
                            and main_mask[nr, nc]):
                        return nc, nr
            for dr in range(-radius + 1, radius):
                for dc in (-radius, radius):
                    nc, nr = c + dc, r + dr
                    if (0 <= nc < blocked.shape[1] and 0 <= nr < blocked.shape[0]
                            and main_mask[nr, nc]):
                        return nc, nr
        raise RuntimeError(f"could not find a main-CC cell near ({c}, {r})")

    sc, sr = world_to_cell(*start, meta)
    gc, gr = world_to_cell(*goal,  meta)
    if sc is None or gc is None:
        raise ValueError(f"start {start} or goal {goal} is outside the mapped area "
                         f"(bounds X[{meta['min_x']:.2f},{meta['max_x']:.2f}] "
                         f"Y[{meta['min_y']:.2f},{meta['max_y']:.2f}])")
    sc_orig, sr_orig = sc, sr
    gc_orig, gr_orig = gc, gr
    sc, sr = snap_to_main(sc, sr)
    gc, gr = snap_to_main(gc, gr)
    vc_start = cell_to_world(sc, sr, meta)
    vc_goal  = cell_to_world(gc, gr, meta)
    start_snap_dist = math.hypot((sc - sc_orig) * res, (sr - sr_orig) * res)
    goal_snap_dist  = math.hypot((gc - gc_orig) * res, (gr - gr_orig) * res)
    if start_snap_dist > 0:
        print(f"[plan] start snapped {start_snap_dist*100:.1f} cm: "
              f"{tuple(round(v, 2) for v in start)} -> {tuple(round(v, 2) for v in vc_start)}")
    if goal_snap_dist > 0:
        print(f"[plan] goal snapped {goal_snap_dist*100:.1f} cm: "
              f"{tuple(round(v, 2) for v in goal)} -> {tuple(round(v, 2) for v in vc_goal)}")

    path = astar(blocked, (sc, sr), (gc, gr))
    if path is None:
        raise RuntimeError("no path found")

    def cells_to_worldlist(cells):
        return [list(cell_to_world(c, r, meta)) for c, r in cells]

    raw_w       = cells_to_worldlist(path)
    waypoints_w = apf_smooth_path(path, blocked, meta, formation_radius)

    headings = []
    for i in range(len(waypoints_w)):
        if i < len(waypoints_w) - 1:
            x0, y0 = waypoints_w[i]
            x1, y1 = waypoints_w[i + 1]
        else:
            x0, y0 = waypoints_w[i - 1]
            x1, y1 = waypoints_w[i]
        headings.append(math.atan2(y1 - y0, x1 - x0))

    def robot_at(vc_xy, yaw, offset_local):
        ox, oy = offset_local
        c, s   = math.cos(yaw), math.sin(yaw)
        return [vc_xy[0] + c * ox - s * oy,
                vc_xy[1] + s * ox + c * oy]

    robots_out = []
    for r in robots:
        track = [robot_at(waypoints_w[i], headings[i], r["offset_local"])
                 for i in range(len(waypoints_w))]
        robots_out.append({
            "name":         r["name"],
            "offset_local": r["offset_local"],
            "color":        r.get("color", "#aaaaaa"),
            "track":        track,
            "start":        track[0]  if track else None,
            "goal":         track[-1] if track else None,
        })

    vc_distance = sum(
        math.hypot(*(np.subtract(waypoints_w[i + 1], waypoints_w[i])))
        for i in range(len(waypoints_w) - 1)
    ) if len(waypoints_w) >= 2 else 0.0

    infl_only = blocked & (np.asarray(grid, dtype=np.int32) != 2)
    infl_rows, infl_cols = np.where(infl_only)
    inflation_cells = [[int(c), int(r)] for c, r in zip(infl_cols, infl_rows)]

    rows, cols = np.where(main_mask)
    if len(rows) > 0:
        nav_bounds = {
            'min_x': float(meta['min_x'] + cols.min() * res),
            'max_x': float(meta['min_x'] + cols.max() * res),
            'min_y': float(meta['min_y'] + rows.min() * res),
            'max_y': float(meta['min_y'] + rows.max() * res),
            'cells': int(len(rows)),
            'area_m2': float(len(rows) * res * res),
        }
    else:
        nav_bounds = None

    return {
        'metadata': {
            'formation_radius':  formation_radius,
            'vc_distance':       vc_distance,
            'travel_time_sec':   vc_distance / 0.18,
            'inflation_iters':   int(math.ceil(formation_radius / res)),
            'inflation_cells':   len(inflation_cells),
            'start_snap_dist':   float(start_snap_dist),
            'goal_snap_dist':    float(goal_snap_dist),
            'navigable_bounds':  nav_bounds,
            'map_bounds':        {
                'min_x': float(meta['min_x']), 'max_x': float(meta['min_x'] + meta['grid_width']  * res),
                'min_y': float(meta['min_y']), 'max_y': float(meta['min_y'] + meta['grid_height'] * res),
            },
        },
        'virtual_center': {
            'start':      list(vc_start),
            'goal':       list(vc_goal),
            'path_raw':   raw_w,
            'waypoints':  waypoints_w,
            'headings':   headings,
        },
        'robots':           robots_out,
        'inflation_cells':  inflation_cells,
    }


# ────────────────────────────────────────────────────────────────────────────
# Standalone CLI (for offline tuning / debugging)
# ────────────────────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--map', default='map.json')
    ap.add_argument('--start',     nargs=2, type=float, metavar=('X', 'Y'), default=None)
    ap.add_argument('--goal',      nargs=2, type=float, metavar=('X', 'Y'), default=None)
    ap.add_argument('--robot-radius',     type=float, default=0.20)
    ap.add_argument('--formation',        type=float, default=0.50)
    ap.add_argument('--formation-radius', type=float, default=None)
    ap.add_argument('--export-json', default='path_plan.json')
    args = ap.parse_args()

    with open(args.map) as f:
        m = json.load(f)
    meta = m['metadata']
    grid = m['grid']
    res  = meta['resolution']

    g_arr = np.asarray(grid, dtype=np.int32)
    ground_rows, ground_cols = np.where(g_arr == 1)
    if len(ground_rows) == 0:
        raise SystemExit("No ground cells in map — cannot plan")
    gx_min = meta['min_x'] + ground_cols.min() * res
    gx_max = meta['min_x'] + ground_cols.max() * res
    gy_min = meta['min_y'] + ground_rows.min() * res
    gy_max = meta['min_y'] + ground_rows.max() * res

    inset = 4 * res + args.robot_radius
    if args.start is None:
        args.start = [gx_min + inset, gy_min + inset]
    if args.goal is None:
        args.goal  = [gx_max - inset, gy_max - inset]

    if args.formation_radius is not None:
        formation_radius = args.formation_radius
    else:
        formation_radius = args.formation / 2 + args.robot_radius

    half = args.formation / 2.0
    robots = [
        {"name": "leader",   "offset_local": [+half, 0.0], "color": "#00e5ff"},
        {"name": "follower", "offset_local": [-half, 0.0], "color": "#ff9933"},
    ]

    plan = compute_plan(map_data=m,
                        start=tuple(args.start),
                        goal=tuple(args.goal),
                        formation_radius=formation_radius,
                        robots=robots)

    if args.export_json:
        with open(args.export_json, 'w') as f:
            json.dump(plan, f, separators=(',', ':'))
        print(f"\n[json] wrote {args.export_json} "
              f"({len(plan['virtual_center']['waypoints'])} VC waypoints)")


if __name__ == '__main__':
    main()
