"""
astar_planner.py
----------------
A* global path planner for the leader robot, with:

  1. Robot-footprint inflation of the occupancy grid
     (so the planner knows the conveyor is ~30 × 30 cm, not a point)
  2. 8-connected A* with Euclidean heuristic on the inflated grid
  3. Path simplification via line-of-sight (Bresenham) — drops collinear
     waypoints so APF gets short straight runs instead of 1-cell zigzags
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
# Path simplification via line-of-sight (Bresenham)
# ────────────────────────────────────────────────────────────────────────────

def bresenham(c0, r0, c1, r1):
    dc = abs(c1 - c0); dr = abs(r1 - r0)
    sc = 1 if c0 < c1 else -1
    sr = 1 if r0 < r1 else -1
    err = dc - dr
    c, r = c0, r0
    while True:
        yield c, r
        if c == c1 and r == r1:
            return
        e2 = 2 * err
        if e2 > -dr:
            err -= dr; c += sc
        if e2 < dc:
            err += dc; r += sr


def line_of_sight(blocked, c0, r0, c1, r1):
    for c, r in bresenham(c0, r0, c1, r1):
        if blocked[r, c]:
            return False
    return True


def simplify(path, blocked):
    if not path or len(path) <= 2:
        return list(path)
    out = [path[0]]
    i = 0
    while i < len(path) - 1:
        j = len(path) - 1
        while j > i + 1:
            c0, r0 = path[i]; c1, r1 = path[j]
            if line_of_sight(blocked, c0, r0, c1, r1):
                break
            j -= 1
        out.append(path[j])
        i = j
    print(f"[simplify] {len(path)} cells -> {len(out)} waypoints")
    return out


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
    simplified = simplify(path, blocked)

    def cells_to_worldlist(cells):
        return [list(cell_to_world(c, r, meta)) for c, r in cells]

    waypoints_w = cells_to_worldlist(simplified)
    raw_w       = cells_to_worldlist(path)

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
