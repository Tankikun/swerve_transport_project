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
from scipy.spatial import cKDTree


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

    Pre-step: drop singleton-noise pixels (an obstacle cell with NO obstacle
    neighbours in its 3×3 window). Earlier versions used binary_opening for
    this, but opening's default cross structuring element silently erodes
    1-cell-thick walls — bad for 2 cm RTAB-Map output where baseboards and
    chair legs can be one pixel wide. The neighbour-count test preserves
    any pixel that's connected to anything else.

    Only TRUE obstacle cells (g==2) are inflated. 'Unknown' cells (g==0) are
    NOT treated as blocked — there are unknown patches inside indoor rooms
    where the camera didn't see (under furniture, occluded floor) and we
    don't want to block planning through them.

    For outdoor / large-area maps, callers can additionally OR with (g==0)
    before passing the grid in."""
    g = np.asarray(grid, dtype=np.int32)
    raw_blocked = (g == 2)
    # ── Pre-filter: keep only pixels with ≥1 obstacle neighbour ──────────
    # This drops salt-and-pepper noise while preserving connected walls
    # (a 1-cell-wide straight wall has 2 neighbours per pixel; opening would
    #  erode it entirely, neighbour-counting keeps every cell).
    KERNEL = np.array([[1, 1, 1], [1, 0, 1], [1, 1, 1]], dtype=np.int8)
    n_neighbors = ndimage.convolve(raw_blocked.astype(np.int8),
                                    KERNEL, mode='constant', cval=0)
    blocked  = raw_blocked & (n_neighbors >= 1)
    iters    = max(1, int(math.ceil(robot_radius / res)))
    inflated = ndimage.binary_dilation(blocked, iterations=iters)
    n_raw = int(raw_blocked.sum())
    n_post_open = int(blocked.sum())
    n_inflated = int(inflated.sum())
    print(f"[inflate] radius={robot_radius:.2f} m  iterations={iters} cells "
          f"(cell={res:.2f} m)")
    print(f"[inflate] obstacle cells: {n_raw} -> {n_post_open} (singleton "
          f"filter dropped {n_raw - n_post_open} pixels) -> {n_inflated} "
          f"(dilation grew by {n_inflated - n_post_open})")
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
    """Standard A* with Euclidean heuristic on a binary blocked-mask grid.

    Args:
        blocked: bool array shape (H, W) — True = robot cannot enter
        start  : (col, row) tuple
        goal   : (col, row) tuple

    Returns:
        list of (col, row) cells from start to goal, or None if unreachable.
    """
    H, W = blocked.shape
    sc, sr = start
    gc, gr = goal
    if blocked[sr, sc]:
        raise ValueError(f"start cell ({sc},{sr}) is blocked")
    if blocked[gr, gc]:
        raise ValueError(f"goal cell ({gc},{gr}) is blocked")

    def h(c, r):  # Euclidean heuristic
        return math.hypot(c - gc, r - gr)

    open_heap = [(h(sc, sr), 0.0, (sc, sr))]   # (f, g, cell)
    came_from = {}
    g_score   = {(sc, sr): 0.0}
    closed    = set()

    while open_heap:
        f, g, (c, r) = heapq.heappop(open_heap)
        if (c, r) in closed:
            continue
        if (c, r) == (gc, gr):
            # reconstruct
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
            # Diagonal corner-cutting prevention: don't squeeze through
            # diagonally adjacent obstacle pairs
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
    """Yield (col, row) cells on the line from (c0, r0) to (c1, r1)."""
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
    """True if the straight line between two cells stays in free space."""
    for c, r in bresenham(c0, r0, c1, r1):
        if blocked[r, c]:
            return False
    return True


def simplify(path, blocked):
    """Drop intermediate waypoints connected by line-of-sight to a future point.

    Walk forward from each anchor as far as you can while line-of-sight holds,
    then anchor at the last visible cell and repeat. Output: a much shorter
    path of straight-line segments. Standard "string pulling" technique."""
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


def apf_refine_path(waypoints_w, blocked, meta, formation_radius,
                    base_step=0.30, dense_step=0.05, dense_threshold=0.30,
                    apf_gain=0.04, max_shift=0.08, iterations=2):
    """A* gives a topologically correct corridor; this function makes it safe
    and smooth before sending to the robot:

      Step 1 — adaptive densification:
        Walk every A* segment and sample intermediate points at a spacing
        that depends on distance to the nearest obstacle.
            far from obstacles  → coarse  (base_step,  default 30 cm)
            near obstacles      → fine    (dense_step, default  5 cm)

      Step 2 — APF repulsion shift:
        For each interior waypoint, sum Khatib-style repulsive forces from
        nearby obstacles and shift the waypoint along that vector. This
        pushes the path away from walls/tires that the inflation halo was
        only just barely clearing. Endpoints (start, goal) are preserved.

    The robot then receives a denser, safer waypoint list — its on-board
    APF is essentially a backup since the laptop pre-shifted the points.

    Returns: refined list of [x, y] waypoints in world frame.
    """
    if len(waypoints_w) < 2:
        return waypoints_w

    res  = float(meta['resolution'])
    mn_x = float(meta['min_x'])
    mn_y = float(meta['min_y'])

    # KD-tree over obstacle cell centers (in world frame) for O(log n) lookup.
    rows, cols = np.where(blocked)
    if len(rows) == 0:
        return waypoints_w
    obs_xy = np.column_stack([
        mn_x + (cols + 0.5) * res,
        mn_y + (rows + 0.5) * res,
    ]).astype(np.float32)
    tree = cKDTree(obs_xy)

    def nearest_d(p):
        d, _ = tree.query(p, k=1)
        return float(d)

    # ─── Step 1: adaptive densification ────────────────────────────────────
    dense = [list(waypoints_w[0])]
    for i in range(len(waypoints_w) - 1):
        x0, y0 = waypoints_w[i]
        x1, y1 = waypoints_w[i + 1]
        seg_len = math.hypot(x1 - x0, y1 - y0)
        if seg_len < 1e-6:
            continue
        # Mid-segment distance to nearest obstacle decides spacing.
        mx, my = (x0 + x1) / 2, (y0 + y1) / 2
        d_obs = nearest_d((mx, my))
        if d_obs < dense_threshold:
            step = dense_step
        elif d_obs < dense_threshold * 2:
            t = (d_obs - dense_threshold) / dense_threshold
            step = dense_step + t * (base_step - dense_step)
        else:
            step = base_step
        n = max(1, int(math.ceil(seg_len / step)))
        for k in range(1, n):
            t = k / n
            dense.append([x0 + t * (x1 - x0), y0 + t * (y1 - y0)])
        dense.append([x1, y1])

    # ─── Step 2: APF repulsion shift (preserve endpoints) ──────────────────
    # D_REP = effective influence radius. Bigger than formation_radius so
    # that the shift kicks in BEFORE the formation footprint touches an
    # obstacle.
    D_REP = max(formation_radius + 0.10, 0.30)
    refined = [list(p) for p in dense]
    n_shifted = 0
    for _ in range(iterations):
        new_pts = [refined[0]]
        for i in range(1, len(refined) - 1):
            x, y = refined[i]
            idxs = tree.query_ball_point((x, y), D_REP)
            fx = fy = 0.0
            for j in idxs:
                ox, oy = obs_xy[j]
                dx, dy = x - ox, y - oy
                d  = math.hypot(dx, dy)
                if d < 1e-3:
                    continue
                # Khatib repulsion: only inside D_REP, monotone in 1/d.
                mag = (1.0 / d - 1.0 / D_REP) / (d * d)
                fx += mag * dx / d
                fy += mag * dy / d
            # Apply gain first, then cap the resulting shift magnitude.
            sx = fx * apf_gain
            sy = fy * apf_gain
            shift_mag = math.hypot(sx, sy)
            if shift_mag > max_shift:
                sx *= max_shift / shift_mag
                sy *= max_shift / shift_mag
            if shift_mag > 1e-4:
                n_shifted += 1
            new_pts.append([float(x + sx), float(y + sy)])
        new_pts.append(refined[-1])
        refined = new_pts

    print(f"[refine] {len(waypoints_w)} A* waypoints -> {len(refined)} "
          f"(adaptive density + APF shift × {iterations}, "
          f"shifted {n_shifted} interior pts)")
    # Cast to plain Python floats so the result is JSON-serializable.
    return [[float(p[0]), float(p[1])] for p in refined]



# ────────────────────────────────────────────────────────────────────────────
# Main
# ────────────────────────────────────────────────────────────────────────────

def visualize_vc(plan, map_data, out_path):
    """Side-by-side matplotlib render of the virtual-center plan."""
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle

    meta = map_data['metadata']; grid = map_data['grid']
    res = meta['resolution']
    g = np.asarray(grid, dtype=np.int32); H, W = g.shape

    img = np.full((H, W, 3), 0.05, dtype=float)
    img[g == 1] = (0.07, 0.15, 0.15)
    img[g == 2] = (0.20, 0.29, 0.37)
    blocked = np.zeros((H, W), dtype=bool)
    for c, r in plan['inflation_cells']:
        blocked[r, c] = True
    img[blocked] = (0.35, 0.20, 0.08)

    extent = [meta['min_x'], meta['min_x'] + W * res,
              meta['min_y'], meta['min_y'] + H * res]

    vc = plan['virtual_center']
    fr_radius = plan['metadata']['formation_radius']

    fig, ax = plt.subplots(figsize=(11, 10), dpi=120)
    ax.imshow(img, origin='lower', extent=extent, interpolation='nearest')

    # Formation envelope at every other waypoint
    for i in range(0, len(vc['waypoints']), max(1, len(vc['waypoints']) // 8)):
        wp = vc['waypoints'][i]
        ax.add_patch(Circle(wp, fr_radius, fill=False,
                            edgecolor=(1.0, 0.5, 0.2, 0.45), linewidth=1.2))

    # Virtual-center path
    xs = [p[0] for p in vc['waypoints']]
    ys = [p[1] for p in vc['waypoints']]
    ax.plot(xs, ys, color='#ffd33f', linewidth=3.5, marker='o',
            markersize=8, markeredgecolor='white', markeredgewidth=1.5,
            label=f'virtual-center path ({len(xs)} wp)', zorder=8)

    # Robots: derived tracks (small dashed lines)
    for r in plan['robots']:
        if not r['track']:
            continue
        rxs = [p[0] for p in r['track']]
        rys = [p[1] for p in r['track']]
        ax.plot(rxs, rys, color=r['color'], linewidth=1.5, linestyle='--',
                alpha=0.7, label=f"{r['name']} (derived from VC + offset)")
        # Robot at start
        ax.add_patch(Circle(r['track'][0], 0.10, fill=True,
                            facecolor=r['color'], alpha=0.7,
                            edgecolor='white', linewidth=1.5, zorder=10))

    # VC start + goal
    ax.plot(*vc['start'], 'o', markersize=14, color='#ffd33f',
            markeredgecolor='white', markeredgewidth=2, zorder=11)
    ax.plot(*vc['goal'],  '+', markersize=22, mew=3, color='#ffd33f', zorder=11)
    ax.text(vc['start'][0], vc['start'][1] + 0.15, 'VC start',
            color='#ffd33f', ha='center', fontweight='bold', zorder=12)
    ax.text(vc['goal'][0],  vc['goal'][1]  + 0.15, 'VC goal',
            color='#ffd33f', ha='center', fontweight='bold', zorder=12)

    ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)')
    ax.set_aspect('equal')
    ax.grid(alpha=0.15)
    ax.set_title(f'Virtual-center plan  (formation_radius={fr_radius:.2f} m)')
    ax.legend(loc='upper right', framealpha=0.9, fontsize=9)
    plt.tight_layout()
    plt.savefig(out_path, dpi=120)
    print(f"[viz] wrote {out_path}")


def arclength_resample(waypoints, target_spacing=0.15):
    """Resample a polyline at uniform arc-length spacing.

    Walks the polyline accumulating distance; emits a sample every
    `target_spacing` meters. Endpoints are always preserved.

    Why: `apf_refine_path` produces densely-packed (~5 cm) points near
    obstacles and sparse (~30 cm) points in open space. Both are outside
    the controller's "sweet spot" — 5 cm is below GOAL_TOL_INTERMEDIATE
    so the Pi pops 4 waypoints in one tick, and 30 cm is past LOOKAHEAD
    so pure-pursuit can momentarily walk past the end of a segment.
    Uniform 12-15 cm spacing falls inside the sweet spot for both.

    APF safety shift is preserved — we resample the SHIFTED polyline,
    so the wall-margin earned by `apf_refine_path` carries through.

    Args:
        waypoints      : list of [x, y] (or longer tuples; only [:2] used)
        target_spacing : meters between adjacent samples (default 0.15)

    Returns:
        list of [x, y] at uniform spacing.
    """
    if len(waypoints) < 2:
        return [list(p[:2]) for p in waypoints]

    pts = [list(p[:2]) for p in waypoints]
    cum = [0.0]
    for i in range(1, len(pts)):
        cum.append(cum[-1] + math.hypot(pts[i][0] - pts[i-1][0],
                                         pts[i][1] - pts[i-1][1]))
    total = cum[-1]
    if total < 1e-9 or target_spacing <= 1e-6:
        return pts[:]

    out = [pts[0]]
    target = target_spacing
    seg = 0
    while target < total:
        while seg < len(pts) - 1 and cum[seg + 1] < target:
            seg += 1
        seg_len = cum[seg + 1] - cum[seg]
        if seg_len < 1e-9:
            target += target_spacing
            continue
        t = (target - cum[seg]) / seg_len
        x = pts[seg][0] + t * (pts[seg + 1][0] - pts[seg][0])
        y = pts[seg][1] + t * (pts[seg + 1][1] - pts[seg][1])
        out.append([x, y])
        target += target_spacing

    out.append(pts[-1])
    print(f"[resample] {len(waypoints)} -> {len(out)} waypoints "
          f"({total:.2f} m at ~{target_spacing*100:.0f} cm spacing)")
    return out


def compute_plan(map_data: dict, start: tuple, goal: tuple,
                 formation_radius: float = 0.45,
                 robots=None,
                 yaw_policy: str = "tangent",
                 start_yaw: float = 0.0,
                 goal_yaw: float = 0.0,
                 target_spacing: float = 0.15) -> dict:
    """Plan ONE path for the virtual center; the formation moves together.

    Args:
        map_data         : the map.json dict (metadata + grid)
        start, goal      : world (x, y) for the virtual center
        formation_radius : the formation's bounding-circle radius in meters.
                           Computed once from individual_robot_radius +
                           offset_from_VC. The PLANNER's only inflation knob.
        robots           : optional list of {name, offset_local, color} where
                           offset_local is (dx, dy) in formation body frame
                           (X forward, Y left). Used only for visualization
                           (each robot's pose is derived from VC + offset at
                           runtime by laplacian_formation_node.py).

    Returns the plan dict that server.py serializes to path_plan.json."""
    if robots is None:
        # Sensible default: 2 robots, leader in front, follower behind
        robots = [
            {"name": "leader",   "offset_local": [+0.25, 0.0], "color": "#00e5ff"},
            {"name": "follower", "offset_local": [-0.25, 0.0], "color": "#ff9933"},
        ]

    meta = map_data['metadata']
    grid = map_data['grid']
    res  = float(meta['resolution'])

    # Compute start/goal cells once. The cascade below handles the rest.
    sc_orig, sr_orig = world_to_cell(*start, meta)
    gc_orig, gr_orig = world_to_cell(*goal,  meta)
    if sc_orig is None or gc_orig is None:
        raise ValueError(f"start {start} or goal {goal} is outside the mapped area "
                         f"(bounds X[{meta['min_x']:.2f},{meta['max_x']:.2f}] "
                         f"Y[{meta['min_y']:.2f},{meta['max_y']:.2f}])")

    # ── Radius cascade ───────────────────────────────────────────────────
    # Try planning at the requested radius. If A* fails (typically because
    # noise pixels collapsed a corridor under inflation), retry at smaller
    # radii. Surfaced in metadata so the GUI can warn that clearance was
    # reduced. Replaces the silent snap-the-goal-up-to-1.2m band-aid.
    blocked = main_mask = sc = sr = gc = gr = path = None
    used_radius = None
    attempted = []
    for scale in (1.0, 0.8, 0.6):
        radius_try = formation_radius * scale
        b_try = inflate_grid(grid, res, radius_try)
        free  = ~b_try
        cc_labels, _ = ndimage.label(free)
        cc_sizes = np.bincount(cc_labels.ravel())
        if cc_sizes.size:
            cc_sizes[0] = 0
        if cc_sizes.size <= 1 or cc_sizes.max() == 0:
            attempted.append((round(radius_try, 3), "everything blocked"))
            continue
        main_cc_try   = int(np.argmax(cc_sizes))
        main_mask_try = (cc_labels == main_cc_try)

        # Bounded snap: never relocate further than 0.5 m. If a click lands
        # deep inside a wall, surface an error instead of silently moving
        # the goal somewhere wrong (the old behavior was up to ~1.2 m).
        max_snap_cells = max(1, int(0.5 / res))
        def _snap(c, r, mm=main_mask_try, b=b_try, cap=max_snap_cells):
            if mm[r, c]:
                return c, r
            for rad in range(1, min(cap + 1, max(b.shape))):
                for dc in range(-rad, rad + 1):
                    for dr in (-rad, rad):
                        nc, nr = c + dc, r + dr
                        if (0 <= nc < b.shape[1] and 0 <= nr < b.shape[0]
                                and mm[nr, nc]):
                            return nc, nr
                for dr in range(-rad + 1, rad):
                    for dc in (-rad, rad):
                        nc, nr = c + dc, r + dr
                        if (0 <= nc < b.shape[1] and 0 <= nr < b.shape[0]
                                and mm[nr, nc]):
                            return nc, nr
            return None, None

        sc_try, sr_try = _snap(sc_orig, sr_orig)
        gc_try, gr_try = _snap(gc_orig, gr_orig)
        if sc_try is None or gc_try is None:
            attempted.append((round(radius_try, 3),
                              "click >0.5 m from any reachable cell"))
            continue

        path_try = astar(b_try, (sc_try, sr_try), (gc_try, gr_try))
        if path_try is None:
            attempted.append((round(radius_try, 3), "no path"))
            continue

        # First success — keep it.
        blocked   = b_try
        main_mask = main_mask_try
        sc, sr    = sc_try, sr_try
        gc, gr    = gc_try, gr_try
        path      = path_try
        used_radius = radius_try
        attempted.append((round(radius_try, 3), "ok"))
        break

    if path is None:
        raise RuntimeError(
            f"no path found at any radius. Attempts: {attempted}. "
            f"Likely the click is unreachable or the map has too much "
            f"noise. Try clicking further from walls."
        )

    radius_reduced = (used_radius < formation_radius)
    if radius_reduced:
        print(f"[plan] ⚠ reduced clearance to {used_radius:.2f} m "
              f"(requested {formation_radius:.2f} m). Cascade: {attempted}")

    vc_start = cell_to_world(sc, sr, meta)
    vc_goal  = cell_to_world(gc, gr, meta)
    start_snap_dist = math.hypot((sc - sc_orig) * res, (sr - sr_orig) * res)
    goal_snap_dist  = math.hypot((gc - gc_orig) * res, (gr - gr_orig) * res)
    if start_snap_dist > 0:
        print(f"[plan] start snapped {start_snap_dist*100:.1f} cm: "
              f"{tuple(round(v, 2) for v in start)} -> "
              f"{tuple(round(v, 2) for v in vc_start)}")
    if goal_snap_dist > 0:
        print(f"[plan] goal snapped {goal_snap_dist*100:.1f} cm: "
              f"{tuple(round(v, 2) for v in goal)} -> "
              f"{tuple(round(v, 2) for v in vc_goal)}")
    simplified = simplify(path, blocked)

    def cells_to_worldlist(cells):
        # Cast to Python float — cell indices may be numpy ints, which
        # produce numpy scalars under arithmetic and break JSON encoding.
        return [[float(v) for v in cell_to_world(c, r, meta)]
                for c, r in cells]

    waypoints_w = cells_to_worldlist(simplified)
    raw_w       = cells_to_worldlist(path)

    # APF refinement: densify near obstacles and shift waypoints away from
    # walls. The robot ends up with a smoother, safer line to follow.
    # Note: pass `used_radius` (the actual clearance achieved) rather than
    # the requested `formation_radius`, so refinement matches the inflation.
    waypoints_w = apf_refine_path(waypoints_w, blocked, meta, used_radius)

    # Arc-length resample: uniform 15 cm spacing for controller-friendly
    # density. APF safety shift is preserved; only the spacing changes.
    if target_spacing > 0:
        waypoints_w = arclength_resample(waypoints_w, target_spacing)

    # ── Heading at each waypoint, dispatched by yaw_policy ─────────────────
    # 'free'           : crab-walk; the formation keeps `start_yaw` throughout.
    # 'tangent'        : heading[i] = direction of segment i→i+1 (path tangent).
    #                    First waypoint forced to `start_yaw`, last to `goal_yaw`.
    # 'smooth_to_goal' : linear interpolate from start_yaw to goal_yaw along
    #                    arc-length progress (0 at start, 1 at goal). Keeps the
    #                    formation slowly rotating into the goal orientation.
    n_wps = len(waypoints_w)
    headings = [0.0] * n_wps
    if yaw_policy == 'free':
        for i in range(n_wps):
            headings[i] = float(start_yaw)
    elif yaw_policy == 'smooth_to_goal':
        # Compute cumulative arc length to interpolate yaw uniformly along path.
        cumlen = [0.0]
        for i in range(1, n_wps):
            cumlen.append(cumlen[-1] + math.hypot(
                waypoints_w[i][0] - waypoints_w[i - 1][0],
                waypoints_w[i][1] - waypoints_w[i - 1][1]))
        total = cumlen[-1] if cumlen[-1] > 1e-9 else 1.0
        # Use shortest-arc interpolation for the yaw delta.
        delta = math.atan2(math.sin(goal_yaw - start_yaw),
                           math.cos(goal_yaw - start_yaw))
        for i in range(n_wps):
            headings[i] = float(start_yaw + delta * (cumlen[i] / total))
    else:  # 'tangent' (default) — path-tangent yaw with start/goal pinning
        for i in range(n_wps):
            if i < n_wps - 1:
                x0, y0 = waypoints_w[i]
                x1, y1 = waypoints_w[i + 1]
            else:
                x0, y0 = waypoints_w[i - 1]
                x1, y1 = waypoints_w[i]
            headings[i] = math.atan2(y1 - y0, x1 - x0)
        # Pin endpoints if caller requested specific orientations.
        if n_wps >= 1:
            headings[0]  = float(start_yaw)
            headings[-1] = float(goal_yaw)

    # ── Each robot's WORLD pose at each waypoint = VC + R(yaw) * offset ────
    # This is what laplacian_formation_node would produce at runtime;
    # we precompute the visual track here just for the viewer.
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

    # ── Path length + travel time for the virtual center ───────────────────
    vc_distance = sum(
        math.hypot(*(np.subtract(waypoints_w[i + 1], waypoints_w[i])))
        for i in range(len(waypoints_w) - 1)
    ) if len(waypoints_w) >= 2 else 0.0

    # ── Inflation halo cells (for the viewer) ──────────────────────────────
    infl_only = blocked & (np.asarray(grid, dtype=np.int32) != 2)
    infl_rows, infl_cols = np.where(infl_only)
    inflation_cells = [[int(c), int(r)] for c, r in zip(infl_cols, infl_rows)]

    # Navigable bounds (where the VC can actually be) so the UI can hint
    # at where clicks should land
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
            'formation_radius':           used_radius,
            'formation_radius_requested': formation_radius,
            'radius_reduced':             bool(radius_reduced),
            'cascade_attempts':           attempted,
            'vc_distance':                vc_distance,
            'travel_time_sec':            vc_distance / 0.18,   # MAX_LINEAR
            'inflation_iters':            int(math.ceil(used_radius / res)),
            'inflation_cells':            len(inflation_cells),
            'start_snap_dist':            float(start_snap_dist),
            'goal_snap_dist':             float(goal_snap_dist),
            'target_spacing':             float(target_spacing),
            'yaw_policy':                 yaw_policy,
            'start_yaw':                  float(start_yaw),
            'goal_yaw':                   float(goal_yaw),
            'navigable_bounds':           nav_bounds,
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


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--map', default='map.json')
    ap.add_argument('--start',     nargs=2, type=float, metavar=('X', 'Y'),
                    default=None,  help='Leader start (x y) world coords')
    ap.add_argument('--goal',      nargs=2, type=float, metavar=('X', 'Y'),
                    default=None,  help='Leader goal  (x y) world coords')
    ap.add_argument('--robot-radius',  type=float, default=0.20,
                    help='Single-robot bounding radius (used to compute formation radius)')
    ap.add_argument('--formation',     type=float, default=0.50,
                    help='Follower distance behind leader (m, formation diameter)')
    ap.add_argument('--formation-radius', type=float, default=None,
                    help='Override the auto-computed formation radius (m)')
    ap.add_argument('--out', default='astar_path.png',
                    help='Output PNG file')
    ap.add_argument('--export-json', default='path_plan.json',
                    help='Output JSON for the HTML viewer (set "" to skip)')
    args = ap.parse_args()

    with open(args.map) as f:
        m = json.load(f)
    meta = m['metadata']
    grid = m['grid']
    res  = meta['resolution']

    print(f"[map] {meta['grid_width']} x {meta['grid_height']} cells @ {res} m")
    print(f"[map] world bounds: x[{meta['min_x']:.2f}, {meta['max_x']:.2f}]  "
          f"y[{meta['min_y']:.2f}, {meta['max_y']:.2f}]")

    # Default start/goal: pick them based on the ACTUAL mapped ground, not
    # the bbox. The grid often has large 'unknown' margins that we can't plan
    # through, so we find ground cells and use their bounds.
    g_arr = np.asarray(grid, dtype=np.int32)
    ground_rows, ground_cols = np.where(g_arr == 1)
    if len(ground_rows) == 0:
        raise SystemExit("No ground cells in map — cannot plan")
    gx_min = meta['min_x'] + ground_cols.min() * res
    gx_max = meta['min_x'] + ground_cols.max() * res
    gy_min = meta['min_y'] + ground_rows.min() * res
    gy_max = meta['min_y'] + ground_rows.max() * res
    print(f"[map] ground bounds: x[{gx_min:.2f}, {gx_max:.2f}]  "
          f"y[{gy_min:.2f}, {gy_max:.2f}]  "
          f"({len(ground_rows)} ground cells)")

    inset = 4 * res + args.robot_radius
    if args.start is None:
        args.start = [gx_min + inset, gy_min + inset]
    if args.goal is None:
        args.goal  = [gx_max - inset, gy_max - inset]
    leader_start = tuple(args.start)
    goal         = tuple(args.goal)

    # Compute the formation radius. Two robots offset by formation/2 around
    # the virtual center, each with robot_radius half-width: the bounding
    # circle of the formation has radius = formation/2 + robot_radius.
    if args.formation_radius is not None:
        formation_radius = args.formation_radius
    else:
        formation_radius = args.formation / 2 + args.robot_radius
    print(f"[plan] formation_radius = {formation_radius:.3f} m  "
          f"(formation/2 = {args.formation/2:.2f} + robot_radius = {args.robot_radius:.2f})")

    # Inflate by FORMATION radius — wherever the virtual center can go, the
    # whole formation can go.
    blocked = inflate_grid(grid, res, formation_radius)

    # Robots in formation: leader at +formation/2 forward, follower at -formation/2
    half = args.formation / 2.0
    robots = [
        {"name": "leader",   "offset_local": [+half, 0.0], "color": "#00e5ff"},
        {"name": "follower", "offset_local": [-half, 0.0], "color": "#ff9933"},
    ]

    # Run the planner — single path for the virtual center.
    plan = compute_plan(map_data=m,
                        start=tuple(args.start),
                        goal=tuple(args.goal),
                        formation_radius=formation_radius,
                        robots=robots)

    # Print path summary
    print()
    print("Virtual-center waypoints (world coords):")
    for i, (x, y) in enumerate(plan['virtual_center']['waypoints']):
        print(f"  [{i:2d}]  ({x:+.3f}, {y:+.3f})  yaw={math.degrees(plan['virtual_center']['headings'][i]):+.0f}°")

    # ── Export JSON for the HTML viewer ──────────────────────────────────
    if args.export_json:
        with open(args.export_json, 'w') as f:
            json.dump(plan, f, separators=(',', ':'))
        print(f"\n[json] wrote {args.export_json} "
              f"({len(plan['inflation_cells'])} inflation cells, "
              f"{len(plan['virtual_center']['waypoints'])} VC waypoints, "
              f"{len(plan['robots'])} robots)")

    # ── PNG via matplotlib (best-effort — viewer is the real interface) ──
    try:
        visualize_vc(plan, m, args.out)
    except Exception as e:
        print(f"[viz] matplotlib render skipped: {e}")


if __name__ == '__main__':
    main()
