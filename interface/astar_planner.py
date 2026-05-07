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


# ────────────────────────────────────────────────────────────────────────────
# Visualization (matplotlib)
# ────────────────────────────────────────────────────────────────────────────

def visualize(meta, grid, blocked, path, simplified,
              leader_start, goal,
              follower_start, follower_path, follower_simplified,
              robot_radius, formation_distance,
              out_path):
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle, Rectangle

    g = np.asarray(grid, dtype=np.int32)
    H, W = g.shape
    res = meta['resolution']

    # Render the base map: 3 colors for unknown / ground / obstacle
    img = np.full((H, W, 3), 0.05, dtype=float)   # unknown = nearly black
    img[g == 1] = (0.07, 0.15, 0.15)              # ground = dark teal
    img[g == 2] = (0.20, 0.29, 0.37)              # obstacle = blue-grey
    inflated_only = blocked & (g != 2) & (g != 0)
    img[inflated_only] = (0.15, 0.10, 0.05)       # inflation halo = warm dark

    extent = [meta['min_x'], meta['min_x'] + W * res,
              meta['min_y'], meta['min_y'] + H * res]

    # ── Two-panel layout: map (LEFT) | path planning (RIGHT) ──────────────
    fig, (axL, axR) = plt.subplots(1, 2, figsize=(20, 11), dpi=120,
                                    gridspec_kw={'width_ratios': [1, 1]})

    # ════════════════════════════════════════════════════════════════════════
    #  LEFT — full map context (point cloud area, walls, inflation, paths)
    # ════════════════════════════════════════════════════════════════════════
    axL.imshow(img, origin='lower', extent=extent, interpolation='nearest')

    if path:
        xs = [meta['min_x'] + (c + 0.5) * res for c, r in path]
        ys = [meta['min_y'] + (r + 0.5) * res for c, r in path]
        axL.plot(xs, ys, color=(0.0, 0.9, 1.0, 0.35), linewidth=1.2,
                 label=f'A* raw ({len(path)} cells)')

    if simplified:
        xs = [meta['min_x'] + (c + 0.5) * res for c, r in simplified]
        ys = [meta['min_y'] + (r + 0.5) * res for c, r in simplified]
        axL.plot(xs, ys, color='#00e5ff', linewidth=3.0, marker='o', markersize=6,
                 label=f'leader ({len(simplified)} waypoints)')

    if follower_simplified:
        xs = [meta['min_x'] + (c + 0.5) * res for c, r in follower_simplified]
        ys = [meta['min_y'] + (r + 0.5) * res for c, r in follower_simplified]
        axL.plot(xs, ys, color='#ff9933', linewidth=2.5, linestyle='--',
                 marker='s', markersize=5,
                 label=f'follower (offset {formation_distance:.2f} m)')

    axL.add_patch(Rectangle((0, 0), 0, 0, color=(0.15, 0.10, 0.05),
                            label=f'inflation halo (r={robot_radius:.2f} m)'))

    axL.add_patch(Circle(leader_start, robot_radius, fill=True,
                         facecolor='#00e5ff', alpha=0.55,
                         edgecolor='#ffffff', linewidth=2, zorder=10))
    axL.text(leader_start[0], leader_start[1] + robot_radius + 0.08,
             'LEADER', color='#00e5ff', ha='center', fontsize=10,
             fontweight='bold', zorder=11)
    axL.plot(goal[0], goal[1], '+', markersize=22, mew=3, color='#00e5ff',
             zorder=10)
    axL.text(goal[0], goal[1] + 0.10, 'GOAL', color='#00e5ff', ha='center',
             fontsize=10, fontweight='bold', zorder=11)
    axL.add_patch(Circle(follower_start, robot_radius, fill=True,
                         facecolor='#ff9933', alpha=0.55,
                         edgecolor='#ffffff', linewidth=2, zorder=10))
    axL.text(follower_start[0], follower_start[1] - robot_radius - 0.10,
             'FOLLOWER', color='#ff9933', ha='center', fontsize=10,
             fontweight='bold', zorder=11)

    axL.set_xlabel('X (m)'); axL.set_ylabel('Y (m)')
    axL.set_title('Map + planned route\n'
                  f'grid {W}×{H} @ {res} m')
    axL.grid(alpha=0.15)
    axL.legend(loc='upper right', framealpha=0.9, fontsize=9)
    axL.set_aspect('equal')

    # ════════════════════════════════════════════════════════════════════════
    #  RIGHT — focused path-planning view (route only, with annotations)
    # ════════════════════════════════════════════════════════════════════════
    # Build a tighter view bounded by the path + a margin
    if simplified:
        xs_all = [cell_to_world(c, r, meta)[0] for c, r in simplified]
        ys_all = [cell_to_world(c, r, meta)[1] for c, r in simplified]
        if follower_simplified:
            xs_all += [cell_to_world(c, r, meta)[0] for c, r in follower_simplified]
            ys_all += [cell_to_world(c, r, meta)[1] for c, r in follower_simplified]
        margin = max(robot_radius * 3, 0.5)
        x0, x1 = min(xs_all) - margin, max(xs_all) + margin
        y0, y1 = min(ys_all) - margin, max(ys_all) + margin
    else:
        x0, x1 = meta['min_x'], meta['min_x'] + W * res
        y0, y1 = meta['min_y'], meta['min_y'] + H * res

    # Show ONLY the obstacle silhouette + inflation halo (no busy ground texture)
    silhouette = np.full((H, W, 4), (0, 0, 0, 0), dtype=float)
    silhouette[g == 2]      = (0.30, 0.40, 0.50, 0.95)   # walls + tires
    silhouette[inflated_only] = (0.55, 0.30, 0.10, 0.45) # inflation halo
    axR.imshow(silhouette, origin='lower', extent=extent, interpolation='nearest')

    # Leader path with per-segment distance + cumulative annotations
    if simplified:
        wxs = [cell_to_world(c, r, meta)[0] for c, r in simplified]
        wys = [cell_to_world(c, r, meta)[1] for c, r in simplified]
        axR.plot(wxs, wys, color='#00e5ff', linewidth=3.5,
                 marker='o', markersize=10, markerfacecolor='#00e5ff',
                 markeredgecolor='white', markeredgewidth=2, zorder=8)
        # Per-segment distances
        cum = 0.0
        for i in range(len(wxs) - 1):
            seg = math.hypot(wxs[i+1] - wxs[i], wys[i+1] - wys[i])
            cum += seg
            mx, my = (wxs[i] + wxs[i+1]) / 2, (wys[i] + wys[i+1]) / 2
            axR.text(mx, my + 0.04, f'{seg:.2f} m',
                     color='#00e5ff', fontsize=9, ha='center',
                     bbox=dict(boxstyle='round,pad=0.15', facecolor='black',
                               edgecolor='#00e5ff', alpha=0.65),
                     zorder=9)
        # Waypoint labels (W0..Wn)
        for i, (wx, wy) in enumerate(zip(wxs, wys)):
            axR.text(wx, wy + robot_radius * 0.55,
                     f'W{i}', color='white', fontsize=8, ha='center',
                     fontweight='bold', zorder=10)
        leader_total = cum
    else:
        leader_total = 0.0

    # Follower path
    if follower_simplified:
        fxs = [cell_to_world(c, r, meta)[0] for c, r in follower_simplified]
        fys = [cell_to_world(c, r, meta)[1] for c, r in follower_simplified]
        axR.plot(fxs, fys, color='#ff9933', linewidth=2.0, linestyle='--',
                 marker='s', markersize=7, markerfacecolor='#ff9933',
                 markeredgecolor='white', markeredgewidth=1.5, zorder=7)

    # Robots + goal (bigger so they read in the focused view)
    axR.add_patch(Circle(leader_start, robot_radius, fill=True,
                         facecolor='#00e5ff', alpha=0.7,
                         edgecolor='white', linewidth=2.5, zorder=10))
    axR.add_patch(Circle(follower_start, robot_radius, fill=True,
                         facecolor='#ff9933', alpha=0.7,
                         edgecolor='white', linewidth=2.5, zorder=10))
    axR.plot(goal[0], goal[1], '+', markersize=28, mew=4, color='#00e5ff',
             zorder=11)

    axR.text(leader_start[0], leader_start[1] + robot_radius + 0.10,
             'LEADER\nstart', color='#00e5ff', ha='center', fontsize=9,
             fontweight='bold', zorder=12)
    axR.text(follower_start[0], follower_start[1] - robot_radius - 0.10,
             'FOLLOWER\nstart', color='#ff9933', ha='center', fontsize=9,
             fontweight='bold', zorder=12)
    axR.text(goal[0] + 0.15, goal[1], 'GOAL', color='#00e5ff',
             ha='left', va='center', fontsize=11, fontweight='bold', zorder=12)

    # Stats panel (top-left of the right axes)
    travel_time = leader_total / 0.18  # MAX_LINEAR from navigation_node.py
    stats = (
        f'Leader path:    {len(simplified) if simplified else 0} waypoints\n'
        f'Total length:   {leader_total:.2f} m\n'
        f'Travel time:    ~{travel_time:.1f} s @ 0.18 m/s\n'
        f'Robot radius:   {robot_radius:.2f} m\n'
        f'Formation gap:  {formation_distance:.2f} m'
    )
    axR.text(0.02, 0.98, stats, transform=axR.transAxes,
             color='#e0e6f0', fontsize=10, fontfamily='monospace',
             ha='left', va='top',
             bbox=dict(boxstyle='round,pad=0.6', facecolor='#0a0c10',
                       edgecolor='#1e2330', alpha=0.92),
             zorder=15)

    axR.set_xlim(x0, x1); axR.set_ylim(y0, y1)
    axR.set_xlabel('X (m)'); axR.set_ylabel('Y (m)')
    axR.set_title('Path planning detail\n(per-segment distances + waypoints)')
    axR.grid(alpha=0.15)
    axR.set_aspect('equal')
    axR.set_facecolor('#0a0c10')

    plt.tight_layout()
    plt.savefig(out_path, dpi=120, facecolor='#0a0c10')
    print(f"[viz] wrote {out_path}")


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


def compute_plan(map_data: dict, start: tuple, goal: tuple,
                 formation_radius: float = 0.45,
                 robots=None) -> dict:
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

    # Inflate by the FORMATION radius (not single-robot radius). Whatever the
    # virtual center can clear, every robot in formation can clear.
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
    # How far did we have to relocate the user's click to reach a navigable cell?
    start_snap_dist = math.hypot((sc - sc_orig) * res, (sr - sr_orig) * res)
    goal_snap_dist  = math.hypot((gc - gc_orig) * res, (gr - gr_orig) * res)
    if start_snap_dist > 0:
        print(f"[plan] start snapped {start_snap_dist*100:.1f} cm: "
              f"{tuple(round(v, 2) for v in start)} -> {tuple(round(v, 2) for v in vc_start)}")
    if goal_snap_dist > 0:
        print(f"[plan] goal snapped {goal_snap_dist*100:.1f} cm: "
              f"{tuple(round(v, 2) for v in goal)} -> {tuple(round(v, 2) for v in vc_goal)}")

    # The single A* path — for the virtual center.
    path = astar(blocked, (sc, sr), (gc, gr))
    if path is None:
        raise RuntimeError("no path found")
    simplified = simplify(path, blocked)

    def cells_to_worldlist(cells):
        return [list(cell_to_world(c, r, meta)) for c, r in cells]

    waypoints_w = cells_to_worldlist(simplified)
    raw_w       = cells_to_worldlist(path)

    # ── Heading at each waypoint (derived from path tangent) ───────────────
    # heading[i] points along the segment leaving waypoint i (or arriving
    # at it, for the final waypoint). Used to orient each robot's offset.
    headings = []
    for i in range(len(waypoints_w)):
        if i < len(waypoints_w) - 1:
            x0, y0 = waypoints_w[i]
            x1, y1 = waypoints_w[i + 1]
        else:
            x0, y0 = waypoints_w[i - 1]
            x1, y1 = waypoints_w[i]
        headings.append(math.atan2(y1 - y0, x1 - x0))

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
            'formation_radius':  formation_radius,
            'vc_distance':       vc_distance,
            'travel_time_sec':   vc_distance / 0.18,   # MAX_LINEAR
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
