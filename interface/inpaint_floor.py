"""
inpaint_floor.py
----------------
Fill in floor holes in friend's map.json without rebuilding the grid.

friend's `db_to_map_json.py` (RANSAC + DBSCAN pipeline) leaves three classes
of cells:
    0 = unknown   (camera never observed this cell)
    1 = ground    (RANSAC floor inlier)
    2 = obstacle  (non-floor cluster point)

The 'unknown' cells are usually:
    a)  outside the room footprint (no point in planning there)
    b)  occluded patches *inside* the room (under chairs, behind tires,
        spots the depth sensor missed) — these are physically floor, the
        camera just didn't see them.

This script fills (b) and leaves (a) alone, using two well-known
morphology + flood-fill steps:

    1. binary_closing on the ground mask  →  fills small gaps
    2. flood-fill from grid boundary       →  any 'unknown' cell that is
                                              walled-off from the boundary
                                              must be inside the room → mark
                                              as ground.

Crucially this DOES NOT rebuild the grid or touch obstacle cells. It only
upgrades unknown→ground. Walls / tires / panels stay exactly as friend's
pipeline placed them.

Run AFTER db_to_map_json.py:

    /usr/bin/python3 db_to_map_json.py --db tb3_1_room.db --output map.json
    /usr/bin/python3 inpaint_floor.py                    # in-place
    /usr/bin/python3 inpaint_floor.py --in map.json --out map_inpainted.json
"""

import argparse
import json
import shutil

import numpy as np
from scipy import ndimage
from scipy.spatial import cKDTree


def inpaint(grid: np.ndarray, closing_iters: int = 2, fill_holes: bool = True):
    """Fill floor holes. Returns the modified grid + count summary."""
    g = grid.copy()
    H, W = g.shape

    cells_before = {
        'unknown':  int((g == 0).sum()),
        'ground':   int((g == 1).sum()),
        'obstacle': int((g == 2).sum()),
    }

    # ── Step 1: morphological closing on the ground mask ────────────────
    # Fills small gaps where the camera saw most of the floor but missed
    # individual cells. Don't overwrite obstacle cells.
    if closing_iters > 0:
        ground_mask = (g == 1)
        closed      = ndimage.binary_closing(ground_mask, iterations=closing_iters)
        new_ground  = closed & (g != 2) & ~ground_mask   # upgrade unknown only
        g[new_ground] = 1
        n_closed_added = int(new_ground.sum())
    else:
        n_closed_added = 0

    # ── Step 2: flood-fill from grid boundary on unknown cells ──────────
    # Any unknown cell reachable from the boundary stays "unknown" (it's
    # outside the room). Any unknown cell NOT reachable must be walled in
    # by either ground or obstacle → must be inside the room → mark ground.
    if fill_holes:
        passable = (g == 0)                  # unknown only — fill flows here
        seed = np.zeros_like(passable)
        seed[0, :]  = passable[0, :]
        seed[-1, :] = passable[-1, :]
        seed[:, 0]  = passable[:, 0]
        seed[:, -1] = passable[:, -1]
        outside = ndimage.binary_propagation(seed, mask=passable)
        interior_holes = passable & ~outside
        g[interior_holes] = 1
        n_flood_added = int(interior_holes.sum())
    else:
        n_flood_added = 0

    cells_after = {
        'unknown':  int((g == 0).sum()),
        'ground':   int((g == 1).sum()),
        'obstacle': int((g == 2).sum()),
    }

    print(f"[inpaint] BEFORE  unknown={cells_before['unknown']:5d}  "
          f"ground={cells_before['ground']:5d}  "
          f"obstacle={cells_before['obstacle']:5d}")
    print(f"[inpaint]   closing ({closing_iters}x) → +{n_closed_added} ground")
    print(f"[inpaint]   flood-fill         → +{n_flood_added} ground")
    print(f"[inpaint] AFTER   unknown={cells_after['unknown']:5d}  "
          f"ground={cells_after['ground']:5d}  "
          f"obstacle={cells_after['obstacle']:5d}")
    print(f"[inpaint]   net change: ground +{cells_after['ground']-cells_before['ground']}, "
          f"unknown {cells_after['unknown']-cells_before['unknown']:+d}")

    return g


def synthesize_floor_points(map_data: dict, density_cm: float = 5.0,
                            search_radius_m: float = 0.08):
    """Add synthetic 3D points for every ground cell that has no real
    point nearby. Each new point sits at (cell_center_x, cell_center_y,
    floor_z) and inherits the median color of the closest real points.

    density_cm:   spacing of synthesized points in centimeters
                  (5cm → one synthetic point every 5cm of ground)
    search_radius_m: a synthesized point is skipped if there's already a
                     real point within this radius (prevents doubling-up)
    """
    meta = map_data['metadata']
    grid = np.asarray(map_data['grid'], dtype=np.int32)
    res  = float(meta['resolution'])
    floor_z = float(meta['floor_z'])
    H, W = grid.shape

    pts  = np.asarray(map_data['points'], dtype=np.float32)
    cols = np.asarray(map_data['colors'], dtype=np.float32)

    # KD-tree on existing FLOOR-BAND point XYs only. Using all points would
    # also include wall bases at floor cells along the perimeter, making us
    # think those XY positions are "covered" when in fact there's no actual
    # floor data above them.
    near_floor = (pts[:, 2] > floor_z - 0.05) & (pts[:, 2] < floor_z + 0.12)
    floor_pts_xy = pts[near_floor, :2]
    if len(floor_pts_xy) == 0:
        print("[3d] no existing floor points to KD-search against")
        return map_data
    tree_xy = cKDTree(floor_pts_xy)

    # Density: 1 synthetic point every `density_cm` cm of floor.
    density_m = density_cm / 100.0
    cells_per_point = max(1, int(round(density_m / res)))

    # Median color of existing floor points (for the synthetic point color)
    if near_floor.sum() > 100:
        floor_color = np.median(cols[near_floor], axis=0)
    else:
        floor_color = np.array([170, 160, 140])   # beige fallback
    print(f"[3d] synthetic floor color (median of {int(near_floor.sum())} "
          f"real floor-band points): RGB={tuple(int(c) for c in floor_color)}")

    # Color jitter — real floor has subtle texture; uniform color reads fake
    color_std = np.array([12.0, 11.0, 10.0])

    # First pass: identify EMPTY ground cells (no floor-band point within
    # search_radius_m), accumulate how big each connected hole is. We then
    # densify each hole proportionally so big holes get many synthetic
    # points and small ones get just a sprinkle.
    cell_xy = []           # (r, c) of every ground cell that needs synth
    for r in range(0, H, cells_per_point):
        for c in range(0, W, cells_per_point):
            if grid[r, c] != 1:
                continue
            wx = meta['min_x'] + (c + 0.5) * res
            wy = meta['min_y'] + (r + 0.5) * res
            d, _ = tree_xy.query([wx, wy], k=1)
            if d > search_radius_m:
                cell_xy.append((r, c))

    rng = np.random.default_rng(0)
    cell_size  = res * cells_per_point         # the "spacing" in meters
    # Points per cell — chosen so the synthetic point density approximates
    # the real-cloud density (~one point per 5–7 mm) within each empty cell
    pts_per_cell = max(4, int((cell_size / 0.007) ** 2))

    new_pts  = np.empty((len(cell_xy) * pts_per_cell, 3), dtype=np.float32)
    new_cols = np.empty((len(cell_xy) * pts_per_cell, 3), dtype=np.float32)
    k = 0
    for (r, c) in cell_xy:
        cx = meta['min_x'] + (c + 0.5) * res
        cy = meta['min_y'] + (r + 0.5) * res
        # Uniform jitter inside the cell footprint (±cell_size/2 each axis)
        jx = (rng.random(pts_per_cell) - 0.5) * cell_size
        jy = (rng.random(pts_per_cell) - 0.5) * cell_size
        # Small Z jitter so the floor doesn't read as a perfect plane
        jz = (rng.random(pts_per_cell) - 0.5) * 0.005
        new_pts[k:k+pts_per_cell, 0] = cx + jx
        new_pts[k:k+pts_per_cell, 1] = cy + jy
        new_pts[k:k+pts_per_cell, 2] = floor_z + jz
        # Color jitter around the median floor color
        col = floor_color + rng.standard_normal((pts_per_cell, 3)) * color_std
        new_cols[k:k+pts_per_cell] = np.clip(col, 0, 255)
        k += pts_per_cell
    new_pts  = new_pts[:k]
    new_cols = new_cols[:k]

    skipped_too_close = (len(range(0, H, cells_per_point))
                         * len(range(0, W, cells_per_point))
                         - len(cell_xy)) if False else 0  # not tracked here

    print(f"[3d] {len(cell_xy)} empty cells found at {density_cm}cm spacing "
          f"(search radius {search_radius_m}m, floor band)")
    print(f"[3d] {pts_per_cell} jittered points per cell "
          f"-> {len(new_pts)} synthetic floor points total")

    if len(new_pts):
        merged_pts  = np.vstack([pts,  new_pts])
        merged_cols = np.vstack([cols, new_cols])
        map_data['points'] = merged_pts.tolist()
        map_data['colors'] = merged_cols.tolist()
        meta['point_count'] = int(len(merged_pts))
    return map_data


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--in',  dest='inp',  default='map.json')
    ap.add_argument('--out', dest='outp', default='map.json')
    ap.add_argument('--closing', type=int, default=2,
                    help='Morphological closing iterations on the ground mask (default 2)')
    ap.add_argument('--no-fill', action='store_true',
                    help='Skip the boundary flood-fill step (only do closing)')
    ap.add_argument('--no-3d',   action='store_true',
                    help='Skip the 3D point synthesis (only patch the 2D grid)')
    ap.add_argument('--density-cm', type=float, default=2.5,
                    help='Spacing of synthesized 3D floor points (default 2.5 cm)')
    ap.add_argument('--search-radius', type=float, default=0.03,
                    help='Skip synthesis if a real floor point exists within this many meters (default 0.03)')
    args = ap.parse_args()

    bak = args.inp + '.beforeInpaint.bak'
    shutil.copy(args.inp, bak)
    print(f"[backup] {args.inp} -> {bak}")

    with open(args.inp) as f:
        m = json.load(f)
    g = np.asarray(m['grid'], dtype=np.uint8)
    print(f"[load] grid shape: {g.shape}")

    g_new = inpaint(g, closing_iters=args.closing, fill_holes=not args.no_fill)
    m['grid'] = g_new.tolist()

    # ── Now also fill the 3D point cloud so the visual floor is solid ──
    if not args.no_3d:
        m = synthesize_floor_points(m, density_cm=args.density_cm,
                                    search_radius_m=args.search_radius)

    with open(args.outp, 'w') as f:
        json.dump(m, f, separators=(',', ':'))
    print(f"[save] wrote {args.outp}")


if __name__ == '__main__':
    main()
