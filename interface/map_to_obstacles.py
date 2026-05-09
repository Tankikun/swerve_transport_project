"""
map_to_obstacles.py
-------------------
Pure-Python helper: read map.json's 2D occupancy grid, extract connected
obstacle blobs, return each as (cx, cy, radius) in WORLD frame coordinates.

Used by:
  * mock_ros_bridge.py   (Mac-side end-to-end test)
  * later: a real ROS 2 publisher node on the Pi (same logic)

The 2D grid in map.json:
  grid[row][col]  →  row indexes Y (forward/back), col indexes X (left/right)
  values: 0=unknown, 1=ground, 2=obstacle

World coordinates (post-cleanup map is z-up REP-103):
  x = min_x + col * resolution
  y = min_y + row * resolution

Earlier versions of this file used a Y-up convention with "X-Z" floor (a
leftover from the pre-RTAB-Map Scaniverse pipeline), which referenced a
non-existent `min_z` metadata key. The current map.json from
db_to_map_json.py / clean_map.py uses Z-up; the floor plane is X-Y; the
metadata exposes `min_x`, `min_y`, `max_x`, `max_y` (no `min_z`).

Usage:
    python3 map_to_obstacles.py --map map.json
    -> prints each obstacle: (cx, cy, radius)

Or import:
    from map_to_obstacles import extract_obstacles
    obs = extract_obstacles('map.json', min_blob_cells=3)
"""

import argparse
import json
import math
from typing import List, Optional, Tuple

import numpy as np
from scipy import ndimage


def extract_obstacles(map_path: str,
                      min_blob_cells: int = 3,
                      max_blob_cells: Optional[int] = None,
                      inflate_radius: float = 0.0,
                      exclude_largest: bool = True,
                      sample_wall_step: int = 0) -> List[Tuple[float, float, float]]:
    """Extract connected obstacle blobs from map.json's 2D grid.

    Args:
        map_path:        path to map.json
        min_blob_cells:  ignore blobs smaller than this many cells (noise)
        max_blob_cells:  ignore blobs larger than this (e.g. walls — None = keep all)
        inflate_radius:  extra meters added to each blob's radius
        exclude_largest: drop the single largest blob (assumed to be the wall outline).
                         APF needs interior obstacles as point-like centroids;
                         a wall centroid would be wrong (it'd push toward centre).
        sample_wall_step: if > 0, after excluding the wall, also sample the wall
                          edge every N cells as small (radius=cell_size) obstacles.
                          0 = disable; 4 is a good default for nav.

    Returns:
        list of (world_x, world_y, radius_m) — coordinates in REP-103 map frame.
    """
    with open(map_path) as f:
        m = json.load(f)

    grid = np.asarray(m['grid'], dtype=np.int32)
    res  = float(m['metadata']['resolution'])
    x0   = float(m['metadata']['min_x'])
    y0   = float(m['metadata']['min_y'])

    obs_mask = (grid == 2)
    labels, n = ndimage.label(obs_mask, structure=np.ones((3, 3)))

    # Find the largest component (likely the wall ring)
    sizes = np.array([(labels == i).sum() for i in range(1, n + 1)])
    largest_idx = int(np.argmax(sizes)) + 1 if len(sizes) > 0 else 0

    blobs = []
    for i in range(1, n + 1):
        cells = np.argwhere(labels == i)
        size  = len(cells)
        if size < min_blob_cells:
            continue
        if max_blob_cells is not None and size > max_blob_cells:
            continue
        if exclude_largest and i == largest_idx:
            # Optionally sample wall cells as small obstacles
            if sample_wall_step > 0:
                for k in range(0, size, sample_wall_step):
                    cy_idx, cx_idx = cells[k]
                    wx = x0 + cx_idx * res
                    wy = y0 + cy_idx * res
                    # Each wall sample = single-cell-radius obstacle
                    blobs.append((float(wx), float(wy),
                                  float(res * 1.2 + inflate_radius)))
            continue

        cy_idx, cx_idx = cells.mean(axis=0)
        area_m2 = size * (res ** 2)
        radius  = math.sqrt(area_m2 / math.pi) + inflate_radius
        radius  = max(radius, res * 1.2)

        wx = x0 + cx_idx * res
        wy = y0 + cy_idx * res
        blobs.append((float(wx), float(wy), float(radius)))

    return blobs


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--map', default='map.json')
    ap.add_argument('--min-cells',  type=int,   default=3)
    ap.add_argument('--max-cells',  type=int,   default=None,
                    help='Drop blobs larger than this many cells (use to skip walls)')
    ap.add_argument('--inflate',    type=float, default=0.0,
                    help='Extra meters added to each blob radius')
    ap.add_argument('--keep-walls', action='store_true',
                    help='Keep the largest blob (otherwise treated as the wall outline and dropped)')
    ap.add_argument('--wall-step',  type=int,   default=4,
                    help='When dropping the wall, sample wall cells every N as small obstacles (0=off)')
    args = ap.parse_args()

    blobs = extract_obstacles(args.map,
                              min_blob_cells=args.min_cells,
                              max_blob_cells=args.max_cells,
                              inflate_radius=args.inflate,
                              exclude_largest=not args.keep_walls,
                              sample_wall_step=args.wall_step)
    print(f'extracted {len(blobs)} obstacle blobs from {args.map}')
    for i, (x, y, r) in enumerate(blobs):
        print(f'  [{i:3d}]  centre=({x:+.2f}, {y:+.2f})  radius={r:.2f} m')


if __name__ == '__main__':
    main()
