"""
db_to_map_json.py — Convert an RTAB-Map .db into the web UI's map.json.

Pipeline:
    rtabmap-export --cloud --ascii  →  ASCII PLY  →  occupancy-grid map.json

Differences vs preprocess.py (which is for Scaniverse .obj):
  * Source is RTAB-Map .db, not a Scaniverse .obj.
  * RTAB-Map's world frame is Z-up (REP-103). The web UI / preprocess.py
    convention is Y-up. We remap on the way in:
        in.x  -> out.x   (horizontal)
        in.y  -> out.z   (horizontal)
        in.z  -> out.y   (height / floor_y)

Usage:
    python3 db_to_map_json.py --db tb3_1_room.db --output map.json
    python3 db_to_map_json.py --ply existing.ply --output map.json   # skip export
    python3 db_to_map_json.py --db ... --resolution 0.03             # finer grid

Requires:
    rtabmap-export  (brew install rtabmap)
    numpy, scipy    (system Python on macOS already has these)
"""

import argparse
import json
import os
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np


def export_db_to_ply(db_path, out_dir,
                     max_range=None,
                     min_range=None,
                     noise_radius=None,
                     noise_k=None,
                     prop_radius_factor=None):
    """Run rtabmap-export with per-camera range and noise filters.

    `max_range` is RTAB-Map's PER-CAMERA distance cap (Option 3 from chat —
    what we actually want). It's NOT a distance from world origin. Each
    point is kept iff captured within max_range of its originating
    keyframe's camera. Default in rtabmap-export is 4 m.

    `noise_radius` + `noise_k` = radius outlier removal at the source:
    drop any point with fewer than `noise_k` neighbors in `noise_radius`.

    `prop_radius_factor` = adaptive proportional-radius noise filter
    (start tuning from 0.01).
    """
    rtabmap_export = shutil.which('rtabmap-export')
    if not rtabmap_export:
        sys.exit("error: rtabmap-export not found. Install with `brew install rtabmap`.")
    name = 'tmp_rtabmap_cloud'
    cmd = [rtabmap_export, '--cloud', '--ascii',
           '--output_dir', out_dir, '--output', name]
    if max_range is not None:
        cmd += ['--max_range', str(max_range)]
    if min_range is not None:
        cmd += ['--min_range', str(min_range)]
    if noise_radius is not None:
        cmd += ['--noise_radius', str(noise_radius)]
    if noise_k is not None:
        cmd += ['--noise_k', str(noise_k)]
    if prop_radius_factor is not None:
        cmd += ['--prop_radius_factor', str(prop_radius_factor)]
    cmd.append(db_path)

    print(f"[export] running: {' '.join(cmd[1:])}")
    subprocess.run(cmd, check=True, timeout=300)

    candidates = sorted(Path(out_dir).glob(f'{name}*cloud*.ply'))
    if not candidates:
        candidates = sorted(Path(out_dir).glob(f'{name}*.ply'))
    if not candidates:
        sys.exit("error: rtabmap-export did not produce a .ply file.")
    return str(candidates[0])


def parse_ascii_ply(path):
    """Read an ASCII PLY into (xyz, rgb) numpy arrays. Tolerates extra
    properties (normals, curvature, etc.) by parsing the header."""
    prop_names = []
    n_vertices = 0
    header_lines = 0
    with open(path) as f:
        for header_lines, line in enumerate(f, start=1):
            line = line.strip()
            if line.startswith('element vertex'):
                n_vertices = int(line.split()[-1])
            elif line.startswith('property'):
                prop_names.append(line.split()[-1])
            elif line == 'end_header':
                break

    has_color = all(c in prop_names for c in ('red', 'green', 'blue'))
    x_idx = prop_names.index('x')
    y_idx = prop_names.index('y')
    z_idx = prop_names.index('z')
    r_idx = prop_names.index('red')   if has_color else -1
    g_idx = prop_names.index('green') if has_color else -1
    b_idx = prop_names.index('blue')  if has_color else -1
    n_props = len(prop_names)

    print(f"[ply] header: {n_vertices} vertices, {n_props} props, color={has_color}")

    # Vectorized read with numpy.loadtxt — fast for ASCII PLY
    data = np.loadtxt(path, skiprows=header_lines, max_rows=n_vertices, dtype=np.float64)
    xyz = data[:, [x_idx, y_idx, z_idx]]
    if has_color:
        rgb = data[:, [r_idx, g_idx, b_idx]].astype(np.uint8)
    else:
        rgb = None
    return xyz, rgb


def build_map_json(xyz, rgb,
                   resolution=0.05,
                   floor_band=0.15,
                   obstacle_min_height=0.1,
                   max_view_points=20000,
                   ceiling_above_floor=None,
                   bbox=None,
                   max_range=None):
    """Build the same map.json structure as preprocess.py but from a Z-up cloud.

    Axis remap (Z-up RTAB-Map -> Y-up web UI):
        in.x -> out.x
        in.y -> out.z
        in.z -> out.y  (height)

    Group-A filters (all optional, applied in order):
      ceiling_above_floor : drop points with height > floor_y + this (meters)
      bbox                : (x_min, x_max, z_min, z_max) — drop points outside
      max_range           : drop points farther than this (meters) from origin
    """
    out_x = xyz[:, 0]
    out_z = xyz[:, 1]
    out_y = xyz[:, 2]

    floor_y = float(np.percentile(out_y, 5))
    print(f"[grid] estimated floor (5th percentile of height): {floor_y:+.3f} m")
    n0 = len(out_x)

    # ---------- Group-A filters ----------
    keep = np.ones(n0, dtype=bool)

    if ceiling_above_floor is not None:
        ceil_y = floor_y + float(ceiling_above_floor)
        m = (out_y >= floor_y - 0.05) & (out_y <= ceil_y)
        dropped = int((~m).sum())
        keep &= m
        print(f"[A1] height cap [{floor_y - 0.05:+.2f}, {ceil_y:+.2f}] m  -> dropped {dropped}")

    if bbox is not None:
        x_lo, x_hi, z_lo, z_hi = bbox
        m = (out_x >= x_lo) & (out_x <= x_hi) & (out_z >= z_lo) & (out_z <= z_hi)
        dropped = int((~(m & keep)).sum() - (~keep).sum())
        keep &= m
        print(f"[A2] bbox crop  x[{x_lo},{x_hi}] z[{z_lo},{z_hi}]  -> dropped {dropped}")

    if max_range is not None:
        # NOTE: this is radial distance from the WORLD ORIGIN — wrong for a
        # moving robot (origin = where the robot started, not where it is).
        # Kept here for backwards compat but DON'T USE; prefer the native
        # rtabmap-export `--rtabmap-max-range` flag (per-camera, correct).
        r2 = out_x * out_x + out_z * out_z
        m = r2 <= float(max_range) ** 2
        dropped = int((~(m & keep)).sum() - (~keep).sum())
        keep &= m
        print(f"[A3 DEPRECATED] origin-radius filter r<={max_range}m  -> dropped {dropped}")
        print(f"[A3 DEPRECATED]   prefer --rtabmap-max-range (per-camera)")

    out_x = out_x[keep]
    out_y = out_y[keep]
    out_z = out_z[keep]
    if rgb is not None:
        rgb = rgb[keep]
    print(f"[filter] kept {len(out_x)} of {n0} points  ({100*len(out_x)/n0:.1f}%)")

    if len(out_x) == 0:
        sys.exit("error: all points filtered out — relax the Group-A flags.")

    min_x, max_x = float(out_x.min()), float(out_x.max())
    min_z, max_z = float(out_z.min()), float(out_z.max())
    width  = int(np.ceil((max_x - min_x) / resolution))
    height = int(np.ceil((max_z - min_z) / resolution))
    print(f"[grid] {width} x {height} cells at {resolution} m  "
          f"(x[{min_x:.2f},{max_x:.2f}], z[{min_z:.2f},{max_z:.2f}])")

    cols = np.clip(((out_x - min_x) / resolution).astype(np.int32), 0, width - 1)
    rows = np.clip(((out_z - min_z) / resolution).astype(np.int32), 0, height - 1)
    h_above = out_y - floor_y

    grid = np.zeros((height, width), dtype=np.uint8)

    # Mark obstacles first (anything tall enough)
    obs_mask = h_above >= obstacle_min_height
    if obs_mask.any():
        grid[rows[obs_mask], cols[obs_mask]] = 2

    # Mark ground where it isn't already obstacle
    flr_mask = h_above < floor_band
    if flr_mask.any():
        flr_rows = rows[flr_mask]
        flr_cols = cols[flr_mask]
        not_obs = grid[flr_rows, flr_cols] != 2
        grid[flr_rows[not_obs], flr_cols[not_obs]] = 1

    print(f"[grid] cells: ground={int((grid==1).sum())} "
          f"obstacle={int((grid==2).sum())} unknown={int((grid==0).sum())}")

    # Dilate obstacles so walls look solid
    try:
        from scipy.ndimage import binary_dilation
        grid[binary_dilation(grid == 2, iterations=1)] = 2
    except ImportError:
        print("[grid] scipy not available, skipping wall dilation")

    # Downsample for the browser's 3D view (over the filtered set)
    n_kept = len(out_x)
    step = max(1, n_kept // max_view_points)
    sel = np.arange(0, n_kept, step)
    pts_list = np.column_stack([out_x[sel], out_y[sel], out_z[sel]]).tolist()
    if rgb is not None:
        cols_list = rgb[sel].tolist()
    else:
        cols_list = [[150, 150, 150]] * len(sel)

    return {
        'metadata': {
            'resolution': resolution,
            'floor_y':    floor_y,
            'min_x':      min_x,
            'min_z':      min_z,
            'max_x':      max_x,
            'max_z':      max_z,
            'grid_width':  width,
            'grid_height': height,
            'point_count': len(pts_list),
        },
        'grid':   grid.tolist(),
        'points': pts_list,
        'colors': cols_list,
    }


def main():
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    src = p.add_mutually_exclusive_group(required=True)
    src.add_argument('--db',  help='RTAB-Map .db (will be exported via rtabmap-export)')
    src.add_argument('--ply', help='Pre-existing ASCII PLY (skip export step)')
    p.add_argument('--output',     default='map.json', help='Output map.json path')
    p.add_argument('--resolution', type=float, default=0.05,
                   help='Grid cell size in meters (default 0.05)')
    p.add_argument('--max-view-points', type=int, default=60000,
                   help='Downsample point cloud to this many points for the 3D view (default 60000 for richer 3D)')
    # ----- Group A filters -----
    p.add_argument('--ceiling-above-floor', type=float, default=None,
                   metavar='M',
                   help='[A1] Drop points more than this height (m) above floor')
    p.add_argument('--bbox', type=float, nargs=4, default=None,
                   metavar=('X_MIN', 'X_MAX', 'Z_MIN', 'Z_MAX'),
                   help='[A2] Crop to this floor bounding box (meters)')
    p.add_argument('--max-range', type=float, default=None, metavar='M',
                   help='[A3, DEPRECATED] Drop points farther than this radial distance from WORLD ORIGIN. '
                        'Wrong for moving-robot mapping. Use --rtabmap-max-range instead.')
    # ----- Option 3: native rtabmap-export filters (per-camera, correct) -----
    p.add_argument('--rtabmap-max-range', type=float, default=None, metavar='M',
                   help='[Opt 3] PER-CAMERA distance cap, applied at export time. '
                        'Each point kept iff captured within M meters of its keyframe.')
    p.add_argument('--rtabmap-min-range', type=float, default=None, metavar='M',
                   help='[Opt 3] Min per-camera distance (drops too-close noise).')
    p.add_argument('--rtabmap-noise-radius', type=float, default=None, metavar='M',
                   help='[Group B] Radius outlier removal at source (e.g. 0.05).')
    p.add_argument('--rtabmap-noise-k', type=int, default=None, metavar='N',
                   help='[Group B] Min neighbors required within --rtabmap-noise-radius (e.g. 5).')
    p.add_argument('--rtabmap-prop-radius', type=float, default=None, metavar='F',
                   help='[Group B] Adaptive proportional-radius noise filter factor (try 0.01).')
    args = p.parse_args()

    cleanup_dir = None
    try:
        if args.db:
            cleanup_dir = tempfile.mkdtemp()
            ply_path = export_db_to_ply(
                args.db, cleanup_dir,
                max_range=args.rtabmap_max_range,
                min_range=args.rtabmap_min_range,
                noise_radius=args.rtabmap_noise_radius,
                noise_k=args.rtabmap_noise_k,
                prop_radius_factor=args.rtabmap_prop_radius,
            )
        else:
            ply_path = args.ply

        xyz, rgb = parse_ascii_ply(ply_path)
        print(f"[parse] loaded {len(xyz)} points")

        out = build_map_json(xyz, rgb,
                             resolution=args.resolution,
                             max_view_points=args.max_view_points,
                             ceiling_above_floor=args.ceiling_above_floor,
                             bbox=tuple(args.bbox) if args.bbox else None,
                             max_range=args.max_range)

        with open(args.output, 'w') as f:
            json.dump(out, f)
        size_mb = os.path.getsize(args.output) / 1024 / 1024
        print(f"[save] wrote {args.output} ({size_mb:.1f} MB)")

    finally:
        if cleanup_dir:
            shutil.rmtree(cleanup_dir, ignore_errors=True)


if __name__ == '__main__':
    main()
