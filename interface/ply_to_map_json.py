"""
ply_to_map_json.py — Convert a PLY point cloud into the web UI's map.json.

Sibling of db_to_map_json.py with the rtabmap-export step removed. Use this
when you already have a PLY (from rtabmap-export, CloudCompare, Open3D,
MeshLab, Scaniverse → Open3D, etc.) and just need the cleaning + occupancy-
grid pipeline.

Pipeline:
    PLY (ASCII or binary)
      → SOR
      → DBSCAN cluster filter
      → RANSAC floor plane
      → floating-cluster filter
      → 2D occupancy grid + downsampled 3D points
      → map.json

Axis convention: Z-up (RTAB-Map / ROS REP-103) preserved end-to-end.
The web UI is also Z-up to match.
    in.x -> out.x   (horizontal, "forward" in ROS frame)
    in.y -> out.y   (horizontal, "left"    in ROS frame)
    in.z -> out.z   (vertical, height)

Differences from db_to_map_json.py:
  • No rtabmap-export call (no Stage 1 per-camera filters; those happen
    upstream when you produce the PLY).
  • Native binary-PLY support (header is always ASCII; body can be either
    ASCII or binary little/big endian).

Usage:
    python3 ply_to_map_json.py --ply room.ply --output map.json
    python3 ply_to_map_json.py --ply room.ply --no-sor                 # baseline
    python3 ply_to_map_json.py --ply room.ply --sor-std-ratio 1.5      # more aggressive
    python3 ply_to_map_json.py --ply room.ply --resolution 0.05        # coarser grid

Requires:
    numpy, scipy, scikit-learn
"""

import argparse
import json
import os
import sys
from pathlib import Path

import numpy as np


# ──────────────────────────────────────────────────────────────────────
# PLY parser — ASCII or binary little/big endian.
# Header is always ASCII regardless of body format.
# ──────────────────────────────────────────────────────────────────────

# Map PLY scalar type names to (numpy dtype char, byte size).
_PLY_TYPE_MAP = {
    'char':   ('i1', 1), 'int8':    ('i1', 1),
    'uchar':  ('u1', 1), 'uint8':   ('u1', 1),
    'short':  ('i2', 2), 'int16':   ('i2', 2),
    'ushort': ('u2', 2), 'uint16':  ('u2', 2),
    'int':    ('i4', 4), 'int32':   ('i4', 4),
    'uint':   ('u4', 4), 'uint32':  ('u4', 4),
    'float':  ('f4', 4), 'float32': ('f4', 4),
    'double': ('f8', 8), 'float64': ('f8', 8),
}


def parse_ply(path):
    """Read a PLY into (xyz, rgb) numpy arrays.

    Auto-detects ASCII vs binary little/big endian from the header. Pulls
    x/y/z + optional red/green/blue from the vertex element regardless of
    property order. List properties (e.g. face indices) inside the vertex
    element are not supported and aren't expected for point clouds.

    Returns:
        xyz : (N, 3) float64 array of positions
        rgb : (N, 3) uint8 array of colours, or None if no colour
    """
    fmt          = None
    n_vertices   = 0
    props        = []        # list of (name, np_type_char, n_bytes)
    header_lines = 0
    in_vertex    = False

    with open(path, 'rb') as f:
        # ── Header (always ASCII) ──
        while True:
            raw = f.readline()
            if not raw:
                sys.exit(f"error: PLY {path} ended before end_header")
            header_lines += 1
            line = raw.strip().decode('ascii', errors='replace')

            if line.startswith('format '):
                fmt = line.split()[1]
            elif line.startswith('element vertex'):
                n_vertices = int(line.split()[-1])
                in_vertex  = True
            elif line.startswith('element '):
                in_vertex = False
            elif line.startswith('property') and in_vertex:
                tokens = line.split()
                if tokens[1] == 'list':
                    sys.exit("error: list properties in vertex element are not supported")
                ptype, pname = tokens[1], tokens[2]
                if ptype not in _PLY_TYPE_MAP:
                    sys.exit(f"error: unknown PLY property type '{ptype}'")
                np_type, n_bytes = _PLY_TYPE_MAP[ptype]
                props.append((pname, np_type, n_bytes))

            if line == 'end_header':
                break

        if fmt is None or n_vertices == 0 or not props:
            sys.exit(f"error: malformed PLY header in {path}")

        prop_names = [p[0] for p in props]
        for required in ('x', 'y', 'z'):
            if required not in prop_names:
                sys.exit(f"error: PLY missing required property '{required}'")
        has_color = all(c in prop_names for c in ('red', 'green', 'blue'))

        print(f"[ply] header: format={fmt}, {n_vertices:,} vertices, "
              f"{len(props)} props, color={has_color}")

        # ── Body ──
        if fmt == 'ascii':
            # We've consumed the header lines off the binary file handle, so
            # the remaining stream IS the body. np.loadtxt wants a path-like
            # OR a file-like that yields strings — easiest: re-open via path
            # and skiprows past the header.
            data = np.loadtxt(path, skiprows=header_lines,
                              max_rows=n_vertices, dtype=np.float64)

            x_idx = prop_names.index('x')
            y_idx = prop_names.index('y')
            z_idx = prop_names.index('z')
            xyz = data[:, [x_idx, y_idx, z_idx]]
            if has_color:
                r_idx = prop_names.index('red')
                g_idx = prop_names.index('green')
                b_idx = prop_names.index('blue')
                rgb = data[:, [r_idx, g_idx, b_idx]].astype(np.uint8)
            else:
                rgb = None

        elif fmt in ('binary_little_endian', 'binary_big_endian'):
            byteorder = '<' if fmt == 'binary_little_endian' else '>'
            dtype = np.dtype([(p[0], byteorder + p[1]) for p in props])
            buf   = f.read(n_vertices * dtype.itemsize)
            if len(buf) != n_vertices * dtype.itemsize:
                sys.exit(f"error: PLY body shorter than header advertised "
                         f"({len(buf)} of {n_vertices * dtype.itemsize} bytes)")
            data = np.frombuffer(buf, dtype=dtype, count=n_vertices)

            xyz = np.column_stack([data['x'].astype(np.float64),
                                   data['y'].astype(np.float64),
                                   data['z'].astype(np.float64)])
            if has_color:
                rgb = np.column_stack([data['red'].astype(np.uint8),
                                       data['green'].astype(np.uint8),
                                       data['blue'].astype(np.uint8)])
            else:
                rgb = None
        else:
            sys.exit(f"error: unsupported PLY format '{fmt}'")

    return xyz, rgb


# ──────────────────────────────────────────────────────────────────────
# Cleaning stages — same algorithms as db_to_map_json.py, just collected
# here so this script is self-contained (no cross-import).
# ──────────────────────────────────────────────────────────────────────

def statistical_outlier_removal(xyz, rgb, nb_neighbors=20, std_ratio=1.5):
    """Drop points whose mean distance to their K nearest neighbours is
    more than `std_ratio` standard deviations above the cloud's mean.
    Pure-scipy implementation (no Open3D required)."""
    try:
        from scipy.spatial import cKDTree
    except ImportError:
        sys.exit("error: scipy required for SOR. Install with `pip3 install scipy`.")

    print(f"[sor] running on {len(xyz):,} points  "
          f"(k={nb_neighbors}, std_ratio={std_ratio})...")

    tree     = cKDTree(xyz)
    dists, _ = tree.query(xyz, k=nb_neighbors + 1)
    mean_d   = dists[:, 1:].mean(axis=1)            # skip self at index 0

    global_mean = float(mean_d.mean())
    global_std  = float(mean_d.std())
    threshold   = global_mean + std_ratio * global_std

    keep     = mean_d < threshold
    kept_xyz = xyz[keep]
    kept_rgb = rgb[keep] if rgb is not None else None

    n_kept = int(keep.sum())
    print(f"[sor] mean-dist global stats: mean={global_mean:.4f}m  "
          f"std={global_std:.4f}m  cutoff={threshold:.4f}m")
    print(f"[sor] kept {n_kept:,} of {len(xyz):,} "
          f"({100*n_kept/len(xyz):.1f}%); "
          f"dropped {len(xyz)-n_kept:,} outliers")
    return kept_xyz, kept_rgb


def dbscan_cluster_filter(xyz, rgb,
                          eps=None, min_samples=10, min_cluster_size=500):
    """Cluster points with DBSCAN; drop noise + clusters smaller than
    min_cluster_size. Catches floating ghost blobs that are dense enough
    to survive SOR but disconnected from real geometry."""
    try:
        from sklearn.cluster import DBSCAN
    except ImportError:
        sys.exit("error: scikit-learn required for DBSCAN. "
                 "Install with `pip3 install scikit-learn`.")

    print(f"[dbscan] clustering {len(xyz):,} points  "
          f"(eps={eps}, min_samples={min_samples})...")

    labels = DBSCAN(eps=eps, min_samples=min_samples, n_jobs=-1).fit(xyz).labels_

    n_clusters = int(labels.max()) + 1 if labels.max() >= 0 else 0
    n_noise    = int((labels == -1).sum())

    if n_clusters > 0:
        sizes = np.bincount(labels[labels >= 0])
    else:
        sizes = np.array([], dtype=np.int64)

    keep_cluster_ids = np.where(sizes >= min_cluster_size)[0]
    keep             = np.isin(labels, keep_cluster_ids)

    kept_xyz = xyz[keep]
    kept_rgb = rgb[keep] if rgb is not None else None
    n_kept   = int(keep.sum())

    big_top = sorted(sizes[keep_cluster_ids].tolist(), reverse=True)[:5]
    print(f"[dbscan] {n_clusters} clusters total; "
          f"{len(keep_cluster_ids)} kept (>= {min_cluster_size} pts), "
          f"{n_clusters - len(keep_cluster_ids)} too-small dropped, "
          f"{n_noise:,} noise dropped")
    if big_top:
        print(f"[dbscan]   top kept cluster sizes: {big_top}")
    print(f"[dbscan] kept {n_kept:,} of {len(xyz):,} "
          f"({100*n_kept/len(xyz):.1f}%)")
    return kept_xyz, kept_rgb


def ransac_floor_plane(xyz,
                       distance_threshold=0.02,
                       n_iterations=1000,
                       max_normal_tilt=0.15,
                       floor_search_band=0.30):
    """RANSAC plane fit on the bottom band of the cloud. Returns
    (floor_z, n_inliers); falls back to 5th-percentile-of-Z heuristic
    if no acceptable plane is found."""
    z_min = float(xyz[:, 2].min())
    z_max = float(xyz[:, 2].max())
    z_band_top = z_min + floor_search_band * (z_max - z_min)

    cand_mask  = xyz[:, 2] <= z_band_top
    candidates = xyz[cand_mask]
    print(f"[ransac] searching for floor in {len(candidates):,} of {len(xyz):,} pts "
          f"(bottom {floor_search_band*100:.0f}% of Z range, Z<={z_band_top:.3f})")

    if len(candidates) < 50:
        fz = float(np.percentile(xyz[:, 2], 5))
        print(f"[ransac] too few candidates, falling back to percentile  -> {fz:+.3f}")
        return fz, 0

    rng    = np.random.default_rng(42)
    n_cand = len(candidates)
    best_inliers = None
    best_count   = 0
    best_normal  = None

    for _ in range(n_iterations):
        idx = rng.choice(n_cand, 3, replace=False)
        p1, p2, p3 = candidates[idx]
        v1, v2 = p2 - p1, p3 - p1
        normal = np.cross(v1, v2)
        nlen   = np.linalg.norm(normal)
        if nlen < 1e-9:
            continue                                  # 3 collinear points
        normal /= nlen
        if normal[2] < 0:                             # always point up
            normal = -normal
        if normal[2] < (1.0 - max_normal_tilt):
            continue                                  # plane not horizontal enough

        d         = -float(normal.dot(p1))
        distances = np.abs(candidates @ normal + d)
        inliers   = distances < distance_threshold
        count     = int(inliers.sum())
        if count > best_count:
            best_inliers = inliers
            best_count   = count
            best_normal  = normal

    if best_inliers is None or best_count < 50:
        fz = float(np.percentile(xyz[:, 2], 5))
        print(f"[ransac] no good plane found (best inliers={best_count}), "
              f"falling back to percentile  -> {fz:+.3f}")
        return fz, 0

    floor_z  = float(np.median(candidates[best_inliers, 2]))
    tilt_deg = float(np.degrees(np.arccos(min(1.0, best_normal[2]))))
    print(f"[ransac] plane normal={best_normal.round(3).tolist()}  "
          f"tilt={tilt_deg:.2f}° from vertical  inliers={best_count:,}")
    print(f"[ransac] floor_z (median of inliers) = {floor_z:+.3f} m")
    return floor_z, best_count


def floor_support_cluster_filter(xyz, rgb, floor_z,
                                 cluster_eps=0.06,
                                 cluster_min_samples=10,
                                 floating_threshold=0.10):
    """Re-cluster with DBSCAN, then drop any cluster whose lowest point
    sits more than `floating_threshold` above the floor. Real obstacles
    touch the floor; floating ghosts hang in mid-air."""
    try:
        from sklearn.cluster import DBSCAN
    except ImportError:
        sys.exit("error: scikit-learn required. Install with `pip3 install scikit-learn`.")

    print(f"[float] re-clustering {len(xyz):,} points  "
          f"(eps={cluster_eps}, min_samples={cluster_min_samples})...")
    labels = DBSCAN(eps=cluster_eps,
                    min_samples=cluster_min_samples,
                    n_jobs=-1).fit(xyz).labels_

    n_clusters = int(labels.max()) + 1 if labels.max() >= 0 else 0
    n_noise    = int((labels == -1).sum())
    print(f"[float] {n_clusters} clusters found, {n_noise:,} noise points")

    keep = np.ones(len(xyz), dtype=bool)
    n_dropped_clusters = 0
    n_dropped_points   = 0

    diag = []
    for cid in range(n_clusters):
        m   = labels == cid
        low = float(xyz[m, 2].min() - floor_z)
        sz  = int(m.sum())
        if low > floating_threshold:
            keep &= ~m
            n_dropped_clusters += 1
            n_dropped_points   += sz
        if cid < 10:
            diag.append((cid, sz, low, low > floating_threshold))

    if diag:
        print("[float]   cluster_id   size   lowest_above_floor   floating?")
        for cid, sz, low, isf in diag:
            print(f"[float]   {cid:>10}  {sz:>6}   {low:+.3f} m            {isf}")

    n_kept = int(keep.sum())
    print(f"[float] dropped {n_dropped_clusters} floating clusters "
          f"({n_dropped_points:,} pts);  "
          f"kept {n_kept:,} of {len(xyz):,} ({100*n_kept/len(xyz):.1f}%)")
    return xyz[keep], (rgb[keep] if rgb is not None else None)


# ──────────────────────────────────────────────────────────────────────
# Stage 3: bin remaining points into the 2D occupancy grid + write JSON.
# ──────────────────────────────────────────────────────────────────────

def build_map_json(xyz, rgb,
                   resolution=0.02,
                   floor_band=0.15,
                   obstacle_min_height=0.10,
                   obstacle_min_points=5,
                   robot_height=0.10,
                   robot_clearance=0.05,
                   floor_z=None,
                   fill_ground=True,
                   fill_ground_close_iters=0):
    """Build map.json. Z-up convention (X, Y horizontal; Z height).

    Obstacle envelope:
        Only points whose height above the floor is in
            [obstacle_min_height,  robot_height + robot_clearance]
        count as obstacles. Anything above that is something the robot
        passes UNDER (table top, hanging cable, ceiling) — its column
        is marked passable / ground if no obstacles sit in the envelope.

    Ground filling (fill_ground=True):
        After the raw point bucketing, any unknown cell that's fully
        enclosed by known cells (ground + the dilated obstacles around
        it) is promoted to ground. This closes natural gaps in sparse
        floor scans so the path planner doesn't see "interior pockets"
        of unknown space inside an otherwise-mapped area.
        fill_ground_close_iters > 0 additionally runs a binary closing
        on the ground mask first (size = N cells), useful when the
        floor scan is patchy enough that unknowns aren't fully enclosed
        yet.
    """
    out_x = xyz[:, 0]
    out_y = xyz[:, 1]
    out_z = xyz[:, 2]

    if floor_z is None:
        floor_z = float(np.percentile(out_z, 5))
        print(f"[grid] estimated floor (5th percentile of Z): {floor_z:+.3f} m")
    else:
        print(f"[grid] floor_z (provided): {floor_z:+.3f} m")

    envelope_top = robot_height + robot_clearance
    print(f"[grid] robot envelope: [{obstacle_min_height:+.2f}, {envelope_top:+.2f}] m "
          f"above floor  (height={robot_height} + clearance={robot_clearance})")

    min_x, max_x = float(out_x.min()), float(out_x.max())
    min_y, max_y = float(out_y.min()), float(out_y.max())
    width  = int(np.ceil((max_x - min_x) / resolution))
    height = int(np.ceil((max_y - min_y) / resolution))
    print(f"[grid] {width} x {height} cells at {resolution} m  "
          f"(x[{min_x:.2f},{max_x:.2f}], y[{min_y:.2f},{max_y:.2f}])")

    cols = np.clip(((out_x - min_x) / resolution).astype(np.int32), 0, width - 1)
    rows = np.clip(((out_y - min_y) / resolution).astype(np.int32), 0, height - 1)
    h_above = out_z - floor_z

    grid = np.zeros((height, width), dtype=np.uint8)

    # 1. Obstacles: enough points inside the robot's vertical envelope.
    obs_mask = (h_above >= obstacle_min_height) & (h_above <= envelope_top)
    if obs_mask.any():
        obs_count = np.zeros((height, width), dtype=np.int32)
        np.add.at(obs_count, (rows[obs_mask], cols[obs_mask]), 1)
        grid[obs_count >= obstacle_min_points] = 2

    # 2. Ground from real floor points.
    flr_mask = h_above < floor_band
    if flr_mask.any():
        flr_rows = rows[flr_mask]
        flr_cols = cols[flr_mask]
        not_obs  = grid[flr_rows, flr_cols] != 2
        grid[flr_rows[not_obs], flr_cols[not_obs]] = 1

    # 3. Passable-under-overhead cells: stuff only above the envelope
    #    (e.g. table top with nothing scanned below it).
    above_mask = h_above > envelope_top
    if above_mask.any():
        above_count = np.zeros((height, width), dtype=np.int32)
        np.add.at(above_count, (rows[above_mask], cols[above_mask]), 1)
        passable_under = (above_count >= obstacle_min_points) & (grid != 2)
        n_under = int(passable_under.sum())
        if n_under:
            print(f"[grid] passable-under-overhead cells promoted to ground: {n_under}")
        grid[passable_under] = 1

    print(f"[grid] cells: ground={int((grid==1).sum())} "
          f"obstacle={int((grid==2).sum())} unknown={int((grid==0).sum())}")

    try:
        from scipy.ndimage import binary_dilation, binary_closing, binary_fill_holes
        scipy_ok = True
    except ImportError:
        scipy_ok = False

    if scipy_ok:
        # Wall dilation (same as before).
        grid[binary_dilation(grid == 2, iterations=1)] = 2

        if fill_ground:
            # Optional pre-step: morphological closing on ground only.
            # Fills small gaps in the ground mask without crossing
            # obstacles. Default iterations=0 means skip — turn on when
            # the floor scan is so patchy that the hole-fill below
            # can't enclose unknown regions on its own.
            if fill_ground_close_iters > 0:
                ground = (grid == 1)
                closed = binary_closing(ground, iterations=fill_ground_close_iters)
                # Don't overwrite obstacles when closing pulls into them.
                added = closed & (grid != 2) & ~ground
                n_close = int(added.sum())
                grid[added] = 1
                print(f"[fill] ground closing (iters={fill_ground_close_iters}): "
                      f"+{n_close:,} ground cells")

            # Main fill: any unknown cell that's fully enclosed by
            # known (ground OR obstacle) cells becomes ground. The
            # boundary of the grid is treated as exterior, so unknown
            # cells touching the edge stay unknown.
            explored      = (grid != 0)
            filled_region = binary_fill_holes(explored)
            new_ground    = filled_region & (grid == 0)
            n_fill        = int(new_ground.sum())
            grid[new_ground] = 1
            print(f"[fill] hole-fill: +{n_fill:,} ground cells in enclosed unknowns")

            print(f"[fill] cells (post): ground={int((grid==1).sum())} "
                  f"obstacle={int((grid==2).sum())} unknown={int((grid==0).sum())}")
    else:
        print("[grid] scipy not available, skipping wall dilation + ground fill")

    pts_list = np.column_stack([out_x, out_y, out_z]).tolist()
    if rgb is not None:
        cols_list = rgb.tolist()
    else:
        cols_list = [[150, 150, 150]] * len(out_x)
    print(f"[view] writing {len(pts_list):,} points to JSON (no downsampling)")

    return {
        'metadata': {
            'resolution':       resolution,
            'floor_z':          floor_z,
            'robot_height':     robot_height,
            'robot_clearance':  robot_clearance,
            'min_x':            min_x,
            'min_y':            min_y,
            'max_x':            max_x,
            'max_y':            max_y,
            'grid_width':       width,
            'grid_height':      height,
            'point_count':      len(pts_list),
            'axis_convention':  'z-up',
        },
        'grid':   grid.tolist(),
        'points': pts_list,
        'colors': cols_list,
    }


# ──────────────────────────────────────────────────────────────────────
# CLI
# ──────────────────────────────────────────────────────────────────────

def main():
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument('--ply',    required=True,
                   help='Path to the input PLY file (ASCII or binary).')
    p.add_argument('--output', default='map.json',
                   help='Output map.json path (default: map.json)')
    p.add_argument('--resolution', type=float, default=0.02,
                   help='Grid cell size in metres (default 0.02 = 2 cm).')

    # ----- Cleaning stages -----
    p.add_argument('--no-sor', action='store_true',
                   help='Disable Statistical Outlier Removal (default: ON).')
    p.add_argument('--sor-neighbors', type=int, default=20,
                   help='SOR: K nearest neighbours per point (default 20).')
    p.add_argument('--sor-std-ratio', type=float, default=1.5,
                   help='SOR: drop if mean-distance > this many sigma '
                        '(default 1.5; lower = more aggressive).')

    p.add_argument('--no-dbscan', action='store_true',
                   help='Disable DBSCAN cluster filter (default: ON).')
    p.add_argument('--dbscan-eps', type=float, default=0.02,
                   help='DBSCAN: max distance between neighbours '
                        '(metres, default 0.02 = 2 cm).')
    p.add_argument('--dbscan-min-samples', type=int, default=10,
                   help='DBSCAN: min neighbours within eps to be a core point '
                        '(default 10).')
    p.add_argument('--dbscan-min-cluster-size', type=int, default=500,
                   help='DBSCAN: drop clusters smaller than this many points '
                        '(default 500).')

    p.add_argument('--no-ransac', action='store_true',
                   help='Disable RANSAC floor detection (default: ON; falls back '
                        'to 5th-percentile-of-Z heuristic).')
    p.add_argument('--ransac-threshold', type=float, default=0.02,
                   help='RANSAC: max distance (m) from plane to count as inlier '
                        '(default 0.02 = 2 cm).')
    p.add_argument('--ransac-tilt', type=float, default=0.15,
                   help='RANSAC: how far the plane normal can tilt from vertical '
                        '(0.0 = perfectly vertical only, 1.0 = no constraint, '
                        'default 0.15).')
    p.add_argument('--ransac-iterations', type=int, default=1000,
                   help='RANSAC: number of random trials (default 1000).')
    p.add_argument('--ransac-search-band', type=float, default=0.30,
                   help='RANSAC: only consider points in the bottom fraction of '
                        'the Z range as floor candidates (default 0.30).')

    p.add_argument('--no-floating', action='store_true',
                   help='Disable floating-cluster filter (default: ON).')
    p.add_argument('--floating-threshold', type=float, default=0.10, metavar='M',
                   help='Floating filter: a cluster is "floating" if its lowest '
                        'point is more than M m above the floor (default 0.10).')
    p.add_argument('--floating-eps', type=float, default=0.06, metavar='M',
                   help='Floating filter: DBSCAN eps for re-clustering '
                        '(default 0.06 = 6 cm).')
    p.add_argument('--floating-min-samples', type=int, default=10,
                   help='Floating filter: DBSCAN min_samples for re-clustering '
                        '(default 10).')

    # ----- Grid stage -----
    p.add_argument('--obstacle-min-points', type=int, default=5,
                   help='Min points above obstacle_min_height needed to mark a '
                        'cell as obstacle (default 5). Higher = noise-suppressed '
                        'but real thin obstacles may disappear.')
    p.add_argument('--robot-height', type=float, default=0.10, metavar='M',
                   help='Robot height in metres (default 0.10). Only points '
                        'within [obstacle_min_height, robot_height + clearance] '
                        'count as obstacles.')
    p.add_argument('--robot-clearance', type=float, default=0.05, metavar='M',
                   help='Vertical safety clearance added on top of robot_height '
                        '(default 0.05 = 5 cm).')

    # ----- Ground fill -----
    p.add_argument('--no-fill-ground', action='store_true',
                   help='Disable filling enclosed unknown cells with ground '
                        '(default: ON). Turn this off if you specifically want '
                        'to see which cells the SLAM scan never covered.')
    p.add_argument('--fill-ground-close', type=int, default=3, metavar='N',
                   help='Pre-fill morphological closing on the ground mask, '
                        'in cells (default 0 = off). Useful when the floor '
                        'scan is so patchy that unknowns are not fully '
                        'enclosed; try 1-3 for sparse scans.')

    args = p.parse_args()

    if not Path(args.ply).is_file():
        sys.exit(f"error: input PLY not found: {args.ply}")

    # ── Load ──
    xyz, rgb = parse_ply(args.ply)
    print(f"[parse] loaded {len(xyz):,} points")

    # ── Cleaning ──
    if not args.no_sor:
        xyz, rgb = statistical_outlier_removal(
            xyz, rgb,
            nb_neighbors=args.sor_neighbors,
            std_ratio   =args.sor_std_ratio)

    if not args.no_dbscan:
        xyz, rgb = dbscan_cluster_filter(
            xyz, rgb,
            eps              = args.dbscan_eps,
            min_samples      = args.dbscan_min_samples,
            min_cluster_size = args.dbscan_min_cluster_size)

    floor_z = None
    if not args.no_ransac:
        floor_z, _ = ransac_floor_plane(
            xyz,
            distance_threshold = args.ransac_threshold,
            n_iterations       = args.ransac_iterations,
            max_normal_tilt    = args.ransac_tilt,
            floor_search_band  = args.ransac_search_band)

    if not args.no_floating:
        fz = floor_z if floor_z is not None else float(np.percentile(xyz[:, 2], 5))
        xyz, rgb = floor_support_cluster_filter(
            xyz, rgb, floor_z=fz,
            cluster_eps         = args.floating_eps,
            cluster_min_samples = args.floating_min_samples,
            floating_threshold  = args.floating_threshold)

    # ── Grid + JSON ──
    out = build_map_json(
        xyz, rgb,
        resolution              = args.resolution,
        obstacle_min_points     = args.obstacle_min_points,
        robot_height            = args.robot_height,
        robot_clearance         = args.robot_clearance,
        floor_z                 = floor_z,
        fill_ground             = not args.no_fill_ground,
        fill_ground_close_iters = args.fill_ground_close,
    )

    with open(args.output, 'w') as f:
        json.dump(out, f)
    size_mb = os.path.getsize(args.output) / 1024 / 1024
    print(f"[save] wrote {args.output} ({size_mb:.1f} MB)")


if __name__ == '__main__':
    main()
