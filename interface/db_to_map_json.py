"""
db_to_map_json.py — Convert an RTAB-Map .db into the web UI's map.json.

Pipeline:
    rtabmap-export --cloud --ascii  →  ASCII PLY  →  SOR  →  occupancy-grid map.json

Axis convention: Z-up (RTAB-Map / ROS REP-103) preserved end-to-end.
The web UI is also Z-up to match.
    in.x -> out.x   (horizontal, "forward" in ROS frame)
    in.y -> out.y   (horizontal, "left"    in ROS frame)
    in.z -> out.z   (vertical, height)

Point-cloud cleaning:
  Stage 1 — rtabmap-export per-camera filters (--rtabmap-*).
            Per-camera = correct for moving robot. Kept on by default.
  Stage 2 — Statistical Outlier Removal on the loaded cloud (--sor-*).
            On by default. Drops isolated noise.
  Stage 3 — DBSCAN cluster filter (--dbscan-*). On by default.
            Drops floating ghost blobs of < min_cluster_size points.

Future (slots reserved in main()):
  - RANSAC plane fit for robust floor detection
  - Voxel downsample for very dense clouds

Usage:
    python3 db_to_map_json.py --db tb3_1_room.db --output map.json
    python3 db_to_map_json.py --db ... --no-sor                       # baseline
    python3 db_to_map_json.py --db ... --sor-std-ratio 1.5            # more aggressive
    python3 db_to_map_json.py --ply existing.ply --output map.json    # skip export

Requires:
    rtabmap-export  (brew install rtabmap)
    numpy, scipy, open3d
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
    """Run rtabmap-export with per-camera range and noise filters."""
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
    """Read an ASCII PLY into (xyz, rgb) numpy arrays."""
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

    data = np.loadtxt(path, skiprows=header_lines, max_rows=n_vertices, dtype=np.float64)
    xyz = data[:, [x_idx, y_idx, z_idx]]
    if has_color:
        rgb = data[:, [r_idx, g_idx, b_idx]].astype(np.uint8)
    else:
        rgb = None
    return xyz, rgb


# ──────────────────────────────────────────────────────────────────────
# Stage 2 dynamic filters (each is its own function so we can add / remove
# them one at a time and watch the diff).
# ──────────────────────────────────────────────────────────────────────

def voxel_downsample(xyz, rgb, voxel_size=0.03):
    """Replace each voxel's points with the mean position and mean colour
    of the points inside it.

    Normalises density: SLAM clouds are highly non-uniform — areas the
    camera observed many times are dense, areas observed once are sparse.
    SOR and DBSCAN both implicitly assume uniform density to set their
    thresholds. Running voxel downsample first makes those filters far
    more predictable.

    voxel_size: cube edge length in metres (default 0.03 = 3 cm).
                Pick smaller than your --resolution so you don't lose
                grid-relevant detail. With 5 cm grid, 3 cm voxel is safe.

    Returns:
        (downsampled_xyz, downsampled_rgb)
    """
    print(f"[voxel] running on {len(xyz):,} points  "
          f"(voxel_size={voxel_size}m)...")

    min_xyz = xyz.min(axis=0)
    cells   = np.floor((xyz - min_xyz) / voxel_size).astype(np.int32)

    unique_cells, inverse, counts = np.unique(
        cells, axis=0, return_inverse=True, return_counts=True)
    n_unique = len(unique_cells)

    sum_xyz = np.zeros((n_unique, 3), dtype=np.float64)
    np.add.at(sum_xyz, inverse, xyz)
    mean_xyz = (sum_xyz / counts[:, None]).astype(np.float32)

    if rgb is not None:
        sum_rgb = np.zeros((n_unique, 3), dtype=np.float64)
        np.add.at(sum_rgb, inverse, rgb)
        mean_rgb = (sum_rgb / counts[:, None]).round().clip(0, 255).astype(np.uint8)
    else:
        mean_rgb = None

    print(f"[voxel] {len(xyz):,} → {n_unique:,} points  "
          f"({100*n_unique/len(xyz):.1f}%, voxel size {voxel_size*100:.1f} cm)")
    return mean_xyz, mean_rgb

def statistical_outlier_removal(xyz, rgb, nb_neighbors=20, std_ratio=None):
    """Drop points whose mean distance to their K nearest neighbours is
    more than `std_ratio` standard deviations above the cloud's mean.

    Pure-scipy implementation (no open3d required). Uses a cKDTree for
    the neighbour query — C-backed, fast on hundreds of thousands of
    points.

    Returns:
        (kept_xyz, kept_rgb)
    """
    try:
        from scipy.spatial import cKDTree
    except ImportError:
        sys.exit("error: scipy required for SOR. Install with `pip3 install scipy`.")

    print(f"[sor] running on {len(xyz):,} points  "
          f"(k={nb_neighbors}, std_ratio={std_ratio})...")

    # Query k+1 because the first neighbour of any point is itself (dist 0).
    tree   = cKDTree(xyz)
    dists, _ = tree.query(xyz, k=nb_neighbors + 1)
    mean_d = dists[:, 1:].mean(axis=1)

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
                          eps=None, min_samples=None, min_cluster_size=None):
    """Cluster points with DBSCAN; drop noise + clusters smaller than
    min_cluster_size.

    Catches a different failure mode than SOR: floating ghost blobs from
    SLAM drift / depth glitches usually contain enough nearby points to
    survive SOR (because within the blob the density is fine), but the
    blob as a whole is small and disconnected from the real geometry —
    which is exactly what cluster filtering throws away.

    eps              : max distance between two points to be considered
                       neighbours, in metres (default 0.06 = 6 cm).
    min_samples      : how many neighbours within eps a point needs to
                       count as a "core" point (DBSCAN density rule).
    min_cluster_size : drop any cluster smaller than this many points.

    Returns:
        (kept_xyz, kept_rgb)
    """
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

    # Per-cluster sizes; -1 (noise) is not a cluster.
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

def floor_support_cluster_filter(xyz, rgb, floor_z,
                                 cluster_eps=0.06,
                                 cluster_min_samples=10,
                                 floating_threshold=0.10):
    """Re-cluster the cloud with DBSCAN, then drop any cluster whose
    LOWEST point sits more than `floating_threshold` above the floor.

    Real obstacles touch the floor (or come within a few cm of it).
    Floating ghosts hang in mid-air with their lowest point well above.
    """
    from sklearn.cluster import DBSCAN
    print(f"[float] re-clustering {len(xyz):,} points  "
          f"(eps={cluster_eps}, min_samples={cluster_min_samples})...")
    labels = DBSCAN(eps=cluster_eps,
                    min_samples=cluster_min_samples,
                    n_jobs=-1).fit(xyz).labels_

    n_clusters = int(labels.max()) + 1 if labels.max() >= 0 else 0
    n_noise    = int((labels == -1).sum())
    print(f"[float] {n_clusters} clusters found, {n_noise:,} noise points")

    keep      = np.ones(len(xyz), dtype=bool)
    n_dropped_clusters = 0
    n_dropped_points   = 0

    # Diagnostic: show the lowest-above-floor for the first 10 clusters
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

def ransac_floor_plane(xyz,
                       distance_threshold=0.02,
                       n_iterations=1000,
                       max_normal_tilt=0.15,
                       floor_search_band=0.30):
    """Fit the floor plane via RANSAC; return a single scalar floor_z
    (median Z of the inlier points).

    Why RANSAC instead of "5th percentile of Z":
      - Robust to obstacles that happen to have low points (a mat, a base of a chair).
      - Robust to noise points *below* the actual floor.
      - Works correctly when the floor itself has small bumps or noise.

    Algorithm:
      Restrict search to the bottom `floor_search_band` of the Z range.
      Repeatedly pick 3 random points, fit a plane through them, discard
      planes whose normal is too far from vertical (so we don't accidentally
      lock onto a wall), count inliers (points within `distance_threshold`
      of the plane). Keep the best.

    Returns:
        (floor_z, n_inliers)
        floor_z: scalar — median Z of the inlier points.
                 Falls back to 5th-percentile heuristic if RANSAC fails.
    """
    z_min = float(xyz[:, 2].min())
    z_max = float(xyz[:, 2].max())
    z_band_top = z_min + floor_search_band * (z_max - z_min)

    cand_mask = xyz[:, 2] <= z_band_top
    candidates = xyz[cand_mask]
    print(f"[ransac] searching for floor in {len(candidates):,} of {len(xyz):,} pts "
          f"(bottom {floor_search_band*100:.0f}% of Z range, Z<={z_band_top:.3f})")

    if len(candidates) < 50:
        fz = float(np.percentile(xyz[:, 2], 5))
        print(f"[ransac] too few candidates, falling back to percentile  -> {fz:+.3f}")
        return fz, 0

    rng     = np.random.default_rng(42)
    n_cand  = len(candidates)
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
            continue                                    # the 3 points were collinear
        normal /= nlen
        if normal[2] < 0:                               # always point up
            normal = -normal
        if normal[2] < (1.0 - max_normal_tilt):
            continue                                    # not horizontal enough; skip

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

# ──────────────────────────────────────────────────────────────────────
# Stage 3: bin remaining points into the 2D occupancy grid.
# ──────────────────────────────────────────────────────────────────────

def build_map_json(xyz, rgb,
                   resolution=None,
                   floor_band=0.15,
                   obstacle_min_height=0.10,
                   obstacle_min_points=None,
                   robot_height=None,           # ← new
                   robot_clearance=None,        # ← new
                   floor_z=None):
    """Build map.json. Z-up convention (X, Y horizontal; Z height).

    Obstacle envelope:
        Only points whose height above the floor is in
            [obstacle_min_height,  robot_height + robot_clearance]
        count as obstacles. Anything above that is something the robot
        passes UNDER (table top, hanging cable, ceiling) — its column
        is marked passable / ground if no obstacles sit in the envelope.
    """
    out_x = xyz[:, 0]
    out_y = xyz[:, 1]    # ← negate
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

    # ── 1. Obstacles: ENOUGH points inside the robot's vertical envelope ──
    obs_mask = (h_above >= obstacle_min_height) & (h_above <= envelope_top)
    if obs_mask.any():
        obs_count = np.zeros((height, width), dtype=np.int32)
        np.add.at(obs_count, (rows[obs_mask], cols[obs_mask]), 1)
        grid[obs_count >= obstacle_min_points] = 2

    # ── 2. Ground from real floor points ──
    flr_mask = h_above < floor_band
    if flr_mask.any():
        flr_rows = rows[flr_mask]
        flr_cols = cols[flr_mask]
        not_obs  = grid[flr_rows, flr_cols] != 2
        grid[flr_rows[not_obs], flr_cols[not_obs]] = 1

    # ── 3. "Passable under": cells with stuff ONLY above the envelope ──
    # If the column has scanned geometry but it's all above the robot's
    # envelope (e.g. a table top at 80 cm with nothing below it down to
    # the floor in the scan), the robot can drive there.
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
        from scipy.ndimage import binary_dilation
        grid[binary_dilation(grid == 2, iterations=1)] = 2
    except ImportError:
        print("[grid] scipy not available, skipping wall dilation")
    # grid = np.fliplr(grid)

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


def main():
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    src = p.add_mutually_exclusive_group(required=True)
    src.add_argument('--db',  help='RTAB-Map .db (will be exported via rtabmap-export)')
    src.add_argument('--ply', help='Pre-existing ASCII PLY (skip export step)')
    p.add_argument('--output',     default='map.json', help='Output map.json path')
    p.add_argument('--resolution', type=float, default=0.02,
                   help='Grid cell size in meters (default 0.05)')
    
    # ----- Stage 1: rtabmap-export per-camera filters -----
    p.add_argument('--rtabmap-max-range', type=float, default=3.0, metavar='M',
                   help='[Stage1] PER-CAMERA distance cap (default 3.0 m).')
    p.add_argument('--rtabmap-min-range', type=float, default=None, metavar='M',
                   help='[Stage1] Min per-camera distance.')
    p.add_argument('--rtabmap-noise-radius', type=float, default=None, metavar='M',
                   help='[Stage1] Radius outlier removal at source.')
    p.add_argument('--rtabmap-noise-k', type=int, default=None, metavar='N',
                   help='[Stage1] Min neighbors required within --rtabmap-noise-radius.')
    p.add_argument('--rtabmap-prop-radius', type=float, default=None, metavar='F',
                   help='[Stage1] Adaptive proportional-radius factor (try 0.01).')

    # ----- Stage 2: dynamic cleaning on the loaded cloud -----
    
    # p.add_argument('--no-voxel', action='store_true',
    #                help='Disable voxel downsampling (default: ON).')
    # p.add_argument('--voxel-size', type=float, default=0.03, metavar='M',
    #                help='Voxel edge length in metres (default 0.03 = 3 cm). '
    #                     'Keep < --resolution to avoid losing grid detail.')
    
    p.add_argument('--no-sor', action='store_true',
                   help='Disable Statistical Outlier Removal (default: ON).')
    p.add_argument('--sor-neighbors', type=int, default=20,
                   help='SOR: K nearest neighbours per point (default 20).')
    p.add_argument('--sor-std-ratio', type=float, default=1.5,
                   help='SOR: drop if mean-distance > this many sigma '
                        '(default 2.0; lower = more aggressive).')

    p.add_argument('--no-dbscan', action='store_true',
                   help='Disable DBSCAN cluster filter (default: ON).')
    p.add_argument('--dbscan-eps', type=float, default=0.02,
                   help='DBSCAN: max distance between neighbours (m, default 0.1).')
    p.add_argument('--dbscan-min-samples', type=int, default=10,
                   help='DBSCAN: min neighbours within eps to be a core point (default 10).')
    p.add_argument('--dbscan-min-cluster-size', type=int, default=500,
                   help='DBSCAN: drop clusters smaller than this many points (default 500).')
    
    p.add_argument('--no-floating', action='store_true',
                   help='Disable floating-cluster filter (default: ON).')
    p.add_argument('--floating-threshold', type=float, default=0.10, metavar='M',
                   help='Floating filter: a cluster is "floating" if its lowest '
                        'point is more than M m above the floor (default 0.10).')
    p.add_argument('--floating-eps', type=float, default=0.02, metavar='M',
                   help='Floating filter: DBSCAN eps for re-clustering (default 0.06).')
    p.add_argument('--floating-min-samples', type=int, default=10,
                   help='Floating filter: DBSCAN min_samples for re-clustering '
                        '(default 10).')

    p.add_argument('--no-ransac', action='store_true',
                   help='Disable RANSAC floor detection (default: ON; falls back to '
                        '5th-percentile-of-Z heuristic).')
    p.add_argument('--ransac-threshold', type=float, default=0.02,
                   help='RANSAC: max distance (m) from plane to count as inlier '
                        '(default 0.02 = 2 cm).')
    p.add_argument('--ransac-tilt', type=float, default=0.15,
                   help='RANSAC: how far the plane normal can tilt from vertical '
                        '(0.0 = perfectly vertical only, 1.0 = no constraint, default 0.15).')
    p.add_argument('--ransac-iterations', type=int, default=1000,
                   help='RANSAC: number of random trials (default 1000).')
    p.add_argument('--ransac-search-band', type=float, default=0.30,
                   help='RANSAC: only consider points in the bottom fraction of '
                        'the Z range as floor candidates (default 0.30).')
    
    p.add_argument('--obstacle-min-points', type=int, default=5,
                   help='Min points above obstacle_min_height needed to mark a '
                        'cell as obstacle (default 3). Higher = noise-suppressed '
                        'but real thin obstacles may disappear.')
    p.add_argument('--robot-height', type=float, default=0.10, metavar='M',
                   help='Robot height in metres (default 0.10). Only points within '
                        '[obstacle_min_height, robot_height + clearance] count as obstacles.')
    p.add_argument('--robot-clearance', type=float, default=0.05, metavar='M',
                   help='Vertical safety clearance added on top of robot_height '
                        '(default 0.05 = 5 cm).')
    
    args = p.parse_args()

    cleanup_dir = None
    try:
        # ── Stage 1 ──────────────────────────────────────────────────────
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
        print(f"[parse] loaded {len(xyz):,} points")

        # ── Stage 2: dynamic filters ────────────────────────────────────
        # if not args.no_voxel:
        #     xyz, rgb = voxel_downsample(xyz, rgb, voxel_size=args.voxel_size)

        if not args.no_sor:
            xyz, rgb = statistical_outlier_removal(
                xyz, rgb,
                nb_neighbors=args.sor_neighbors,
                std_ratio=args.sor_std_ratio)

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
            
        # TODO (later): RANSAC plane fit for robust floor detection

        # ── Stage 3 ──────────────────────────────────────────────────────
        out = build_map_json(xyz, rgb,
                             resolution=args.resolution,
                             obstacle_min_points=args.obstacle_min_points,
                             robot_height=args.robot_height,
                             robot_clearance=args.robot_clearance,
                             floor_z=floor_z)

        with open(args.output, 'w') as f:
            json.dump(out, f)
        size_mb = os.path.getsize(args.output) / 1024 / 1024
        print(f"[save] wrote {args.output} ({size_mb:.1f} MB)")

    finally:
        if cleanup_dir:
            shutil.rmtree(cleanup_dir, ignore_errors=True)


if __name__ == '__main__':
    main()