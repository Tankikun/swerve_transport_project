#!/usr/bin/env python3
"""
clean_map.py — single-file map cleanup pipeline.

Six stages, all inline:
  1. rtabmap-export       — per-camera distance + noise filters
  2. Statistical Outlier  — drop isolated noise points
  3. DBSCAN clustering    — drop tiny floating ghost blobs
  4. RANSAC floor plane   — robust floor_z (vs. 5th-percentile fallback)
  5. Floating-cluster     — re-cluster, drop blobs hanging in mid-air
  6. Grid build + inpaint — 2D occupancy grid + closing + flood-fill +
                            synthetic floor points for the viewer

Plus a top-down verification JPEG (2D grid + cleaned point cloud).

Usage:
    /usr/bin/python3 clean_map.py path/to/your.db
    /usr/bin/python3 clean_map.py path/to/your.db --output ./room2.json
    /usr/bin/python3 clean_map.py path/to/your.ply         # skip Stage 1
    /usr/bin/python3 clean_map.py room.db --no-inpaint --no-preview

Tuning knobs (defaults match tb3_1_room — a 3-4 m square indoor lab):
    --resolution 0.05            (5 cm cells; default 0.02)
    --robot-height 0.20          (taller robot stack)
    --robot-clearance 0.10       (more headroom)
    --dbscan-min-cluster-size 200  (less aggressive — keep small obstacles)

Run with /usr/bin/python3 (Homebrew 3.14 has broken pyexpat).
Requires: numpy, scipy, scikit-learn, matplotlib, rtabmap-export.
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


# ════════════════════════════════════════════════════════════════════════
# Stage 1 — rtabmap-export: .db → ASCII PLY with per-camera filters
# ════════════════════════════════════════════════════════════════════════

def export_db_to_ply(db_path, out_dir,
                     max_range=None, min_range=None,
                     noise_radius=None, noise_k=None,
                     prop_radius_factor=None):
    """Run rtabmap-export with per-camera range and noise filters."""
    rtabmap_export = shutil.which("rtabmap-export")
    if not rtabmap_export:
        sys.exit("error: rtabmap-export not found. Install with `brew install rtabmap`.")
    name = "tmp_rtabmap_cloud"
    cmd = [rtabmap_export, "--cloud", "--ascii",
           "--output_dir", out_dir, "--output", name]
    if max_range is not None:          cmd += ["--max_range", str(max_range)]
    if min_range is not None:          cmd += ["--min_range", str(min_range)]
    if noise_radius is not None:       cmd += ["--noise_radius", str(noise_radius)]
    if noise_k is not None:            cmd += ["--noise_k", str(noise_k)]
    if prop_radius_factor is not None: cmd += ["--prop_radius_factor", str(prop_radius_factor)]
    cmd.append(db_path)

    print(f"[stage1] rtabmap-export {' '.join(cmd[1:])}")
    subprocess.run(cmd, check=True, timeout=300)

    candidates = sorted(Path(out_dir).glob(f"{name}*cloud*.ply"))
    if not candidates:
        candidates = sorted(Path(out_dir).glob(f"{name}*.ply"))
    if not candidates:
        sys.exit("error: rtabmap-export did not produce a .ply file.")
    return str(candidates[0])


def parse_ascii_ply(path):
    """Read an ASCII PLY into (xyz, rgb) numpy arrays."""
    prop_names, n_vertices, header_lines = [], 0, 0
    with open(path) as f:
        for header_lines, line in enumerate(f, start=1):
            line = line.strip()
            if line.startswith("element vertex"):
                n_vertices = int(line.split()[-1])
            elif line.startswith("property"):
                prop_names.append(line.split()[-1])
            elif line == "end_header":
                break

    has_color = all(c in prop_names for c in ("red", "green", "blue"))
    x_idx = prop_names.index("x")
    y_idx = prop_names.index("y")
    z_idx = prop_names.index("z")
    r_idx = prop_names.index("red")   if has_color else -1
    g_idx = prop_names.index("green") if has_color else -1
    b_idx = prop_names.index("blue")  if has_color else -1

    print(f"[ply] header: {n_vertices:,} vertices, {len(prop_names)} props, color={has_color}")
    data = np.loadtxt(path, skiprows=header_lines, max_rows=n_vertices, dtype=np.float64)
    xyz = data[:, [x_idx, y_idx, z_idx]]
    rgb = data[:, [r_idx, g_idx, b_idx]].astype(np.uint8) if has_color else None
    return xyz, rgb


# ════════════════════════════════════════════════════════════════════════
# Stage 2 — Statistical Outlier Removal
# ════════════════════════════════════════════════════════════════════════

def statistical_outlier_removal(xyz, rgb, nb_neighbors=20, std_ratio=1.5):
    """Drop points whose mean distance to K nearest neighbours exceeds
    `std_ratio` standard deviations above the cloud's mean."""
    from scipy.spatial import cKDTree
    print(f"[stage2-sor] {len(xyz):,} points  k={nb_neighbors}  std_ratio={std_ratio}")
    tree = cKDTree(xyz)
    dists, _ = tree.query(xyz, k=nb_neighbors + 1)
    mean_d = dists[:, 1:].mean(axis=1)
    threshold = float(mean_d.mean() + std_ratio * mean_d.std())
    keep = mean_d < threshold
    n_kept = int(keep.sum())
    print(f"[stage2-sor] kept {n_kept:,} of {len(xyz):,} ({100*n_kept/len(xyz):.1f}%); "
          f"dropped {len(xyz)-n_kept:,}")
    return xyz[keep], (rgb[keep] if rgb is not None else None)


# ════════════════════════════════════════════════════════════════════════
# Stage 3 — DBSCAN cluster filter
# ════════════════════════════════════════════════════════════════════════

def dbscan_cluster_filter(xyz, rgb, eps=0.02, min_samples=10, min_cluster_size=500):
    """Cluster points with DBSCAN; drop noise + clusters smaller than
    min_cluster_size. Catches floating ghost blobs that survive SOR."""
    from sklearn.cluster import DBSCAN
    print(f"[stage3-dbscan] clustering {len(xyz):,} points  eps={eps}  min_samples={min_samples}")
    labels = DBSCAN(eps=eps, min_samples=min_samples, n_jobs=-1).fit(xyz).labels_
    n_clusters = int(labels.max()) + 1 if labels.max() >= 0 else 0
    n_noise = int((labels == -1).sum())
    sizes = np.bincount(labels[labels >= 0]) if n_clusters else np.array([], dtype=np.int64)
    keep_ids = np.where(sizes >= min_cluster_size)[0]
    keep = np.isin(labels, keep_ids)
    n_kept = int(keep.sum())
    big_top = sorted(sizes[keep_ids].tolist(), reverse=True)[:5]
    print(f"[stage3-dbscan] {n_clusters} clusters; {len(keep_ids)} kept (>= {min_cluster_size} pts); "
          f"{n_clusters - len(keep_ids)} too-small dropped; {n_noise:,} noise dropped")
    if big_top:
        print(f"[stage3-dbscan]   top kept cluster sizes: {big_top}")
    print(f"[stage3-dbscan] kept {n_kept:,} of {len(xyz):,} ({100*n_kept/len(xyz):.1f}%)")
    return xyz[keep], (rgb[keep] if rgb is not None else None)


# ════════════════════════════════════════════════════════════════════════
# Stage 4 — RANSAC floor plane fit
# ════════════════════════════════════════════════════════════════════════

def ransac_floor_plane(xyz, distance_threshold=0.02, n_iterations=1000,
                       max_normal_tilt=0.15, floor_search_band=0.30):
    """Fit the floor plane via RANSAC. Restrict candidates to the bottom
    `floor_search_band` of Z; reject planes whose normal tilts more than
    `max_normal_tilt` from vertical (so we don't lock onto a wall).

    Returns (floor_z, n_inliers). Falls back to 5th percentile if no good
    plane is found.
    """
    z_min, z_max = float(xyz[:, 2].min()), float(xyz[:, 2].max())
    z_band_top = z_min + floor_search_band * (z_max - z_min)
    candidates = xyz[xyz[:, 2] <= z_band_top]
    print(f"[stage4-ransac] {len(candidates):,} of {len(xyz):,} pts in bottom "
          f"{floor_search_band*100:.0f}% (Z<={z_band_top:.3f})")

    if len(candidates) < 50:
        fz = float(np.percentile(xyz[:, 2], 5))
        print(f"[stage4-ransac] too few candidates → fallback percentile {fz:+.3f}")
        return fz, 0

    rng = np.random.default_rng(42)
    best_count, best_inliers, best_normal = 0, None, None

    for _ in range(n_iterations):
        idx = rng.choice(len(candidates), 3, replace=False)
        p1, p2, p3 = candidates[idx]
        normal = np.cross(p2 - p1, p3 - p1)
        nlen = np.linalg.norm(normal)
        if nlen < 1e-9:
            continue
        normal /= nlen
        if normal[2] < 0:
            normal = -normal
        if normal[2] < (1.0 - max_normal_tilt):
            continue
        d = -float(normal.dot(p1))
        distances = np.abs(candidates @ normal + d)
        inliers = distances < distance_threshold
        count = int(inliers.sum())
        if count > best_count:
            best_count, best_inliers, best_normal = count, inliers, normal

    if best_inliers is None or best_count < 50:
        fz = float(np.percentile(xyz[:, 2], 5))
        print(f"[stage4-ransac] no good plane (best={best_count}) → fallback percentile {fz:+.3f}")
        return fz, 0

    floor_z = float(np.median(candidates[best_inliers, 2]))
    tilt = float(np.degrees(np.arccos(min(1.0, best_normal[2]))))
    print(f"[stage4-ransac] normal={best_normal.round(3).tolist()}  "
          f"tilt={tilt:.2f}°  inliers={best_count:,}  floor_z={floor_z:+.3f}")
    return floor_z, best_count


# ════════════════════════════════════════════════════════════════════════
# Stage 5 — Floating-cluster filter (re-DBSCAN, drop mid-air ghosts)
# ════════════════════════════════════════════════════════════════════════

def floor_support_cluster_filter(xyz, rgb, floor_z,
                                 cluster_eps=0.02, cluster_min_samples=10,
                                 floating_threshold=0.10):
    """Re-cluster, then drop any cluster whose LOWEST point sits more
    than `floating_threshold` above the floor. Real obstacles touch the
    floor; ghosts hang in mid-air."""
    from sklearn.cluster import DBSCAN
    print(f"[stage5-float] re-clustering {len(xyz):,} points  eps={cluster_eps}")
    labels = DBSCAN(eps=cluster_eps, min_samples=cluster_min_samples, n_jobs=-1).fit(xyz).labels_
    n_clusters = int(labels.max()) + 1 if labels.max() >= 0 else 0
    n_noise = int((labels == -1).sum())
    print(f"[stage5-float] {n_clusters} clusters, {n_noise:,} noise points")

    keep = np.ones(len(xyz), dtype=bool)
    n_drop_clusters = 0
    n_drop_points = 0
    diag = []
    for cid in range(n_clusters):
        m = labels == cid
        low = float(xyz[m, 2].min() - floor_z)
        sz = int(m.sum())
        if low > floating_threshold:
            keep &= ~m
            n_drop_clusters += 1
            n_drop_points += sz
        if cid < 10:
            diag.append((cid, sz, low, low > floating_threshold))
    if diag:
        print("[stage5-float]   cid    size   lowest_above_floor   floating?")
        for cid, sz, low, isf in diag:
            print(f"[stage5-float]   {cid:>3}  {sz:>6}   {low:+.3f} m              {isf}")
    n_kept = int(keep.sum())
    print(f"[stage5-float] dropped {n_drop_clusters} floating clusters ({n_drop_points:,} pts); "
          f"kept {n_kept:,} of {len(xyz):,} ({100*n_kept/len(xyz):.1f}%)")
    return xyz[keep], (rgb[keep] if rgb is not None else None)


# ════════════════════════════════════════════════════════════════════════
# Stage 6a — Build 2D occupancy grid (axis convention: Z-up)
# ════════════════════════════════════════════════════════════════════════

def build_map_json(xyz, rgb, resolution, obstacle_min_height=0.10,
                   obstacle_min_points=5, robot_height=0.10,
                   robot_clearance=0.05, floor_z=None):
    """Bin points into a 2D occupancy grid in the X-Y plane (Z-up).

    Obstacle envelope: only points whose height above the floor is in
    [obstacle_min_height, robot_height + robot_clearance] count as
    obstacles. Anything taller is "passable under" (table tops, hanging
    cables) and the cell is promoted to ground if no obstacles sit lower.
    """
    out_x, out_y, out_z = xyz[:, 0], xyz[:, 1], xyz[:, 2]

    if floor_z is None:
        floor_z = float(np.percentile(out_z, 5))
        print(f"[stage6-grid] estimated floor (5th %ile of Z): {floor_z:+.3f} m")
    else:
        print(f"[stage6-grid] floor_z (provided): {floor_z:+.3f} m")

    envelope_top = robot_height + robot_clearance
    print(f"[stage6-grid] robot envelope: [{obstacle_min_height:+.2f}, "
          f"{envelope_top:+.2f}] m above floor "
          f"(height={robot_height} + clearance={robot_clearance})")

    min_x, max_x = float(out_x.min()), float(out_x.max())
    min_y, max_y = float(out_y.min()), float(out_y.max())
    width = int(np.ceil((max_x - min_x) / resolution))
    height = int(np.ceil((max_y - min_y) / resolution))
    print(f"[stage6-grid] {width} × {height} cells @ {resolution} m  "
          f"(x[{min_x:.2f},{max_x:.2f}]  y[{min_y:.2f},{max_y:.2f}])")

    cols = np.clip(((out_x - min_x) / resolution).astype(np.int32), 0, width - 1)
    rows = np.clip(((out_y - min_y) / resolution).astype(np.int32), 0, height - 1)
    h_above = out_z - floor_z

    grid = np.zeros((height, width), dtype=np.uint8)

    # 1. Obstacles: enough points inside the robot envelope
    obs_mask = (h_above >= obstacle_min_height) & (h_above <= envelope_top)
    if obs_mask.any():
        obs_count = np.zeros((height, width), dtype=np.int32)
        np.add.at(obs_count, (rows[obs_mask], cols[obs_mask]), 1)
        grid[obs_count >= obstacle_min_points] = 2

    # 2. Ground from real floor points
    floor_band = 0.15
    flr_mask = h_above < floor_band
    if flr_mask.any():
        not_obs = grid[rows[flr_mask], cols[flr_mask]] != 2
        grid[rows[flr_mask][not_obs], cols[flr_mask][not_obs]] = 1

    # 3. Passable-under-overhead: cells with stuff ONLY above the envelope
    above_mask = h_above > envelope_top
    if above_mask.any():
        above_count = np.zeros((height, width), dtype=np.int32)
        np.add.at(above_count, (rows[above_mask], cols[above_mask]), 1)
        passable_under = (above_count >= obstacle_min_points) & (grid != 2)
        n_under = int(passable_under.sum())
        if n_under:
            print(f"[stage6-grid] passable-under-overhead promoted to ground: {n_under}")
        grid[passable_under] = 1

    print(f"[stage6-grid] cells: ground={int((grid==1).sum())}  "
          f"obstacle={int((grid==2).sum())}  unknown={int((grid==0).sum())}")

    # 1-cell wall dilation
    try:
        from scipy.ndimage import binary_dilation
        grid[binary_dilation(grid == 2, iterations=1)] = 2
    except ImportError:
        print("[stage6-grid] scipy missing — skipping wall dilation")

    pts_list = np.column_stack([out_x, out_y, out_z]).tolist()
    cols_list = rgb.tolist() if rgb is not None else [[150, 150, 150]] * len(out_x)
    print(f"[stage6-grid] writing {len(pts_list):,} points to JSON")

    return {
        "metadata": {
            "resolution":       resolution,
            "floor_z":          floor_z,
            "robot_height":     robot_height,
            "robot_clearance":  robot_clearance,
            "min_x":            min_x,
            "min_y":            min_y,
            "max_x":            max_x,
            "max_y":            max_y,
            "grid_width":       width,
            "grid_height":      height,
            "point_count":      len(pts_list),
            "axis_convention":  "z-up",
        },
        "grid":   grid.tolist(),
        "points": pts_list,
        "colors": cols_list,
    }


# ════════════════════════════════════════════════════════════════════════
# Stage 6b — Inpaint floor (closing + flood-fill + synthetic 3D points)
# ════════════════════════════════════════════════════════════════════════

def inpaint_grid(grid: np.ndarray, closing_iters: int = 2, fill_holes: bool = True):
    """Fill floor holes the camera missed. Two well-known steps:
       (a) morphological closing on the ground mask (small gaps)
       (b) flood-fill from boundary — any unknown cell walled off from
           the boundary must be inside the room → mark as ground.
    Never touches obstacle cells.
    """
    from scipy import ndimage

    g = grid.copy()
    before = {"unk": int((g == 0).sum()), "gnd": int((g == 1).sum()),
              "obs": int((g == 2).sum())}

    if closing_iters > 0:
        ground_mask = (g == 1)
        closed = ndimage.binary_closing(ground_mask, iterations=closing_iters)
        new_ground = closed & (g != 2) & ~ground_mask
        g[new_ground] = 1
        n_closed = int(new_ground.sum())
    else:
        n_closed = 0

    if fill_holes:
        passable = (g == 0)
        seed = np.zeros_like(passable)
        seed[0, :]  = passable[0, :];  seed[-1, :] = passable[-1, :]
        seed[:, 0]  = passable[:, 0];  seed[:, -1] = passable[:, -1]
        outside = ndimage.binary_propagation(seed, mask=passable)
        interior_holes = passable & ~outside
        g[interior_holes] = 1
        n_flood = int(interior_holes.sum())
    else:
        n_flood = 0

    after = {"unk": int((g == 0).sum()), "gnd": int((g == 1).sum()),
             "obs": int((g == 2).sum())}
    print(f"[inpaint] before  unk={before['unk']:5d}  gnd={before['gnd']:5d}  obs={before['obs']:5d}")
    print(f"[inpaint]   closing ({closing_iters}x) → +{n_closed} ground")
    print(f"[inpaint]   flood-fill         → +{n_flood} ground")
    print(f"[inpaint] after   unk={after['unk']:5d}  gnd={after['gnd']:5d}  obs={after['obs']:5d}")
    return g


def synthesize_floor_points(map_data: dict, density_cm: float = 2.5,
                            search_radius_m: float = 0.03):
    """Generate synthetic 3D floor points at every ground cell that has
    no real point nearby. Visualization only — does NOT change the grid."""
    from scipy.spatial import cKDTree

    meta = map_data["metadata"]
    grid = np.asarray(map_data["grid"], dtype=np.int32)
    res = float(meta["resolution"])
    floor_z = float(meta["floor_z"])
    H, W = grid.shape
    pts  = np.asarray(map_data["points"], dtype=np.float32)
    cols = np.asarray(map_data["colors"], dtype=np.float32)

    near_floor = (pts[:, 2] > floor_z - 0.05) & (pts[:, 2] < floor_z + 0.12)
    floor_xy = pts[near_floor, :2]
    if len(floor_xy) == 0:
        print("[inpaint-3d] no existing floor points to KD-search against")
        return map_data
    tree_xy = cKDTree(floor_xy)

    cells_per_point = max(1, int(round((density_cm / 100.0) / res)))
    floor_color = (np.median(cols[near_floor], axis=0)
                   if near_floor.sum() > 100 else np.array([170, 160, 140]))
    print(f"[inpaint-3d] median floor color RGB={tuple(int(c) for c in floor_color)}")

    cell_xy = []
    for r in range(0, H, cells_per_point):
        for c in range(0, W, cells_per_point):
            if grid[r, c] != 1:
                continue
            wx = meta["min_x"] + (c + 0.5) * res
            wy = meta["min_y"] + (r + 0.5) * res
            d, _ = tree_xy.query([wx, wy], k=1)
            if d > search_radius_m:
                cell_xy.append((r, c))

    rng = np.random.default_rng(0)
    cell_size = res * cells_per_point
    pts_per_cell = max(4, int((cell_size / 0.007) ** 2))
    color_std = np.array([12.0, 11.0, 10.0])

    new_pts = np.empty((len(cell_xy) * pts_per_cell, 3), dtype=np.float32)
    new_cols = np.empty((len(cell_xy) * pts_per_cell, 3), dtype=np.float32)
    k = 0
    for (r, c) in cell_xy:
        cx = meta["min_x"] + (c + 0.5) * res
        cy = meta["min_y"] + (r + 0.5) * res
        jx = (rng.random(pts_per_cell) - 0.5) * cell_size
        jy = (rng.random(pts_per_cell) - 0.5) * cell_size
        jz = (rng.random(pts_per_cell) - 0.5) * 0.005
        new_pts[k:k+pts_per_cell, 0] = cx + jx
        new_pts[k:k+pts_per_cell, 1] = cy + jy
        new_pts[k:k+pts_per_cell, 2] = floor_z + jz
        col = floor_color + rng.standard_normal((pts_per_cell, 3)) * color_std
        new_cols[k:k+pts_per_cell] = np.clip(col, 0, 255)
        k += pts_per_cell
    new_pts = new_pts[:k]
    new_cols = new_cols[:k]

    print(f"[inpaint-3d] {len(cell_xy)} empty cells found → {len(new_pts)} synthetic points")
    if len(new_pts):
        merged_pts  = np.vstack([pts, new_pts])
        merged_cols = np.vstack([cols, new_cols])
        map_data["points"] = merged_pts.tolist()
        map_data["colors"] = merged_cols.tolist()
        meta["point_count"] = int(len(merged_pts))
    return map_data


# ════════════════════════════════════════════════════════════════════════
# Verification preview JPEG
# ════════════════════════════════════════════════════════════════════════

def render_preview(map_data: dict, out_path: Path, source_name: str):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.colors import ListedColormap

    grid = np.asarray(map_data["grid"], dtype=np.int32)
    md = map_data["metadata"]
    pts = np.asarray(map_data.get("points", []), dtype=np.float32)

    fig, axes = plt.subplots(1, 2, figsize=(14, 7))

    cmap = ListedColormap([(0.85, 0.85, 0.85), (1.0, 1.0, 1.0), (0.85, 0.10, 0.10)])
    extent = [md["min_x"], md["max_x"], md["min_y"], md["max_y"]]
    axes[0].imshow(grid, origin="lower", cmap=cmap, vmin=0, vmax=2,
                   extent=extent, interpolation="nearest")
    axes[0].set_xlabel("X (m)")
    axes[0].set_ylabel("Y (m)")
    axes[0].set_aspect("equal")
    unk = int((grid == 0).sum())
    gnd = int((grid == 1).sum())
    obs = int((grid == 2).sum())
    axes[0].set_title(
        f"2D occupancy grid\n"
        f"unknown {unk:,}   ground {gnd:,}   obstacle {obs:,}\n"
        f"{md['grid_width']}×{md['grid_height']} @ {md['resolution']} m   "
        f"floor_z={md.get('floor_z', 0):+.3f} m"
    )

    if len(pts) > 0:
        floor = md.get("floor_z", float(np.percentile(pts[:, 2], 5)))
        shown = pts
        if len(shown) > 80_000:
            idx = np.random.default_rng(0).choice(len(shown), 80_000, replace=False)
            shown = shown[idx]
        z = shown[:, 2] - floor
        sc = axes[1].scatter(shown[:, 0], shown[:, 1], c=z, s=0.3,
                             cmap="viridis", vmin=-0.05, vmax=0.4)
        axes[1].set_aspect("equal")
        axes[1].set_xlim(md["min_x"], md["max_x"])
        axes[1].set_ylim(md["min_y"], md["max_y"])
        axes[1].set_xlabel("X (m)")
        axes[1].set_ylabel("Y (m)")
        axes[1].set_title(
            f"cleaned point cloud (top-down)\n"
            f"{len(shown):,} of {md.get('point_count', len(pts)):,} shown"
        )
        plt.colorbar(sc, ax=axes[1], fraction=0.046, pad=0.04,
                     label="height above floor (m)")
    else:
        axes[1].text(0.5, 0.5, "no point data", ha="center", va="center",
                     transform=axes[1].transAxes)

    fig.suptitle(f"{source_name}  —  cleaned map preview", fontsize=13)
    fig.tight_layout(rect=[0, 0, 1, 0.96])
    fig.savefig(out_path, dpi=140, bbox_inches="tight")
    plt.close(fig)


# ════════════════════════════════════════════════════════════════════════
# Main pipeline
# ════════════════════════════════════════════════════════════════════════

def main():
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument("source", help="Path to RTAB-Map .db OR existing .ply")
    p.add_argument("--output", default="map.json", help="Output map.json (default: ./map.json)")

    g = p.add_argument_group("grid + envelope")
    g.add_argument("--resolution", type=float, default=0.02, help="Cell size, m (default 0.02)")
    g.add_argument("--robot-height", type=float, default=0.10, help="m (default 0.10)")
    g.add_argument("--robot-clearance", type=float, default=0.05, help="m (default 0.05)")
    g.add_argument("--obstacle-min-points", type=int, default=5,
                   help="Min points/cell to mark obstacle (default 5)")

    g = p.add_argument_group("stage toggles (skip a stage entirely)")
    g.add_argument("--no-sor",      action="store_true", help="Skip Stage 2 (SOR)")
    g.add_argument("--no-dbscan",   action="store_true", help="Skip Stage 3 (DBSCAN)")
    g.add_argument("--no-ransac",   action="store_true", help="Skip Stage 4 (RANSAC floor)")
    g.add_argument("--no-floating", action="store_true", help="Skip Stage 5 (floating cluster)")
    g.add_argument("--no-inpaint",  action="store_true", help="Skip Stage 6b (floor inpaint)")
    g.add_argument("--no-3d",       action="store_true", help="Skip synthetic 3D floor points")
    g.add_argument("--no-preview",  action="store_true", help="Skip the JPEG preview")

    g = p.add_argument_group("Stage 1 — rtabmap-export")
    g.add_argument("--rtabmap-max-range", type=float, default=3.0, help="m (default 3.0)")
    g.add_argument("--rtabmap-min-range", type=float, default=None)
    g.add_argument("--rtabmap-noise-radius", type=float, default=None)
    g.add_argument("--rtabmap-noise-k", type=int, default=None)
    g.add_argument("--rtabmap-prop-radius", type=float, default=None)

    g = p.add_argument_group("Stage 2 — SOR")
    g.add_argument("--sor-neighbors", type=int, default=20)
    g.add_argument("--sor-std-ratio", type=float, default=1.5)

    g = p.add_argument_group("Stage 3 — DBSCAN")
    g.add_argument("--dbscan-eps", type=float, default=0.02)
    g.add_argument("--dbscan-min-samples", type=int, default=10)
    g.add_argument("--dbscan-min-cluster-size", type=int, default=500,
                   help="Lower (e.g. 200) to keep small obstacles like poles or cones")

    g = p.add_argument_group("Stage 4 — RANSAC floor")
    g.add_argument("--ransac-threshold", type=float, default=0.02)
    g.add_argument("--ransac-tilt", type=float, default=0.15)
    g.add_argument("--ransac-iterations", type=int, default=1000)
    g.add_argument("--ransac-search-band", type=float, default=0.30)

    g = p.add_argument_group("Stage 5 — floating-cluster")
    g.add_argument("--floating-threshold", type=float, default=0.10,
                   help="m above floor (default 0.10)")
    g.add_argument("--floating-eps", type=float, default=0.02)
    g.add_argument("--floating-min-samples", type=int, default=10)

    g = p.add_argument_group("inpaint")
    g.add_argument("--closing", type=int, default=2, help="Morphological closing iters")
    g.add_argument("--no-fill", action="store_true", help="Skip boundary flood-fill")
    g.add_argument("--density-cm", type=float, default=2.5)
    g.add_argument("--search-radius", type=float, default=0.03)

    args = p.parse_args()

    src = Path(args.source).expanduser().resolve()
    if not src.exists():
        sys.exit(f"error: source not found: {src}")
    if src.suffix.lower() not in (".db", ".ply"):
        sys.exit(f"error: source must be .db or .ply, got {src.suffix!r}")

    out = Path(args.output).expanduser().resolve()
    out.parent.mkdir(parents=True, exist_ok=True)

    cleanup_dir = None
    try:
        # ── Stage 1 ──
        if src.suffix.lower() == ".db":
            cleanup_dir = tempfile.mkdtemp()
            ply_path = export_db_to_ply(
                str(src), cleanup_dir,
                max_range=args.rtabmap_max_range,
                min_range=args.rtabmap_min_range,
                noise_radius=args.rtabmap_noise_radius,
                noise_k=args.rtabmap_noise_k,
                prop_radius_factor=args.rtabmap_prop_radius,
            )
        else:
            ply_path = str(src)

        xyz, rgb = parse_ascii_ply(ply_path)
        print(f"[parse] loaded {len(xyz):,} points")

        # ── Stage 2 ──
        if not args.no_sor:
            xyz, rgb = statistical_outlier_removal(
                xyz, rgb, nb_neighbors=args.sor_neighbors,
                std_ratio=args.sor_std_ratio)

        # ── Stage 3 ──
        if not args.no_dbscan:
            xyz, rgb = dbscan_cluster_filter(
                xyz, rgb, eps=args.dbscan_eps,
                min_samples=args.dbscan_min_samples,
                min_cluster_size=args.dbscan_min_cluster_size)

        # ── Stage 4 ──
        floor_z = None
        if not args.no_ransac:
            floor_z, _ = ransac_floor_plane(
                xyz, distance_threshold=args.ransac_threshold,
                n_iterations=args.ransac_iterations,
                max_normal_tilt=args.ransac_tilt,
                floor_search_band=args.ransac_search_band)

        # ── Stage 5 ──
        if not args.no_floating:
            fz = floor_z if floor_z is not None else float(np.percentile(xyz[:, 2], 5))
            xyz, rgb = floor_support_cluster_filter(
                xyz, rgb, floor_z=fz,
                cluster_eps=args.floating_eps,
                cluster_min_samples=args.floating_min_samples,
                floating_threshold=args.floating_threshold)

        # ── Stage 6a — grid build ──
        map_data = build_map_json(
            xyz, rgb,
            resolution=args.resolution,
            obstacle_min_points=args.obstacle_min_points,
            robot_height=args.robot_height,
            robot_clearance=args.robot_clearance,
            floor_z=floor_z,
        )

        # ── Stage 6b — inpaint (in-memory, on the grid) ──
        if not args.no_inpaint:
            grid = np.asarray(map_data["grid"], dtype=np.uint8)
            grid = inpaint_grid(grid, closing_iters=args.closing,
                                fill_holes=not args.no_fill)
            map_data["grid"] = grid.tolist()
            if not args.no_3d:
                map_data = synthesize_floor_points(
                    map_data, density_cm=args.density_cm,
                    search_radius_m=args.search_radius)

        # ── Save ──
        with open(out, "w") as f:
            json.dump(map_data, f, separators=(",", ":"))
        print(f"[save] wrote {out} ({os.path.getsize(out)/1024/1024:.1f} MB)")

        # ── Preview ──
        preview = out.with_suffix(".jpg")
        if not args.no_preview:
            try:
                render_preview(map_data, preview, src.name)
                print(f"[preview] wrote {preview}")
            except Exception as e:
                print(f"[preview] failed: {e}")

        # ── Summary ──
        md = map_data["metadata"]
        g  = np.asarray(map_data["grid"])
        print()
        print("=" * 64)
        print(f"  output:   {out}")
        if not args.no_preview:
            print(f"  preview:  {preview}")
        print(f"  bounds:   X[{md['min_x']:+.2f}, {md['max_x']:+.2f}]  "
              f"Y[{md['min_y']:+.2f}, {md['max_y']:+.2f}]")
        print(f"  grid:     {md['grid_width']} × {md['grid_height']} @ {md['resolution']} m")
        print(f"  floor_z:  {md.get('floor_z', 0):+.3f} m")
        print(f"  cells:    unknown={int((g==0).sum()):,}   "
              f"ground={int((g==1).sum()):,}   obstacle={int((g==2).sum()):,}")
        print(f"  points:   {md.get('point_count', 0):,}")
        print("=" * 64)

    finally:
        if cleanup_dir:
            shutil.rmtree(cleanup_dir, ignore_errors=True)


if __name__ == "__main__":
    main()
