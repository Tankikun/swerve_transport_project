"""
post_align.py
Post-process map.json from db_to_map_json.py:
  1. SOR (statistical outlier removal) — drops isolated speckle
  2. Auto-rotate so walls align with X/Z grid (PCA on wall points)
  3. Optional left-right flip if the world frame is mirrored vs reality
  4. Rebuild the 2D occupancy grid with density-based wall extraction
     (dense+straight = real wall; sparse cells = noise, dropped)

Usage:
    /usr/bin/python3 post_align.py                          # in-place on map.json
    /usr/bin/python3 post_align.py --in map.json --out clean.json --flip-x
    /usr/bin/python3 post_align.py --no-rotate --no-sor     # only rebuild 2D grid

Run AFTER db_to_map_json.py.
"""

import argparse
import json
import numpy as np


def sor(pts, cols, k=20, sigma=1.0):
    """Statistical outlier removal: drop points whose mean distance to
    their k nearest neighbors is more than sigma stddevs above the mean."""
    from scipy.spatial import cKDTree
    tree = cKDTree(pts)
    d, _ = tree.query(pts, k=k + 1)
    mean_d = d[:, 1:].mean(axis=1)
    thresh = mean_d.mean() + sigma * mean_d.std()
    keep = mean_d < thresh
    print(f"[sor] kept {keep.sum()} of {len(pts)} ({100*keep.mean():.1f}%) "
          f"thresh={thresh:.4f} m")
    return pts[keep], (cols[keep] if cols is not None else None)


def rotate_align(pts, floor_y):
    """Rotate around vertical Y-axis so wall points align with X/Z axes."""
    walls = pts[pts[:, 1] > floor_y + 0.20][:, [0, 2]]
    if len(walls) < 100:
        print("[rotate] too few wall points, skipping")
        return pts, 0.0
    walls -= walls.mean(axis=0)
    _, _, Vt = np.linalg.svd(walls, full_matrices=False)
    angle = np.arctan2(Vt[0, 1], Vt[0, 0])
    angle_q = angle % (np.pi / 2)
    if angle_q > np.pi / 4:
        angle_q -= np.pi / 2
    rot = -angle_q
    print(f"[rotate] detected {np.degrees(angle_q):+.2f}° off-axis, "
          f"applying {np.degrees(rot):+.2f}°")

    # Recenter at origin (XZ only) before rotating
    xz_center = pts[:, [0, 2]].mean(axis=0)
    pts[:, 0] -= xz_center[0]
    pts[:, 2] -= xz_center[1]

    c, s = np.cos(rot), np.sin(rot)
    new_x = c * pts[:, 0] - s * pts[:, 2]
    new_z = s * pts[:, 0] + c * pts[:, 2]
    pts[:, 0] = new_x
    pts[:, 2] = new_z
    return pts, np.degrees(rot)


def rebuild_grid(pts, floor_y, res=0.05, wall_min_h=0.10,
                 wall_density_frac=0.15, floor_density_frac=0.05):
    """Density-based 2D occupancy grid. Sparse cells are dropped as noise."""
    x_min, x_max = float(pts[:, 0].min()) - res, float(pts[:, 0].max()) + res
    z_min, z_max = float(pts[:, 2].min()) - res, float(pts[:, 2].max()) + res
    W = int(np.ceil((x_max - x_min) / res))
    H = int(np.ceil((z_max - z_min) / res))

    xi = np.clip(((pts[:, 0] - x_min) / res).astype(np.int32), 0, W - 1)
    zi = np.clip(((pts[:, 2] - z_min) / res).astype(np.int32), 0, H - 1)

    wall_mask  = pts[:, 1] > floor_y + wall_min_h
    floor_mask = (pts[:, 1] >= floor_y - 0.05) & (pts[:, 1] <= floor_y + wall_min_h)

    wall_counts  = np.zeros((W, H), dtype=np.int32)
    floor_counts = np.zeros((W, H), dtype=np.int32)
    np.add.at(wall_counts,  (xi[wall_mask],  zi[wall_mask]),  1)
    np.add.at(floor_counts, (xi[floor_mask], zi[floor_mask]), 1)

    wall_thr  = max(1, int(wall_counts.max()  * wall_density_frac))
    floor_thr = max(1, int(floor_counts.max() * floor_density_frac))

    grid = np.zeros((W, H), dtype=np.uint8)   # 0=unknown 1=ground 2=obstacle
    grid[floor_counts >= floor_thr] = 1
    grid[wall_counts  >= wall_thr ] = 2

    raw_obs = int((wall_counts > 0).sum())
    kept_obs = int((grid == 2).sum())
    print(f"[grid] {W}x{H} cells at {res}m  "
          f"ground={int((grid==1).sum())} obstacle={kept_obs} "
          f"(dropped {raw_obs - kept_obs} sparse cells as noise)")

    meta = {"grid_width": W, "grid_height": H, "resolution": res,
            "min_x": x_min, "max_x": x_max, "min_z": z_min, "max_z": z_max}
    return grid.T.tolist(), meta


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--in",  dest="inp",  default="map.json")
    ap.add_argument("--out", dest="outp", default="map.json")
    ap.add_argument("--no-sor",    action="store_true")
    ap.add_argument("--no-rotate", action="store_true")
    ap.add_argument("--flip-x",    action="store_true",
                    help="Mirror cloud left<->right (use if walls are on wrong side)")
    ap.add_argument("--sor-sigma", type=float, default=1.0)
    ap.add_argument("--res",       type=float, default=0.05)
    args = ap.parse_args()

    with open(args.inp) as f:
        m = json.load(f)
    pts  = np.asarray(m["points"], dtype=np.float32)
    cols = np.asarray(m["colors"], dtype=np.float32) if "colors" in m else None
    floor_y = float(m["metadata"]["floor_y"])
    print(f"[load] {len(pts)} points  floor_y={floor_y:.3f}")

    if not args.no_sor:
        pts, cols = sor(pts, cols, sigma=args.sor_sigma)

    if not args.no_rotate:
        pts, _ = rotate_align(pts, floor_y)

    if args.flip_x:
        pts[:, 0] *= -1.0
        print("[flip] mirrored X axis")

    grid, gmeta = rebuild_grid(pts, floor_y, res=args.res)

    m["points"] = pts.tolist()
    if cols is not None:
        m["colors"] = cols.tolist()
    m["grid"] = grid
    m["metadata"].update(gmeta)
    m["metadata"]["point_count"] = len(pts)

    with open(args.outp, "w") as f:
        json.dump(m, f, separators=(",", ":"))
    print(f"[save] wrote {args.outp}  ({len(pts)} points)")


if __name__ == "__main__":
    main()
