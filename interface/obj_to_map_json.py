"""
obj_to_map_json.py — Convert an OBJ mesh into the web UI's map.json.

Sibling of db_to_map_json.py / ply_to_map_json.py for the case where your
input is a textured mesh (Scaniverse, photogrammetry, `rtabmap-export
--mesh`, MeshLab export, etc.).

OBJ files are typically MESHES, not point clouds — relatively few vertices
but full surface coverage via triangles, with colour stored in a .mtl-
referenced texture image rather than per-vertex. So this parser:

  1. Reads `v` / `vt` / `f` lines from the OBJ.
  2. Locates the texture via `mtllib` → `map_Kd`.
  3. Area-weighted-samples N points across each triangle (barycentric
     coordinates, vectorised across all triangles in one numpy pass).
  4. Looks up colour for each sample point via texture UV interpolation
     (or per-vertex RGB if the OBJ has the `v x y z r g b` extension).

The downstream pipeline (SOR → DBSCAN → RANSAC → floating-cluster →
2D occupancy grid) is the same as ply_to_map_json.py and produces an
identical map.json schema, so the GUI doesn't care which input you used.

Pipeline:
    OBJ + MTL + texture
      → triangle surface sampling
      → SOR
      → DBSCAN cluster filter
      → RANSAC floor plane
      → floating-cluster filter
      → fill enclosed unknown cells with ground
      → map.json

Axis convention: Z-up (RTAB-Map / ROS REP-103) preserved end-to-end.
The web UI is also Z-up to match.
    in.x -> out.x   (horizontal, "forward" in ROS frame)
    in.y -> out.y   (horizontal, "left"    in ROS frame)
    in.z -> out.z   (vertical, height)

Note on coordinate systems: many scan-app OBJ exports (Scaniverse, etc.)
are Y-UP, not Z-up. If the resulting map looks like the floor is on a
side, pass `--swap-yz` to swap Y and Z (with a sign flip on the new Y
to keep the handedness right). Quick rule of thumb: load the OBJ in
MeshLab; if the room sits flat on the XZ plane in MeshLab's default
view, you need --swap-yz.

Usage:
    python3 obj_to_map_json.py --obj room.obj --output map.json
    python3 obj_to_map_json.py --obj room.obj --swap-yz                    # Y-up source
    python3 obj_to_map_json.py --obj room.obj --samples-per-m2 50000       # denser
    python3 obj_to_map_json.py --obj room.obj --no-texture                 # grey output
    python3 obj_to_map_json.py --obj room.obj --resolution 0.05            # coarser grid

Requires:
    numpy, scipy, scikit-learn, pillow (pillow only needed for texture lookup;
    --no-texture skips it).
"""

import argparse
import json
import os
import sys
from pathlib import Path

import numpy as np


# ──────────────────────────────────────────────────────────────────────
# OBJ + MTL parser, with area-weighted triangle surface sampling.
# ──────────────────────────────────────────────────────────────────────

def _parse_face_token(tok):
    """`f` line uses `v[/vt[/vn]]` for each corner. Return (v_idx, vt_idx)
    as 1-based ints; vt_idx is None if absent."""
    parts  = tok.split('/')
    v_idx  = int(parts[0])
    vt_idx = int(parts[1]) if len(parts) > 1 and parts[1] else None
    return v_idx, vt_idx


def _resolve_mtl_texture(mtl_path):
    """Parse an .mtl file; return PIL.Image of the first map_Kd found
    (RGB), or None if no texture / Pillow not installed / file missing."""
    if not mtl_path.is_file():
        print(f"[mtl] not found: {mtl_path}")
        return None
    try:
        from PIL import Image
    except ImportError:
        print(f"[mtl] PIL/Pillow not installed; install with "
              f"`pip3 install pillow` to use textures (or pass --no-texture).")
        return None

    with open(mtl_path) as f:
        for line in f:
            tok = line.strip().split()
            if not tok or tok[0] != 'map_Kd':
                continue
            # map_Kd [-options ...] <filename>
            # Filename is the last token. If it contains spaces it'd be
            # quoted; we don't try to handle that exotic case.
            tex_name = tok[-1]
            tex_path = mtl_path.parent / tex_name
            if not tex_path.is_file():
                # Some exporters write absolute paths
                alt = Path(tex_name)
                if alt.is_file():
                    tex_path = alt
                else:
                    print(f"[mtl] map_Kd → {tex_name}: file not found near "
                          f"{mtl_path}")
                    return None
            print(f"[mtl] loading texture {tex_path}")
            return Image.open(tex_path).convert('RGB')
    print(f"[mtl] {mtl_path} has no map_Kd directive")
    return None


def parse_obj(path,
              samples_per_m2=30000.0,
              prefer_texture=True,
              swap_yz=False,
              max_points=None,
              seed=42):
    """Read an OBJ mesh into (xyz, rgb) numpy arrays via triangle surface
    sampling.

    Parameters:
        path            : OBJ file path.
        samples_per_m2  : target surface point density. 30000 = 3 per cm²,
                          giving roughly 300k points for a typical 10 m²
                          room — enough for the cleaning pipeline to behave
                          like a SLAM cloud.
        prefer_texture  : if True and the OBJ ships with a texture, sample
                          colour from the texture; else use per-vertex RGB
                          if available; else default grey.
        swap_yz         : OBJ source is Y-up. Map (x, y, z) → (x, -z, y)
                          to convert to ROS Z-up while preserving handedness.
        max_points      : optional hard cap on total samples. If the
                          area-weighted target exceeds this, scale
                          samples_per_m2 down to fit.
        seed            : RNG seed for reproducible sampling.

    Returns:
        xyz : (N, 3) float64
        rgb : (N, 3) uint8
    """
    obj_path = Path(path).resolve()
    if not obj_path.is_file():
        sys.exit(f"error: OBJ file not found: {obj_path}")
    obj_dir = obj_path.parent

    verts        = []     # list of (x, y, z)
    vert_colors  = []     # list of (r, g, b) ints 0-255, or None
    tex_coords   = []     # list of (u, v)
    faces        = []     # list of [(v_idx, vt_idx_or_None), …] (1-based)
    mtllib_paths = []

    with open(obj_path) as f:
        for raw in f:
            line = raw.strip()
            if not line or line.startswith('#'):
                continue
            tok = line.split()
            head = tok[0]

            if head == 'v':
                vals = tok[1:]
                x, y, z = float(vals[0]), float(vals[1]), float(vals[2])
                verts.append((x, y, z))
                if len(vals) >= 6:
                    # `v x y z r g b` extension. RGB might be 0-1 floats or
                    # 0-255 ints depending on exporter; auto-detect by
                    # checking if any value > 1.
                    raw_rgb = [float(vals[3]), float(vals[4]), float(vals[5])]
                    if max(raw_rgb) <= 1.0:
                        rgb_int = tuple(int(round(c * 255)) for c in raw_rgb)
                    else:
                        rgb_int = tuple(int(round(c)) for c in raw_rgb)
                    vert_colors.append(rgb_int)
                else:
                    vert_colors.append(None)
            elif head == 'vt':
                tex_coords.append((float(tok[1]), float(tok[2])))
            elif head == 'f':
                # n-gon → fan-triangulate. Robust to any face length.
                idxs = [_parse_face_token(t) for t in tok[1:]]
                for i in range(1, len(idxs) - 1):
                    faces.append([idxs[0], idxs[i], idxs[i + 1]])
            elif head == 'mtllib':
                # Path may contain spaces; rejoin everything after the keyword.
                mtllib_paths.append(obj_dir / ' '.join(tok[1:]))

    print(f"[obj] {len(verts):,} vertices, {len(faces):,} triangles "
          f"(post fan-triangulation), {len(tex_coords):,} UVs, "
          f"{len(mtllib_paths)} mtllib")

    if not verts or not faces:
        sys.exit("error: OBJ has no vertices or no faces")

    verts_np = np.array(verts, dtype=np.float64)

    # ── Optional Y-up → Z-up axis swap ───────────────────────────────
    if swap_yz:
        # (x, y, z) → (x, -z, y). Preserves right-handedness.
        x = verts_np[:, 0]
        y = verts_np[:, 1]
        z = verts_np[:, 2]
        verts_np = np.column_stack([x, -z, y])
        print("[obj] applied Y-up → Z-up swap: (x, y, z) → (x, -z, y)")

    # ── Resolve colour source ────────────────────────────────────────
    has_vert_color = any(c is not None for c in vert_colors)
    texture_img    = None
    if prefer_texture and not has_vert_color and tex_coords and mtllib_paths:
        for mtl_path in mtllib_paths:
            texture_img = _resolve_mtl_texture(mtl_path)
            if texture_img is not None:
                break

    tex_array = None
    if texture_img is not None:
        tex_array = np.asarray(texture_img)         # H × W × 3 uint8
        print(f"[obj] colour source: texture {tex_array.shape[1]}x"
              f"{tex_array.shape[0]}")
    elif has_vert_color:
        print(f"[obj] colour source: per-vertex RGB")
    else:
        print(f"[obj] colour source: default grey "
              f"(no per-vertex RGB, no usable texture)")

    # ── Vectorised triangle surface sampling ─────────────────────────
    # Build the per-face vertex/uv index arrays.
    n_tri = len(faces)
    v_face = np.empty((n_tri, 3), dtype=np.int64)
    vt_face = np.full((n_tri, 3), -1, dtype=np.int64)
    for i, face in enumerate(faces):
        for j in range(3):
            v_face[i, j]  = face[j][0] - 1
            if face[j][1] is not None:
                vt_face[i, j] = face[j][1] - 1

    p1 = verts_np[v_face[:, 0]]
    p2 = verts_np[v_face[:, 1]]
    p3 = verts_np[v_face[:, 2]]
    areas = 0.5 * np.linalg.norm(np.cross(p2 - p1, p3 - p1), axis=1)
    total_area = float(areas.sum())
    print(f"[obj] total mesh surface area: {total_area:.3f} m²")

    # Per-face sample count, with optional total cap.
    target_samples = areas * samples_per_m2
    n_per_face = np.maximum(1, np.round(target_samples).astype(np.int64))
    total = int(n_per_face.sum())
    if max_points is not None and total > max_points:
        scale = max_points / total
        samples_per_m2 *= scale
        n_per_face = np.maximum(1, np.round(areas * samples_per_m2).astype(np.int64))
        total = int(n_per_face.sum())
        print(f"[obj] capped samples_per_m2 → {samples_per_m2:.0f} "
              f"to honour --max-points={max_points}")
    print(f"[obj] sampling {total:,} points "
          f"(target density {samples_per_m2:.0f} per m²)")

    # Expand to per-sample face indices.
    face_idx = np.repeat(np.arange(n_tri), n_per_face)

    # Random barycentric coords. Sample (u, v) uniform in [0,1]² and
    # reflect across the diagonal so the (u, v, w=1-u-v) triple is
    # uniform on the simplex.
    rng = np.random.default_rng(seed)
    u = rng.random(total)
    v = rng.random(total)
    flip = (u + v) > 1.0
    u[flip] = 1.0 - u[flip]
    v[flip] = 1.0 - v[flip]
    w = 1.0 - u - v

    # Sample positions = u·p1 + v·p2 + w·p3 (per sample).
    p1_e = p1[face_idx]
    p2_e = p2[face_idx]
    p3_e = p3[face_idx]
    xyz = (u[:, None] * p1_e + v[:, None] * p2_e + w[:, None] * p3_e)

    # ── Sample colours ───────────────────────────────────────────────
    if has_vert_color:
        # Build a (V, 3) array of vertex colours; missing ones become grey.
        vcol = np.full((len(verts_np), 3), 150, dtype=np.float64)
        for vi, c in enumerate(vert_colors):
            if c is not None:
                vcol[vi] = c
        c1 = vcol[v_face[:, 0]][face_idx]
        c2 = vcol[v_face[:, 1]][face_idx]
        c3 = vcol[v_face[:, 2]][face_idx]
        rgb = (u[:, None] * c1 + v[:, None] * c2 + w[:, None] * c3)
        rgb = np.clip(rgb, 0, 255).astype(np.uint8)

    elif tex_array is not None:
        # Faces with all three vt indices defined are sampled from the
        # texture; faces missing UVs fall back to grey.
        rgb = np.full((total, 3), 150, dtype=np.uint8)

        face_has_uvs = (vt_face >= 0).all(axis=1)
        sample_has_uvs = face_has_uvs[face_idx]
        valid = np.where(sample_has_uvs)[0]
        if valid.size:
            uvs_arr = np.array(tex_coords, dtype=np.float64)
            v_idxs = face_idx[valid]
            uv1 = uvs_arr[vt_face[v_idxs, 0]]
            uv2 = uvs_arr[vt_face[v_idxs, 1]]
            uv3 = uvs_arr[vt_face[v_idxs, 2]]
            uv = (u[valid, None] * uv1
                  + v[valid, None] * uv2
                  + w[valid, None] * uv3)

            tex_h, tex_w = tex_array.shape[:2]
            # OBJ uses V-up, image rows are V-down → flip V.
            px = np.clip((uv[:, 0] * tex_w).astype(np.int32), 0, tex_w - 1)
            py = np.clip(((1.0 - uv[:, 1]) * tex_h).astype(np.int32),
                         0, tex_h - 1)
            rgb[valid] = tex_array[py, px, :3]

        n_grey = total - int(sample_has_uvs.sum())
        if n_grey:
            print(f"[obj] {n_grey:,} samples on faces without UVs → grey")
    else:
        rgb = np.full((total, 3), 150, dtype=np.uint8)

    return xyz, rgb


# ──────────────────────────────────────────────────────────────────────
# Cleaning stages — same algorithms as db_to_map_json.py / ply_to_map_json.py.
# ──────────────────────────────────────────────────────────────────────

def statistical_outlier_removal(xyz, rgb, nb_neighbors=20, std_ratio=1.5):
    """Drop points whose mean distance to their K nearest neighbours is
    more than `std_ratio` standard deviations above the cloud's mean."""
    try:
        from scipy.spatial import cKDTree
    except ImportError:
        sys.exit("error: scipy required for SOR. Install with `pip3 install scipy`.")

    print(f"[sor] running on {len(xyz):,} points  "
          f"(k={nb_neighbors}, std_ratio={std_ratio})...")

    tree     = cKDTree(xyz)
    dists, _ = tree.query(xyz, k=nb_neighbors + 1)
    mean_d   = dists[:, 1:].mean(axis=1)

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
                          eps=0.02, min_samples=10, min_cluster_size=500):
    """Cluster with DBSCAN; drop noise + clusters smaller than min_cluster_size."""
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
    sizes = np.bincount(labels[labels >= 0]) if n_clusters > 0 else np.array([], dtype=np.int64)

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
    """RANSAC plane fit on the bottom band of the cloud; returns
    (floor_z, n_inliers). Falls back to 5th percentile if no plane found."""
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
            continue
        normal /= nlen
        if normal[2] < 0:
            normal = -normal
        if normal[2] < (1.0 - max_normal_tilt):
            continue

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
    """Drop clusters whose lowest point is more than floating_threshold
    above the floor — they're floating ghosts, not real obstacles."""
    try:
        from sklearn.cluster import DBSCAN
    except ImportError:
        sys.exit("error: scikit-learn required.")

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
# Stage 3: bin into 2D occupancy grid, fill ground holes, write JSON.
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
    """Build map.json. Z-up convention (X, Y horizontal; Z height)."""
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

    obs_mask = (h_above >= obstacle_min_height) & (h_above <= envelope_top)
    if obs_mask.any():
        obs_count = np.zeros((height, width), dtype=np.int32)
        np.add.at(obs_count, (rows[obs_mask], cols[obs_mask]), 1)
        grid[obs_count >= obstacle_min_points] = 2

    flr_mask = h_above < floor_band
    if flr_mask.any():
        flr_rows = rows[flr_mask]
        flr_cols = cols[flr_mask]
        not_obs  = grid[flr_rows, flr_cols] != 2
        grid[flr_rows[not_obs], flr_cols[not_obs]] = 1

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
        grid[binary_dilation(grid == 2, iterations=1)] = 2

        if fill_ground:
            if fill_ground_close_iters > 0:
                ground = (grid == 1)
                closed = binary_closing(ground, iterations=fill_ground_close_iters)
                added  = closed & (grid != 2) & ~ground
                n_close = int(added.sum())
                grid[added] = 1
                print(f"[fill] ground closing (iters={fill_ground_close_iters}): "
                      f"+{n_close:,} ground cells")

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
    p.add_argument('--obj',    required=True,
                   help='Path to the input OBJ file. Adjacent .mtl + texture '
                        'will be auto-discovered via mtllib + map_Kd.')
    p.add_argument('--output', default='map.json',
                   help='Output map.json path (default: map.json)')
    p.add_argument('--resolution', type=float, default=0.02,
                   help='Grid cell size in metres (default 0.02 = 2 cm).')

    # ----- OBJ-specific sampling + colour -----
    p.add_argument('--samples-per-m2', type=float, default=30000.0,
                   help='Surface sampling density in points per m² '
                        '(default 30000 ≈ 3 per cm²).')
    p.add_argument('--max-points', type=int, default=None,
                   help='Cap on total samples; scales density down to fit. '
                        'Default: unlimited.')
    p.add_argument('--no-texture', action='store_true',
                   help='Skip texture lookup; use per-vertex RGB if any, '
                        'else default grey. Useful when Pillow isn\'t '
                        'available or you only want geometry.')
    p.add_argument('--swap-yz', action='store_true',
                   help='OBJ source is Y-up (e.g. Scaniverse, Blender export) '
                        '— remap (x, y, z) → (x, -z, y) to convert to ROS Z-up.')

    # ----- Cleaning stages (same as ply_to_map_json.py) -----
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
                   help='DBSCAN: max distance between neighbours (m, default 0.02).')
    p.add_argument('--dbscan-min-samples', type=int, default=10,
                   help='DBSCAN: min neighbours within eps to be a core point.')
    p.add_argument('--dbscan-min-cluster-size', type=int, default=500,
                   help='DBSCAN: drop clusters smaller than this many points.')

    p.add_argument('--no-ransac', action='store_true',
                   help='Disable RANSAC floor detection (default: ON).')
    p.add_argument('--ransac-threshold', type=float, default=0.02,
                   help='RANSAC: max distance from plane to count as inlier (m).')
    p.add_argument('--ransac-tilt', type=float, default=0.15,
                   help='RANSAC: how far the plane normal can tilt from vertical.')
    p.add_argument('--ransac-iterations', type=int, default=1000,
                   help='RANSAC: number of random trials (default 1000).')
    p.add_argument('--ransac-search-band', type=float, default=0.30,
                   help='RANSAC: bottom fraction of Z range to search.')

    p.add_argument('--no-floating', action='store_true',
                   help='Disable floating-cluster filter (default: ON).')
    p.add_argument('--floating-threshold', type=float, default=0.10, metavar='M',
                   help='A cluster is "floating" if its lowest point is '
                        'more than M m above the floor (default 0.10).')
    p.add_argument('--floating-eps', type=float, default=0.06, metavar='M',
                   help='Floating filter: DBSCAN eps for re-clustering.')
    p.add_argument('--floating-min-samples', type=int, default=10,
                   help='Floating filter: DBSCAN min_samples for re-clustering.')

    # ----- Grid stage -----
    p.add_argument('--obstacle-min-points', type=int, default=5,
                   help='Min points above obstacle_min_height for a cell '
                        'to be marked obstacle.')
    p.add_argument('--robot-height', type=float, default=0.10, metavar='M',
                   help='Robot height in metres (default 0.10).')
    p.add_argument('--robot-clearance', type=float, default=0.05, metavar='M',
                   help='Vertical safety clearance on top of robot_height.')

    # ----- Ground fill -----
    p.add_argument('--no-fill-ground', action='store_true',
                   help='Disable filling enclosed unknown cells with ground.')
    p.add_argument('--fill-ground-close', type=int, default=0, metavar='N',
                   help='Pre-fill morphological closing on the ground mask '
                        '(default 0 = off).')

    args = p.parse_args()

    # ── Load + sample the mesh ───────────────────────────────────────
    xyz, rgb = parse_obj(
        args.obj,
        samples_per_m2 = args.samples_per_m2,
        prefer_texture = not args.no_texture,
        swap_yz        = args.swap_yz,
        max_points     = args.max_points,
    )
    print(f"[parse] sampled {len(xyz):,} points from mesh")

    # ── Cleaning ─────────────────────────────────────────────────────
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

    # ── Grid + JSON ──────────────────────────────────────────────────
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
