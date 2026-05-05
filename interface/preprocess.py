"""
preprocess.py
Converts a Scaniverse .obj mesh into a clean occupancy grid JSON for the web UI.
Separates ground (walkable, clickable) from obstacles (walls, furniture).

Usage:
    python3 preprocess.py --input map.obj --output map.json --resolution 0.05

Dependencies:
    pip3 install trimesh numpy
"""

import argparse
import json
import numpy as np

try:
    import trimesh
except ImportError:
    print("trimesh not installed. Run: pip3 install trimesh")
    exit(1)


def preprocess_obj(input_path, output_path, resolution=0.05,
                   floor_band=0.15, obstacle_min_height=0.1):
    """
    Loads a .obj mesh and builds a 2D occupancy grid.

    Args:
        input_path         : path to .obj file from Scaniverse
        output_path        : path to save output .json
        resolution         : grid cell size in meters (smaller = more detail)
        floor_band         : height range above floor to consider as ground (meters)
        obstacle_min_height: minimum height above floor to count as obstacle (meters)
    """

    print(f"Loading: {input_path}")
    mesh = trimesh.load(input_path, force="mesh")
    print(f"Mesh loaded: {len(mesh.vertices)} vertices, {len(mesh.faces)} faces")

    vertices = np.asarray(mesh.vertices)

    # --- Detect floor level (lowest 5th percentile of Y axis) ---
    floor_y = float(np.percentile(vertices[:, 1], 5))
    print(f"Estimated floor Y: {floor_y:.3f}m")

    # --- Compute 2D bounding box (X and Z axes, top-down view) ---
    min_x, min_z = vertices[:, 0].min(), vertices[:, 2].min()
    max_x, max_z = vertices[:, 0].max(), vertices[:, 2].max()

    width  = int(np.ceil((max_x - min_x) / resolution))
    height = int(np.ceil((max_z - min_z) / resolution))
    print(f"Grid size: {width} x {height} cells at {resolution}m resolution")

    # --- Build occupancy grid ---
    # 0 = unknown, 1 = ground (walkable), 2 = obstacle
    grid = np.zeros((height, width), dtype=np.uint8)

    for v in vertices:
        vx, vy, vz = v
        col = int((vx - min_x) / resolution)
        row = int((vz - min_z) / resolution)

        col = np.clip(col, 0, width  - 1)
        row = np.clip(row, 0, height - 1)

        height_above_floor = vy - floor_y

        if height_above_floor < floor_band:
            # Ground level — only mark as ground if not already obstacle
            if grid[row, col] != 2:
                grid[row, col] = 1
        elif height_above_floor >= obstacle_min_height:
            # Obstacle — overrides ground
            grid[row, col] = 2

    # --- Dilate obstacles slightly so walls look solid ---
    from scipy.ndimage import binary_dilation
    obstacle_mask   = grid == 2
    dilated         = binary_dilation(obstacle_mask, iterations=1)
    grid[dilated]   = 2

    ground_count   = int((grid == 1).sum())
    obstacle_count = int((grid == 2).sum())
    print(f"Ground cells: {ground_count} | Obstacle cells: {obstacle_count}")

    # --- Also export point cloud for 3D view (downsampled) ---
    try:
        import open3d as o3d
        pcd      = mesh.as_open3d
        pcd_down = pcd.voxel_down_sample(voxel_size=0.05)
        pts      = np.asarray(pcd_down.points)
        if pcd_down.has_colors():
            cols = (np.asarray(pcd_down.colors) * 255).astype(int).tolist()
        else:
            cols = [[150, 150, 150]] * len(pts)
        pts_list = pts.tolist()
    except Exception:
        # Fallback: use raw vertices if open3d not available
        step     = max(1, len(vertices) // 20000)
        pts      = vertices[::step]
        cols     = [[150, 150, 150]] * len(pts)
        pts_list = pts.tolist()

    # --- Export ---
    output = {
        "metadata": {
            "resolution"    : resolution,
            "floor_y"       : floor_y,
            "min_x"         : float(min_x),
            "min_z"         : float(min_z),
            "max_x"         : float(max_x),
            "max_z"         : float(max_z),
            "grid_width"    : width,
            "grid_height"   : height,
            "point_count"   : len(pts_list)
        },
        "grid"   : grid.tolist(),   # 2D array: 0=unknown, 1=ground, 2=obstacle
        "points" : pts_list,        # for 3D view
        "colors" : cols
    }

    print(f"Saving to: {output_path}")
    with open(output_path, "w") as f:
        json.dump(output, f)

    size_mb = os.path.getsize(output_path) / 1024 / 1024
    print(f"Done. File size: {size_mb:.1f} MB")


if __name__ == "__main__":
    import os
    parser = argparse.ArgumentParser(description="Preprocess OBJ map to occupancy grid")
    parser.add_argument("--input",      default="map.obj",  help="Input .obj file")
    parser.add_argument("--output",     default="map.json", help="Output .json file")
    parser.add_argument("--resolution", type=float, default=0.05, help="Grid resolution in meters")
    args = parser.parse_args()

    preprocess_obj(args.input, args.output, resolution=args.resolution)