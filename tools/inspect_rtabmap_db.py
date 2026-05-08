#!/usr/bin/env python3
"""
inspect_rtabmap_db.py — Lightweight RTAB-Map database inspector.

Visualizes a `.db` file produced by RTAB-Map mapping runs (e.g. the output
of `rtabmap_laptop_mapping.launch.py` from this repo). Designed as a fast,
freeze-free alternative to `rtabmap-databaseViewer` for verifying a map
before using it for localization in `rtabmap_laptop_localization.launch.py`.

Shows:
  - Summary stats (nodes / links / loop closures / trajectory length / file size)
  - 2D top-down trajectory plot, nodes colored by acquisition order
  - Loop closures highlighted as red dashed connectors
  - Start (green) / End (red) markers

Usage:
    python3 tools/inspect_rtabmap_db.py ~/maps/tb3_1_room.db
    python3 tools/inspect_rtabmap_db.py                  # opens file picker
    python3 tools/inspect_rtabmap_db.py <db> --save out.png

Requires:
    - Python 3.8+ (built-in sqlite3, struct, tempfile)
    - matplotlib                            (pip install matplotlib)
    - rtabmap CLI tools                     (brew install rtabmap)  [optional]
        Used to extract poses via `rtabmap-export`. If unavailable, falls
        back to a direct SQLite parse of the pose BLOB column.
"""

import argparse
import os
import shutil
import sqlite3
import struct
import subprocess
import sys
import tempfile
from pathlib import Path


# ---------------------------------------------------------------------
#  Summary statistics — pure SQLite, version-tolerant
# ---------------------------------------------------------------------
def db_summary(db_path):
    conn = sqlite3.connect(db_path)
    cur = conn.cursor()
    s = {'file_size_mb': os.path.getsize(db_path) / 1024 / 1024}

    cur.execute("SELECT COUNT(*) FROM Node")
    s['nodes'] = cur.fetchone()[0]

    cur.execute("SELECT COUNT(*) FROM Link")
    s['links_total'] = cur.fetchone()[0]

    try:
        cur.execute("SELECT COUNT(*) FROM Link WHERE type=0")
        s['links_neighbor'] = cur.fetchone()[0]
        cur.execute("SELECT COUNT(*) FROM Link WHERE type IN (1, 4)")
        s['loop_closures'] = cur.fetchone()[0]
    except sqlite3.OperationalError:
        s['links_neighbor'] = '?'
        s['loop_closures'] = '?'

    try:
        cur.execute("SELECT version FROM Admin LIMIT 1")
        s['rtabmap_version'] = cur.fetchone()[0]
    except sqlite3.OperationalError:
        s['rtabmap_version'] = 'unknown'

    try:
        cur.execute("SELECT COUNT(DISTINCT mapId) FROM Node")
        s['maps'] = cur.fetchone()[0]
    except sqlite3.OperationalError:
        s['maps'] = '?'

    conn.close()
    return s


# ---------------------------------------------------------------------
#  Pose extraction — primary: rtabmap-export, fallback: direct SQLite
# ---------------------------------------------------------------------
def extract_poses_via_export(db_path):
    """Fallback: use `rtabmap-export --poses --poses_format 1` (TUM format).

    TUM line: `timestamp x y z qx qy qz qw`. Used only if the direct SQLite
    parse fails for some reason (corrupt blob, unsupported version).

    Notes on rtabmap-export CLI:
      `--output_dir DIR`  controls the directory.
      `--output NAME`     just the file's NAME suffix (NOT a full path).
                          Result lands at  <output_dir>/<NAME>_poses.txt.
    """
    rtabmap_export = shutil.which('rtabmap-export')
    if not rtabmap_export:
        return None
    with tempfile.TemporaryDirectory() as tmpdir:
        try:
            subprocess.run(
                [rtabmap_export, '--poses', '--poses_format', '1',
                 '--output_dir', tmpdir, '--output', 'traj',
                 db_path],
                check=True, capture_output=True, timeout=120,
            )
        except (subprocess.CalledProcessError, subprocess.TimeoutExpired,
                FileNotFoundError):
            return None

        candidates = sorted(Path(tmpdir).glob('*poses*.txt'))
        if not candidates:
            candidates = sorted(Path(tmpdir).glob('*.txt'))
        if not candidates:
            return None

        ids, xs, ys, zs = [], [], [], []
        for i, raw in enumerate(candidates[0].read_text().splitlines()):
            raw = raw.strip()
            if not raw or raw.startswith('#'):
                continue
            parts = raw.split()
            # TUM: timestamp x y z qx qy qz qw   (need at least 4 cols)
            if len(parts) < 4:
                continue
            try:
                x, y, z = float(parts[1]), float(parts[2]), float(parts[3])
            except ValueError:
                continue
            ids.append(i)
            xs.append(x)
            ys.append(y)
            zs.append(z)
        return (ids, xs, ys, zs) if ids else None


def extract_poses_via_sqlite(db_path):
    """Fallback: parse the pose BLOB column directly.

    For RTAB-Map 0.20+ the pose column is just **12 raw float32** values
    (row-major 3x4 transformation matrix), no cv::Mat header. The
    translation is at indices [3], [7], [11].
    Older versions may include a 16-byte header (rows/cols/type/channels);
    we try both.
    """
    conn = sqlite3.connect(db_path)
    cur = conn.cursor()
    cur.execute("SELECT id, pose FROM Node ORDER BY id")
    ids, xs, ys, zs = [], [], [], []
    for node_id, blob in cur.fetchall():
        if blob is None:
            continue
        floats = None
        # Modern format: 48 bytes = 12 floats, no header
        if len(blob) == 48:
            try:
                floats = struct.unpack('<12f', blob)
            except struct.error:
                pass
        # Legacy format: 16-byte cv::Mat header + 12 floats = 64 bytes
        elif len(blob) == 64:
            try:
                rows, cols, _type, _ch = struct.unpack('<iiii', blob[:16])
                if rows == 3 and cols == 4:
                    floats = struct.unpack('<12f', blob[16:16 + 48])
            except struct.error:
                pass
        if floats is None:
            continue
        # Translation = column 3 of the 3x4 matrix (row-major)
        ids.append(node_id)
        xs.append(floats[3])
        ys.append(floats[7])
        zs.append(floats[11])
    conn.close()
    return (ids, xs, ys, zs) if ids else None


def loop_closure_pairs(db_path):
    """List of (from_id, to_id) for loop-closure links (types 1 and 4)."""
    conn = sqlite3.connect(db_path)
    cur = conn.cursor()
    try:
        cur.execute("SELECT from_id, to_id FROM Link WHERE type IN (1, 4)")
        rows = cur.fetchall()
    except sqlite3.OperationalError:
        rows = []
    conn.close()
    return rows


# ---------------------------------------------------------------------
#  Render
# ---------------------------------------------------------------------
def render(db_path, out_image=None):
    import matplotlib.pyplot as plt
    import numpy as np

    print(f"\n[inspect] {db_path}")
    s = db_summary(db_path)

    print(f"  RTAB-Map version : {s.get('rtabmap_version')}")
    print(f"  File size        : {s['file_size_mb']:.1f} MB")
    print(f"  Maps             : {s.get('maps')}")
    print(f"  Nodes            : {s['nodes']}")
    print(f"  Links total      : {s['links_total']}")
    print(f"    neighbor       : {s.get('links_neighbor')}")
    print(f"    loop closures  : {s.get('loop_closures')}")

    # Direct SQLite is fast, version-tolerant, and gives ALL nodes (not just
    # the ones that participated in optimization). Use it as primary.
    print("\n[poses] reading pose blobs from SQLite ...")
    poses = extract_poses_via_sqlite(db_path)
    if poses is None:
        print("[poses] SQLite parse failed — falling back to rtabmap-export ...")
        poses = extract_poses_via_export(db_path)

    if poses is None:
        print("[poses] could not extract any poses; plotting summary only.")
        ids, xs, ys, zs = [], [], [], []
    else:
        ids, xs, ys, zs = poses
        print(f"[poses] extracted {len(ids)} poses.")

    traj_length_m = None
    if len(xs) > 1:
        dx = np.diff(xs)
        dy = np.diff(ys)
        traj_length_m = float(np.sum(np.sqrt(dx * dx + dy * dy)))
        print(f"  Trajectory length: {traj_length_m:.2f} m")

    lcs = loop_closure_pairs(db_path)

    fig = plt.figure(figsize=(12, 7))
    fig.suptitle(f"RTAB-Map DB: {Path(db_path).name}", fontsize=13, fontweight='bold')

    # Left: summary text
    ax_text = fig.add_subplot(1, 2, 1)
    ax_text.axis('off')
    lines = [
        f"File         : {Path(db_path).name}",
        f"Size         : {s['file_size_mb']:.1f} MB",
        f"RTAB version : {s.get('rtabmap_version')}",
        f"Maps         : {s.get('maps')}",
        "",
        f"Nodes (keyframes): {s['nodes']}",
        f"Links total      : {s['links_total']}",
        f"  neighbor       : {s.get('links_neighbor')}",
        f"  loop closures  : {s.get('loop_closures')}",
    ]
    if traj_length_m is not None:
        lines += ["", f"Trajectory length: {traj_length_m:.2f} m"]
        # Heuristic indicator of map quality
        loops = s.get('loop_closures')
        if isinstance(loops, int):
            if loops == 0:
                quality = "⚠ no loop closures (map quality unverified)"
            elif loops < 5:
                quality = f"~ {loops} loop closures (modest)"
            else:
                quality = f"✓ {loops} loop closures (good)"
            lines += ["", f"Map quality      : {quality}"]
    ax_text.text(0.02, 0.98, "\n".join(lines), va='top', ha='left',
                 family='monospace', fontsize=11)

    # Right: 2D trajectory
    ax_plot = fig.add_subplot(1, 2, 2)
    ax_plot.set_aspect('equal')
    ax_plot.set_title("Trajectory (top-down, X-Y)")
    ax_plot.set_xlabel("X (m)")
    ax_plot.set_ylabel("Y (m)")
    ax_plot.grid(True, alpha=0.3)

    if len(xs) > 1:
        ax_plot.plot(xs, ys, '-', color='steelblue', linewidth=1.5, alpha=0.6)
        sc = ax_plot.scatter(xs, ys, c=range(len(xs)),
                             cmap='viridis', s=18, zorder=3)
        plt.colorbar(sc, ax=ax_plot, label='Node order (start → end)', fraction=0.04)

        # Loop closures
        if lcs:
            id_to_xy = {nid: (x, y) for nid, x, y in zip(ids, xs, ys)}
            n_drawn = 0
            for from_id, to_id in lcs:
                if from_id in id_to_xy and to_id in id_to_xy:
                    fx, fy = id_to_xy[from_id]
                    tx_, ty_ = id_to_xy[to_id]
                    ax_plot.plot([fx, tx_], [fy, ty_],
                                 'r--', linewidth=0.8, alpha=0.6)
                    n_drawn += 1
            if n_drawn:
                ax_plot.plot([], [], 'r--', label=f"{n_drawn} loop closures")

        ax_plot.plot(xs[0], ys[0], 'go', markersize=12, label='Start')
        ax_plot.plot(xs[-1], ys[-1], 'ro', markersize=12, label='End')
        ax_plot.legend(loc='best', fontsize=9)
    else:
        ax_plot.text(0.5, 0.5, "No poses extracted",
                     ha='center', va='center',
                     transform=ax_plot.transAxes, color='gray')

    plt.tight_layout()
    if out_image:
        plt.savefig(out_image, dpi=120, bbox_inches='tight')
        print(f"[saved] {out_image}")
    else:
        plt.show()


# ---------------------------------------------------------------------
#  CLI entry
# ---------------------------------------------------------------------
def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('db_path', nargs='?', help='Path to an rtabmap .db file')
    p.add_argument('--save', metavar='OUT.png',
                   help='Save figure to PNG instead of showing the window')
    args = p.parse_args()

    if not args.db_path:
        try:
            from tkinter import Tk, filedialog
            root = Tk()
            root.withdraw()
            args.db_path = filedialog.askopenfilename(
                title="Open RTAB-Map .db file",
                filetypes=[("RTAB-Map database", "*.db"), ("All files", "*.*")],
            )
            root.destroy()
        except Exception as e:
            print(f"No file argument and no GUI available ({e}).")
            print("Usage: python3 inspect_rtabmap_db.py /path/to/rtabmap.db")
            sys.exit(1)
        if not args.db_path:
            print("No file selected.")
            sys.exit(0)

    if not os.path.exists(args.db_path):
        print(f"error: file not found: {args.db_path}")
        sys.exit(1)

    render(args.db_path, out_image=args.save)


if __name__ == '__main__':
    main()
