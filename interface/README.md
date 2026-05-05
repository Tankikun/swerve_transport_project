# robot_path_planner — Pre-Mapping Cleanup Guide

This folder contains the **Mac-side web GUI** for visualizing maps from the
F_Senior swerve transport project, plus the **`db_to_map_json.py` pipeline**
that converts an RTAB-Map `.db` into a clean point cloud the GUI can render.

The hard part isn't running the GUI — it's **cleaning the raw RTAB-Map cloud
so the result is verifiable**. Raw RTAB-Map output has 1+ million points
with ~30% noise (sensor speckle, far-range depth haze, ceiling fixtures).
This README documents how the cleanup works and how to tune it.

---

## Pipeline overview

```
tb3_1_room.db                                    (~115 MB, RTAB-Map mapping run)
        │
        ▼ rtabmap-export --cloud --ascii          [Stage 1 — at the source]
        │   --max_range, --noise_radius, --noise_k, --prop_radius_factor
        │
        ▼ ASCII PLY (~600K-1M points after Stage 1 filtering)
        │
        ▼ db_to_map_json.py                       [Stage 2 — post-hoc]
        │   --ceiling-above-floor, --bbox
        │   axis remap (Z-up → Y-up)
        │   build 2D occupancy grid + downsample for 3D view
        │
        ▼ map.json  (~3 MB)
        │
        ▼ server.py (Flask, port 5002)
        │
        ▼ index.html (Three.js 3D + canvas 2D)    [Stage 3 — display polish]
            color = original RGB
            grid + axes for spatial reference
            auto-fit camera, double-click to reset
```

---

## Live localization viewer

Once a `map.json` is loaded, `ros_pose_bridge.py` streams the robot's live
`map -> base_link` TF to the Flask server so the GUI shows a moving robot
icon and a `LOC: …` health badge. Operator runbook: see
[`RUN_LOCALIZATION_VIEWER.md`](RUN_LOCALIZATION_VIEWER.md).

---

## Quick start

```bash
# One-time on Mac:
brew install rtabmap                 # gives rtabmap-export + rtabmap-info

# Each cycle (after dropping a new tb3_1_room.db into this folder):
/usr/bin/python3 db_to_map_json.py \
    --db tb3_1_room.db \
    --output map.json \
    --ceiling-above-floor 1.5 \
    --bbox -4 4 -4 4 \
    --rtabmap-max-range 3.0 \
    --rtabmap-noise-radius 0.05 \
    --rtabmap-noise-k 10 \
    --rtabmap-prop-radius 0.01

lsof -ti:5002 | xargs kill 2>/dev/null
/usr/bin/python3 server.py --map map.json --port 5002 &

# Open http://localhost:5002 — the cloud auto-fits the panel.
```

> **Use `/usr/bin/python3`**, not `python3`. The Homebrew Python 3.14 on this
> Mac has a broken `pyexpat` (ImportError) that breaks Flask. The system
> Python 3.9 has all required packages (`numpy`, `scipy`, `flask`, `trimesh`).

---

## Stage 1 — RTAB-Map native filters (the big wins)

These run **inside `rtabmap-export`** — RTAB-Map knows which keyframe captured
each point, so its filters are per-camera (correct for moving robots).

| Flag | What it does | Recommended | Effect |
|---|---|---|---|
| `--rtabmap-max-range M` | Drop points farther than M meters from **the camera that captured them** (NOT from world origin!) | `3.0` | Kills far-range depth haze where OAK-D Lite is unreliable (>3m) |
| `--rtabmap-min-range M` | Drop points closer than M meters | (skip) | OAK-D close-range noise is rare |
| `--rtabmap-noise-radius R` | Radius outlier removal: each point needs ≥ K neighbors in R | `0.05` | Kills isolated speckles |
| `--rtabmap-noise-k K` | Min neighbor count for the above | `10` (strict) or `5` (lenient) | Higher = stricter cleanup |
| `--rtabmap-prop-radius F` | Adaptive proportional-radius noise filter | `0.01` | RTAB-Map's smart density-aware outlier killer |

Typical impact on a 1M-point cloud:
- `--max_range 3.0` alone: **~10% of points** (the far-range halo)
- `noise_radius 0.05 noise_k 10` together: **another ~25-30% of points**
- `prop_radius 0.01` adds another ~5%
- **Total: ~600-700K points after Stage 1** (down from 1M)

### Why "max-range from origin" was wrong

The original `--max-range` flag in `db_to_map_json.py` (still kept for backward
compat but **deprecated**) measured distance from the world origin. For a
moving robot this is wrong — origin is just where the robot started. A point
captured when the robot was 3 m away can be 5+ m from origin and still
legitimate. Use `--rtabmap-max-range` instead — it's per-camera and correct.

---

## Stage 2 — Post-hoc Group A filters

These run in `db_to_map_json.py` after the PLY is parsed, before the 2D grid
is built. They handle things RTAB-Map's native filters can't.

| Flag | What it does | Recommended | When to use |
|---|---|---|---|
| `--ceiling-above-floor M` | Drop points more than M meters above the detected floor | `1.5` (keeps walls) or `0.5` (only obstacles below camera) | Always — kills ceiling fixtures, lights, etc. |
| `--bbox X_MIN X_MAX Z_MIN Z_MAX` | Crop to a floor bounding box (rectangular) | `-4 4 -4 4` (small room) | Always — limits the displayed area |
| `--max-range M` | DEPRECATED — radial from origin | (don't use) | Replaced by `--rtabmap-max-range` |

The floor is auto-detected as the 5th-percentile of the height (Y) axis.
Points below `floor − 5 cm` are also dropped (catches sub-floor noise).

---

## Stage 3 — Display polish (`index.html`)

Once the data is clean, rendering matters. These are baked into `index.html`
and don't require flags:

| Feature | Where | Effect |
|---|---|---|
| **Auto-fit camera** | `fitCameraToCloud()` | The cloud always fills the 3D panel after every refresh + window resize |
| **Default angle** `theta=0, phi=0.45` | `DEFAULT_VIEW` const | Slight tilt from top-down — same orientation every refresh |
| **Double-click to reset** | `dblclick` handler on canvas3d | Snaps back to default angle + auto-fit |
| **Larger point size 0.05** with `sizeAttenuation` | `PointsMaterial` | Cloud reads as solid surfaces, not dust |
| **Floor grid (1 m spacing)** | `THREE.GridHelper` | Spatial reference at floor height |
| **Origin axes** (R=X, G=Y, B=Z, 1m) | `THREE.AxesHelper` | Always know which way is forward / up |
| **60K display points** (default) | `--max-view-points` | 3× richer than the original 20K |
| **Press `O` to toggle overlay** | `keydown` handler | A/B with the Scaniverse `.obj` if loaded |

---

## Tuning guide — what to change when

| Symptom | Try |
|---|---|
| Cloud has scattered specks far from main shape | Increase `--rtabmap-noise-k` (8 → 12) or shrink `--rtabmap-noise-radius` (0.05 → 0.03) |
| Walls cut off / room looks too small | Loosen `--bbox` (e.g. `-6 6 -6 6`) and/or raise `--ceiling-above-floor` (1.5 → 2.5) |
| Far end of room is missing | Raise `--rtabmap-max-range` (3.0 → 4.0 or 5.0) — your sensor reach |
| 3D view too cluttered | Lower `--max-view-points` (60000 → 30000) |
| 2D top-down looks like a circle, not a room | The **per-camera** max-range is correct; the OLD `--max-range` (post-hoc, from origin) was making the circle. Make sure you're NOT passing `--max-range` |
| Cloud looks tilted or 2D grid is wrong shape | Source OBJ uses Y-up but `db_to_map_json.py` already remaps Z-up. If you converted via `preprocess.py` from a `.db`-derived OBJ, that script needs an `--up-axis z` patch (not yet implemented) |

---

## Comparing to a baseline (.obj overlay)

To overlay your Scaniverse `.obj` as a ghost on top of the RTAB-Map cloud
for accuracy verification:

```bash
# One-time: convert the .obj
/usr/bin/python3 preprocess.py --input map.obj --output map_obj.json --resolution 0.05

# Run the server with both maps:
/usr/bin/python3 server.py --map map.json --overlay map_obj.json --port 5002
```

In the browser, **press `O`** to toggle the ghost overlay on/off.

---

## Convert `.db` to `.obj` (textured mesh, for use in other tools)

```bash
rtabmap-export --mesh --texture --output_dir . --output tb3_1_room_from_db tb3_1_room.db
# Produces:
#   tb3_1_room_from_db_mesh.obj  (geometry)
#   tb3_1_room_from_db_mesh.mtl  (material file)
#   tb3_1_room_from_db_mesh.jpg  (texture atlas)
# All three must stay together (the .obj references the .mtl which references the .jpg).
```

Open in: macOS Preview (drag-and-drop), Blender, MeshLab, or any 3D viewer.

---

## File index

| File | Purpose |
|---|---|
| `index.html` | Web UI (Three.js 3D + canvas 2D + Flask client) |
| `server.py` | Flask backend (port 5002, routes `/`, `/map`, `/map_overlay`, `/goal`) |
| `db_to_map_json.py` | **Cleanup pipeline** — RTAB-Map `.db` → cleaned `map.json` |
| `preprocess.py` | Original Scaniverse `.obj` → `map.json` (Y-up only) |
| `map.json` | Currently-served map (output of `db_to_map_json.py`) |
| `map_obj.json` | Optional Scaniverse-derived overlay |
| `tb3_1_room.db` | Source RTAB-Map database |
| `map.obj`, `map.ply` | Original Scaniverse exports |
| `*.bak` | Snapshots before each tuning round (rollback safety) |

---

## Rollback

Each tuning round saved a `*.bak` snapshot. To undo a round:

```bash
# Roll back the converter:
cp db_to_map_json.py.beforeOpt3.bak db_to_map_json.py
# Roll back the served map:
cp map.json.beforeOpt3.bak map.json
# Roll back the frontend:
cp index.html.beforeAutoFit.bak index.html
# Restart server:
lsof -ti:5002 | xargs kill && /usr/bin/python3 server.py --map map.json --port 5002 &
```

Available snapshots:
- `*.scaniverse.bak` — original (Scaniverse-only) map
- `*.beforeGroupA.bak` — before Group A filters added
- `*.beforeOpt3.bak` — before per-camera `rtabmap-export` flags
- `*.before3DPolish.bak` — before 3D rendering tweaks (point size, grid, axes)
- `*.beforeAutoFit.bak` — before camera auto-fit
- `*.beforeOverlay.bak` — before A/B overlay support

---

## Mapping advice (separate from this cleanup pipeline)

The cleanup can only fix so much. **Better source data beats more aggressive
filtering every time.** When doing a fresh mapping run:

1. Drive **slowly** (0.1-0.15 m/s) — fast motion → blur → bad ORB → bad map
2. **Pause + spin 360°** at every station — captures features from all angles
3. **Multiple passes** through the same area — drives loop closures (the main quality signal)
4. **Return to start** — forces a global loop closure that tightens the trajectory
5. Avoid: glass, mirrors, blank walls, fast turns, dim light

Quality check after a run (uses the inspector tool in the friend's repo):
```bash
/usr/bin/python3 ~/Documents/F_Senior/swerve_transport_project/tools/inspect_rtabmap_db.py \
    tb3_1_room.db
```
A healthy map has at least **20-30 loop closures** for a small room
(your 158-node example had 10 — modest, hence the noise).
