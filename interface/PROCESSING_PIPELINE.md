# Processing Pipeline: From Saved Map to Clean Localization Data

This document explains **the *idea* of the processing pipeline** —
the conceptual flow, what each step is trying to accomplish, and why.
For specific commands and flags, see [`README.md`](README.md).

---

## Big picture

A robot can't localize against a noisy, raw map — it needs a **clean,
structured representation** of the environment. The pipeline takes a
fresh-from-the-robot RTAB-Map database (full of sensor noise, drift,
ceiling fixtures, far-range haze) and turns it into a clean map suitable
for both a human operator (the web UI) and the robot's localization stack.

```
                ┌─────────────────────────────────────────┐
                │  1. SAVED MAP (rtabmap.db)              │
                │     raw 3D point cloud of the room      │
                │     ~1M points, ~30% noise              │
                └────────────────┬────────────────────────┘
                                 │
                                 ▼
                ┌─────────────────────────────────────────┐
                │  2. OPEN + EXPORT TO PLY                │
                │     rtabmap-export reads the .db        │
                │     reconstructs the cloud from         │
                │     stored keyframes + depth + poses    │
                └────────────────┬────────────────────────┘
                                 │
                                 ▼
                ┌─────────────────────────────────────────┐
                │  3. STAGE 1 CLEANING (per-camera)       │
                │     remove far-range haze + speckles    │
                │     while we still know which keyframe  │
                │     captured each point                 │
                └────────────────┬────────────────────────┘
                                 │
                                 ▼
                ┌─────────────────────────────────────────┐
                │  4. PARSE + AXIS REMAP                  │
                │     PLY → numpy, Z-up → Y-up            │
                └────────────────┬────────────────────────┘
                                 │
                                 ▼
                ┌─────────────────────────────────────────┐
                │  5. STAGE 2 CLEANING (geometric)        │
                │     drop ceiling, crop to room bbox     │
                └────────────────┬────────────────────────┘
                                 │
                                 ▼
                ┌─────────────────────────────────────────┐
                │  6. BUILD OCCUPANCY GRID + DOWNSAMPLE   │
                │     2D grid for navigation              │
                │     ~60K points for browser 3D view     │
                └────────────────┬────────────────────────┘
                                 │
                                 ▼
                ┌─────────────────────────────────────────┐
                │  7. OUTPUT: map.json                    │
                │     ready for web UI + downstream       │
                │     ROS localization                    │
                └─────────────────────────────────────────┘
```

---

## Stage 0 — How the saved map got there

Before this pipeline runs, the robot did a **mapping run**:

- RTAB-Map ran in `mode:=mapping` on the Pi (or on the laptop if split-execution)
- The OAK-D Lite streamed stereo + RGB into `depthai-ros`
- `rtabmap_odom` computed visual odometry from the stereo pair
- `rtabmap_slam` accumulated keyframes whenever the scene changed enough,
  ran loop closure detection, and optimized the global pose graph
- When the operator stopped the run, RTAB-Map saved the result as
  **`rtabmap.db`** (a SQLite file containing keyframes, descriptors,
  depth images, poses, and the optimization graph)

The saved `.db` is **everything the robot saw** — including unwanted
data: ceiling lights it briefly looked at, noisy depth readings beyond
the camera's reliable range, and the thousands of points it observed
while spinning between mapping stations.

**Key idea:** the .db is faithful raw data, not a "clean map." Cleaning
is our job, downstream.

---

## Stage 1 — Open and export

`rtabmap-export` is RTAB-Map's CLI tool for converting a saved
database into common formats (PLY, OBJ, mesh, etc.).

What it does internally:

1. **Reads the optimization graph** to get the final corrected pose of every keyframe
2. For each keyframe: reads its **depth image** + **camera intrinsics** + **pose**
3. Backprojects every depth pixel into 3D using the keyframe's pose
4. Aggregates into one **assembled cloud** (PLY)

**Why this step matters for cleaning:** while RTAB-Map is doing the
backprojection, it still knows *which keyframe captured each point*.
This is the only moment in the pipeline where we can do **per-camera
cleaning** (Stage 1 cleaning, next).

After export, the keyframe association is lost — the PLY is just a flat
list of 3D points.

---

## Stage 2 — Stage 1 Cleaning (per-camera, at source)

This happens *during* the export, by passing flags to `rtabmap-export`.
It's the most physically correct cleaning we can do because RTAB-Map
knows the camera-to-point relationship.

### 2a. Per-camera range cap (`--max_range`)

**Problem:** OAK-D Lite stereo depth becomes unreliable past ~3 meters.
Beyond that, every depth pixel adds a noisy point in roughly the right
direction but at the wrong distance — the "halo" of haze around the
real geometry.

**Wrong approach:** drop points more than R meters from the world origin.
This is wrong because the robot moved during mapping — origin is just
where the robot started, not where it is. A point captured when the
robot was 3 m away can be 5+ m from origin and still legitimate.

**Right approach (what `--max_range` does):** drop points more than R
meters from **the keyframe that captured them**. Each pose contributes
a sphere of valid observations of radius R. Together they form a
"tube" along the trajectory, not a circle around origin.

### 2b. Radius outlier removal (`--noise_radius` + `--noise_k`)

**Problem:** stereo matching produces occasional false depth values —
isolated 3D points scattered across empty space. They look like
"speckle" noise, no two near each other.

**Approach:** for each point, count its neighbors within radius R.
If fewer than K neighbors, the point is statistically isolated — drop
it. Real surfaces are dense; noise speckles are sparse. The filter
exploits this asymmetry.

### 2c. Adaptive proportional radius (`--prop_radius_factor`)

**Problem:** point density isn't uniform — it's higher near the
camera (more pixels per square meter) and lower far away. A fixed-radius
filter (2b) can be too lenient near the camera and too strict far away.

**Approach:** scale the search radius proportionally to expected density.
RTAB-Map computes the expected neighbor count at each point's distance
and drops points that fall short — a smarter version of 2b.

**End of Stage 1:** typical reduction is **~30-40% of points removed**
without losing real geometry. The cloud is tighter and cleaner.

---

## Stage 3 — Parse + axis remap

Now the PLY file is on disk. We parse it into numpy arrays, then
adjust the coordinate system.

### 3a. Parse the ASCII PLY

Read the header to find vertex count + property layout (each row may
have x, y, z, normals, RGB, curvature). Use `numpy.loadtxt` to read
the body in one shot — fast for ~1M lines.

### 3b. Axis remap: Z-up → Y-up

**Problem:** RTAB-Map uses **REP-103** convention (X forward, Y left,
**Z up**). The web UI was originally written for Scaniverse-exported
OBJs which use **Y-up**. If we don't remap, the floor appears as a
vertical wall in the browser.

**Approach:** swap axes during the read:
- `output.x  =  input.x`   (horizontal stays horizontal)
- `output.y  =  input.z`   (vertical becomes vertical)
- `output.z  =  input.y`   (other horizontal stays horizontal)

This is just renaming, no rotation — preserves all relative geometry.

---

## Stage 4 — Stage 2 Cleaning (geometric, post-hoc)

Now that the cloud is in the same convention as the visualization,
apply the geometric filters we couldn't apply at source.

### 4a. Auto-detect floor

Take the **5th percentile of the height (Y) axis**. Why not the minimum?
Because the minimum is usually a single noise point below the actual
floor. The 5th percentile is robust to outliers but still anchored
to the real floor surface.

### 4b. Height cap (`--ceiling-above-floor`)

**Problem:** the OAK-D pointed up briefly during the mapping run. It
captured ceiling fixtures, lights, edges of doorframes — none of
which matter for ground-based navigation, but they pollute the
visualization.

**Approach:** drop any point with `y > floor + ceiling_above_floor`.
Typically `1.5 m` keeps walls visible (good for context) while
killing ceiling clutter. For navigation-only maps, `0.5 m` is even
tighter (only obstacles below camera height).

### 4c. Bounding box (`--bbox`)

**Problem:** even with per-camera range filtering (Stage 1), some points
land far outside the actual room — typically when the robot looked
through a doorway during mapping.

**Approach:** declare a rectangular bounding box `[x_min, x_max,
z_min, z_max]` matching the actual room footprint. Drop everything
outside. Crude but effective; needs to be tuned per room.

**End of Stage 2:** the cloud is now bounded, ceiling-free, and
spatially focused on the area we care about.

---

## Stage 5 — Build occupancy grid + downsample

Two outputs now get derived from the cleaned cloud, for two different
consumers.

### 5a. 2D occupancy grid (for navigation)

**Idea:** project the 3D cloud onto the floor plane (top-down view) at
some fixed cell resolution (e.g. 5 cm per cell). For each cell, decide:
- `0` = unknown (no points)
- `1` = ground (only points near floor height)
- `2` = obstacle (points above some height threshold)

The grid is what a path planner needs: a 2D image where it can
search for free paths between cells.

### 5b. Downsampled point cloud (for the 3D view)

**Problem:** the cleaned cloud is still ~600K points. Sending all of
them to the browser is wasteful; rendering them all is laggy.

**Approach:** uniform stride downsampling — keep every Nth point so
that the total ≤ a target (e.g. 60K points). This preserves the
overall shape and density distribution while being browser-friendly.

---

## Stage 6 — Output: map.json

The final structure (loaded by `server.py`, served on `/map`):

```json
{
  "metadata": {
    "resolution": 0.05,
    "floor_y":   -0.119,
    "min_x": -1.47, "max_x": 3.90,
    "min_z": -3.96, "max_z": 1.79,
    "grid_width":  108,
    "grid_height": 115,
    "point_count": 60000
  },
  "grid":   [[0,0,1,1,2, ...], ...],   // 2D occupancy
  "points": [[x,y,z], ...],            // 3D for the browser view
  "colors": [[r,g,b], ...]             // per-point colors
}
```

---

## How this serves localization

The cleaned `map.json` is the **shared reference** between:

1. **The operator** (sees it in the web UI; clicks to set goals)
2. **The path planner** (uses the 2D occupancy grid to find routes)
3. **The localization node** (compares live OAK-D observations against
   the same map to estimate the robot's current pose)

If the cleaned map is geometrically accurate (matches the real room),
all three benefit. If it's noisy or skewed (drift, bad calibration,
unfiltered haze), the operator sees garbage AND the localizer drifts.

This is why the cleanup pipeline matters: **garbage in = garbage
localization**. Better source data (a careful mapping run) + better
cleanup (this pipeline) = better localization.

---

## Where each problem gets solved (cheat sheet)

| Symptom in the raw .db | Cleaned by |
|---|---|
| Far-range depth haze | Stage 1: `--max_range` (per-camera) |
| Isolated noise speckles | Stage 1: `--noise_radius` + `--noise_k` |
| Sparse far points missed by fixed-radius filter | Stage 1: `--prop_radius_factor` |
| Floor appearing vertical in browser | Stage 3: axis remap |
| Ceiling fixtures, sky points | Stage 4: `--ceiling-above-floor` |
| Stuff outside the room (through doorways) | Stage 4: `--bbox` |
| 2D grid showing a circle instead of a room | Use Stage 1 `--max_range` (per-camera, correct) instead of the deprecated post-hoc `--max-range` (origin-radius, wrong) |
| Trajectory bending or drift | **Not fixable here** — it's a mapping-run problem; redo the mapping with slower driving + more loop closures |
| Wrong overall scale | **Not fixable here** — it's a calibration issue; verify with `depth_check.py` and recalibrate if confirmed bad |

---

## Read next

- [`README.md`](README.md) — exact commands and flag-by-flag tuning
- [`depth_check.py`](depth_check.py) — tape-measure verification of calibration
- The mapping-quality recipe (in `README.md` § "Mapping advice") — how
  to produce a `.db` that needs less cleaning in the first place
