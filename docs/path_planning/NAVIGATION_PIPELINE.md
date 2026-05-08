# Navigation Pipeline — Technical Reference

Companion to **README.md**. This file is the engineering spec: every data
boundary, every JSON field, every ROS topic, plus academic citations for
each algorithm in the stack. Read this when the README is too vague.

---

## 1. End-to-end workflow

```
+---------------------+        (1)        +---------------------+
| RTAB-Map .db file   |  rtabmap-export   | ASCII PLY cloud     |
| (tb3_1_room.db)     | ----------------> | (xyz + rgb)         |
+---------------------+                   +---------------------+
                                                    |
                                                    | (2) SOR + DBSCAN, build_map_json
                                                    v
                                          +---------------------+
                                          | map.json (raw)      |
                                          +---------------------+
                                                    |
                                       (3) inpaint_floor.py (in-place)
                                                    v
                                          +---------------------+
                                          | map.json (filled)   |
                                          +---------------------+
                                                    |
                                       (4) tighten_obstacles.py
                                                    v
                                          +---------------------+
                                          | map.json (cleaned)  |
                                          +---------------------+
                                                    |
                                       (5) Flask server.py loads it on startup
                                                    v
                                          +---------------------+
                                          | server.py + browser |
                                          | (index.html, three.js)
                                          +---------------------+
                                                    |
                                       (6) user clicks Plan  ->  POST /plan
                                                    v
                                       compute_plan() -> path_plan.json on disk
                                                    |
                                       (7) Send Goal -> publishWaypointsToROS
                                                    v
                                          +---------------------+
                                          | rosbridge_server    |
                                          | (websocket, port 9090)
                                          +---------------------+
                                                    |
                                       (8) /navigation/waypoints (PoseArray)
                                                    v
                                          +---------------------+
                                          | navigation_node.py  |
                                          | (Pi, ROS 2)         |
                                          | _waypoints_cb       |
                                          +---------------------+
                                                    |
                                       (9) /virtual_center/cmd_vel (Twist)
                                                    v
                                          laplacian_formation_node -> per-wheel
                                                    |
                                       (10) serial -> OpenCR firmware -> motors
```

Steps 1–7 run on the **Mac**. Steps 8–10 run on the **Raspberry Pi**.

---

## 2. Mac side vs Pi side

| Layer | Where it runs | Lives in |
|---|---|---|
| Map building (rtabmap-export, SOR, DBSCAN, RANSAC) | Mac | `db_to_map_json.py` |
| Floor inpainting (closing + flood-fill) | Mac | `inpaint_floor.py` |
| Obstacle tightening (3D-evidence rescue) | Mac | `tighten_obstacles.py` |
| A\* + APF refine + arc-resample (planning) | Mac | `astar_planner.py:compute_plan` |
| HTTP/JSON server, `/plan` endpoint, `path_plan.json` writer | Mac | `server.py` |
| 3D viewer, click-to-set-pose, **Send Goal** button | Mac (browser) | `index.html` |
| ROS bridge (websocket → ROS 2 topic) | either Mac or Pi | `rosbridge_server` (external) |
| Pure-pursuit + APF safety + ramp + 2-phase arrival | **Pi** | `swerve_transport_project/.../navigation_node.py` |
| EKF (odometry fusion) | **Pi** | `swerve_transport_project/.../ekf_node` |
| Formation control (leader → follower offsets) | **Pi** | `swerve_transport_project/.../laplacian_formation_node.py` |
| Per-wheel kinematics, serial protocol | **Pi** | swerve_formation package |
| Motor PID + encoder loop | OpenCR microcontroller | OpenCR C++ firmware |

---

## 3. Data schemas at each boundary

### 3.1 `map.json` (Mac, written by `db_to_map_json.py` ± `inpaint_floor.py` ± `tighten_obstacles.py`)

| Field | Python type | Shape / units | Description |
|---|---|---|---|
| `metadata.resolution` | float | meters per cell | grid cell size (default 2 cm) |
| `metadata.floor_z` | float | meters (Z-up) | RANSAC floor height |
| `metadata.robot_height` | float | meters | passes through to envelope filter |
| `metadata.robot_clearance` | float | meters | added on top of `robot_height` |
| `metadata.min_x, min_y, max_x, max_y` | float | meters, world frame | bbox of the cloud |
| `metadata.grid_width, grid_height` | int | cells | shape of `grid` |
| `metadata.point_count` | int | count | length of `points` |
| `metadata.axis_convention` | str | `"z-up"` | REP-103 frame tag |
| `grid` | `list[list[int]]` | `H × W`, values `{0,1,2}` | 0 = unknown, 1 = ground, 2 = obstacle |
| `points` | `list[[float, float, float]]` | `N × 3`, meters | downsampled cloud (for the 3D viewer) |
| `colors` | `list[[int, int, int]]` | `N × 3`, 0–255 | per-point RGB |

`inpaint_floor.py` only mutates `grid` (unknown → ground via closing + flood-fill). `tighten_obstacles.py` only mutates `grid` (ground → obstacle when above-floor evidence exceeds a threshold). Neither rewrites `metadata` or `points`.

### 3.2 `path_plan.json` (Mac, written by `server.py:/plan`)

| Field | Python type | Shape / units | Description |
|---|---|---|---|
| `metadata.formation_radius` | float | meters | radius **actually used** (post-cascade) |
| `metadata.formation_radius_requested` | float | meters | what the caller asked for |
| `metadata.radius_reduced` | bool | — | `true` if cascade had to shrink |
| `metadata.cascade_attempts` | `list[[float, str]]` | (radius_m, status) | e.g. `[[0.45,"no path"], [0.36,"ok"]]` |
| `metadata.vc_distance` | float | meters | total polyline length of the VC path |
| `metadata.travel_time_sec` | float | seconds | `vc_distance / 0.18` (MAX_LINEAR estimate) |
| `metadata.inflation_iters` | int | cells | `ceil(used_radius / resolution)` |
| `metadata.inflation_cells` | int | cell count | obstacle-halo cells (for viewer) |
| `metadata.start_snap_dist` | float | meters | how far the start was relocated |
| `metadata.goal_snap_dist` | float | meters | same for the goal |
| `metadata.target_spacing` | float | meters | arc-length resampling spacing (0 = off) |
| `metadata.yaw_policy` | str | `"tangent"` / `"free"` / `"smooth_to_goal"` | controller behavior selector |
| `metadata.start_yaw` | float | radians | requested start yaw |
| `metadata.goal_yaw` | float | radians | requested goal yaw |
| `metadata.navigable_bounds` | dict or null | min/max x/y, cells, area_m² | reachable region (main connected component) |
| `metadata.map_bounds` | dict | min/max x/y in meters | grid extent in world frame |
| `virtual_center.start` | `[float, float]` | meters | snapped VC start |
| `virtual_center.goal` | `[float, float]` | meters | snapped VC goal |
| `virtual_center.path_raw` | `list[[float, float]]` | meters | unsimplified A\* output |
| `virtual_center.waypoints` | `list[[float, float]]` | meters | simplified + APF-shifted + arc-resampled |
| `virtual_center.headings` | `list[float]` | radians | path-tangent yaw at each waypoint |
| `robots[i]` | dict | per robot | `{name, offset_local:[dx,dy], color, track:list[[x,y]], start, goal}` |
| `inflation_cells` | `list[[int, int]]` | (col, row) pairs | halo cells (for viewer overlay) |
| `seq` | int | monotonic counter | bumped per replan |
| `wall_clock_iso` | str | ISO-8601 UTC | timestamp |
| `start_source` | str | `"explicit (user click)"` or live-pose | added by `/plan` only |

### 3.3 What the **Pi actually receives** (the headline answer)

The browser publishes via rosbridge ([index.html:1320](index.html)):

- **Topic**: `/navigation/waypoints`
- **Message type**: `geometry_msgs/msg/PoseArray`

PoseArray wire format:

| Field | Type | Value |
|---|---|---|
| `header.frame_id` | string | `"map"` |
| `header.stamp` | `builtin_interfaces/Time` | `{secs:0, nsecs:0}` (browser does NOT stamp time — see §4 risks) |
| `poses[i].position.x` | float64 | meters, world X (= `wp[0]`) |
| `poses[i].position.y` | float64 | meters, world Y (= `wp[1]`) |
| `poses[i].position.z` | float64 | hardcoded `0.0` |
| `poses[i].orientation.{x, y}` | float64 | `0` (yaw-only quaternion) |
| `poses[i].orientation.z` | float64 | `sin(yaw / 2)` |
| `poses[i].orientation.w` | float64 | `cos(yaw / 2)` |

For all interior poses, `yaw = headings[i]` (path tangent). For the **final pose**, `yaw = goal.yawRad` if the user moved the orientation slider, otherwise the last heading. This is how `goal_yaw` survives the rosbridge crossing.

**The Pi receives ONLY the PoseArray.** The entire `metadata` block (cascade_attempts, vc_distance, navigable_bounds, etc.) lives in `path_plan.json` for the GUI's benefit and is NOT transmitted. `_waypoints_cb` ([navigation_node.py:243](../swerve_transport_project/ros2_ws/src/swerve_formation/swerve_formation/navigation_node.py)) decodes each Pose into `np.array([x, y, yaw])` — that's all the Pi sees: a list of `(x, y, yaw)` triples in the `map` frame.

### 3.4 Why `geometry_msgs/PoseArray`?

It's the **ROS-standard "ordered list of 6-DoF poses."** Nav2's planners (`PathToPoses`, `ComputePathThroughPoses`) and most ROS 2 visualisation tools (RViz `PoseArray` display, `nav2_smoother`) consume exactly this type. By emitting it directly, the Pi-side `navigation_node` is interchangeable with any Nav2-compatible planner — the laptop just slots in where Nav2 would. A custom dict would have meant a custom `.msg` definition, package rebuild on the Pi, and rosbridge-side type registration. PoseArray is the lowest-friction choice.

---

## 4. Open questions / risks

1. **Metadata is stripped at the rosbridge boundary.** If `goal_yaw` ever needs to differ from the final-segment tangent (it currently does — see [index.html:1329-1335](index.html)), it is encoded only in `poses[-1].orientation`. Anyone reusing this pipeline must remember the `metadata` block is Mac-only.
2. **Header stamp is hardcoded `{0, 0}`** ([index.html:1345](index.html)). Anything downstream that filters by message age (tf2 lookups, message_filters) will see "ancient" messages. The Pi node ignores the stamp, but RViz playback and rosbag introspection look wrong.
3. **`frame_id` is hardcoded `"map"`.** If the EKF publishes odom under a different parent frame, waypoints will be misinterpreted. There is no TF check on the Pi side.
4. **The 0.18 m/s travel-time estimate is a UI hint only.** Firmware cap is 0.198 m/s — off by ~10%.
5. **Path is published one-shot, no ack.** A dropped websocket frame loses the entire plan; the Pi has no `/navigation/waypoints/status` topic to surface failure. Pressing **Send Goal** again is the only recovery.

---

## 5. Algorithms & Citations

This stack is built almost entirely from classical, peer-reviewed algorithms — the contribution of the work is the integration and tuning, not new theory. Below is the canonical reference for each component.

### Map preparation (Stage 1)

1. **RANSAC plane fit** — Fischler & Bolles · 1981 · *Communications of the ACM* 24(6):381–395 · [DOI](https://dl.acm.org/doi/10.1145/358669.358692) — random-sample consensus for robust model fitting under heavy outliers; here used to extract the dominant floor plane.
2. **DBSCAN** — Ester, Kriegel, Sander & Xu · 1996 · *Proc. 2nd KDD*, 226–231 · [PDF](https://file.biolab.si/papers/1996-DBSCAN-KDD.pdf) — density-based spatial clustering that finds arbitrary-shape clusters and rejects noise; used to drop small disconnected obstacle blobs.
3. **Statistical Outlier Removal (SOR)** — Rusu, Marton, Blodow, Dolha & Beetz · 2008 · *Robotics and Autonomous Systems* 56(11):927–941 · [DOI](https://www.sciencedirect.com/science/article/abs/pii/S0921889008001140) — per-point mean-k-NN-distance thresholding (μ ± k·σ); the canonical PCL formulation re-implemented here in pure SciPy.
4. **Morphological closing & flood-fill** — no single canonical paper. Soille · 2003 · *Morphological Image Analysis: Principles and Applications*, 2nd ed., Springer · [book](https://link.springer.com/book/10.1007/978-3-662-05088-0); and Vincent · 1993 · *IEEE Trans. Image Processing* 2(2):176–201 · [IEEE](https://ieeexplore.ieee.org/document/217222/) — efficient queue-based reconstruction that underlies modern flood-fill.
5. **Minkowski-sum C-space inflation** — Latombe · 1991 · *Robot Motion Planning*, Kluwer (Springer reprint) · [book](https://link.springer.com/book/10.1007/978-1-4615-4022-9) — inflate occupancy by robot radius via binary dilation so the robot can be planned as a point.
6. **Binary opening (noise pre-filter)** — Soille 2003 (above). Erosion-then-dilation removes isolated single-pixel speckle before inflation.

### Path planning (Stage 2)

7. **A\* search** — Hart, Nilsson & Raphael · 1968 · *IEEE Trans. Systems Science and Cybernetics* 4(2):100–107 · [IEEE](https://ieeexplore.ieee.org/document/4082128/) — optimal best-first search under an admissible heuristic.
8. **Bresenham's line algorithm** — Bresenham · 1965 · *IBM Systems Journal* 4(1):25–30 · [IEEE](https://ieeexplore.ieee.org/document/5388473/) — integer-only incremental line rasterisation, used here for grid line-of-sight tests.
9. **String-pulling / any-angle smoothing (Theta\*-family)** — Daniel, Nash, Koenig & Felner · 2010 · *Journal of Artificial Intelligence Research* 39:533–579 · [JAIR](https://jair.org/index.php/jair/article/view/10676) — removes grid-aligned heading bias by replacing intermediate vertices with line-of-sight shortcuts.
10. **Khatib Artificial Potential Field** — Khatib · 1986 · *Int. Journal of Robotics Research* 5(1):90–98 · [Sage](https://journals.sagepub.com/doi/10.1177/027836498600500106) — conic attraction to the goal plus 1/d² obstacle repulsion for real-time refinement.
11. **Pure-pursuit waypoint follower** — Coulter · 1992 · *CMU-RI-TR-92-01*, Carnegie Mellon Robotics Institute · [PDF](https://www.ri.cmu.edu/pub_files/pub3/coulter_r_craig_1992_1/coulter_r_craig_1992_1.pdf) — geometric path tracker that aims at a look-ahead point on the path.
12. **Trapezoidal velocity profile** — no single canonical paper. Lynch & Park · 2017 · *Modern Robotics: Mechanics, Planning, and Control*, Cambridge University Press, §9.2 · [book](https://www.cambridge.org/core/books/modern-robotics/57C3BB1C6D5CB40320FA96E5FA3BCEC6).

### Multi-robot formation (Stage 3, planned)

13. **Laplacian consensus formation control** — Olfati-Saber, Fax & Murray · 2007 · *Proceedings of the IEEE* 95(1):215–233 · [IEEE](https://ieeexplore.ieee.org/document/4118472/) — survey/foundation of graph-Laplacian consensus protocols for networked multi-agent coordination, including formation control.

### Utilities

14. **Arc-length resampling of polylines** — no single canonical paper. Farin · 2002 · *Curves and Surfaces for CAGD: A Practical Guide*, 5th ed., Morgan Kaufmann · [book](https://www.sciencedirect.com/book/9781558607378/curves-and-surfaces-for-cagd) — standard CG/CAGD chord-length parameterisation.
15. **RTAB-Map RGB-D Visual SLAM** — Labbé & Michaud · 2019 · *Journal of Field Robotics* 36(2):416–446 · [Wiley](https://onlinelibrary.wiley.com/doi/abs/10.1002/rob.21831) — real-time appearance-based mapping with memory management; the upstream source of `tb3_1_room.db`.

### Implementation map

| Algorithm | File |
|---|---|
| 1, 2, 3 | `db_to_map_json.py` |
| 4 | `inpaint_floor.py` |
| 5, 6, 7, 8, 9 | `astar_planner.py` |
| 10 | `astar_planner.py:apf_refine_path` and `navigation_node.py:_apf_velocity` |
| 11 | `sim_navigator.py:pure_pursuit_target` and `navigation_node.py:_pure_pursuit_target` |
| 12 | `sim_navigator.py:_apply_ramp` and `navigation_node.py:_apply_ramp` |
| 13 | `swerve_transport_project/.../laplacian_formation_node.py` (planned) |
| 14 | `astar_planner.py:arclength_resample` |
| 15 | upstream — `tb3_1_room.db` is generated by RTAB-Map |
