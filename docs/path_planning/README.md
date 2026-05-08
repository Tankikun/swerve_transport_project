# Robot Path Planner

A laptop-side planning brain that turns a 3D room scan into a safe, smooth route for a small mobile robot — and lets you preview that route in the browser before sending it to the hardware.

## What this is

This project takes a 3D scan of a real room, cleans it into a simple top-down floor plan, and figures out a route from where the robot is to where you want it to go. You click a goal in a 3D web view, the planner draws the route in yellow, and a browser-based simulation plays back what the robot *would* do on that route. When everything looks right, the same route is shipped over to a Raspberry Pi on the robot, which actually drives it. Think of this repo as the part that does the *thinking* — the Pi does the *moving*.

## The big picture

```
   ┌───────────────┐    ┌──────────────┐    ┌───────────────┐    ┌───────────────┐
   │ 3D scan (.db) │    │ Cleaned 2D   │    │ Planner       │    │ JSON path     │
   │ from RTAB-Map │───▶│ floor plan   │───▶│ (A* + smooth) │───▶│ (waypoints +  │
   │ mapping run   │    │ (map.json)   │    │               │    │  yaw + meta)  │
   └───────────────┘    └──────────────┘    └───────────────┘    └───────┬───────┘
       db_to_map_json.py   inpaint_floor.py    astar_planner.py          │
       + tighten_obstacles.py                  compute_plan()            │
                                                                         │
                                       ┌─────────────────────────────────┴─────────┐
                                       │                                           │
                                       ▼                                           ▼
                              ┌─────────────────┐                          ┌────────────────┐
                              │ Browser preview │                          │ Real robot     │
                              │ (mock executor, │                          │ (Raspberry Pi  │
                              │  yellow path,   │                          │  + ROS 2 node, │
                              │  2D path tab)   │                          │  swerve drive) │
                              └─────────────────┘                          └────────────────┘
                              sim_navigator.py                             navigation_node.py
                              + index.html                                 (in friend's repo)
```

## How it works (5 stages, plain English)

1. **Stage 1 — From 3D scan to 2D floor plan.** *(map preparation)* The robot drives around the room with a depth camera and builds up a 3D point cloud (`tb3_1_room.db`). `db_to_map_json.py` exports that cloud, throws away noise (Statistical Outlier Removal — drops lonely floating points; DBSCAN — drops small disconnected blobs), and slices it into a simple top-down grid where every 5 cm cell is labelled "ground", "obstacle" or "unknown". `inpaint_floor.py` then fills in floor holes the camera missed, and `tighten_obstacles.py` rescues short objects (panel rims, tire bases) that the floor-detection step accidentally labelled as ground.

2. **Stage 2 — Drawing the route.** *(global path planning with A\*)* Given the cleaned grid and a click-selected goal, `astar_planner.compute_plan()` first inflates every obstacle by the robot's footprint so the planner can safely treat the robot as a single point. Then it runs *A\** — the classic shortest-path search that explores cells in order of "how far have I gone + how far do I still have to go" — to produce a sequence of grid waypoints from start to goal.

3. **Stage 3 — Smoothing and safety nudges.** *(path refinement)* The raw A\* output has 1-cell zigzags and skims close to walls. `apf_refine_path()` re-samples the line at finer spacing near obstacles, then nudges every waypoint *away* from nearby walls using a soft repulsive force (an *Artificial Potential Field*, or APF — imagine each wall blowing a gentle wind that pushes the path outward). Finally `arclength_resample()` lays the points down at uniform 12–15 cm intervals so the robot's onboard controller has neither too few nor too many to chew on.

4. **Stage 4 — Hand-off to the robot.** *(JSON path file)* The planner saves the finished route to `path_plan.json` — a plain text file listing every waypoint with its (x, y) position and yaw (which way the robot should be facing there), plus metadata like the inflation radius that was actually used. The browser reads this file and offers two views: the 3D map with the route drawn on top, and a clean 2D "path-only" tab that shows exactly what the robot will receive.

5. **Stage 5 — Execution.** *(Pi takes over)* The Raspberry Pi on the robot is subscribed to a standard ROS topic. When the browser publishes the path on that topic (or the user uploads the JSON file directly), the Pi's `navigation_node.py` runs its own onboard control loop — closing the small remaining gap between "planned point" and "actual position" with a velocity ramp, a yaw controller, and the same APF math as a safety net. The laptop is no longer in the loop once execution starts.

## What this project IS NOT

This is the **planning brain**, not the robot. The simulation in the browser is a *mock* — `sim_navigator.py` pretends to be a real robot driving the planned path, so you can preview the motion before committing to hardware. **Real execution happens on the Raspberry Pi**, in a separate codebase (the swerve-drive project). If the simulation looks fine but the real robot misbehaves, the bug is almost certainly on the Pi side, not here.

## Quick start

> **Python note:** use `/usr/bin/python3` (the system Python). Homebrew's Python 3.14 currently ships a broken `pyexpat` that crashes some of the dependencies; the system Python avoids it.

```bash
# 1. Generate / verify map.json from your latest mapping .db
/usr/bin/python3 db_to_map_json.py --db tb3_1_room.db --output map.json
/usr/bin/python3 inpaint_floor.py
/usr/bin/python3 tighten_obstacles.py

# 2. Start the planner server (Flask, listens on :5002)
/usr/bin/python3 server.py --map map.json --port 5002

# 3. (Optional) Start the mock executor for live preview
/usr/bin/python3 sim_navigator.py

# 4. Open the GUI
open http://localhost:5002
```

In the browser: click **Sim Pose** (or wait for a real localized pose), click somewhere on the floor to drop a goal, press **Plan Path**, then watch the yellow route appear. Switch to the **2D Path** tab to see exactly what would be sent to the Pi.

## What lives where

| File | Role | Open this when you need to… |
|---|---|---|
| `db_to_map_json.py` | RTAB-Map `.db` → `map.json` (3D cloud → 2D grid) | …regenerate the map after a new mapping run |
| `inpaint_floor.py` | Fills floor holes inside the room | …the planner refuses to route through a clearly-walkable patch |
| `tighten_obstacles.py` | Promotes short objects from "ground" to "obstacle" | …the planner thinks a tire stack or panel is floor |
| `astar_planner.py` | A* + APF smoothing + arc-length resample | …tune planner clearance, smoothness, or step size |
| `server.py` | Flask backend (map, plan, pose endpoints) on :5002 | …change endpoints, ports, or the wire format |
| `index.html` | The web UI: 3D view, click-to-goal, Plan Path, 2D path view | …adjust visuals, button behaviour, or rosbridge bindings |
| `sim_navigator.py` | ROS-free mock of the robot for end-to-end preview | …test the full pipeline without real hardware |
| `ros_pose_bridge.py` | Streams the live robot pose from ROS TF into the GUI | …the LOC pill is stuck on `NO BRIDGE` |
| `map_to_obstacles.py` | Extracts obstacle blobs as (x, y, radius) circles | …the navigation node needs an obstacle list |
| `mock_ros_bridge.py` | Stand-in for the Pi-side ROS bridge during Mac dev | …you're testing on a laptop with no ROS install |
| `path_plan.json` | The current plan (regenerated on every Plan Path click) | …debug what the Pi is about to receive |
| `map.json` | The currently-served floor plan | …confirm preprocessing produced something sane |

## What gets sent to the Pi

When you click **Plan Path**, the planner writes `path_plan.json` — a Python dictionary serialized as JSON, listing every waypoint as `(x, y, yaw)` plus formation metadata. The browser reads that file back, draws it, and re-publishes it on the standard ROS topic `/navigation/waypoints` as a `geometry_msgs/PoseArray` message (via rosbridge over WebSocket). The Pi subscribes to that topic, drops the points into its own waypoint queue, and starts driving. The full topic table, message schemas, and the leader's per-tick control loop are documented in **[`NAVIGATION_PIPELINE.md`](NAVIGATION_PIPELINE.md)** — go there for the technical reference.

## Glossary

- **A\*** — a shortest-path search algorithm; explores grid cells in order of "cost so far + estimated remaining cost."
- **APF (Artificial Potential Field)** — treats the goal as an attractive force and obstacles as repulsive forces; the robot follows the resulting "wind."
- **holonomic** — can move in any direction without first turning; this robot can strafe sideways.
- **swerve** — a wheel module that can both spin (drive) and pivot (steer) independently. Four of these give the robot its holonomic motion.
- **virtual center** — an imaginary point in the middle of the formation; the planner plans for *that* point and each robot keeps a fixed offset from it.
- **lookahead** — how far ahead on the path the controller is aiming at any instant.
- **formation** — the group of robots moving together while keeping a fixed shape.
- **occupancy grid** — a 2D top-down array where each cell is "ground", "obstacle", or "unknown".
- **inflation** — bloating obstacles by the robot's radius so the planner can treat the robot as a point.
- **EKF (Extended Kalman Filter)** — a sensor-fusion math trick that combines wheel odometry and visual pose into a smooth best-guess of where the robot is.
- **RANSAC** — fits a flat plane through a noisy point cloud; used here to find the floor.
- **DBSCAN** — clusters nearby points and tags lonely ones as outliers; used here to drop ghost blobs.
- **rosbridge** — a small WebSocket server that lets a browser talk to ROS as if it were a native client.
- **PoseArray** — a standard ROS message containing an ordered list of `(position, orientation)` poses; the format the Pi expects waypoints in.

## Tuning knobs you'll actually touch

| Parameter | Where | Raise it to… | Lower it to… |
|---|---|---|---|
| `formation_radius` | `compute_plan()` arg in `server.py` | get a wider berth around walls; refuses tighter gaps | squeeze through narrower doorways; less safety margin |
| `target_spacing` | `arclength_resample()` default (0.15 m) | give the controller fewer, longer waypoints (smoother but lazier) | give it more, shorter ones (twitchier but tighter tracking) |
| `yaw_policy` | `compute_plan()` arg | set to `"tangent"` to face direction of motion, or `"hold"` to keep start yaw |  |
| `--dbscan-min-cluster-size` | `db_to_map_json.py` flag | drop more ghost blobs; risk losing real small objects | keep more detail; risk more noise in the map |
| `--robot-height` | `db_to_map_json.py` flag | only count obstacles up to this height; ignore tall stuff | include more of the column above the robot |

## Project structure

```
robot_path_planner/
├── README.md                 ← you are here
├── NAVIGATION_PIPELINE.md    ← end-to-end control flow + JSON/ROS schemas + citations
├── RUNBOOK.md                ← step-by-step operator manual + smoke test + troubleshooting
│
├── db_to_map_json.py         ← stage 1a: 3D cloud → 2D grid
├── inpaint_floor.py          ← stage 1b: fill floor holes
├── tighten_obstacles.py      ← stage 1c: rescue short obstacles
│
├── astar_planner.py          ← stages 2-3: A* + APF refinement + resample
├── map_to_obstacles.py       ← grid → obstacle circles for the controller
│
├── server.py                 ← Flask backend (port 5002)
├── ros_pose_bridge.py        ← ROS TF → HTTP relay for the live-pose pill
├── mock_ros_bridge.py        ← laptop-only stand-in for the above
│
├── sim_navigator.py          ← ROS-free mock of the robot driving the path
├── index.html                ← web UI (3D view + 2D path tab)
├── path_viewer.html          ← lightweight standalone path-only viewer
│
├── map.json                  ← currently-served floor plan
├── path_plan.json            ← latest planned route (overwritten on Plan)
├── tb3_1_room.db             ← source RTAB-Map mapping run
└── run_test.sh               ← convenience launcher
```

## Where to read next

- **[`NAVIGATION_PIPELINE.md`](NAVIGATION_PIPELINE.md)** — full control flow from browser click to wheel command. Covers the Mac↔Pi data boundary, the exact `geometry_msgs/PoseArray` shape sent to the robot, every `path_plan.json` field, and academic citations for all 15 algorithms in the stack.
- **[`RUNBOOK.md`](RUNBOOK.md)** — operator manual: prerequisites, first-run sequence, day-to-day commands, curl smoke test, and the top-5 failure modes with fixes. Aim is "from a fresh checkout to a working **Plan Path** click in under ten minutes."
- The header docstring at the top of each `.py` file is a self-contained mini-spec for that file. They are the source of truth when this README and the code disagree.

## Acknowledgements / citations

This project stands on classical mobile-robotics work — A\* search, Khatib's potential-field method for obstacle avoidance, Laplacian consensus for multi-robot formation control, and the RANSAC/DBSCAN family of cloud-cleaning algorithms. Full citations and the specific formulations used live in `NAVIGATION_PIPELINE.md` alongside the technical write-up of each layer.
