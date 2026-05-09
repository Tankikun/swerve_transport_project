# Runbook

Companion to **README.md** and **NAVIGATION_PIPELINE.md**. This file is
the operator's manual: someone who has never touched the system before
should be able to follow it top-to-bottom and reach a successful
**Plan Path** click in under ten minutes.

> **Scope.** This runbook covers the **Mac-side dev sandbox** (Flask + rosbridge
> + sim_navigator). The deployed system uses `path_planner_node` as a ROS 2
> node — its bring-up is documented in `swerve_bringup/launch/` in the team
> repo, not here.

---

## Prerequisites

- **Python interpreter:** `/usr/bin/python3` (Apple's system Python).
  Do **not** use Homebrew Python 3.14 — its `pyexpat` is broken and Flask /
  scipy will crash on import.
- **pip packages** (everything the system imports across `server.py`,
  `sim_navigator.py`, `mock_ros_bridge.py`, `db_to_map_json.py`, and the
  `interface/clean_map.py` alternative):

  ```bash
  /usr/bin/python3 -m pip install --user \
      Flask flask-cors numpy scipy scikit-learn matplotlib requests Pillow \
      websockets open3d
  ```

- **System tool** (only needed when generating `map.json` from a `.db`):

  ```bash
  brew install rtabmap
  ```

---

## First-run sequence (you have a `.db` file but no `map.json`)

Working directory: `~/Documents/F_Senior/robot_path_planner/`.

1. **(Optional) Sanity-check the .db loads:**

   ```bash
   rtabmap-databaseViewer tb3_1_room.db
   ```

2. **Convert + clean the cloud into `map.json`:**

   ```bash
   /usr/bin/python3 db_to_map_json.py --db tb3_1_room.db --output map.json
   ```

   Defaults are tuned for a 3–4 m indoor room: 2 cm grid, SOR + DBSCAN on,
   per-camera max-range 3.0 m. For a noisier scan try
   `--sor-std-ratio 1.0` or `--dbscan-min-cluster-size 200`.

   **Single-file alternative** lives in the swerve project at
   `~/Documents/F_Senior/swerve_transport_project/interface/clean_map.py`.
   It runs the same six stages (rtabmap-export → SOR → DBSCAN → RANSAC
   floor → floating-cluster → grid build) **plus** floor inpainting and
   a top-down JPEG preview, all inline:

   ```bash
   /usr/bin/python3 clean_map.py tb3_1_room.db --output map.json
   ```

3. **(Only if you used `db_to_map_json.py`)** post-process:

   ```bash
   /usr/bin/python3 inpaint_floor.py
   /usr/bin/python3 tighten_obstacles.py
   ```

   `clean_map.py` already does inpainting; skip these if you went that route.

4. **Verify the map looks sane:**

   ```bash
   /usr/bin/python3 -c "import json; m=json.load(open('map.json')); print(m['metadata'])"
   ```

   You should see `resolution`, `min_x`/`min_y`, `grid_width`/`grid_height`,
   and `point_count` in the thousands. If `point_count == 0` or the grid is
   1×1, the cleanup was too aggressive — re-run with looser thresholds.

---

## Day-to-day sequence (you already have a `map.json`)

Three terminals, all in `~/Documents/F_Senior/robot_path_planner/`.

1. **Terminal 1 — Flask + planner:**

   ```bash
   /usr/bin/python3 server.py --map map.json --port 5002
   ```

2. **Terminal 2 — mock executor** (drives the cyan robot indicator in
   the GUI; lets you preview motion):

   ```bash
   /usr/bin/python3 sim_navigator.py
   ```

3. **Terminal 3 (optional) — rosbridge stand-in** (needed only for the
   GUI's "ROS Connected" pill to go green and for **Send Goal** to publish):

   ```bash
   /usr/bin/python3 mock_ros_bridge.py --map map.json --port 9090
   ```

4. Open **`http://localhost:5002`** in a browser.

`./run_test.sh map.json` does steps 1 + 3 in one shot, but **not** step 2 —
start `sim_navigator.py` in another terminal afterward.

---

## Operating the GUI

Click in this order — `sim_navigator` will refuse to start until step 1 is done:

1. Click **Sim Pose** to drop the simulated robot at a starting (x, y, yaw).
2. Click anywhere on the green floor in the 3D view to set a goal.
3. Drag the orientation slider for the goal yaw.
4. Click **Plan Path** — `server.py` runs A\* + APF refinement, writes
   `path_plan.json`, and the browser polls it. A yellow polyline appears.
   The browser AUTO-publishes the new waypoints on `/navigation/waypoints`
   (`geometry_msgs/PoseArray`) as soon as it sees the new file — you do
   not need to click anything else for the Pi to receive them.
5. Switch to the **2D Path** tab to inspect the exact `(x, y, yaw)`
   waypoints that will be sent to the robot.
6. (Optional) **Send Goal** publishes a single `geometry_msgs/PoseStamped`
   on `/goal_pose` — that's the input to `path_planner_node` in the
   deployed system. It does NOT publish the waypoint list; that
   already happened in step 4. If a goal goes stale, re-run Plan Path
   (Send Goal would only re-publish the goal pose, not a new path).

---

## Verifying it works (curl smoke test)

```bash
# 1. Index page
curl -s -o /dev/null -w "%{http_code}\n" http://localhost:5002/
#   → 200

# 2. Map JSON keys
curl -s http://localhost:5002/map | \
  /usr/bin/python3 -c "import json,sys; d=json.load(sys.stdin); print(sorted(d.keys()), d['metadata']['point_count'])"
#   → ['colors', 'grid', 'metadata', 'points'] 416779

# 3. Plan a path explicitly (no pose dependency)
curl -s -X POST http://localhost:5002/plan \
     -H 'Content-Type: application/json' \
     -d '{"start":{"x":-4.36,"y":-3.33},"goal":{"x":-3.0,"y":-1.0},"yaw_policy":"free"}'
#   → {"status":"ok","seq":1,"vc_distance":3.44,"waypoints":28}

# 4. Pose relay (returns available:false until sim_navigator posts)
curl -s http://localhost:5002/pose
```

---

## Common failures and fixes

| Symptom | Cause | Fix |
|---|---|---|
| `FileNotFoundError: 'map.json'` | `server.py` started in a directory that has no map | `cd` into the project dir, or pass `--map /full/path/to/map.json` |
| `NotOpenSSLWarning: urllib3 ... LibreSSL` | macOS system Python links LibreSSL | Harmless — ignore the line |
| `[sim] FATAL: no localized pose in /pose` | `sim_navigator.py` started before any pose was set | Click **Sim Pose** in the browser first, then re-run `sim_navigator.py` |
| Goal click does nothing / `click >0.5m from any reachable cell` | Clicked outside the navigable region (gray) or inside an inflation halo | Click closer to the green floor and at least one robot-radius (~0.2 m) clear of walls |
| Browser shows "ROS Disconnected" | `mock_ros_bridge.py` isn't running | Start it on port 9090 (Terminal 3 above). Without it, planning still works — only **Send Goal** and live `/odom` markers are disabled. |

---

## Stopping cleanly

```bash
pkill -9 -f "sim_navigator"
pkill -9 -f "server.py.*5002"
pkill -9 -f "mock_ros_bridge"
```

Or by port (matches what `run_test.sh` does on startup):

```bash
lsof -ti:5002 | xargs kill
lsof -ti:9090 | xargs kill
```
