# interface/ — Web GUI for RTAB-Map localization

This folder contains the **laptop-side web GUI** for visualizing the SLAM
map and watching the robot localize against it in real time. The GUI is
3D-only: a textured point cloud rendered with Three.js, with a click-to-set-goal
floor plane and a click-to-set-initial-pose tool.

For the data-preparation pipeline that turns an RTAB-Map `.db` into the
`map.json` this GUI consumes, see your branch's preprocessing docs (the
preprocessing has changed over time; whichever script is in this folder
right now is the one to use).

---

## Operator runbook

To actually run the localization viewer end-to-end (5 terminals + a
browser), open **[`RUN_LOCALIZATION_VIEWER.md`](RUN_LOCALIZATION_VIEWER.md)**.
It's a self-contained, step-by-step guide that takes you from a powered-off
robot to a green `LOC: LIVE` pill in ~10 minutes.

---

## What's in this folder

| File | Purpose |
|---|---|
| `index.html` | The web UI itself — Three.js 3D point cloud, click-to-goal floor raycast, "Set Initial Pose" tool, LOC status pill |
| `server.py` | Flask backend serving `index.html`, `map.json`, and the `/pose` + `/set_initial_pose` mailbox endpoints |
| `ros_pose_bridge.py` | rclpy node that streams `map → base_link` TF to the server every 100 ms and republishes GUI initial-pose hints to RTAB-Map's `/initialpose` |
| `db_to_map_json.py` | Preprocessing — converts an RTAB-Map `.db` into the `map.json` the GUI consumes |
| `map.json` | Currently-served map (regenerate from your `.db` when needed) |
| `RUN_LOCALIZATION_VIEWER.md` | Step-by-step runbook for the localization viewer |
| `README.md` | (this file) |

---

## How the live-pose loop fits together

```
                          ┌────────────────────────┐
                          │       Browser          │
                          │   (index.html, 3D)     │
                          └───┬──────────────────▲─┘
                              │                  │
              POST            │ GET /pose        │
   /set_initial_pose          │ (every 200 ms)   │
                              │                  │
                              ▼                  │
                          ┌────────────────────────┐
                          │      server.py         │
                          │  (Flask, port 5002)    │
                          └───▲──────────────────┬─┘
                              │                  │
              GET             │ POST             │
   /set_initial_pose          │ /pose            │
   (every 100 ms)             │ (every 100 ms)   │
                              │                  ▼
                          ┌────────────────────────┐
                          │  ros_pose_bridge.py    │
                          │  (rclpy node)          │
                          └───▲──────────────────┬─┘
                              │                  │
                  TF lookup   │                  │ Publish PoseWithCovarianceStamped
       map -> tb3_1_base_link │                  │ to /initialpose (single shot
                              │                  │ per new GUI hint)
                              │                  ▼
                          ┌────────────────────────┐
                          │       RTAB-Map         │
                          │  (laptop_localization) │
                          └────────────────────────┘
```

Two independent roles for the bridge:

1. **Live pose feed** — every 10 Hz, look up the canonical
   `map → {robot_id}_base_link` TF and POST it to `/pose`. The browser
   polls `/pose` to drive the LOC status pill (`LIVE` / `DEAD-RECK` /
   `SEARCHING` / `STALE`) and the cyan cone marker.
2. **Initial-pose republisher** — every 10 Hz, GET `/set_initial_pose`.
   The browser writes there when the user uses the 📍 button. When the
   `seq` increments, the bridge publishes one
   `PoseWithCovarianceStamped` to `/initialpose` for RTAB-Map to seed
   its current pose estimate. This is the same topic AMCL / RViz
   "2D Pose Estimate" use, so RTAB-Map converges in 1–2 seconds
   instead of doing a slow global re-localization search.

The server is intentionally a tiny, stateless mailbox — it doesn't
queue or order anything, it just remembers the latest message in each
direction.

---

## Quick start

Once you have a `map.json` ready in this folder:

```bash
# Terminal 2 (laptop): web backend
cd interface
python3 server.py --map map.json --port 5002

# Terminal 4 (laptop, separate sourced shell with ROS): live-pose bridge
source /opt/ros/humble/setup.bash
source ~/swerve_transport_project/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=30
python3 ros_pose_bridge.py --ros-args -p robot_id:=tb3_1

# Browser
open http://localhost:5002
```

Plus T1 (Pi sensors), T3 (RTAB-Map in localization mode), and T5
(teleop) — see the runbook for the full sequence. Order matters; the
bridge will warn loudly if RTAB-Map isn't producing the
`map → tb3_1_base_link` TF yet.

---

## GUI controls

In the 3D panel:

| Action | Result |
|---|---|
| **Drag** | Orbit the camera around the cloud's centre |
| **Scroll** | Zoom in/out |
| **Double-click** | Refit the camera so the whole cloud fills the panel |
| **Short click on the floor** (no drag) | Raycast onto the floor plane → set goal at world (X, Y, floor_z). Drag-clicks (>5 px motion) are filtered out |
| **📍 Set Initial Pose button** | Enter `PLACING` mode. Next click anchors a position; mouse-move thereafter sets yaw; second click confirms and POSTs to `/set_initial_pose` |
| **Escape** | Cancel an active "Set Initial Pose" without sending |

In the bottom bar:

- **X / Y / Z** show the world coordinates of the most recent floor click
- **Orientation slider** (0–359°) sets the goal yaw; the cyan goal-arrow
  rotates live as you drag the slider
- **Send Goal** publishes the current `(X, Y, Z, yaw)` to ROS topic
  `/goal_pose` via rosbridge (Nav2-compatible). If rosbridge isn't
  reachable on `ws://localhost:9090`, the button shows `Failed`.

---

## Status pill cheat sheet

| Pill | Color | Meaning |
|---|---|---|
| `LOC: LIVE x.xx,y.yy` | 🟢 green | Localized; visual match within last 5 s |
| `LOC: DEAD-RECK Xs` | 🟠 orange | TF tracking but no fresh visual match in X seconds |
| `LOC: SEARCHING Xs` | 🔴 red | RTAB-Map hasn't matched yet |
| `LOC: STALE Xs` | 🔴 red | Server hasn't received a pose POST in > 2 s — bridge died |
| `LOC: NO BRIDGE` | 🔴 red | Server up but no bridge has ever posted — bridge never started |
| `LOC: SERVER DOWN` | 🔴 red | Browser can't reach `/pose` — Flask server died |

Full troubleshooting for each of these states is in the runbook.

---

## Mapping advice (separate from this folder's runtime)

The localization can only do as well as the underlying map. **Better
source data beats more aggressive cleanup every time.** When doing a
fresh mapping run:

1. Drive **slowly** (0.1–0.15 m/s) — fast motion → blur → bad ORB → bad map
2. **Pause + spin 360°** at every station — captures features from all angles
3. **Multiple passes** through the same area — drives loop closures (the main quality signal)
4. **Return to start** — forces a global loop closure that tightens the trajectory
5. Avoid: glass, mirrors, blank walls, fast turns, dim light

A healthy mapping run for a small room produces 20–30 loop closures.
If your `.db` shows fewer, expect localization to be twitchy or
require initial-pose hints in more areas.
