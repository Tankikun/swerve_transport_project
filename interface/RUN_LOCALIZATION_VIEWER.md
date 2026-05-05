# Live Localization Viewer — Operator Runbook

A green/orange/red status pill plus a robot icon that moves on the 2D and 3D
maps as the robot drives. Single-glance answer to **"is localization working
right now?"**.

---

## What this gives you

| Element | What it tells you |
|---|---|
| **LOC status pill** (header, far right) | Localization health, color-coded |
| **Cyan triangle** (2D map) | Robot's current pose in the map frame |
| **Cyan cone + disk** (3D map) | Same pose, in 3D, sitting on the floor |

The viewer is decoupled from the rosbridge code: it works whether or not the
existing `/tb3_0/pose` rosbridge subscriber is connected. The only required
moving parts are the localization stack, `ros_pose_bridge.py`, and the Flask
server.

---

## Prerequisites

1. The localization stack is running per **`LOCALIZATION_RUN_LAPTOP.md`**
   (RTAB-Map in localization mode + the camera + robot). Confirm with:
   ```bash
   ros2 topic list | grep slam/pose
   ros2 run tf2_ros tf2_echo map tb3_1_base_link
   ```
2. `requests` Python package installed in whatever Python runs
   `ros_pose_bridge.py`:
   ```bash
   pip install requests
   ```
3. A current `map.json` for the same physical room. If you've redone the
   mapping run, regenerate it (next step).

---

## Step 1 — Update map.json

Run **on the Mac** (or wherever `rtabmap-export` is installed):

```bash
cd interface/
./regenerate_map.sh ~/maps/tb3_1_room.db map.json
```

Or call `db_to_map_json.py` directly with custom flags — see
`interface/README.md` for the full flag table.

Sanity check: open the file size — a healthy map.json is ~2-5 MB. If it's
under 200 KB, the cleanup over-filtered.

---

## Step 2 — Start the Flask server

```bash
cd interface/
python3 server.py --map map.json --port 5002
```

Leave this terminal open. You should see:

```
Loading map: map.json
Map loaded: 60000 points
Server running at http://localhost:5002
```

---

## Step 3 — Start the ROS pose bridge

In a **new terminal** with ROS sourced:

```bash
source ~/swerve_transport_project/ros2_ws/install/setup.bash
python3 interface/ros_pose_bridge.py --ros-args -p robot_id:=tb3_1
```

Expected output:

```
[INFO] [ros_pose_bridge]: ros_pose_bridge: robot_id=tb3_1 -> http://localhost:5002/pose @ 10.0 Hz
```

If you see `WARN: TF map -> tb3_1_base_link not available` repeated every
5 s, the localization stack hasn't matched yet — drive the robot a bit
until RTAB-Map finds its first visual match.

---

## Step 4 — Open the GUI

Browser → **http://localhost:5002**

The header should show three pills. The leftmost two are Run's existing
"Awaiting map…" / "ROS Disconnected". The rightmost is the new
**`LOC: …`** pill from this viewer.

Drive the robot via teleop (or `cmd_vel`). You should see:

- The 2D triangle slide along its path
- The 3D cone follow on the same path, oriented with the robot's heading
- The pill stay green-ish while RTAB-Map is happy

---

## What the badge means

| Badge | Color | Meaning | Likely cause |
|---|---|---|---|
| `LOC: LIVE x.xx,y.yy` | green | TF `map -> base_link` is live and a visual match landed in the last 5 s | Healthy localization |
| `LOC: DEAD-RECK Xs` | orange | TF still tracking via odometry but RTAB-Map hasn't matched in X s | Featureless area, mapping hole, sensor occlusion |
| `LOC: SEARCHING Xs` | red | No `map -> base_link` TF available | RTAB-Map hasn't found its first match yet, or the localization stack is down |
| `LOC: STALE Xs` | red | Pose data on the server is older than 2 s | `ros_pose_bridge.py` died or its host can't reach the Flask server |
| `LOC: NO BRIDGE` | red | Flask server is up but no bridge has ever POSTed a pose | `ros_pose_bridge.py` was never started |
| `LOC: SERVER DOWN` | red | Browser can't reach `/pose` | `server.py` is not running, or port 5002 is blocked |

---

## Troubleshooting

| Symptom | Fix |
|---|---|
| Pill stays `LOC: SEARCHING` for >30 s | Drive the robot 30-60 cm in a featureful area — RTAB-Map needs visual texture for the first match. If still none, `ros2 topic echo /tb3_1/slam/pose` should be silent — confirm the localization stack actually runs. |
| Pill flips between `LIVE` and `DEAD-RECK` | Normal in low-feature corridors. If it happens in well-lit rooms, the map is poor — re-mapping run is in order. |
| Marker is in the wrong room corner | Map frame mismatch. `map.json` was generated from a different `.db` than what RTAB-Map is currently using for localization. Regenerate with the matching `.db`. |
| Marker is at the right place but rotated 180° | Yaw sign convention regression. Check the firmware odom yaw fix is still in place (`turtlebot3_conveyor.ino` should negate `wz` per REP-103). |
| `ros_pose_bridge.py` crashes immediately | Likely missing `requests` (`pip install requests`) or wrong ROS sourcing (`echo $ROS_DISTRO` should show `humble`). |
| `requests.exceptions.ConnectionError` flooding the bridge log | Flask server isn't reachable. Confirm `curl http://localhost:5002/` returns the HTML. The bridge already swallows these errors silently — if you see them you may have an old version. |
| Status pill says `LIVE` but the marker doesn't appear | Browser console likely has an error. Open DevTools (F12), reload, check for `mapData.metadata` undefined or a similar map-load failure. |
| Marker disappears when the robot moves | `livePose.x / livePose.y` left the map's bounding box. Either the robot really did leave the mapped area, or the map was generated with too tight a `--bbox`. |

---

## Cross-network setup

If the Flask server runs on a different machine than the bridge:

```bash
python3 interface/ros_pose_bridge.py --ros-args \
    -p robot_id:=tb3_1 \
    -p server_url:=http://<server-ip>:5002/pose
```

Make sure the server is listening on `0.0.0.0` (it is by default in
`server.py`'s `run()`) and the firewall allows port 5002.
