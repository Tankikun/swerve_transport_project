# RTAB-Map Branch — Session Summary

Branch: `feature/rtab-map`
Goal of the branch: stand up RTAB-Map visual SLAM on the OAK-D Lite
so the formation has world-frame localization and the EKF stops
drifting (root-cause of the 7° rotation drift documented in
`TIER1_NOTES.md`).

This file is the human-readable progress record. For end-to-end
operating procedure see `MAPPING_RUN_GUIDE.md`. For deep technical
detail see `CAMERA_NOTES.md`.

---

## Where this branch stands now

| Step                                         | Status |
|----------------------------------------------|--------|
| 1. Install `depthai-ros`                     | ✅ done by Earth (apt install on clean network) |
| 2. Verify camera publishes ROS topics        | ✅ verified on pi2 (oak_camera_node) |
| 3. Camera→base_link static TF                | ✅ in `oak_camera.launch.py` |
| 4. Install `rtabmap-ros`                     | ✅ done by Earth (apt install) |
| 5. Mapping launch (all-on-pi)                | ✅ `rtabmap_mapping.launch.py` |
| 5a. Localization launch + slam pose relay    | ✅ `rtabmap_localization.launch.py` + `slam_pose_relay_node` |
| 5b. Per-robot namespacing of rtabmap topics  | ✅ both launches remap localization_pose / info / cloud_map / mapData to `/{robot_id}/rtabmap/*` (CodeRabbit on PR #3) |
| 5c. Split-execution launches (pi=sensors, laptop=SLAM) | ✅ `rtabmap_pi_sensors.launch.py` + `rtabmap_laptop_mapping.launch.py` + `rtabmap_laptop_localization.launch.py` |
| **6. Mapping run (drive room with teleop)**  | ⏭ **next — operator does this** |
| 7. Localization run + EKF loop closure       | ⏭ after step 6 |

Pi2 is fully equipped (camera + udev rule + depthai SDK + all
rtabmap apt packages + our custom code, latest branch built clean).
Verified by `inventory_pi2.sh` and `final_verify.sh`.

---

## Camera mount geometry (recorded for the project)

The OAK-D Lite on tb3_1 is mounted at:

| arg | value | meaning |
|---|---|---|
| `cam_x` | `+0.128 m` | 128 mm forward of `base_link` |
| `cam_y` | ` 0.000 m` | centred laterally |
| `cam_z` | `-0.0175 m` | 17.5 mm **below** `base_link` |

`base_link` for tb3_1 is defined at the **top of the chassis where the
payload sits**, so the OAK-D being 17.5 mm below is correct (negative
Z). Rotation from `base_link` to `oak_rgb_camera_optical_frame` is the
standard ROS optical-from-body rotation (roll = pitch = yaw = -π/2).

These values are passed via launch args; defaults remain the
"placeholder" 0.10 / 0.00 / 0.15 to keep the launch self-contained,
but every operator command should override with the measured values.

---

## What this session actually did (chronological)

### Reading Tan's tweaks
Pulled `feature/rtab-map` to local; Tan + CodeRabbit had added 3
commits on top of the previous push:
- `cam_x/y/z` exposed as launch args on both mapping and
  localization launches (passes through to `oak_camera.launch.py`).
- `db_path` directory creation: `os.path.dirname(db_path) or '.'`
  instead of falling back to `/tmp` — saner default for relative
  paths.
- `rtabmap` odom subscription remapped from `/{robot_id}/odom` (raw
  wheel) to `/{robot_id}/ekf/odom` (EKF-fused). In mapping mode this
  is harmless. In localization mode it creates a feedback loop
  (rtabmap → ekf → rtabmap); see Troubleshooting in
  `MAPPING_RUN_GUIDE.md` for the rollback if jumps appear.

### Inventory pi2
Confirmed Earth's apt installs landed cleanly:
- `ros-humble-depthai-*` — 6 packages
- `ros-humble-rtabmap-*` — 14 packages including `rtabmap_slam`,
  `rtabmap_conversions`, `rtabmap_msgs`, `rtabmap_viz`
- depthai 3.5.0 Python SDK
- udev rule `80-movidius.rules` present (camera accessible without sudo)
- OAK-D-LITE detected on USB (Intel Movidius MyriadX)
- 45 GB free disk, 7.2 GB free RAM, 41 °C idle, no throttling history

### Sync + rebuild on pi2
- rsync latest `feature/rtab-map` source → pi2 `~/ros2_ws/src/`
- First rebuild attempt failed: stale broken symlink to
  `single_robot_nav.launch.py` (file from a different branch left
  behind in the build cache from a previous nav-test rebuild).
- Cleaned by `rm -rf ~/ros2_ws/{build,install}/swerve_bringup` and
  rebuilding fresh.
- Both launch files now installed:
  `rtabmap_mapping.launch.py`, `rtabmap_localization.launch.py`
- Both new executables present:
  `oak_camera_node`, `slam_pose_relay_node`
- Both launches parse cleanly with the new `cam_x/y/z` args (verified
  via `ros2 launch ... --show-args`).

### Documentation produced this session
- `MAPPING_RUN_GUIDE.md` — operator-facing run procedure for step 6
  (terminal layouts, driving rules, troubleshooting). Now covers
  both the SPLIT (recommended) and ALL-ON-PI (fallback) paths.
- `RTAB_SESSION_SUMMARY.md` — this file.

### Follow-up changes after the initial push (Tan + CodeRabbit feedback)

- **Per-robot namespacing of rtabmap topics**. CodeRabbit caught
  that rtabmap publishes `localization_pose`, `info`, `cloud_map`,
  `mapData` to GLOBAL topic names by default — meaning two robots
  running localization concurrently would consume each other's
  poses. Added explicit remappings in both
  `rtabmap_mapping.launch.py` and `rtabmap_localization.launch.py`
  to scope every output to `/{robot_id}/rtabmap/*`. The
  `slam_pose_relay`'s `in_topic` was updated to match.
- **Split-execution architecture**. Tan + the team raised the
  thermal concern of running rtabmap_slam on the Pi 4 (visual
  feature extraction + graph optimisation under sustained mapping
  pushes the Pi to 70-80 °C). Added three new launches that split
  the work: pi runs sensors only, laptop runs rtabmap_slam. The
  all-on-pi launches remain as a fallback. See `MAPPING_RUN_GUIDE.md`
  Path (A) vs Path (B).

---

## Files touched / produced (across the whole branch)

```
ros2_ws/src/swerve_formation/swerve_formation/
    oak_camera_node.py            # custom OAK-D ROS publisher (depthai 3.x)
    slam_pose_relay_node.py       # rtabmap pose → ekf_node format

ros2_ws/src/swerve_formation/setup.py
    # registered both new console_script entry points

ros2_ws/src/swerve_bringup/launch/
    oak_camera.launch.py                       # camera + base_link↔optical TF
    rtabmap_mapping.launch.py                  # all-on-pi: mapping
    rtabmap_localization.launch.py             # all-on-pi: localization (closes EKF loop)
    rtabmap_pi_sensors.launch.py               # split: pi-side sensors only
    rtabmap_laptop_mapping.launch.py           # split: laptop-side rtabmap (mapping)
    rtabmap_laptop_localization.launch.py      # split: laptop-side rtabmap (localization)

CAMERA_NOTES.md             # deep-dive: depthai 3.x quirks, apt mirror workaround
MAPPING_RUN_GUIDE.md        # top-level index pointing to per-side guides
MAPPING_RUN_PI.md           # operator procedure — pi side (sensors)
MAPPING_RUN_LAPTOP.md       # operator procedure — laptop side (rtabmap + teleop)
RTAB_SESSION_SUMMARY.md     # THIS FILE
```

---

## Next concrete step

Operator runs the mapping launch with the measured mount offsets
(128, 0, -17.5 mm) on pi2, teleops through the room, saves the .db.
Then re-launches with `rtabmap_localization.launch.py` to verify the
robot localizes itself in the saved map and `/tb3_1/ekf/odom`
becomes drift-corrected.

Exact commands and driving rules are in `MAPPING_RUN_GUIDE.md`.

After that, the same procedure repeated on pi1 (tb3_0) once Earth
applies the apt installs there. Both robots will then localize against
their own .db (or, eventually, a shared single .db once both robots
have been used for mapping the same room).

---

## Known things to address before merging to main

- Camera mount geometry IS now measured (above) — but the launch
  defaults still show the placeholder. Either bake the real values
  in or document that operators must always override. Current
  preference: keep defaults harmless and require an explicit pass.
- The `/ekf/odom` feedback loop in localization mode is unproven —
  watch first localization run for jumpy poses; if seen, revert that
  remap (see Troubleshooting in `MAPPING_RUN_GUIDE.md`).
- Localization launch has not yet been hardware-tested. Step 6 is
  the gate.
