# mocap_test/ — MoCap accuracy benchmark for the swerve formation

A scripted closed-loop trajectory used to measure motion-tracking
accuracy against a motion-capture ground truth, across all three of
the swerve drive's motion primitives:

1. **Strafing** (pure translation, no rotation)
2. **Arc turn** (simultaneous translation + rotation)
3. **Pivot turn** (pure in-place rotation)

The trajectory is closed (start = end), so you can run it back-to-back
to look at how error accumulates over multiple loops.

---

## What this branch adds

| File | Purpose |
|---|---|
| `mocap_test/trajectory_publisher.py` | ROS 2 node that publishes `/virtual_center/cmd_vel` along the U-loop. Open-loop, time-based. |
| `mocap_test/README.md` | This file. |

Nothing else in the repo is modified. The script depends on the
existing `laplacian_formation_node` (already on `main`) to translate
the virtual-centre twist into per-robot `/{robot_id}/cmd_vel`.

---

## The trajectory

A closed loop traced as 5 legs. The "U" is the first 3 legs (strafe,
arc, strafe); legs D and E close the bottom of the U back to the start.

```
              Y (m, world "north")
                   ^
                   |
               1.0 +   3 <===============  2
                   |  / \                  ^
                   | /   \                 |
                   ||     |  arc CCW       |
                   ||     |  R = 0.5 m     |  leg A
                   ||     |  (leg B)       |  1.0 m
                   ||     |                |
               0.5 +v     |                |
                   ||     |                |
                   ||leg C|                |
                   ||1 m  |                |
                   |v     |                |
               0.0 +  4 ================ 1   <-- START (heading 90 deg)
                   | (5)  leg E
                   |  ^   1.0 m
                   |  |
                   |pivot +90 deg
                   |
                   +----+----+----+--------> X (m, world "east")
                      -1.0 -0.5  0.0
```

### Per-leg motion (body-frame, virtual centre)

| Leg | From -> To | Motion | cmd_vel (default) | Duration (default) |
|-----|------------|--------|-------------------|--------------------|
| A   | 1 -> 2     | Strafe forward 1.0 m       | `vx=0.10, wz=0`    | 10.0 s |
| B   | 2 -> 3     | Arc CCW 180 deg, R=0.5 m   | `vx=0.07, wz=0.15` | 20.94 s |
| C   | 3 -> 4     | Strafe forward 1.0 m       | `vx=0.10, wz=0`    | 10.0 s |
| D   | 4 -> 5     | Pivot +90 deg in place     | `vx=0,    wz=0.20` | 7.85 s |
| E   | 5 -> 1     | Strafe forward 1.0 m       | `vx=0.10, wz=0`    | 10.0 s |

A 2 s zero-twist pause is held at every checkpoint (between legs) so
each transition shows up as a flat segment in MoCap and odometry traces.

Total wall-clock time per loop: roughly **70 s** at default speed
(including pauses), or **140 s** with `--slow`.

### Checkpoint world-frame poses

| # | x (m) | y (m) | yaw (deg) | Pointing |
|---|-------|-------|-----------|----------|
| 1 |  0.0  |  0.0  |    90     | North (start) |
| 2 |  0.0  |  1.0  |    90     | North |
| 3 | -1.0  |  1.0  |   270     | South |
| 4 | -1.0  |  0.0  |   270     | South |
| 5 | -1.0  |  0.0  |     0     | East  |
| 1 |  0.0  |  0.0  |     0     | East (loop closes; heading does not match start) |

Note: the loop closes in *position* but not in *heading* (start is
North, end is East). Run two loops back-to-back to see the heading
walk; that's a useful indicator of yaw drift independent of position.

---

## Why these speeds

Speeds were chosen so the **outer robot of the largest tested
formation (D=0.9 m) stays inside the per-wheel cap (`MAX_WHEEL_LINEAR
= 0.18 m/s`)** during the arc, *without* the laplacian controller
needing to scale the formation down. That keeps the arc shape circular
across all formation sizes — otherwise the controller's saturation
would distort the timing on larger formations.

For the arc (worst case: outer robot of a side-by-side formation):

```
v_robot_center = vx_vc + omega_vc * (D / 2)
v_wheel_max    = v_robot_center + omega_vc * 0.212    # chassis half-diagonal
```

| D     | vx_vc=0.07, omega_vc=0.15 | vs cap 0.18 m/s |
|-------|---------------------------|-----------------|
| 0.5 m | 0.135 m/s wheel max       | 25 % margin     |
| 0.7 m | 0.151 m/s wheel max       | 16 % margin     |
| 0.9 m | 0.169 m/s wheel max       |  6 % margin     |

If you push the speeds higher, `laplacian_formation_node` will
auto-scale the formation; the trajectory still traces correctly but
slower than the script expects, which corrupts segmentation by
checkpoint timestamp. Use `--slow` if you want to drop the speeds
further (everything halves).

---

## Formation distances under test

The user-defined sweep is **D = 50 cm, 70 cm, 90 cm** between the two
robots' base_link origins. The same script drives all three — the
distance is set in the laplacian launch parameters, not in this
script:

```bash
# Each robot launches its formation node with my_offset / neighbor_offsets.
# For D=0.7 with VC at the midpoint, side-by-side along body Y:
#   tb3_0:  my_offset:=[0.0, +0.35]   neighbor_offsets:=[0.0, -0.35]
#   tb3_1:  my_offset:=[0.0, -0.35]   neighbor_offsets:=[0.0, +0.35]
```

Reasonability check (chassis is 0.50 m x 0.35 m):

| D     | Gap between chassis | Notes |
|-------|---------------------|-------|
| 0.50 m | 0.15 m               | Tight but safe; chassis edges 15 cm apart side-by-side. |
| 0.70 m | 0.35 m               | Comfortable working distance. |
| 0.90 m | 0.55 m               | Largest meaningful spacing inside a 2 m x 2 m MoCap volume. |

---

## How to run

You need 5 things up before launching the trajectory:

1. **MoCap publishing** — rigid body for each robot, published to e.g.
   `/tb3_0/mocap/pose` and `/tb3_1/mocap/pose` (PoseStamped, 100 Hz+).
   Bridge depends on which MoCap system you use (OptiTrack, Vicon, etc.).
2. **Per-robot stack** — Pi sensor + base launch on each robot, plus
   `laplacian_formation_node` configured for the chosen formation D.
3. **Time sync** — chrony / NTP / PTP between MoCap host and robots.
   Skew >10 ms shows up as a phantom yaw error during the arc.
4. **Bag recording** — see below.
5. **Robot placement** — place the formation centroid at the MoCap
   origin with each robot facing world +Y (north). Ground-truth pose
   at start should be `(x, y, theta) = (0, 0, pi/2)`.

### Bag the run

```bash
ros2 bag record \
  /virtual_center/cmd_vel \
  /tb3_0/mocap/pose \
  /tb3_1/mocap/pose \
  /tb3_0/odom \
  /tb3_1/odom \
  /tb3_0/ekf/odom \
  /tb3_1/ekf/odom \
  /tb3_0/cmd_vel \
  /tb3_1/cmd_vel \
  /tb3_0/imu \
  /tb3_1/imu \
  /tb3_0/joint_states \
  /tb3_1/joint_states \
  -o run_D$(printf "%02d" $D_CM)_$(date +%Y%m%d_%H%M)
```

(Set `D_CM=50`, `70`, or `90` per run so the bag name reflects the
formation distance.)

### Drive the trajectory

In a sourced ROS 2 shell on the laptop:

```bash
python3 mocap_test/trajectory_publisher.py
# or, half-speed:
python3 mocap_test/trajectory_publisher.py --slow
```

The script publishes Twist at 20 Hz to `/virtual_center/cmd_vel`,
holds zero between legs, and exits when the loop is complete. Stops
the formation cleanly on Ctrl-C.

### After the run

For each leg, slice the bag at the checkpoint pause boundaries (each
flat zero-velocity segment in `/virtual_center/cmd_vel` marks a
checkpoint), then compute per-segment error:

| Leg | Per-segment metrics |
|-----|---------------------|
| A, C, E | translation magnitude (MoCap vs odom), cross-track in body frame |
| B       | translation + rotation jointly; APE per pose vs ground-truth arc |
| D       | rotation only; integrated yaw vs MoCap yaw |

`evo` (https://github.com/MichaelGrupp/evo) handles all of these:

```bash
evo_traj bag2 run_D70_*.bag /tb3_0/mocap/pose --ref /tb3_0/ekf/odom --align --plot
evo_ape  bag2 run_D70_*.bag /tb3_0/mocap/pose /tb3_0/ekf/odom --align
evo_rpe  bag2 run_D70_*.bag /tb3_0/mocap/pose /tb3_0/ekf/odom --align --pose_relation angle_deg
```

The interesting comparison is: **same trajectory, different formation
distances**. If error scales linearly with D, the dominant term is
formation-mechanical (one robot dragging the other through wheel
slip). If error is roughly D-invariant, the dominant term is per-robot
odometry calibration — and you should treat each robot independently.
