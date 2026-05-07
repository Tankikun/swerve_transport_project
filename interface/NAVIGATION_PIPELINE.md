# Navigation Pipeline — `navigation_node.py` end-to-end

This document walks the *control flow* from "user clicks goal in the
browser" all the way to "individual robot wheels turn." It covers the
five layers your friend's swerve-formation system uses.

```
       BROWSER                      SERVER                ROBOT (running navigation_node)
  ┌────────────────┐         ┌────────────────────┐    ┌──────────────────────────────────┐
  │  index.html    │  POST   │   server.py        │    │  navigation_node.py              │
  │  3D view       │ /plan   │                    │    │                                  │
  │  click goal    │ ──────▶ │  → astar_planner   │    │  /navigation/goal       (Twist)  │
  │  click Plan    │         │     compute_plan   │    │  /navigation/waypoints  (PoseArr)│
  │                │         │                    │    │  /navigation/obstacles  (PoseArr)│
  │  yellow path   │ ◀──poll │  writes            │    │                                  │
  │  appears       │ /path…  │  path_plan.json    │    │  → APF planner (10 Hz)           │
  └────────────────┘         └────────────────────┘    │  → velocity ramp                 │
                                                       │  → /virtual_center/cmd_vel       │
                                                       │     (Twist)                      │
                                                       └────────────┬─────────────────────┘
                                                                    │
                                                       ┌────────────▼─────────────────────┐
                                                       │  laplacian_formation_node (each  │
                                                       │  robot runs its own copy)        │
                                                       │                                  │
                                                       │  reads /virtual_center/cmd_vel   │
                                                       │  reads own /ekf/odom             │
                                                       │  computes own offset correction  │
                                                       │  publishes /{robot_id}/cmd_vel   │
                                                       └────────────┬─────────────────────┘
                                                                    │
                                                       ┌────────────▼─────────────────────┐
                                                       │  Swerve drive module (firmware)  │
                                                       │  applies wheel velocities        │
                                                       └──────────────────────────────────┘
```

The five layers, top-to-bottom, are:

1. **Goal pose acquisition** — where the goal comes from
2. **Grid-based global planning** — A\* on the inflated occupancy grid
3. **Local control + velocity shaping** — APF + velocity ramp + slow-on-approach
4. **Formation keeping** — virtual center + Laplacian consensus
5. **Wheel-level execution** — swerve module response

---

## Layer 1 — Goal pose acquisition

Two paths feed the navigation node a goal:

| Source | Topic | Trigger |
|---|---|---|
| Web UI "Send Goal" button | `/goal_pose` (PoseStamped, Nav2 standard) | User clicks goal in 3D view, presses "Send Goal" |
| Web UI "Plan Path" button | `/path_plan.json` (HTTP, polled by UI) | User clicks Plan; result is a precomputed waypoint list |
| Direct CLI / external | `/navigation/goal` (Twist, navigation_node native) | Programmatic |

Internally `navigation_node.py` accepts:

```python
self.create_subscription(Twist,     '/navigation/goal',      self._goal_cb,      10)
self.create_subscription(PoseArray, '/navigation/waypoints', self._waypoints_cb, 10)
```

The Twist convention used (line 87, navigation_node.py):

```
linear.x   = goal x in map frame
linear.y   = goal y in map frame
angular.z  = goal yaw (rad)
```

When a goal arrives, the leader's `_goal_cb` clears any pending waypoints
and stores the new goal as `self._current_wp`.

---

## Layer 2 — Grid-based global planning (A\*)

Triggered when the user presses **Plan Path** in the web UI. Server-side flow:

1. `POST /plan` arrives at `server.py` with `{start: {x, y}, goal: {x, y}}`
2. Server calls `astar_planner.compute_plan(map, start, goal, formation_radius)`
3. `compute_plan()` does:
   1. **Inflate** the occupancy grid by `formation_radius` (so the virtual
      center can never be placed where any robot in the formation would
      collide)
   2. **A\*** with 8-connected Euclidean heuristic on the inflated grid
   3. **Simplify** with line-of-sight (Bresenham) — drops collinear waypoints
   4. **Per-robot tracks**: each robot's expected path = VC waypoints +
      its `offset_local` rotated by the path heading
4. Server writes `path_plan.json` with `seq` counter
5. Browser polls `path_plan.json`, renders yellow line + envelope rings on the floor

Key A\* properties (`astar_planner.py`):

- **Heuristic**: `math.hypot(c - gc, r - gr)` — Euclidean distance, admissible & consistent
- **Cost per move**: 1.0 for cardinal, √2 for diagonal
- **Corner-cutting prevention**: rejects diagonals that squeeze through obstacle pairs
- **Inflation**: `ndimage.binary_dilation(obstacles, iterations=ceil(radius/cell))`

The output of A\* is fed to the navigation node as a `PoseArray` on
`/navigation/waypoints`. Each pose is a (x, y, yaw) target; the leader
visits them in order.

---

## Layer 3 — Local control + velocity shaping

This is where your friend's APF + velocity ramp lives, in
`navigation_node.py:_apf_velocity()` (lines 195–248) and
`_apply_ramp()` (lines 257–276).

### 3a — Attractive force (toward current waypoint)

```python
to_goal = goal - position
d_goal  = ||to_goal||
f_att   = K_ATT * to_goal / d_goal       # unit vector × K_ATT
```

This is a **conic** attraction (constant magnitude regardless of distance),
which prevents the robot from lunging at far-away goals.

### 3b — Repulsive force (away from each obstacle)

```python
for each (cx, cy, r) in obstacles:
    r_eff = r + SAFETY                   # inflate by formation half-width
    d     = (distance from robot to obstacle surface)
    if d < D_REP:
        f_rep += K_REP * (1/d - 1/D_REP) / d² * (away direction)
```

Khatib's classical formulation. Force grows as 1/d² near the obstacle and
goes to zero at `D_REP = 1.2 m`. `SAFETY = formation half-width` ensures
the **whole formation** clears, not just the virtual center.

### 3c — Distance-based slowdown ⭐ (this is the bit you asked about)

```python
speed = min(MAX_LINEAR, f_mag)           # cap at robot top speed (0.18 m/s)
if d_goal < SLOW_RADIUS:                 # within 40 cm of waypoint
    speed *= d_goal / SLOW_RADIUS        # linearly scale to zero
v_des = (speed / f_mag) * f_total
```

The closer the formation gets to its waypoint, the slower it moves. At
`d_goal = 0.40 m` it's at full speed; at `d_goal = 0.20 m` it's at half
speed; at `d_goal = 0` it's stopped. **This is what keeps the robot on
track** — no overshoot, no oscillation around the waypoint.

### 3d — Velocity ramp (anti-jerk)

```python
dv     = v_des - v_actual
max_dv = ACC_MAX * DT                    # 0.15 m/s² × 0.05 s = 7.5 mm
if ||dv|| > max_dv:
    v_actual += (max_dv / ||dv||) * dv   # take only the allowed step
else:
    v_actual = v_des
```

Same idea on yaw: `omega_actual` only changes by `±ALPHA_MAX·DT` per tick.
This filter sits **between** APF and the publish, so even if APF demands
a sudden velocity flip, the wheels never get told to flip suddenly. Saves
the gear teeth and keeps the formation tight.

### 3e — Heading control

```python
desired_heading = atan2(v_des.y, v_des.x)    # face direction of motion
omega_des       = K_HEADING * wrap(desired_heading - current_heading)
```

Proportional yaw controller. Drives the formation's facing direction to
match its velocity vector. Near the goal (when `||v_des|| < 5 cm/s`),
switches to the goal's specified yaw instead.

### 3f — Output

```python
cmd.linear.x  = vx_body            # forward in the formation's own frame
cmd.linear.y  = vy_body            # left in the formation's own frame
cmd.angular.z = omega_actual
self._cmd_pub.publish(cmd)         # → /virtual_center/cmd_vel
```

---

## Layer 4 — Formation keeping (Laplacian consensus)

The leader's `navigation_node` only cares about the **virtual center**.
Each individual robot runs its own copy of `laplacian_formation_node.py`
that:

1. Subscribes to `/virtual_center/cmd_vel` (the leader's published command)
2. Subscribes to its own `/ekf/odom` (where it actually is)
3. Knows its **assigned offset** from the virtual center (e.g. leader =
   +25 cm forward, follower = −25 cm)
4. Runs a **Laplacian consensus** correction: each tick, compute the
   delta between current-offset and desired-offset, generate a velocity
   correction that closes the gap, add it to the virtual-center cmd_vel
5. Publish to its own `/{robot_id}/cmd_vel`

This is the "ghostlab follower car" pattern. The leader follows the path,
the follower stays at its offset, the formation holds shape because each
robot independently corrects toward the virtual center.

The math (simplified, single-axis):

```
own_offset_world  = own_pose - virtual_center_pose      (current)
want_offset_world = R(yaw_VC) * own_offset_local        (desired, rotated to world)
correction        = K_FORMATION * (want_offset_world - own_offset_world)
cmd_robot         = cmd_VC + correction
```

`K_FORMATION` is the consensus gain. Higher = tighter formation, more
oscillation. Lower = loose formation, smoother but laggy.

The "Laplacian" name comes from the fact that this is a special case of
graph-Laplacian consensus dynamics — the robots converge on a fixed
relative configuration around the virtual center.

---

## Layer 5 — Obstacle avoidance (cross-cutting)

Two kinds of obstacles enter the system, both via `/navigation/obstacles`
(PoseArray, line 33 of navigation_node.py):

| Source | What it represents |
|---|---|
| `map_to_obstacles_node` (extracts blobs from `map.json` once at startup) | Static map obstacles (walls, tire stack, the static panel) |
| Some perception node (future, not yet wired) | Dynamic obstacles (other robots in the swarm, people, moving objects) |

Each obstacle is published as `(pose.position.x, pose.position.y,
pose.position.z=radius)`. The APF in Layer 3b automatically generates a
repulsive force away from each one, with `SAFETY` inflation matching the
formation's footprint.

For static obstacles, A\* in Layer 2 already accounted for them
(inflation makes the path avoid them at the global scale). The APF
repulsion in Layer 3 is then a **safety net** for things that change
between plan and execute (like a robot drifting slightly off-path).

---

## Putting it all together — one tick of the leader's control loop

```
1. _control_loop() fires (20 Hz)
   ├─ if not leader → return
   ├─ if no waypoint → return
   │
2. Check goal-tolerance (5 cm)
   ├─ if reached → pop next waypoint or finish
   │
3. APF compute_velocity()
   ├─ f_att = K_ATT × unit(goal − pos)            ← attractive
   ├─ f_rep = Σ K_REP × decay(d) × n̂              ← repulsive (each obstacle)
   ├─ speed = clamp(||f_total||, 0, MAX_LINEAR)
   ├─ if d_goal < SLOW_RADIUS:                    ← *distance-based slowdown*
   │     speed *= d_goal / SLOW_RADIUS            ←   keeps the formation
   ├─ v_des = (speed / ||f_total||) × f_total      ←   on track
   ├─ omega_des = K_HEADING × yaw_error
   │
4. Velocity ramp
   ├─ v_actual += clamp(v_des − v_actual, ±ACC_MAX·DT)
   ├─ omega_actual += clamp(omega_des − omega_actual, ±ALPHA_MAX·DT)
   │
5. World → body frame transform
   ├─ vx_body =  v_actual.x cos(θ) + v_actual.y sin(θ)
   ├─ vy_body = −v_actual.x sin(θ) + v_actual.y cos(θ)
   │
6. Publish
   ├─ /virtual_center/cmd_vel (Twist)             ← read by every robot
   ├─ /navigation/status      (String)            ← UI / monitoring
```

Each follower does (also at 20 Hz):

```
1. Read /virtual_center/cmd_vel
2. Read own /ekf/odom
3. Look up assigned offset from VC
4. Compute consensus correction
5. Publish /{robot_id}/cmd_vel  ← swerve drive
```

---

## Tuning cheat sheet

| What you'd change | Where | Effect |
|---|---|---|
| Make formation tighter | `K_FORMATION` ↑ in laplacian_formation_node | Robots correct faster, may oscillate |
| Slower deceleration on approach | `SLOW_RADIUS` ↑ in navigation_node | Smoother stop, longer braking distance |
| Avoid obstacles harder | `K_REP` ↑ or `D_REP` ↑ | Wider berth around obstacles |
| Smoother turns | `ALPHA_MAX` ↓ | Less yaw jerk, slower direction changes |
| Higher top speed | `MAX_LINEAR` ↑ | Watch motor current limits |
| Plan paths more conservatively | `formation_radius` ↑ in server.py call | Inflated grid eats more space; avoids tight gaps |

---

## Quick references

- Path planner: `astar_planner.py:compute_plan()`
- Local control: `navigation_node.py:_apf_velocity()` and `_apply_ramp()`
- Formation keeping: `laplacian_formation_node.py`
- Obstacle source: `map_to_obstacles.py` extracts from `map.json`
- Web UI: `index.html` (3D view + click-to-goal + Plan Path)
- HTTP backend: `server.py` (`/plan`, `/goal`, `/pose`, `/set_initial_pose`)
- Live pose: `ros_pose_bridge.py` (TF lookup → POST /pose)

---

## Why the velocity stays on track — short answer

Three layers stack to keep the robot on its planned path:

1. **A\*** gives a globally optimal path that respects obstacles + formation footprint
2. **APF attraction** at each waypoint pulls the formation toward it
3. **Distance-based slowdown** (`speed *= d_goal / SLOW_RADIUS` when within 40 cm)
   ensures the formation **decelerates** rather than overshooting

If the formation drifts off the planned path (e.g., a follower lags), the
**Laplacian consensus** in Layer 4 corrects each robot back toward its
assigned offset around the virtual center. The virtual center itself
stays on the A\*-planned path because Layer 3 keeps it there.

The combination is robust: A\* handles "where," APF + slowdown handle
"how fast," and Laplacian consensus handles "stay in formation while
doing both."
