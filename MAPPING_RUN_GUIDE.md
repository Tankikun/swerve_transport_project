# Mapping Run Guide (Step 6 of the RTAB-Map plan)

The mapping run is split across two machines:

- **Pi (sensors)** — runs the camera, TF, wheel odometry, EKF
- **Laptop (SLAM)** — runs rtabmap_slam in mapping mode

Each side has its own self-contained guide. Read the one for
whichever side you're sitting at.

| Machine | What runs there | Guide |
|---|---|---|
| pi2 | sensors only (camera + TF + base + EKF) | **[`MAPPING_RUN_PI.md`](./MAPPING_RUN_PI.md)** |
| laptop | rtabmap + teleop + monitoring | **[`MAPPING_RUN_LAPTOP.md`](./MAPPING_RUN_LAPTOP.md)** |

---

## Workflow at a glance

1. Operator on pi2 follows `MAPPING_RUN_PI.md` and starts the sensor
   launch. Leaves it running.
2. Operator on the laptop follows `MAPPING_RUN_LAPTOP.md`:
   - Pre-flight checks that pi2's topics are visible
   - Starts rtabmap on the laptop
   - Starts teleop on the laptop
   - Drives the robot through the room
   - Ctrl+C the rtabmap launch when done — `.db` is saved on the
     laptop at `~/maps/tb3_1_room.db`
3. Pi-side operator Ctrl+Cs the sensor launch.

If both operators are the same person, just keep three terminals
open and follow both files in parallel.

---

## Why split

rtabmap_slam under sustained mapping pushes the Pi 4 to 70-80 °C
and risks thermal throttling. Splitting — Pi streams sensors only,
laptop runs the SLAM — keeps the Pi at ~60 °C and gives rtabmap
~10× the spare CPU. The only cost is bandwidth (5–10 MB/s of image
data over LAN), which is well within budget.

An **ALL-ON-PI fallback** is documented at the bottom of
`MAPPING_RUN_PI.md` for cases where the network can't keep up
(camera rate < 1 Hz on the laptop side).

---

## After mapping — localization

Same split-execution pattern, with different launches. See the
"After mapping — switch to localization" section in
`MAPPING_RUN_LAPTOP.md`. Both robots should run their own copy of
the localization stack against their own (or a shared) `.db`.
