# RPi Utility Scripts

Deploy to the Raspberry Pi home directory (`~/`) and add aliases to `~/.bashrc`.

## `odom_watch.py` — Live odometry display

Real-time terminal display: position x/y, heading (°), velocity bars, compass.

```bash
# Add to ~/.bashrc
alias odom="source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash && python3 ~/odom_watch.py"

# Run
odom             # watches /robot1/odom
odom /tb3_0      # watches /tb3_0/odom
```

## `teleop_twist_keyboard.py` — Standard keyboard teleop

Faithful re-implementation of the ROS2 `teleop_twist_keyboard` package.  
Default speed: **0.10 m/s** (safe for 3D-printed steering joints).  
Max effective speed: **~0.20 m/s** (XL430 motor physical limit).

```bash
# Add to ~/.bashrc
alias teleop2="source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash && python3 ~/teleop_twist_keyboard.py --ros-args -r __ns:=/robot1"

# Run (bringup must be running first)
teleop2
```

### Key layout
```
Normal:          Holonomic strafing (Shift):
  u  i  o          U  I  O
  j  k  l          J  K  L       J = strafe left
  m  ,  .          M  <  >       L = strafe right

k / Space = stop     w/x = linear ±10%     e/c = angular ±10%
```
