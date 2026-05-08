"""
obstacle_avoidance_lib.py
-------------------------
Pure-numpy helper functions for `obstacle_avoidance_node`. Lives in its
own module — and has zero ROS imports — so the math can be unit-tested
on any Python with numpy installed (laptop, CI, anywhere).

The Node in `obstacle_avoidance_node.py` imports from here. Anything
ROS-typed (sensor_msgs/Image conversion, etc.) lives in the Node file,
not here.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


@dataclass
class AvoidanceParams:
    """Tuning parameters; mirrors the Node's ros params 1:1."""
    swathe_top_frac:   float = 0.40
    swathe_bot_frac:   float = 0.65
    min_valid_mm:      int   = 250
    max_valid_mm:      int   = 4000
    avoid_range_mm:    int   = 1200
    lateral_gain:      float = 0.10
    speed_scale_floor: float = 0.40


def find_closest_in_swathe(depth_mm: np.ndarray,
                           p: AvoidanceParams):
    """
    Return (closest_mm, col_u) for the closest valid pixel inside the
    forward swathe, or (None, 0.0) if no pixel is valid.

    `depth_mm` is a 2-D uint16 array of millimetres (zeros = invalid).
    `col_u` is the normalised column of the closest pixel, in [-1, +1]
    where -1 = far-left of image, +1 = far-right.
    """
    h, w = depth_mm.shape
    if h < 4 or w < 4:
        return None, 0.0

    top = int(p.swathe_top_frac * h)
    bot = int(p.swathe_bot_frac * h)
    if bot <= top + 1:
        return None, 0.0

    swathe = depth_mm[top:bot, :]
    valid = (swathe >= p.min_valid_mm) & (swathe <= p.max_valid_mm)
    if not valid.any():
        return None, 0.0

    sentinel = np.uint16(min(p.max_valid_mm + 1, 65535))
    masked = np.where(valid, swathe, sentinel)
    flat_idx = int(masked.argmin())
    sw_w = swathe.shape[1]
    col = flat_idx % sw_w
    depth = int(masked.flat[flat_idx])

    col_u = (col - (sw_w / 2.0)) / max(sw_w / 2.0, 1.0)
    return depth, float(col_u)


def compute_avoidance(closest_mm,
                      col_u: float,
                      raw_x: float,
                      raw_y: float,
                      raw_wz: float,
                      p: AvoidanceParams):
    """
    Return (out_x, out_y, out_wz, state_str). Pure function.

    `closest_mm` may be None (depth-stale or no-valid-depth) — the raw
    twist is then passed through unchanged with state='no-obstacle'.
    """
    if closest_mm is None:
        return raw_x, raw_y, raw_wz, 'no-obstacle'

    if closest_mm >= p.avoid_range_mm:
        return raw_x, raw_y, raw_wz, f'clear  closest={closest_mm/1000.0:.2f}m'

    urgency = 1.0 - (float(closest_mm) / float(p.avoid_range_mm))
    urgency = max(0.0, min(1.0, urgency))

    # Coordinate sign: col_u > 0 means obstacle is on the right side of
    # the image — which, for a forward-facing camera with the standard
    # ROS optical-from-body rotation in oak_camera.launch.py, is the
    # robot's right (body -y). To push AWAY we want body +y (left), so
    # the sign of v_y_add MUST match the sign of col_u.
    push_dir = math.copysign(1.0, col_u) if abs(col_u) > 1e-3 else 1.0
    v_y_add = p.lateral_gain * push_dir * urgency

    scale = 1.0 - (1.0 - p.speed_scale_floor) * urgency
    out_x  = raw_x  * scale
    out_y  = raw_y  + v_y_add
    out_wz = raw_wz * scale
    state = (
        f'AVOID  closest={closest_mm/1000.0:.2f}m  '
        f'col_u={col_u:+.2f}  push_y={v_y_add:+.2f}  scale={scale:.2f}'
    )
    return out_x, out_y, out_wz, state
