"""
test_obstacle_logic.py
----------------------
Pure-logic tests for obstacle_avoidance_node's two helper functions
(`find_closest_in_swathe` and `compute_avoidance`). No ROS context, no
camera, no robot — runs anywhere that has numpy + the package source
on the PYTHONPATH.

Run on the laptop before deploying:

    cd <repo_root>
    PYTHONPATH=ros2_ws/src/swerve_formation python -m pytest tools/test_obstacle_logic.py -v

Or invoke directly:

    PYTHONPATH=ros2_ws/src/swerve_formation python tools/test_obstacle_logic.py

The point of these tests: prove the avoidance math handles the
scenarios we care about (clear, obstacle on left, obstacle on right,
obstacle dead-centre, no valid pixels) BEFORE we ever try this on real
hardware. If any of these fail, the demo will be wrong on the floor.
"""

from __future__ import annotations

import sys

import numpy as np

# Allow running under colcon test (cwd = package dir) or directly from
# repo root (cwd = repo root, PYTHONPATH set above).
try:
    from swerve_formation.obstacle_avoidance_lib import (
        AvoidanceParams,
        compute_avoidance,
        find_closest_in_swathe,
    )
except ImportError:  # pragma: no cover — running from repo root w/o PYTHONPATH
    import os
    sys.path.insert(0, os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        '..', 'ros2_ws', 'src', 'swerve_formation',
    ))
    from swerve_formation.obstacle_avoidance_lib import (    # noqa: E402
        AvoidanceParams,
        compute_avoidance,
        find_closest_in_swathe,
    )


# ────────────────────── helpers ───────────────────────────────────────────

H, W = 400, 640        # match OAK-D Lite stereo native size
DEFAULT_PARAMS = AvoidanceParams()


def _empty_depth_mm() -> np.ndarray:
    """All zeros = all invalid (depthai_ros_driver writes 0 for invalid)."""
    return np.zeros((H, W), dtype=np.uint16)


def _far_wall_mm(distance_mm: int = 3000) -> np.ndarray:
    """Uniform far wall — should always read clear."""
    return np.full((H, W), distance_mm, dtype=np.uint16)


def _wall_with_box(box_distance_mm: int,
                   col_lo: int,
                   col_hi: int,
                   wall_distance_mm: int = 3500) -> np.ndarray:
    """A far wall plus a closer 'box' filling columns [col_lo, col_hi)."""
    arr = np.full((H, W), wall_distance_mm, dtype=np.uint16)
    arr[:, col_lo:col_hi] = box_distance_mm
    return arr


# ────────────────────── find_closest_in_swathe ────────────────────────────

def test_clear_far_wall_reports_wall_distance():
    """Wall at 3 m, swathe reports the wall, no avoidance needed."""
    arr = _far_wall_mm(3000)
    closest, _col_u = find_closest_in_swathe(arr, DEFAULT_PARAMS)
    assert closest == 3000
    # col_u is undefined for a uniform field (argmin lands on the first
    # pixel by tiebreak); we only assert the distance here.


def test_no_valid_depth_returns_none():
    """All-zero depth (all invalid) → None, 0.0."""
    arr = _empty_depth_mm()
    closest, col_u = find_closest_in_swathe(arr, DEFAULT_PARAMS)
    assert closest is None
    assert col_u == 0.0


def test_box_on_left_negative_col_u():
    """Box in left third of image → col_u should be < 0."""
    arr = _wall_with_box(box_distance_mm=900, col_lo=80, col_hi=160)
    closest, col_u = find_closest_in_swathe(arr, DEFAULT_PARAMS)
    assert closest == 900
    assert col_u < -0.3


def test_box_on_right_positive_col_u():
    """Box in right third of image → col_u should be > 0."""
    arr = _wall_with_box(box_distance_mm=900, col_lo=480, col_hi=560)
    closest, col_u = find_closest_in_swathe(arr, DEFAULT_PARAMS)
    assert closest == 900
    assert col_u > 0.3


def test_box_too_close_clipped_by_min_valid():
    """Object closer than min_valid_mm is masked out, far wall wins."""
    arr = _wall_with_box(box_distance_mm=100, col_lo=200, col_hi=400,
                         wall_distance_mm=2500)
    closest, _ = find_closest_in_swathe(arr, DEFAULT_PARAMS)
    assert closest == 2500   # the 100 mm box was below min_valid_mm=250


def test_floor_outside_swathe_ignored():
    """Floor at the bottom of the image is below swathe_bot_frac → ignored."""
    arr = _far_wall_mm(3000)
    # Put a 'floor' at 500 mm in the bottom 30% of the image — outside
    # the default 0.40–0.65 swathe band.
    arr[int(0.7 * H):, :] = 500
    closest, _ = find_closest_in_swathe(arr, DEFAULT_PARAMS)
    assert closest == 3000


# ────────────────────── compute_avoidance ─────────────────────────────────

def test_no_obstacle_passes_through():
    """closest_mm = None → raw twist passes through unchanged."""
    out_x, out_y, out_wz, state = compute_avoidance(
        None, 0.0, 0.10, 0.0, 0.0, DEFAULT_PARAMS
    )
    assert out_x == 0.10
    assert out_y == 0.0
    assert out_wz == 0.0
    assert state == 'no-obstacle'


def test_far_obstacle_passes_through():
    """closest_mm beyond avoid_range_mm → raw twist passes through."""
    out_x, out_y, out_wz, state = compute_avoidance(
        2500, 0.5, 0.10, 0.0, 0.0, DEFAULT_PARAMS
    )
    assert out_x == 0.10
    assert out_y == 0.0
    assert state.startswith('clear')


def test_obstacle_on_right_pushes_left():
    """Obstacle right of centre (col_u > 0) → push +y (body left)."""
    out_x, out_y, out_wz, state = compute_avoidance(
        600, +0.5, 0.10, 0.0, 0.0, DEFAULT_PARAMS
    )
    assert out_y > 0.0
    assert out_x < 0.10        # speed attenuated when close
    assert state.startswith('AVOID')


def test_obstacle_on_left_pushes_right():
    """Obstacle left of centre (col_u < 0) → push -y (body right)."""
    out_x, out_y, out_wz, state = compute_avoidance(
        600, -0.5, 0.10, 0.0, 0.0, DEFAULT_PARAMS
    )
    assert out_y < 0.0
    assert out_x < 0.10


def test_avoidance_urgency_scales_lateral():
    """Closer obstacles → larger |out_y|, lower out_x."""
    out_x_far,  out_y_far,  *_ = compute_avoidance(
        1100, 0.5, 0.10, 0.0, 0.0, DEFAULT_PARAMS
    )
    out_x_near, out_y_near, *_ = compute_avoidance(
        300, 0.5, 0.10, 0.0, 0.0, DEFAULT_PARAMS
    )
    assert abs(out_y_near) > abs(out_y_far)
    assert out_x_near < out_x_far


def test_lateral_gain_capped_at_unity_urgency():
    """Even at urgency=1 (closest_mm = 0), |v_y_add| ≤ lateral_gain."""
    out_x, out_y, out_wz, state = compute_avoidance(
        0, 0.5, 0.10, 0.0, 0.0, DEFAULT_PARAMS
    )
    # Equality is the spec: urgency=1 → v_y_add = lateral_gain.
    assert abs(out_y) <= DEFAULT_PARAMS.lateral_gain + 1e-9


def test_speed_scale_floor_respected():
    """At urgency=1, out_x = raw_x * speed_scale_floor."""
    out_x, _, _, _ = compute_avoidance(
        0, 0.5, 0.10, 0.0, 0.0, DEFAULT_PARAMS
    )
    expected = 0.10 * DEFAULT_PARAMS.speed_scale_floor
    assert abs(out_x - expected) < 1e-9


def test_angular_z_attenuated_with_speed():
    """When avoiding, raw angular.z is scaled by the same factor as linear.x."""
    raw_wz = 0.20
    _, _, out_wz, _ = compute_avoidance(
        0, 0.5, 0.10, 0.0, raw_wz, DEFAULT_PARAMS
    )
    assert abs(out_wz - raw_wz * DEFAULT_PARAMS.speed_scale_floor) < 1e-9


# ────────────────────── direct invocation ─────────────────────────────────

if __name__ == '__main__':
    # Run all tests in this file even without pytest installed.
    import inspect
    funcs = [f for name, f in inspect.getmembers(sys.modules[__name__],
             inspect.isfunction) if name.startswith('test_')]
    failed = 0
    for fn in funcs:
        try:
            fn()
            print(f'PASS  {fn.__name__}')
        except AssertionError as e:
            failed += 1
            print(f'FAIL  {fn.__name__}: {e}')
    if failed:
        print(f'\n{failed}/{len(funcs)} tests failed.')
        sys.exit(1)
    print(f'\nAll {len(funcs)} tests passed.')
