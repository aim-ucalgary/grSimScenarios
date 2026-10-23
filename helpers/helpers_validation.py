"""Assertions against measured state.

Raise AssertionError when a check fails.
Missing or stale observations must be rejected before comparison.
"""


def assert_near_position(actual, expected, *, tolerance_m):
    """Check distance between two (x, y) positions."""
    raise NotImplementedError


def assert_heading(actual_rad, expected_rad, *, tolerance_rad):
    """Check heading using the shortest angular difference."""
    raise NotImplementedError


def assert_stopped(
    velocity_samples, *,
    linear_tolerance_mps, angular_tolerance_radps, duration_s,
):
    """Check timestamped velocities cover a stopped interval."""
    raise NotImplementedError


def assert_ball_displacement(
    initial_position, final_position, *,
    min_distance_m, max_distance_m=None,
):
    """Check net ball displacement against supplied bounds."""
    raise NotImplementedError