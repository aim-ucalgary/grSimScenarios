"""Assertions against measured state.

Raise AssertionError when a check fails.
Missing or stale observations must be rejected before comparison.
"""

import math
from .helpers_geometry import distance_to, angle_error


def assert_near_position(actual, expected, *, tolerance_m):
    """Check distance between two (x, y) positions.

    Inputs:
        actual (tuple[float, float]): Observed (x, y) position in metres.
        expected (tuple[float, float]): Target (x, y) position in metres.
        tolerance_m (float): Maximum acceptable Euclidean distance in metres.

    Outputs:
        None

    Raises:
        AssertionError: If observation is missing or distance exceeds tolerance_m.

    Example:
        >>> assert_near_position((1.01, 2.01), (1.00, 2.00), tolerance_m=0.03)
    """
    if actual is None or expected is None:
        raise AssertionError("Cannot compare position: observation is missing")
    dist = distance_to(actual, expected)
    if dist > tolerance_m:
        raise AssertionError(
            f"Position error {dist:.4f}m exceeds tolerance {tolerance_m:.4f}m "
            f"(actual={actual}, expected={expected})"
        )


def assert_heading(actual_rad, expected_rad, *, tolerance_rad):
    """Check heading using the shortest angular difference.

    Inputs:
        actual_rad (float): Observed robot heading in radians.
        expected_rad (float): Expected target heading in radians.
        tolerance_rad (float): Maximum acceptable absolute angular difference in radians.

    Outputs:
        None

    Raises:
        AssertionError: If observation is missing or angle difference exceeds tolerance_rad.

    Example:
        >>> assert_heading(3.14, -3.14, tolerance_rad=0.05)
    """
    if actual_rad is None or expected_rad is None:
        raise AssertionError("Cannot compare heading: observation is missing")
    err = abs(angle_error(actual_rad, expected_rad))
    if err > tolerance_rad:
        raise AssertionError(
            f"Heading error {err:.4f}rad exceeds tolerance {tolerance_rad:.4f}rad "
            f"(actual={actual_rad}, expected={expected_rad})"
        )


def assert_stopped(
    velocity_samples, *,
    linear_tolerance_mps, angular_tolerance_radps, duration_s,
):
    """Check timestamped velocities cover a stopped interval.

    Inputs:
        velocity_samples (list[tuple]): Sequence of (timestamp_s, (vx, vy, omega)) samples.
        linear_tolerance_mps (float): Maximum allowed linear speed in m/s.
        angular_tolerance_radps (float): Maximum allowed rotational speed in rad/s.
        duration_s (float): Minimum duration in seconds that the samples must span.

    Outputs:
        None

    Raises:
        AssertionError: If samples do not span duration_s or any speed exceeds tolerance.

    Example:
        >>> samples = [(0.0, (0.01, 0.0, 0.0)), (0.3, (0.0, 0.0, 0.0))]
        >>> assert_stopped(samples, linear_tolerance_mps=0.05, angular_tolerance_radps=0.05, duration_s=0.25)
    """
    if not velocity_samples:
        raise AssertionError("No velocity samples provided")

    timestamps = [s[0] for s in velocity_samples]
    total_duration = max(timestamps) - min(timestamps)
    if total_duration < duration_s:
        raise AssertionError(
            f"Samples cover {total_duration:.3f}s, which is less than required {duration_s:.3f}s"
        )

    for sample in velocity_samples:
        t = sample[0]
        v = sample[1:] if len(sample) > 2 else sample[1]
        vx = v[0]
        vy = v[1]
        omega = v[2] if len(v) > 2 else 0.0

        linear_speed = math.hypot(vx, vy)
        if linear_speed > linear_tolerance_mps:
            raise AssertionError(
                f"Linear speed {linear_speed:.4f}m/s exceeds tolerance {linear_tolerance_mps:.4f}m/s at t={t:.3f}"
            )
        if abs(omega) > angular_tolerance_radps:
            raise AssertionError(
                f"Angular speed {abs(omega):.4f}rad/s exceeds tolerance {angular_tolerance_radps:.4f}rad/s at t={t:.3f}"
            )


def assert_ball_displacement(
    initial_position, final_position, *,
    min_distance_m, max_distance_m=None,
):
    """Check net ball displacement against supplied bounds.

    Inputs:
        initial_position (tuple[float, float]): (x, y) start point of ball in metres.
        final_position (tuple[float, float]): (x, y) end point of ball in metres.
        min_distance_m (float): Minimum required travel distance in metres.
        max_distance_m (float, optional): Maximum allowed travel distance in metres.

    Outputs:
        None

    Raises:
        AssertionError: If ball position is missing or displacement is out of bounds.

    Example:
        >>> assert_ball_displacement((0.0, 0.0), (1.5, 0.0), min_distance_m=1.0, max_distance_m=2.0)
    """
    if initial_position is None or final_position is None:
        raise AssertionError("Ball position observation is missing")
    dist = distance_to(initial_position, final_position)
    if dist < min_distance_m:
        raise AssertionError(
            f"Ball displacement {dist:.4f}m is below min {min_distance_m:.4f}m"
        )
    if max_distance_m is not None and dist > max_distance_m:
        raise AssertionError(
            f"Ball displacement {dist:.4f}m exceeds max {max_distance_m:.4f}m"
        )
