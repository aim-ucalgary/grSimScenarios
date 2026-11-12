"""Pure geometry calculations; no networking.

Positions: (x, y) in metres. Angles: radians.
"""

import math


def distance_to(position, target):
    """Return Euclidean distance in metres.

    Inputs:
        position (tuple[float, float]): Starting (x, y) coordinates in metres.
        target (tuple[float, float]): Target (x, y) coordinates in metres.

    Outputs:
        float: Non-negative Euclidean distance in metres.

    Example:
        >>> distance_to((0.0, 0.0), (3.0, 4.0))
        5.0
    """
    return math.hypot(float(target[0]) - float(position[0]), float(target[1]) - float(position[1]))


def bearing_to(position, target):
    """Return the field-relative heading toward a target.

    Inputs:
        position (tuple[float, float]): Starting (x, y) coordinates in metres.
        target (tuple[float, float]): Target (x, y) coordinates in metres.

    Outputs:
        float: Angle in radians in [-pi, pi], counter-clockwise from field +X.

    Example:
        >>> bearing_to((0.0, 0.0), (1.0, 0.0))
        0.0
        >>> bearing_to((0.0, 0.0), (0.0, 1.0))
        1.5707963267948966  # pi / 2
    """
    return math.atan2(float(target[1]) - float(position[1]), float(target[0]) - float(position[0]))


def angle_error(target_rad, current_rad):
    """Return the shortest signed angular difference.

    Inputs:
        target_rad (float): Target orientation in radians.
        current_rad (float): Current orientation in radians.

    Outputs:
        float: Shortest signed difference (target - current) wrapped to [-pi, pi].
               Positive indicates a counter-clockwise rotation; negative indicates clockwise.

    Example:
        >>> angle_error(math.pi, -math.pi)
        0.0
        >>> round(angle_error(0.1, 0.0), 2)
        0.1
    """
    diff = float(target_rad) - float(current_rad)
    return math.atan2(math.sin(diff), math.cos(diff))


def global_to_local(vx_mps, vy_mps, heading_rad):
    """Return robot-relative (forward, left) velocity.

    Inputs:
        vx_mps (float): Field-relative velocity along +X in m/s.
        vy_mps (float): Field-relative velocity along +Y in m/s.
        heading_rad (float): Robot orientation in radians counter-clockwise from +X.

    Outputs:
        tuple[float, float]: (forward_mps, left_mps) in robot coordinates.

    Example:
        # A robot moving along field +X while facing field +Y (pi / 2 rad)
        # is sliding to its right flank (left velocity = -1.0 m/s):
        >>> global_to_local(1.0, 0.0, math.pi / 2)
        (0.0, -1.0)
    """
    vx, vy, h = float(vx_mps), float(vy_mps), float(heading_rad)
    forward = vx * math.cos(h) + vy * math.sin(h)
    left = -vx * math.sin(h) + vy * math.cos(h)
    return (forward, left)


def local_to_global(forward_mps, left_mps, heading_rad):
    """Return field-relative (vx, vy) velocity.

    Inputs:
        forward_mps (float): Velocity along robot front face in m/s.
        left_mps (float): Velocity along robot left flank in m/s.
        heading_rad (float): Robot orientation in radians counter-clockwise from +X.

    Outputs:
        tuple[float, float]: (vx_mps, vy_mps) in field coordinates.

    Example:
        # A robot driving forward at 1.0 m/s while facing field +Y (pi / 2 rad)
        # moves along field +Y:
        >>> local_to_global(1.0, 0.0, math.pi / 2)
        (0.0, 1.0)
    """
    f, l, h = float(forward_mps), float(left_mps), float(heading_rad)
    vx = f * math.cos(h) - l * math.sin(h)
    vy = f * math.sin(h) + l * math.cos(h)
    return (vx, vy)
