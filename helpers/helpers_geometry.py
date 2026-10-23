"""Pure geometry calculations; no networking.

Positions: (x, y) in metres. Angles: radians.
"""


def distance_to(position, target):
    """Return Euclidean distance in metres."""
    raise NotImplementedError


def bearing_to(position, target):
    """Return the field-relative heading toward a target."""
    raise NotImplementedError


def angle_error(target_rad, current_rad):
    """Return the shortest signed angular difference."""
    raise NotImplementedError


def global_to_local(vx_mps, vy_mps, heading_rad):
    """Return robot-relative (forward, left) velocity."""
    raise NotImplementedError


def local_to_global(forward_mps, left_mps, heading_rad):
    """Return field-relative (vx, vy) velocity."""
    raise NotImplementedError