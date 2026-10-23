"""Vision observations and freshness checks.

Normalize positions to metres and headings to radians.
Never substitute a zero pose for missing observations.
"""


def read_vision(sock, *, timeout_s=1.0):
    """Receive and decode a vision packet from a configured socket."""
    raise NotImplementedError


def get_robot_pose(world_state, team, robot_id, *, max_age_s=0.5):
    """Return a fresh robot pose or raise if unavailable."""
    raise NotImplementedError


def get_ball_state(world_state, *, max_age_s=0.5):
    """Return a fresh ball estimate or raise if unavailable."""
    raise NotImplementedError


def require_fresh_state(last_received_monotonic_s, *, max_age_s=0.5):
    """Raise if the observation is missing or too old."""
    raise NotImplementedError


def estimate_velocity(previous_position, current_position, *, dt_s):
    """Return velocity from positions in metres and a positive interval."""
    raise NotImplementedError