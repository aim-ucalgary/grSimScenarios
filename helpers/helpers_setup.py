"""Scene initialization through the simulation-control endpoint.

Positions: metres. Heading: radians. Team: 'blue' or 'yellow'.
"""


def teleport_robot(
    sock, endpoint, team, robot_id, *,
    x_m, y_m, heading_rad=0.0,
    vx_mps=0.0, vy_mps=0.0, omega_radps=0.0,
):
    """Set a robot's pose and initial velocity."""
    raise NotImplementedError


def teleport_ball(
    sock, endpoint, *,
    x_m, y_m, z_m=0.0,
    vx_mps=0.0, vy_mps=0.0, vz_mps=0.0,
):
    """Set the ball's position and initial velocity."""
    raise NotImplementedError


def set_robot_present(sock, endpoint, team, robot_id, *, present):
    """Enable or remove an existing robot slot."""
    raise NotImplementedError


def apply_initial_state(sock, endpoint, state):
    """Apply a team-defined scene description."""
    raise NotImplementedError


def reset_scene(sock, endpoint, initial_state):
    """Restore the supplied initial scene, including velocities."""
    raise NotImplementedError