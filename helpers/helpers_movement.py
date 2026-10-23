"""Robot velocity commands.

Linear velocity: m/s. Angular velocity: rad/s.
Send functions send once; duration-based functions block.
"""


def send_global_velocity(
    sock, endpoint, robot_id, *,
    vx_mps, vy_mps, omega_radps=0.0,
):
    """Send velocity in field coordinates."""
    raise NotImplementedError


def send_local_velocity(
    sock, endpoint, robot_id, *,
    forward_mps, left_mps, omega_radps=0.0,
):
    """Send velocity in robot coordinates."""
    raise NotImplementedError


def hold_velocity(
    sock, endpoint, robot_id, *,
    vx_mps, vy_mps, omega_radps=0.0,
    duration_s, rate_hz=50.0,
):
    """Repeat field-relative velocity; caller is responsible for stopping."""
    raise NotImplementedError


def ramp_velocity(
    sock, endpoint, robot_id, *,
    start_velocity, end_velocity, duration_s, rate_hz=50.0,
):
    """Interpolate field (vx, vy, omega) commands; caller owns stopping."""
    raise NotImplementedError


def stop_robot(sock, endpoint, robot_id, *, repeat_for_s=0.25, rate_hz=50.0):
    """Repeat zero translational and angular velocity commands."""
    raise NotImplementedError


def stop_robots(sock, endpoint, robot_ids, *, repeat_for_s=0.25, rate_hz=50.0):
    """Stop the specified robots on one team."""
    raise NotImplementedError