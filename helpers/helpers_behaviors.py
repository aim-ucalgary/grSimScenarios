"""Feedback-based robot behaviors."""



def rotate_to(
    sock, endpoint, robot_id, get_pose, *,
    heading_rad, tolerance_rad=0.05, timeout_s=5.0,
):
    """Rotate to a target heading using feedback."""
    raise NotImplementedError


def face_point(
    sock, endpoint, robot_id, get_pose, *,
    x_m, y_m, timeout_s=5.0,
):
    """Rotate toward a field position."""
    raise NotImplementedError


def hold_pose(
    sock, endpoint, robot_id, get_pose, *,
    x_m, y_m, heading_rad, duration_s,
):
    """Maintain a pose for a bounded duration."""
    raise NotImplementedError


def follow_waypoints(
    sock, endpoint, robot_id, get_pose, *,
    waypoints, tolerance_m=0.05, timeout_s=30.0,
):
    """Visit (x, y) targets within an overall deadline."""
    raise NotImplementedError


def approach_ball(
    sock, endpoint, robot_id, get_pose, get_ball, *,
    approach_heading_rad, timeout_s=10.0,
):
    """Approach the ball from a specified direction."""
    raise NotImplementedError


def pass_to(
    sock, endpoint, robot_id, get_pose, get_ball, *,
    target_x_m, target_y_m, kick_speed_mps, timeout_s=10.0,
):
    """Coordinate positioning, aiming, and a kick request."""
    raise NotImplementedError