"""Skill: drive a robot from its current position to a target point.

Units: metres, seconds, radians. Field frame, counter-clockwise positive.

Built on helpers.behaviors: it uses the same control loop, so it times out,
stops the robot when vision is missing, and always sends a zero command
before returning (also on exceptions such as KeyboardInterrupt).
"""

from time import monotonic

from ._compat import load

_b = load("behaviors")
HEADING_TOLERANCE_RAD = _b.HEADING_TOLERANCE_RAD
MAX_SPEED_MPS = _b.MAX_SPEED_MPS
POSITION_TOLERANCE_M = _b.POSITION_TOLERANCE_M
_check_positive = _b._check_positive
_drive_command = _b._drive_command
_run = _b._run


def go_to_point(
    sock, endpoint, robot_id, get_pose, *,
    x_m, y_m, heading_rad=None,
    tolerance_m=POSITION_TOLERANCE_M,
    tolerance_rad=HEADING_TOLERANCE_RAD,
    max_speed_mps=MAX_SPEED_MPS,
    timeout_s=10.0,
):
    """Drive to a field position, optionally ending at a target heading.

    Inputs:
        sock (socket.socket): UDP command socket.
        endpoint (tuple[str, int]): Target team address tuple.
        robot_id (int): Non-negative robot ID.
        get_pose (callable): Returns (x_m, y_m, heading_rad) or None.
        x_m (float): Target X coordinate in metres.
        y_m (float): Target Y coordinate in metres.
        heading_rad (float, optional): Final heading in radians. If None, the
            heading seen at the first valid sample is held.
        tolerance_m (float, optional): Position tolerance in metres.
        tolerance_rad (float, optional): Heading tolerance in radians.
        max_speed_mps (float, optional): Speed cap in m/s.
        timeout_s (float, optional): Maximum execution seconds. Default is 10.0s.

    Outputs:
        bool: True if the pose was reached and held for SETTLE_CYCLES samples,
            False on timeout.

    Example:
        >>> go_to_point(sock, endpoint, 0, get_pose, x_m=1.5, y_m=-0.5,
        ...             heading_rad=math.pi / 2)
    """
    _check_positive("tolerance_m", tolerance_m)
    _check_positive("tolerance_rad", tolerance_rad)
    _check_positive("max_speed_mps", max_speed_mps)
    _check_positive("timeout_s", timeout_s)
    held = heading_rad

    def step():
        nonlocal held
        pose = get_pose()
        if pose is None:
            return None, False
        if held is None:
            held = pose[2]
        command, dist, heading_err = _drive_command(
            pose, x_m, y_m, held, max_speed=max_speed_mps)
        ok = dist <= tolerance_m and (
            heading_rad is None or heading_err <= tolerance_rad)
        return command, ok

    return _run(sock, endpoint, robot_id, monotonic() + timeout_s, step)
