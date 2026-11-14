"""Skill: drive to the ball and kick it toward a field position.

Units: metres, seconds, radians. Field frame, counter-clockwise positive.

Built on helpers.behaviors: approach_ball lines the robot up behind the ball
(going around it if needed) and creeps to the kicker face, then this skill
requests a kick while pushing forward until the ball visibly moves. If the
kick misses, it re-approaches and tries again until the deadline.
"""

import math
import time

from helpers import behaviors as _b

# Auto kick speed assumes the ball decelerates uniformly: v = sqrt(2 * a * d).
# This is a starting estimate; tune it from the live test (see live_kick_to.py).
BALL_DECEL_MPS2 = 1.4
MIN_AUTO_KICK_MPS = 1.5
MAX_AUTO_KICK_MPS = 6.5   # SSL rules cap shots at 6.5 m/s


def estimate_kick_speed(distance_m):
    """Kick speed (m/s) expected to roll a ball roughly distance_m and stop."""
    v = math.sqrt(2.0 * BALL_DECEL_MPS2 * max(distance_m, 0.0))
    print(f"v={v}")
    
    return max(MIN_AUTO_KICK_MPS, min(MAX_AUTO_KICK_MPS, v))


def kick_to(
    sock, endpoint, robot_id, get_pose, get_ball, *,
    target_x_m, target_y_m, kick_speed_mps=None,
    max_attempts=3, timeout_s=20.0,
):
    """Approach the ball and kick it toward (target_x_m, target_y_m).

    Inputs:
        sock (socket.socket): UDP command socket.
        endpoint (tuple[str, int]): Target team address tuple.
        robot_id (int): Non-negative robot ID.
        get_pose (callable): Returns (x_m, y_m, heading_rad) or None.
        get_ball (callable): Returns (x_m, y_m) or None.
        target_x_m (float): X coordinate the ball should travel toward, metres.
        target_y_m (float): Y coordinate the ball should travel toward, metres.
        kick_speed_mps (float, optional): Kick speed in (0, 10] m/s. If None, it
            is estimated from the ball-to-target distance (see estimate_kick_speed).
        max_attempts (int, optional): Approach-and-kick tries before giving up.
        timeout_s (float, optional): Overall deadline in seconds. Default is 20.0s.

    Outputs:
        bool: True once the ball moved after a kick request, False otherwise.
            This confirms the kick happened, not that the ball reached the target.

    Raises:
        ValueError: On invalid arguments or a target on top of the ball.

    Example:
        >>> kick_to(sock, endpoint, 0, get_pose, get_ball,
        ...         target_x_m=2.0, target_y_m=1.0)
    """
    _b._check_positive("timeout_s", timeout_s)
    if not isinstance(max_attempts, int) or max_attempts < 1:
        raise ValueError(f"max_attempts must be a positive int, got {max_attempts!r}")
    if kick_speed_mps is not None and (
            not math.isfinite(kick_speed_mps)
            or not 0 < kick_speed_mps <= _b.MAX_KICK_SPEED_MPS):
        raise ValueError(
            f"kick_speed_mps must be in (0, {_b.MAX_KICK_SPEED_MPS}], got {kick_speed_mps!r}")

    deadline = time.monotonic() + timeout_s

    for _ in range(max_attempts):
        # Wait (stationary) for the ball to be visible.
        ball = get_ball()
        while ball is None:
            if time.monotonic() >= deadline:
                return False
            time.sleep(_b.CONTROL_PERIOD_S)
            ball = get_ball()

        dist = math.hypot(target_x_m - ball[0], target_y_m - ball[1])
        if dist < 0.1:
            raise ValueError("target is on top of the ball")
        heading = math.atan2(target_y_m - ball[1], target_x_m - ball[0])
        speed = kick_speed_mps if kick_speed_mps is not None else estimate_kick_speed(dist)

        remaining = deadline - time.monotonic()
        if remaining <= 0:
            return False
        if not _b.approach_ball(sock, endpoint, robot_id, get_pose, get_ball,
                                approach_heading_rad=heading, timeout_s=remaining):
            continue  # try again if time remains

        start_ball = get_ball()
        if start_ball is None:
            continue
        kick_deadline = min(deadline, time.monotonic() + _b.KICK_WINDOW_S)

        def step():
            pose, ball_now = get_pose(), get_ball()
            if pose is None:
                return None, False
            w = max(-_b.MAX_ANGULAR_RADPS, min(
                _b.MAX_ANGULAR_RADPS,
                _b.KP_ANGULAR * _b.angle_error(heading, pose[2])))
            command = {
                "vx": _b.KICK_PUSH_MPS * math.cos(heading),
                "vy": _b.KICK_PUSH_MPS * math.sin(heading),
                "w": w,
                "kick_speed": speed,
            }
            moved = ball_now is not None and math.hypot(
                ball_now[0] - start_ball[0],
                ball_now[1] - start_ball[1]) >= _b.BALL_MOVED_M
            return command, moved

        if _b._run(sock, endpoint, robot_id, kick_deadline, step, settle_cycles=1):
            return True

    return False
