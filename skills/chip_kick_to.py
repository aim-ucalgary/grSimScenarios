
"""Skill: drive to the ball and chip-kick it toward a field position.
 
Units: metres, seconds, radians. Field frame, counter-clockwise positive.
 
Built on helpers.behaviors: approach_ball lines the robot up behind the ball
(going around it if needed) and creeps to the kicker face, then this skill
requests a chip kick while pushing forward until the ball visibly moves. If the
kick misses, it re-approaches and tries again until the deadline.
"""
 
import math
import time
 
from helpers import behaviors as _b
 
# Auto chip speed assumes a ballistic flight with a fixed launch angle:
# landing distance d = v^2 * sin(2 * theta) / g  ->  v = sqrt(d * g / sin(2 * theta)).
# The ball keeps bouncing/rolling after the first landing, so this targets the
# FIRST landing point. Tune CHIP_ANGLE_RAD and the speed limits on the real
# robots (and in grsim, which models chip differently).
GRAVITY_MPS2 = 9.81
CHIP_ANGLE_RAD = math.radians(45.0)  # fixed chipper angle; change to match your kicker
MIN_AUTO_CHIP_MPS = 1.5
MAX_AUTO_CHIP_MPS = 6.5   # SSL rules cap shots at 6.5 m/s
CHIP_ANGLE_DEG = 45.0 
 
def estimate_chip_speed(distance_m):
    """Chip speed (m/s) expected to make the ball first land roughly distance_m away."""
    v = math.sqrt(
        max(distance_m, 0.0) * GRAVITY_MPS2 / math.sin(2.0 * CHIP_ANGLE_RAD))
    return max(MIN_AUTO_CHIP_MPS, min(MAX_AUTO_CHIP_MPS, v))
 
 
def chip_kick_to(
    sock, endpoint, robot_id, get_pose, get_ball, *,
    target_x_m, target_y_m, chip_speed_mps=None,
    max_attempts=3, timeout_s=20.0,
):
    """Approach the ball and chip-kick it toward (target_x_m, target_y_m).
 
    Inputs:
        sock (socket.socket): UDP command socket.
        endpoint (tuple[str, int]): Target team address tuple.
        robot_id (int): Non-negative robot ID.
        get_pose (callable): Returns (x_m, y_m, heading_rad) or None.
        get_ball (callable): Returns (x_m, y_m) or None.
        target_x_m (float): X coordinate the ball should land near, metres.
        target_y_m (float): Y coordinate the ball should land near, metres.
        chip_speed_mps (float, optional): Chip speed in (0, 10] m/s. If None, it
            is estimated from the ball-to-target distance (see estimate_chip_speed).
        max_attempts (int, optional): Approach-and-kick tries before giving up.
        timeout_s (float, optional): Overall deadline in seconds. Default is 20.0s.
 
    Outputs:
        bool: True once the ball moved after a chip request, False otherwise.
            This confirms the kick happened, not that the ball landed at the target.
 
    Raises:
        ValueError: On invalid arguments or a target on top of the ball.
 
    Example:
        >>> chip_kick_to(sock, endpoint, 0, get_pose, get_ball,
        ...              target_x_m=2.0, target_y_m=1.0)
    """
    _b._check_positive("timeout_s", timeout_s)
    if not isinstance(max_attempts, int) or max_attempts < 1:
        raise ValueError(f"max_attempts must be a positive int, got {max_attempts!r}")
    if chip_speed_mps is not None and (
            not math.isfinite(chip_speed_mps)
            or not 0 < chip_speed_mps <= _b.MAX_KICK_SPEED_MPS):
        raise ValueError(
            f"chip_speed_mps must be in (0, {_b.MAX_KICK_SPEED_MPS}], got {chip_speed_mps!r}")
 
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
        speed = chip_speed_mps if chip_speed_mps is not None else estimate_chip_speed(dist)
 
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
                "kick_angle": CHIP_ANGLE_DEG,  # nonzero angle = chip
            }
            moved = ball_now is not None and math.hypot(
                ball_now[0] - start_ball[0],
                ball_now[1] - start_ball[1]) >= _b.BALL_MOVED_M
            return command, moved
 
        if _b._run(sock, endpoint, robot_id, kick_deadline, step, settle_cycles=1):
            return True
 
    return False