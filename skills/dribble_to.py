"""Skill: dribble the ball to a field position.

Units: metres, seconds, radians. Field frame, counter-clockwise positive.

The robot first gets to the ball with helpers.behaviors.approach_ball, facing
the direction of travel. It then spins up the dribbler and drives toward the
target while keeping its heading fixed, so the ball stays on the dribbler.
If the ball slips away, the robot stops and re-acquires it. There is no limit
on dribbling distance here; the caller is responsible for any such rule.

The dribbler switches off when the skill returns, because the final stop
command zeroes everything.
"""

import math
import time

from helpers import actuators
from helpers import behaviors as _b

DRIBBLE_SPEED_MPS = 0.5     # cruise speed while carrying the ball
DRIBBLE_START_MPS = 0.1     # speed cap at the start of the ramp
DRIBBLE_ACCEL_MPS2 = 1.0    # ramp rate; gentle so the ball isn't shaken loose
DRIBBLER_SPINUP_S = 0.3     # stand still with the dribbler on before moving
HOLD_SLACK_M = 0.04         # ball may sit this far beyond the kicker face
HOLD_LATERAL_M = 0.05       # ... and this far off the robot's centreline
LOST_CYCLES = 5             # consecutive cycles out of hold zone = ball lost


def _dribble_phase(sock, endpoint, robot_id, get_pose, get_ball, *,
                   target, heading, deadline, speed, tolerance_m, rpm):
    """Carry the ball to target. Returns 'arrived', 'lost' or 'timeout'."""
    contact = _b.ROBOT_FRONT_M + _b.BALL_RADIUS_M
    dx, dy = math.cos(heading), math.sin(heading)
    # Robot-centre point at which the ball sits on the target.
    rx, ry = target[0] - dx * contact, target[1] - dy * contact

    outcome = None
    spinup = math.ceil(DRIBBLER_SPINUP_S / _b.CONTROL_PERIOD_S)
    cap = DRIBBLE_START_MPS
    lost = 0

    def step():
        nonlocal outcome, spinup, cap, lost
        pose, ball = get_pose(), get_ball()
        if pose is None or ball is None:
            # Keep the dribbler spinning while stopped through a vision dropout.
            return {"dribbler_rpm": rpm}, False
        px, py, ph = pose

        # Is the ball still on the dribbler? (robot frame: f ahead, l left)
        rel_x, rel_y = ball[0] - px, ball[1] - py
        f = rel_x * math.cos(ph) + rel_y * math.sin(ph)
        l = -rel_x * math.sin(ph) + rel_y * math.cos(ph)
        held = f <= contact + HOLD_SLACK_M and abs(l) <= HOLD_LATERAL_M
        lost = 0 if held else lost + 1
        if lost >= LOST_CYCLES:
            outcome = "lost"
            return None, True

        if math.hypot(ball[0] - target[0], ball[1] - target[1]) <= tolerance_m:
            outcome = "arrived"
            return {"dribbler_rpm": rpm}, True

        if spinup > 0:
            spinup -= 1
            command, _, _ = _b._drive_command(pose, px, py, heading, max_speed=0.0)
        else:
            cap = min(speed, cap + DRIBBLE_ACCEL_MPS2 * _b.CONTROL_PERIOD_S)
            command, _, _ = _b._drive_command(pose, rx, ry, heading, max_speed=cap)
        command["dribbler_rpm"] = rpm
        return command, False

    _b._run(sock, endpoint, robot_id, deadline, step, settle_cycles=1)
    return outcome or "timeout"


def dribble_to(
    sock, endpoint, robot_id, get_pose, get_ball, *,
    x_m, y_m, speed_mps=DRIBBLE_SPEED_MPS, tolerance_m=0.05,
    dribbler_rpm=actuators.DRIBBLER_ON_RPM, max_reacquire=2, timeout_s=20.0,
):
    """Dribble the ball until it is within tolerance of (x_m, y_m).

    Inputs:
        sock (socket.socket): UDP command socket.
        endpoint (tuple[str, int]): Target team address tuple.
        robot_id (int): Non-negative robot ID.
        get_pose (callable): Returns (x_m, y_m, heading_rad) or None.
        get_ball (callable): Returns (x_m, y_m) or None.
        x_m (float): Target X coordinate for the ball, metres.
        y_m (float): Target Y coordinate for the ball, metres.
        speed_mps (float, optional): Cruise speed while dribbling. Default 0.5 m/s.
        tolerance_m (float, optional): Ball-to-target distance counted as arrived.
        dribbler_rpm (float, optional): Dribbler speed command.
        max_reacquire (int, optional): Times to re-approach after losing the ball.
        timeout_s (float, optional): Overall deadline in seconds. Default is 20.0s.

    Outputs:
        bool: True if the ball ended within tolerance of the target, False on
            timeout, repeated ball loss, or missing vision.

    Example:
        >>> dribble_to(sock, endpoint, 0, get_pose, get_ball, x_m=1.0, y_m=0.5)
    """
    _b._check_positive("speed_mps", speed_mps)
    _b._check_positive("tolerance_m", tolerance_m)
    _b._check_positive("dribbler_rpm", dribbler_rpm)
    _b._check_positive("timeout_s", timeout_s)
    if not isinstance(max_reacquire, int) or isinstance(max_reacquire, bool) or max_reacquire < 0:
        raise ValueError(f"max_reacquire must be a non-negative int, got {max_reacquire!r}")

    deadline = time.monotonic() + timeout_s
    target = (float(x_m), float(y_m))

    for _ in range(max_reacquire + 1):
        # Wait (stationary) for the ball to be visible.
        ball = get_ball()
        while ball is None:
            if time.monotonic() >= deadline:
                return False
            time.sleep(_b.CONTROL_PERIOD_S)
            ball = get_ball()

        if math.hypot(target[0] - ball[0], target[1] - ball[1]) <= tolerance_m:
            return True  # already there

        remaining = deadline - time.monotonic()
        if remaining <= 0:
            return False

        heading = math.atan2(target[1] - ball[1], target[0] - ball[0])
        if not _b.approach_ball(sock, endpoint, robot_id, get_pose, get_ball,
                                approach_heading_rad=heading, timeout_s=remaining):
            continue  # try again if time remains

        outcome = _dribble_phase(
            sock, endpoint, robot_id, get_pose, get_ball,
            target=target, heading=heading, deadline=deadline,
            speed=speed_mps, tolerance_m=tolerance_m, rpm=dribbler_rpm)
        if outcome == "arrived":
            return True
        if outcome == "timeout":
            return False
        # outcome == "lost": loop to re-acquire

    return False
