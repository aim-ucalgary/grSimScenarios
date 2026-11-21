"""Skill: go to the ball, hold it at its coordinates, and spin 360 degrees in place.

Units: metres, seconds, radians. Field frame, counter-clockwise positive.

Self-contained (the dribbling pieces are copied from dribble_to):
  1. approach_ball gets the robot onto the ball (always, never skipped).
  2. _dribble_phase stands still with the dribbler on so it can grip the ball,
     then, only if the ball has been nudged away from where it started, carries
     it back to those coordinates.
  3. _turn_with_ball spins the robot in place through 360 degrees.

If the ball is lost at any point, the robot re-acquires it, carries it back to
its original coordinates and tries again.

The dribbler switches off when the skill returns, because the final stop
command zeroes everything.
"""

import math
import time

from helpers import actuators
from helpers import behaviors as _b

DRIBBLE_SPEED_MPS = 0.5     # cruise speed while carrying the ball back
DRIBBLE_START_MPS = 0.1     # speed cap at the start of the ramp
DRIBBLE_ACCEL_MPS2 = 1.0    # ramp rate; gentle so the ball isn't shaken loose
DRIBBLER_SPINUP_S = 0.3     # stand still with the dribbler on before moving
HOLD_SLACK_M = 0.04        # ball may sit this far beyond the kicker face
HOLD_LATERAL_M = 0.05       # ... and this far off the robot's centreline
LOST_CYCLES = 5             # consecutive cycles out of hold zone = ball lost

# Spin in place (after the ball is secured)
TURN_RATE_RADPS = 0.5       # slow; counter-clockwise positive, negative = clockwise
TURN_ACCEL_RADPS2 = 1.0     # angular speed ramp-up
TURN_TIMEOUT_FACTOR = 1.5   # turn time budget = factor * ideal time + 2 s


def _ball_held(pose, ball, contact):
    """True if the ball is within the dribbler zone (robot frame: f ahead, l left)."""
    px, py, ph = pose
    rel_x, rel_y = ball[0] - px, ball[1] - py
    f = rel_x * math.cos(ph) + rel_y * math.sin(ph)
    l = -rel_x * math.sin(ph) + rel_y * math.cos(ph)
    return f <= contact + HOLD_SLACK_M and abs(l) <= HOLD_LATERAL_M


def _dribble_phase(sock, endpoint, robot_id, get_pose, get_ball, *,
                   target, heading, deadline, speed, tolerance_m, rpm):
    """Grip the ball and carry it to target. Returns 'arrived', 'lost' or 'timeout'."""
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

        lost = 0 if _ball_held(pose, ball, contact) else lost + 1
        if lost >= LOST_CYCLES:
            outcome = "lost"
            return None, True

        if spinup > 0:
            # Stand still with the dribbler on so it can grip the ball. Arrival is
            # deliberately not judged yet: when the ball is already at the target,
            # skipping this wait would start the turn before the ball is gripped.
            spinup -= 1
            command, _, _ = _b._drive_command(pose, px, py, heading, max_speed=0.0)
            command["dribbler_rpm"] = rpm
            return command, False

        at_contact = math.hypot(px - rx, py - ry) <= 0.01   # robot is at the grip point
        if (math.hypot(ball[0] - target[0], ball[1] - target[1]) <= tolerance_m
                and at_contact):
            outcome = "arrived"
            return {"dribbler_rpm": rpm}, True

        cap = min(speed, cap + DRIBBLE_ACCEL_MPS2 * _b.CONTROL_PERIOD_S)
        command, _, _ = _b._drive_command(pose, rx, ry, heading, max_speed=cap)
        command["dribbler_rpm"] = rpm
        return command, False

    _b._run(sock, endpoint, robot_id, deadline, step, settle_cycles=1)
    return outcome or "timeout"


def _turn_with_ball(sock, endpoint, robot_id, get_pose, get_ball, *,
                    rate_radps, rpm, total_rad=2.0 * math.pi):
    """Spin in place by total_rad radians with the dribbler on.

    Sends only an angular velocity, so the robot rotates about its own centre.
    The turn has its own time budget, since the dribble deadline may be nearly
    used up by now. Returns True if the full turn was completed with the ball
    still on the dribbler, False if the ball is lost or the budget runs out.
    """
    contact = _b.ROBOT_FRONT_M + _b.BALL_RADIUS_M
    sign = 1.0 if rate_radps > 0 else -1.0
    rate = abs(rate_radps)
    deadline = time.monotonic() + TURN_TIMEOUT_FACTOR * total_rad / rate + 2.0

    prev_heading = None
    turned = 0.0
    w_cap = 0.0
    lost = 0
    failed = False

    def step():
        nonlocal prev_heading, turned, w_cap, lost, failed
        pose, ball = get_pose(), get_ball()
        if pose is None or ball is None:
            return {"dribbler_rpm": rpm}, False
        if prev_heading is None:
            prev_heading = pose[2]

        lost = 0 if _ball_held(pose, ball, contact) else lost + 1
        if lost >= LOST_CYCLES:
            failed = True
            return None, True

        turned += sign * _b.angle_error(pose[2], prev_heading)
        prev_heading = pose[2]
        if turned >= total_rad:
            return {"dribbler_rpm": rpm}, True

        w_cap = min(rate, w_cap + TURN_ACCEL_RADPS2 * _b.CONTROL_PERIOD_S)
        return {"w": sign * w_cap, "dribbler_rpm": rpm}, False

    _b._run(sock, endpoint, robot_id, deadline, step, settle_cycles=1)
    return (not failed) and turned >= total_rad


def turn_with_ball(
    sock, endpoint, robot_id, get_pose, get_ball, *,
    turn_rate_radps=TURN_RATE_RADPS, tolerance_m=0.05,
    dribbler_rpm=actuators.DRIBBLER_ON_RPM, max_reacquire=2, timeout_s=20.0,
):
    """Go to the ball, hold it at its coordinates, and spin 360 degrees in place.

    Inputs:
        sock (socket.socket): UDP command socket.
        endpoint (tuple[str, int]): Target team address tuple.
        robot_id (int): Non-negative robot ID.
        get_pose (callable): Returns (x_m, y_m, heading_rad) or None.
        get_ball (callable): Returns (x_m, y_m) or None.
        turn_rate_radps (float, optional): Turning speed in rad/s. Default 0.5.
            Positive = counter-clockwise, negative = clockwise.
        tolerance_m (float, optional): How far the ball may be from its original
            coordinates and still count as "there".
        dribbler_rpm (float, optional): Dribbler speed command.
        max_reacquire (int, optional): Times to re-approach after losing the ball.
        timeout_s (float, optional): Deadline for the approach and dribble, in
            seconds. Default 20.0s. The 360-degree turn has its own time budget.

    Outputs:
        bool: True if the full 360-degree turn was completed with the ball on the
            dribbler; False on timeout, repeated ball loss, or missing vision.

    Example:
        >>> turn_with_ball(sock, endpoint, 0, get_pose, get_ball, turn_rate_radps=0.8)
    """
    if (not isinstance(turn_rate_radps, (int, float)) or isinstance(turn_rate_radps, bool)
            or not math.isfinite(turn_rate_radps) or turn_rate_radps == 0):
        raise ValueError(f"turn_rate_radps must be a non-zero finite number, got {turn_rate_radps!r}")
    _b._check_positive("tolerance_m", tolerance_m)
    _b._check_positive("dribbler_rpm", dribbler_rpm)
    _b._check_positive("timeout_s", timeout_s)
    if not isinstance(max_reacquire, int) or isinstance(max_reacquire, bool) or max_reacquire < 0:
        raise ValueError(f"max_reacquire must be a non-negative int, got {max_reacquire!r}")

    deadline = time.monotonic() + timeout_s
    target = None  # the ball's coordinates, fixed at the first sighting

    for _ in range(max_reacquire + 1):
        # Wait (stationary) for vision of both the robot and the ball.
        pose, ball = get_pose(), get_ball()
        while pose is None or ball is None:
            if time.monotonic() >= deadline:
                return False
            time.sleep(_b.CONTROL_PERIOD_S)
            pose, ball = get_pose(), get_ball()
        if target is None:
            target = (float(ball[0]), float(ball[1]))

        remaining = deadline - time.monotonic()
        if remaining <= 0:
            return False

        if math.hypot(target[0] - ball[0], target[1] - ball[1]) > tolerance_m:
            # Ball has moved: line up behind it to carry it back (as dribble_to does).
            heading = math.atan2(target[1] - ball[1], target[0] - ball[0])
        else:
            # Ball is where it started: go straight at it from where we are.
            heading = math.atan2(ball[1] - pose[1], ball[0] - pose[0])

        if not _b.approach_ball(sock, endpoint, robot_id, get_pose, get_ball,
                                approach_heading_rad=heading, timeout_s=remaining):
            continue  # try again if time remains

        outcome = _dribble_phase(
            sock, endpoint, robot_id, get_pose, get_ball,
            target=target, heading=heading, deadline=deadline,
            speed=DRIBBLE_SPEED_MPS, tolerance_m=tolerance_m, rpm=dribbler_rpm)
        if outcome == "timeout":
            return False
        if outcome == "lost":
            continue

        # Ball is secured at its coordinates: spin a full circle.
        if _turn_with_ball(sock, endpoint, robot_id, get_pose, get_ball,
                           rate_radps=turn_rate_radps, rpm=dribbler_rpm):
            return True
        # Ball lost during the turn: loop re-acquires it and tries again.

    return False