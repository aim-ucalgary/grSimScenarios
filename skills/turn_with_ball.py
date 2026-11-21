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

TURN_MIN_RADPS = 0.15       # slowest speed while closing in on the final heading
TURN_TAPER_GAIN = 2.0       # w <= gain * remaining_angle, so it slows down at the end
TURN_DONE_RAD = 0.03        # ~1.7 degrees: close enough to count as facing the target


def _turn_with_ball(sock, endpoint, robot_id, get_pose, get_ball, *,
                    rate_radps, rpm, total_rad):
    """Spin in place by total_rad radians (>0) with the dribbler on.

    The sign of rate_radps sets the direction (positive = counter-clockwise).
    Returns True if the turn was completed with the ball still on the
    dribbler, False if the ball is lost or the time budget runs out.
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
        remaining = total_rad - turned
        if remaining <= TURN_DONE_RAD:
            return {"dribbler_rpm": rpm}, True

        w_cap = min(rate, w_cap + TURN_ACCEL_RADPS2 * _b.CONTROL_PERIOD_S)
        w = min(w_cap, max(TURN_MIN_RADPS, TURN_TAPER_GAIN * remaining))
        return {"w": sign * min(w, rate), "dribbler_rpm": rpm}, False

    _b._run(sock, endpoint, robot_id, deadline, step, settle_cycles=1)
    return (not failed) and (total_rad - turned) <= TURN_DONE_RAD

def turn_with_ball(
    sock, endpoint, robot_id, get_pose, get_ball, *,
    x_m, y_m, turn_rate_radps=TURN_RATE_RADPS, tolerance_m=0.05,
    dribbler_rpm=actuators.DRIBBLER_ON_RPM, max_reacquire=2, timeout_s=20.0,
):
    """Go to the ball, hold it at its coordinates, and turn in place to face (x_m, y_m).

    Inputs:
        x_m, y_m (float): Point the robot should end up facing, metres.
        turn_rate_radps (float, optional): Maximum turning speed in rad/s.
            Only the magnitude is used; the shortest direction is chosen.
        (other inputs as before)

    Outputs:
        bool: True if the robot ended up facing (x_m, y_m) with the ball on the
            dribbler; False on timeout, repeated ball loss, or missing vision.
    """
    if (not isinstance(turn_rate_radps, (int, float)) or isinstance(turn_rate_radps, bool)
            or not math.isfinite(turn_rate_radps) or turn_rate_radps == 0):
        raise ValueError(f"turn_rate_radps must be a non-zero finite number, got {turn_rate_radps!r}")
    for name, v in (("x_m", x_m), ("y_m", y_m)):
        if not isinstance(v, (int, float)) or isinstance(v, bool) or not math.isfinite(v):
            raise ValueError(f"{name} must be a finite number, got {v!r}")
    _b._check_positive("tolerance_m", tolerance_m)
    _b._check_positive("dribbler_rpm", dribbler_rpm)
    _b._check_positive("timeout_s", timeout_s)
    if not isinstance(max_reacquire, int) or isinstance(max_reacquire, bool) or max_reacquire < 0:
        raise ValueError(f"max_reacquire must be a non-negative int, got {max_reacquire!r}")

    look_at = (float(x_m), float(y_m))
    deadline = time.monotonic() + timeout_s
    target = None  # the ball's coordinates, fixed at the first sighting

    for _ in range(max_reacquire + 1):
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
            heading = math.atan2(target[1] - ball[1], target[0] - ball[0])
        else:
            heading = math.atan2(ball[1] - pose[1], ball[0] - pose[0])

        if not _b.approach_ball(sock, endpoint, robot_id, get_pose, get_ball,
                                approach_heading_rad=heading, timeout_s=remaining):
            continue

        outcome = _dribble_phase(
            sock, endpoint, robot_id, get_pose, get_ball,
            target=target, heading=heading, deadline=deadline,
            speed=DRIBBLE_SPEED_MPS, tolerance_m=tolerance_m, rpm=dribbler_rpm)
        if outcome == "timeout":
            return False
        if outcome == "lost":
            continue

        # Ball is secured: work out how far to turn to face the look-at point.
        pose = get_pose()
        while pose is None:
            if time.monotonic() >= deadline:
                return False
            time.sleep(_b.CONTROL_PERIOD_S)
            pose = get_pose()

        desired = math.atan2(look_at[1] - pose[1], look_at[0] - pose[0])
        delta = _b.angle_error(desired, pose[2])   # signed, shortest, in (-pi, pi]
        if abs(delta) <= TURN_DONE_RAD:
            return True                            # already facing it

        rate = math.copysign(abs(turn_rate_radps), delta)
        if _turn_with_ball(sock, endpoint, robot_id, get_pose, get_ball,
                           rate_radps=rate, rpm=dribbler_rpm, total_rad=abs(delta)):
            return True
        # Ball lost during the turn: loop re-acquires it and tries again.

    return False