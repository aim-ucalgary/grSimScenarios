"""Skill: intercept a moving ball and catch it with the dribbler.

Units: metres, seconds, radians. Field frame, counter-clockwise positive.

The ball position and velocity are read from vision (velocity is estimated from
recent get_ball samples unless a get_ball_vel callable is given). The skill
predicts where the rolling ball can be reached first, drives there while
turning the dribbler face toward the incoming ball, and cushions the ball by
moving with it just before contact. On first contact it latches: planning stops,
the cushion ramps to zero, heading is held, and it succeeds HOLD_S seconds later
if the ball is still on the dribbler (otherwise it releases and chases again).

Ground (rolling) balls only: the prediction ignores chipped/airborne balls.
"""

import math
import time
from collections import deque

from helpers import behaviors as _b

# Ball model: uniform deceleration, same estimate as skills/kick_to.py.
BALL_DECEL_MPS2 = 1.4  # tuned for grSim; change for real robots

# Robot motion (starting estimates, tune live).
ROBOT_MAX_SPEED_MPS = 2.0
KP_LINEAR = 5.0
DRIBBLER_RPM = 3000.0         # CHECK: units/scale expected by your sim/robots
CAPTURE_OFFSET_M = 0.10       # robot centre -> ball centre when ball touches dribbler

# Ball velocity estimation.
MIN_MOVING_MPS = 0.15         # below this the ball is treated as stationary
VEL_WINDOW_S = 0.15           # least-squares window over recent samples
PRE_SAMPLE_S = 0.12           # stationary sampling time before moving

# Intercept planning.
INTERCEPT_DT_S = 0.02
INTERCEPT_MARGIN_S = 0.10     # robot must be able to arrive this much early

# Cushioning: when the ball is this close, move along with it.
CUSHION_DIST_M = 0.35
CUSHION_GAIN = 0.6

# "Ball is in the dribbler" test (robot frame: x forward, y left).
CAPTURE_FWD_RANGE_M = (0.05, 0.14)
CAPTURE_LAT_M = 0.04
# Once latched, the ball only counts as lost if it leaves this wider zone.
RELEASE_FWD_M = 0.17
RELEASE_LAT_M = 0.07
CUSHION_RAMP_S = 0.15         # latched: ramp cushion velocity to zero over this time
HOLD_S = 0.3                  # latched this long (robot stopped) -> success


class _VelocityEstimator:
    """Least-squares ball velocity from recent distinct vision samples."""

    def __init__(self):
        self.samples = deque()  # (t, x, y)

    def update(self, ball, now):
        if ball is not None and (
                not self.samples or (ball[0], ball[1]) != self.samples[-1][1:]):
            self.samples.append((now, ball[0], ball[1]))
        while self.samples and now - self.samples[0][0] > VEL_WINDOW_S:
            self.samples.popleft()

    def velocity(self):
        n = len(self.samples)
        if n < 2:
            return 0.0, 0.0
        tm = sum(s[0] for s in self.samples) / n
        xm = sum(s[1] for s in self.samples) / n
        ym = sum(s[2] for s in self.samples) / n
        denom = sum((s[0] - tm) ** 2 for s in self.samples)
        if denom < 1e-6:
            return 0.0, 0.0
        vx = sum((s[0] - tm) * (s[1] - xm) for s in self.samples) / denom
        vy = sum((s[0] - tm) * (s[2] - ym) for s in self.samples) / denom
        return vx, vy


def _plan_intercept(robot_xy, ball_xy, vel):
    """Return (target_x, target_y, heading, ux, uy) for the robot centre.

    (ux, uy) is the unit direction the ball travels; the robot faces against it.
    """
    speed = math.hypot(*vel)
    if speed < MIN_MOVING_MPS:
        heading = math.atan2(ball_xy[1] - robot_xy[1], ball_xy[0] - robot_xy[0])
        ux, uy = -math.cos(heading), -math.sin(heading)
        return (ball_xy[0] + CAPTURE_OFFSET_M * ux,
                ball_xy[1] + CAPTURE_OFFSET_M * uy, heading, ux, uy)

    ux, uy = vel[0] / speed, vel[1] / speed
    heading = math.atan2(-uy, -ux)
    t_stop = speed / BALL_DECEL_MPS2
    s_stop = speed * speed / (2.0 * BALL_DECEL_MPS2)

    def target_at(s):
        return (ball_xy[0] + ux * s + CAPTURE_OFFSET_M * ux,
                ball_xy[1] + uy * s + CAPTURE_OFFSET_M * uy)

    t = 0.0
    while t <= t_stop:
        s = speed * t - 0.5 * BALL_DECEL_MPS2 * t * t
        tx, ty = target_at(s)
        reach_s = math.hypot(tx - robot_xy[0], ty - robot_xy[1]) / ROBOT_MAX_SPEED_MPS
        if reach_s + INTERCEPT_MARGIN_S <= t:
            return tx, ty, heading, ux, uy
        t += INTERCEPT_DT_S
    tx, ty = target_at(s_stop)  # cannot beat the ball: wait where it stops
    return tx, ty, heading, ux, uy


def receive_ball(
    sock, endpoint, robot_id, get_pose, get_ball, *,
    get_ball_vel=None, dribbler_rpm=DRIBBLER_RPM, timeout_s=10.0,
):
    """Intercept a moving ball and catch it in the dribbler.

    Inputs:
        sock (socket.socket): UDP command socket.
        endpoint (tuple[str, int]): Target team address tuple.
        robot_id (int): Non-negative robot ID.
        get_pose (callable): Returns (x_m, y_m, heading_rad) or None.
        get_ball (callable): Returns (x_m, y_m) or None.
        get_ball_vel (callable, optional): Returns (vx_mps, vy_mps) or None. If
            omitted, velocity is estimated from successive get_ball samples.
        dribbler_rpm (float, optional): Dribbler speed while receiving.
        timeout_s (float, optional): Overall deadline in seconds. Default is 10.0s.

    Outputs:
        bool: True once the ball is held in front of the dribbler, False on
            timeout. Note the robot is stopped when this returns.

    Raises:
        ValueError: On invalid arguments.

    Example:
        >>> receive_ball(sock, endpoint, 0, get_pose, get_ball)
    """
    _b._check_positive("timeout_s", timeout_s)
    if not math.isfinite(dribbler_rpm) or dribbler_rpm < 0:
        raise ValueError(f"dribbler_rpm must be >= 0, got {dribbler_rpm!r}")

    deadline = time.monotonic() + timeout_s
    est = _VelocityEstimator()

    # Wait (stationary) for the ball to be visible, then sample its motion.
    ball = get_ball()
    while ball is None:
        if time.monotonic() >= deadline:
            return False
        time.sleep(_b.CONTROL_PERIOD_S)
        ball = get_ball()
    sample_end = time.monotonic() + PRE_SAMPLE_S
    while time.monotonic() < sample_end:
        est.update(get_ball(), time.monotonic())
        time.sleep(_b.CONTROL_PERIOD_S)

    last_ball = [ball]
    latch = {}  # set on first contact: freezes planning so we stop and hold

    def step():
        pose, ball_now = get_pose(), get_ball()
        now = time.monotonic()
        if pose is None:
            return None, False
        if ball_now is not None:
            last_ball[0] = ball_now
        est.update(ball_now, now)
        b = last_ball[0]

        dx, dy = b[0] - pose[0], b[1] - pose[1]
        fwd = dx * math.cos(pose[2]) + dy * math.sin(pose[2])
        lat = -dx * math.sin(pose[2]) + dy * math.cos(pose[2])

        # Latched: ball is on the dribbler. No more planning (the ball's pose and
        # velocity estimates are noisy at contact). Ramp the cushion to zero,
        # hold heading, keep the dribbler on, then report success.
        if latch:
            if fwd <= RELEASE_FWD_M and abs(lat) <= RELEASE_LAT_M:
                elapsed = now - latch["t"]
                k = max(0.0, 1.0 - elapsed / CUSHION_RAMP_S)
                w = max(-_b.MAX_ANGULAR_RADPS, min(
                    _b.MAX_ANGULAR_RADPS,
                    _b.KP_ANGULAR * _b.angle_error(latch["heading"], pose[2])))
                command = {"vx": latch["vx"] * k, "vy": latch["vy"] * k,
                           "w": w, "dribbler_rpm": dribbler_rpm}
                return command, elapsed >= HOLD_S
            latch.clear()  # ball escaped: go back to chasing it

        if get_ball_vel is not None:
            vel = get_ball_vel() or (0.0, 0.0)
        else:
            vel = est.velocity()
        if math.hypot(*vel) < MIN_MOVING_MPS:
            vel = (0.0, 0.0)

        # First contact: latch and start the controlled stop.
        if (CAPTURE_FWD_RANGE_M[0] <= fwd <= CAPTURE_FWD_RANGE_M[1]
                and abs(lat) <= CAPTURE_LAT_M):
            latch.update(t=now, heading=pose[2],
                         vx=CUSHION_GAIN * vel[0], vy=CUSHION_GAIN * vel[1])
            return {"vx": latch["vx"], "vy": latch["vy"], "w": 0.0,
                    "dribbler_rpm": dribbler_rpm}, False

        tx, ty, heading, ux, uy = _plan_intercept((pose[0], pose[1]), b, vel)

        vx = KP_LINEAR * (tx - pose[0])
        vy = KP_LINEAR * (ty - pose[1])

        # Cushion: once the ball is close, move along with it to soak up speed.
        dribble_pt = (pose[0] + CAPTURE_OFFSET_M * math.cos(pose[2]),
                      pose[1] + CAPTURE_OFFSET_M * math.sin(pose[2]))
        if math.hypot(b[0] - dribble_pt[0], b[1] - dribble_pt[1]) < CUSHION_DIST_M:
            vx += CUSHION_GAIN * vel[0]
            vy += CUSHION_GAIN * vel[1]

        norm = math.hypot(vx, vy)
        if norm > ROBOT_MAX_SPEED_MPS:
            vx, vy = vx * ROBOT_MAX_SPEED_MPS / norm, vy * ROBOT_MAX_SPEED_MPS / norm

        w = max(-_b.MAX_ANGULAR_RADPS, min(
            _b.MAX_ANGULAR_RADPS,
            _b.KP_ANGULAR * _b.angle_error(heading, pose[2])))

        return {"vx": vx, "vy": vy, "w": w, "dribbler_rpm": dribbler_rpm}, False

    return bool(_b._run(sock, endpoint, robot_id, deadline, step, settle_cycles=1))