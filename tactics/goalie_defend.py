"""Skill: goalie_defend, a classical goalkeeper.

Units: metres, seconds, radians. Field frame, counter-clockwise positive.

Three behaviours, chosen every control cycle:

1. Shot: if the ball is moving toward the goal line and will reach it inside the
   posts (using the same uniform-deceleration ball model as kick_to), slide to
   where its path crosses the line (just in front of it) and block there. Shots
   do not turn while doing it, and even slow, decelerating balls still count as
   shots (the old cut-off made the goalie give up on a ball that slowed down).
2. Idle: otherwise stand on the line from the goal centre to the ball, a short
   way out ("arc" defence), so the angle to both posts is covered.
3. Clear: if the ball comes to rest in front of the goal, kick it away toward
   the side it is on (kick_to), then go back to defending.

The goalie faces the ball, except while blocking a shot or when the ball is
within TURN_FREEZE_GAP_M of its body: then it does not turn at all (a ball at
contact makes the "face the ball" angle swing wildly and the robot would rotate
away from the shot, and rotation also uses up wheel speed that the sideways
move needs). It never
leaves the posts' width.
"""

import math
import time

from helpers import behaviors as _b
from skills.kick_to import kick_to, BALL_DECEL_MPS2
from skills.receive_ball import _VelocityEstimator

GOALIE_DEPTH_M = 0.15       # centre this far in front of the goal line (minimum)
GOALIE_ARC_M = 0.5          # idle distance from the goal centre toward the ball
GOALIE_KP = 8.0
GOALIE_MAX_SPEED_MPS = 2.5
SHOT_MIN_MPS = 0.3          # ball slower than this is not treated as a shot
SHOT_MARGIN_M = 0.10        # also defend shots this far outside the posts
ROBOT_RADIUS_M = 0.09
BALL_RADIUS_M = 0.0215
TURN_FREEZE_GAP_M = 0.05    # ball closer than this to the robot's body: stop turning

CLEAR_MIN_M = 0.30          # ball must be this far from the goal line to be cleared
CLEAR_MAX_M = 1.20
CLEAR_SIDE_M = 0.8          # ... and within the posts' width plus this
CLEAR_STILL_MPS = 0.2
CLEAR_HOLD_S = 0.5          # ball must be still this long before we go for it
CLEAR_TIMEOUT_S = 8.0
CLEAR_TARGET_DIST_M = 3.0   # clearance target: this far out from the goal line
CLEAR_TARGET_SIDE_M = 2.0   # ... and this far to the ball's side

_ZERO = {"vx": 0.0, "vy": 0.0, "w": 0.0}


def goalie_target(ball, vel, goal_x, goal_y, half_width, inward_sign):
    """Where the goalie should stand: ((x, y), mode) with mode 'shot' or 'idle'.

    A shot is met where the ball's path crosses the line (just in front of it).
    """
    depth_x = goal_x + inward_sign * GOALIE_DEPTH_M
    if ball is None:
        return (depth_x, goal_y), "idle"

    def clamp_y(y):
        return max(goal_y - half_width, min(goal_y + half_width, y))

    speed = math.hypot(*vel)
    if speed >= SHOT_MIN_MPS:
        ux, uy = vel[0] / speed, vel[1] / speed
        if abs(ux) > 1e-6 and (goal_x - ball[0]) * ux > 0:       # heading at the goal line
            s_line = (goal_x - ball[0]) / ux                       # distance along its path
            if speed * speed >= 2.0 * BALL_DECEL_MPS2 * s_line:    # it gets there
                y_cross = ball[1] + uy * s_line
                if abs(y_cross - goal_y) <= half_width + SHOT_MARGIN_M:
                    return (depth_x, clamp_y(y_cross)), "shot"       # block at the crossing point

    dx, dy = ball[0] - goal_x, ball[1] - goal_y
    dist = math.hypot(dx, dy)
    if dist < 1e-3 or dx * inward_sign <= 0:                       # ball level with/behind the line
        return (depth_x, goal_y), "idle"
    d = max(GOALIE_DEPTH_M, min(GOALIE_ARC_M, 0.5 * dist))
    x = goal_x + dx / dist * d
    y = goal_y + dy / dist * d
    if (x - goal_x) * inward_sign < GOALIE_DEPTH_M:
        x = depth_x
    return (x, clamp_y(y)), "idle"


def goalie_defend(
    sock, endpoint, goalie_id, get_pose, get_ball, *,
    goal_x_m, goal_y_m=0.0, goal_half_width_m=0.5,
    clear_ball=True, should_stop=None, timeout_s=30.0,
):
    """Defend a goal: block shots, cover the angle, and clear loose balls.

    Inputs:
        sock (socket.socket): UDP command socket.
        endpoint (tuple[str, int]): Target team address tuple.
        goalie_id (int): Non-negative robot ID.
        get_pose (callable): Returns (x_m, y_m, heading_rad) or None.
        get_ball (callable): Returns (x_m, y_m) or None.
        goal_x_m (float): X of the goal line being defended, metres.
        goal_y_m (float, optional): Y of the goal centre. Default 0.0.
        goal_half_width_m (float, optional): Half the goal width, metres.
            Division B goals are 1.0 m wide (0.5), Division A 1.8 m (0.9).
        clear_ball (bool, optional): Kick away a ball that stops in front of the goal.
        should_stop (callable, optional): Returns True to end the skill early.
        timeout_s (float, optional): Maximum run time in seconds. Default is 30.0s.

    Outputs:
        bool: True if no goal was conceded while running, False otherwise. The
            goalie keeps defending after a goal (it does not stop).

    Raises:
        ValueError: On invalid arguments.

    Example:
        >>> goalie_defend(sock, endpoint, 0, get_pose, get_ball, goal_x_m=-4.5)
    """
    _b._check_positive("timeout_s", timeout_s)
    _b._check_positive("goal_half_width_m", goal_half_width_m)

    inward_sign = 1.0 if goal_x_m < 0 else -1.0   # +1 when the field is at larger x
    inward_heading = 0.0 if goal_x_m < 0 else math.pi
    deadline = time.monotonic() + timeout_s
    est = _VelocityEstimator()
    state = {"ball": None, "goals": 0, "in_net": False,
             "still_since": None, "clear_from": None, "stopped": False}

    def in_clear_zone(b):
        along = (b[0] - goal_x_m) * inward_sign
        return (CLEAR_MIN_M <= along <= CLEAR_MAX_M
                and abs(b[1] - goal_y_m) <= goal_half_width_m + CLEAR_SIDE_M)

    def step():
        if should_stop is not None and should_stop():
            state["stopped"] = True
            return _ZERO, True
        pose, ball_now = get_pose(), get_ball()
        now = time.monotonic()
        if pose is None:
            return None, False
        if ball_now is not None:
            est.update(ball_now, now)
            state["ball"] = ball_now
        b = state["ball"]
        vel = est.velocity()
        speed = math.hypot(*vel)

        if b is not None:
            in_net = ((b[0] - goal_x_m) * inward_sign < 0
                      and abs(b[1] - goal_y_m) <= goal_half_width_m)
            if in_net and not state["in_net"]:
                state["goals"] += 1
            state["in_net"] = in_net
            if clear_ball and not in_net and speed < CLEAR_STILL_MPS and in_clear_zone(b):
                state["still_since"] = state["still_since"] or now
                if now - state["still_since"] >= CLEAR_HOLD_S:
                    state["clear_from"] = b
                    state["still_since"] = None
                    return _ZERO, True
            else:
                state["still_since"] = None

        (tx, ty), mode = goalie_target(b, vel, goal_x_m, goal_y_m, goal_half_width_m,
                                       inward_sign)
        vx, vy = GOALIE_KP * (tx - pose[0]), GOALIE_KP * (ty - pose[1])
        norm = math.hypot(vx, vy)
        if norm > GOALIE_MAX_SPEED_MPS:
            vx, vy = vx * GOALIE_MAX_SPEED_MPS / norm, vy * GOALIE_MAX_SPEED_MPS / norm
        if b is None:
            face, gap = inward_heading, math.inf
        else:
            face = math.atan2(b[1] - pose[1], b[0] - pose[0])
            gap = math.hypot(b[0] - pose[0], b[1] - pose[1]) - ROBOT_RADIUS_M - BALL_RADIUS_M
        if mode == "shot" or gap < TURN_FREEZE_GAP_M:
            # Blocking a shot: all wheel speed goes to moving (turning would eat into it).
            # Ball at contact: the bearing swings wildly, so stop turning.
            w = 0.0
        else:
            w = max(-_b.MAX_ANGULAR_RADPS, min(
                _b.MAX_ANGULAR_RADPS, _b.KP_ANGULAR * _b.angle_error(face, pose[2])))
        return {"vx": vx, "vy": vy, "w": w}, False

    while time.monotonic() < deadline:
        state["clear_from"] = None
        _b._run(sock, endpoint, goalie_id, deadline, step, settle_cycles=1)
        if state["stopped"] or state["clear_from"] is None:
            break
        # A ball is resting in front of the goal: kick it out toward its own side.
        b = state["clear_from"]
        side = 1.0 if b[1] >= goal_y_m else -1.0
        kick_to(sock, endpoint, goalie_id, get_pose, get_ball,
                target_x_m=goal_x_m + inward_sign * CLEAR_TARGET_DIST_M,
                target_y_m=goal_y_m + side * CLEAR_TARGET_SIDE_M,
                timeout_s=max(0.1, min(CLEAR_TIMEOUT_S, deadline - time.monotonic())))

    return state["goals"] == 0