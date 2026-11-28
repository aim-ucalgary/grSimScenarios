"""Tactic: direct_shot_at_goal, a classical shot at an open goal.

Units: metres, seconds, radians. Field frame, counter-clockwise positive.

Looks at every enemy (including their goalie), finds which parts of the goal are
open as seen from the ball, and aims at the middle of the widest open gap (kept
clear of the posts). The shot itself is kick_to at full kick speed. The aim is
chosen once, from the enemy positions when the tactic starts.
"""

import math
import time

from helpers import behaviors as _b
from skills.kick_to import kick_to, MAX_AUTO_KICK_MPS

POST_MARGIN_M = 0.12          # keep the aim point this far inside the posts
ENEMY_BLOCK_RADIUS_M = 0.15   # robot radius (0.09) + ball radius (0.0215) + slack
AIM_STEP_M = 0.02             # resolution along the goal line
MIN_SHOT_DISTANCE_M = 0.3     # ball must be at least this far in front of the goal line


def _segment_distance(p, a, b):
    """Distance from point p to segment a-b."""
    abx, aby = b[0] - a[0], b[1] - a[1]
    length2 = abx * abx + aby * aby
    t = 0.0 if length2 == 0 else max(0.0, min(1.0, ((p[0] - a[0]) * abx + (p[1] - a[1]) * aby) / length2))
    return math.hypot(p[0] - (a[0] + t * abx), p[1] - (a[1] + t * aby))


def plan_goal_shot(ball_xy, goal_x, goal_y, half_width, enemies, *,
                   post_margin_m=POST_MARGIN_M, block_radius_m=ENEMY_BLOCK_RADIUS_M):
    """Choose where on the goal line to aim. Pure function (no robot commands).

    Inputs:
        ball_xy (tuple[float, float]): Ball position, metres.
        goal_x, goal_y (float): Goal-line X and goal-centre Y, metres.
        half_width (float): Half the goal width, metres.
        enemies (list[tuple]): Enemy positions; only the first two items (x, y) are used.

    Outputs:
        dict: aim_y (float), clear (bool: some lane is free), free_angle_rad (width
            of the chosen gap seen from the ball; 0 if none), clearance_m (gap
            between the ball's path and the nearest enemy along the chosen aim).

    Raises:
        ValueError: If the ball is less than MIN_SHOT_DISTANCE_M from the goal line
            or behind it.
    """
    dx = abs(goal_x - ball_xy[0])
    if dx < MIN_SHOT_DISTANCE_M:
        raise ValueError(f"ball is only {dx:.2f} m from the goal line")
    lo = goal_y - half_width + post_margin_m
    hi = goal_y + half_width - post_margin_m
    if lo >= hi:
        raise ValueError("goal too narrow for post_margin_m")
    enemy_xy = [(e[0], e[1]) for e in enemies]

    n = int((hi - lo) / AIM_STEP_M) + 1
    ys = [lo + (hi - lo) * k / max(n - 1, 1) for k in range(n)]

    def clearance(y):
        end = (goal_x, y)
        return min((_segment_distance(e, ball_xy, end) - block_radius_m for e in enemy_xy),
                   default=math.inf)

    free = [clearance(y) >= 0.0 for y in ys]

    def phi(y):  # angle of the lane, measured from the ball's straight-to-goal axis
        return math.atan((y - ball_xy[1]) / dx)

    # Widest run of free lanes, by angle.
    best, k = None, 0
    while k < n:
        if free[k]:
            j = k
            while j + 1 < n and free[j + 1]:
                j += 1
            width = phi(ys[j]) - phi(ys[k])
            if best is None or width > best[0]:
                best = (width, k, j)
            k = j + 1
        else:
            k += 1

    if best is not None:
        width, k, j = best
        aim_y = ball_xy[1] + dx * math.tan(0.5 * (phi(ys[k]) + phi(ys[j])))
        return {"aim_y": aim_y, "clear": True, "free_angle_rad": width,
                "clearance_m": clearance(aim_y)}
    # Nothing is open: aim where the nearest enemy is furthest from the path.
    aim_y = max(ys, key=clearance)
    return {"aim_y": aim_y, "clear": False, "free_angle_rad": 0.0,
            "clearance_m": clearance(aim_y)}


def direct_shot_at_goal(
    sock, endpoint, robot_id, get_pose, get_ball, get_enemies, *,
    goal_x_m, goal_y_m=0.0, goal_half_width_m=0.5,
    kick_speed_mps=None, require_clear=False, timeout_s=25.0,
):
    """Drive to the ball and shoot at the most open part of the goal.

    Inputs:
        sock (socket.socket): UDP command socket.
        endpoint (tuple[str, int]): Target team address tuple.
        robot_id (int): Non-negative robot ID of the shooter.
        get_pose (callable): Returns (x_m, y_m, heading_rad) or None.
        get_ball (callable): Returns (x_m, y_m) or None.
        get_enemies (callable): Returns a list of enemy positions, each a tuple
            whose first two items are (x_m, y_m). Include their goalie. May be empty.
        goal_x_m (float): X of the goal line being shot at, metres.
        goal_y_m (float, optional): Y of the goal centre. Default 0.0.
        goal_half_width_m (float, optional): Half the goal width, metres.
        kick_speed_mps (float, optional): Kick speed. Default is the maximum.
        require_clear (bool, optional): If True and no lane is open, return
            False without shooting.
        timeout_s (float, optional): Overall deadline in seconds. Default is 25.0s.

    Outputs:
        bool: True once the ball moved after the kick request, False otherwise.
            This confirms the kick happened, not that it scored.

    Raises:
        ValueError: On invalid arguments or a ball too close to the goal line.

    Example:
        >>> direct_shot_at_goal(sock, endpoint, 0, get_pose, get_ball, get_enemies,
        ...                     goal_x_m=4.5)
    """
    _b._check_positive("timeout_s", timeout_s)
    _b._check_positive("goal_half_width_m", goal_half_width_m)
    deadline = time.monotonic() + timeout_s

    # Wait (stationary) for the ball to be visible.
    ball = get_ball()
    while ball is None:
        if time.monotonic() >= deadline:
            return False
        time.sleep(_b.CONTROL_PERIOD_S)
        ball = get_ball()

    plan = plan_goal_shot(ball, goal_x_m, goal_y_m, goal_half_width_m,
                          list(get_enemies() or []))
    if require_clear and not plan["clear"]:
        return False

    return bool(kick_to(
        sock, endpoint, robot_id, get_pose, get_ball,
        target_x_m=goal_x_m, target_y_m=plan["aim_y"],
        kick_speed_mps=MAX_AUTO_KICK_MPS if kick_speed_mps is None else kick_speed_mps,
        timeout_s=max(0.1, deadline - time.monotonic())))