"""Tactics: penalty_them (we defend) and penalty_us (we shoot).

Units: metres, seconds, radians. Field frame, counter-clockwise positive.

penalty_them: the goalie stands on the goal line and slides along it to block
(centred until the ball is kicked, then at the point where the ball will cross
the line). Every other robot waits 1 m behind the ball, on the far side from our
goal. Returns when the shot is over (ball crossed the line or stopped).

penalty_us: the shooter drives to the ball and lines up at the kicker face
facing the opponent goal, WITHOUT kicking. Every other robot waits 1 m behind
the ball, on the far side from the opponent goal, and keeps clear of the
shooter's approach lane. Returns when the shooter is ready; call kick_to or
chip_kick_to afterwards (for example on the referee's normal-start command).

"Behind the ball" is measured along the line from the ball to the goal being
shot at. Several waiting robots are spread sideways along the line 1 m behind
the ball. Which robot takes which spot is chosen to minimise the total driving
distance (optimal assignment), which also guarantees no two straight paths cross,
so the robots do not get in each other's way when lining up.

Robots are given as a dict {robot_id: get_pose} so each robot has its own pose
getter (the same style as the passing tactics).
"""

import math
import threading
import time
from concurrent.futures import ThreadPoolExecutor

from helpers import behaviors as _b
from skills.go_to_point import go_to_point  # adjust to where go_to_point lives
from skills.receive_ball import _VelocityEstimator

BEHIND_BALL_M = 1.0
ROBOT_SPACING_M = 0.4          # sideways gap between waiting robots
SHOOTER_LANE_M = 0.5           # penalty_us: keep waiting robots this far off the shot line
HOLD_CHUNK_S = 0.4             # go_to_point isn't interruptible: hold in short chunks

# Goalie (starting estimates, tune live).
ROBOT_RADIUS_M = 0.09
GOALIE_KP = 6.0
GOALIE_MAX_SPEED_MPS = 2.0
BALL_PREDICT_MIN_MPS = 0.5     # ball must be this fast along x to predict a crossing

# Shot-over detection for penalty_them.
KICK_DETECT_M = 0.10           # ball moved this far from the penalty mark = kicked
BALL_STOPPED_MPS = 0.2
BALL_STOPPED_HOLD_S = 0.3


def _check_ids(robot_id, other_ids, get_pose_by_id):
    ids = [robot_id] + list(other_ids)
    if len(set(ids)) != len(ids):
        raise ValueError(f"robot ids must all differ, got {ids!r}")
    missing = [i for i in ids if i not in get_pose_by_id]
    if missing:
        raise ValueError(f"get_pose_by_id has no entry for robot(s) {missing!r}")


def _behind_positions(ball, goal, n, lane_m=0.0):
    """Positions 1 m behind the ball (away from goal) for n robots, plus heading.

    Robots are spread sideways, ROBOT_SPACING_M apart. With lane_m > 0 nobody is
    placed within lane_m of the ball-goal line (alternating sides, nearest first);
    with lane_m == 0 they are centred on it.
    """
    dx, dy = goal[0] - ball[0], goal[1] - ball[1]
    dist = math.hypot(dx, dy)
    if dist < 0.1:
        raise ValueError("ball is on top of the goal")
    dx, dy = dx / dist, dy / dist
    px, py = -dy, dx  # sideways
    heading = math.atan2(dy, dx)
    positions = []
    for k in range(n):
        if lane_m > 0:
            side = 1 if k % 2 == 0 else -1
            offset = side * (lane_m + (k // 2) * ROBOT_SPACING_M)
        else:
            offset = (k - (n - 1) / 2.0) * ROBOT_SPACING_M
        positions.append((ball[0] - BEHIND_BALL_M * dx + offset * px,
                          ball[1] - BEHIND_BALL_M * dy + offset * py))
    return positions, heading


def _hungarian(cost):
    """Optimal assignment for a square cost matrix; returns col index per row.

    Minimising the total distance also means no two robots' straight paths cross
    (swapping a crossing pair is always strictly shorter).
    """
    n = len(cost)
    INF = float("inf")
    u, v = [0.0] * (n + 1), [0.0] * (n + 1)
    match = [0] * (n + 1)   # match[col] = row (1-based)
    way = [0] * (n + 1)
    for i in range(1, n + 1):
        match[0] = i
        j0 = 0
        minv = [INF] * (n + 1)
        used = [False] * (n + 1)
        while True:
            used[j0] = True
            i0, delta, j1 = match[j0], INF, 0
            for j in range(1, n + 1):
                if not used[j]:
                    cur = cost[i0 - 1][j - 1] - u[i0] - v[j]
                    if cur < minv[j]:
                        minv[j], way[j] = cur, j0
                    if minv[j] < delta:
                        delta, j1 = minv[j], j
            for j in range(n + 1):
                if used[j]:
                    u[match[j]] += delta
                    v[j] -= delta
                else:
                    minv[j] -= delta
            j0 = j1
            if match[j0] == 0:
                break
        while j0:
            j1 = way[j0]
            match[j0] = match[j1]
            j0 = j1
    result = [0] * n
    for j in range(1, n + 1):
        result[match[j] - 1] = j - 1
    return result


def _assign_slots(robot_ids, positions, get_pose_by_id):
    """Match robots to waiting spots by minimum total distance; returns spot per robot."""
    poses = []
    end = time.monotonic() + 0.5   # give vision a moment if a robot is not seen yet
    for rid in robot_ids:
        pose = get_pose_by_id[rid]()
        while pose is None and time.monotonic() < end:
            time.sleep(_b.CONTROL_PERIOD_S)
            pose = get_pose_by_id[rid]()
        poses.append(pose)
    # A robot we cannot see gets zero cost everywhere (it takes whatever is left).
    cost = [[0.0 if p is None else math.hypot(p[0] - x, p[1] - y) for (x, y) in positions]
            for p in poses]
    return [positions[c] for c in _hungarian(cost)]


def _hold(sock, endpoint, robot_id, get_pose, pos, heading, stop, deadline):
    """Drive to pos/heading and keep holding it until stop is set."""
    while not stop.is_set():
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            return False
        go_to_point(sock, endpoint, robot_id, get_pose,
                    x_m=pos[0], y_m=pos[1], heading_rad=heading,
                    timeout_s=min(HOLD_CHUNK_S, remaining))
    return True


def _wait_for_ball(get_ball, deadline):
    ball = get_ball()
    while ball is None:
        if time.monotonic() >= deadline:
            return None
        time.sleep(_b.CONTROL_PERIOD_S)
        ball = get_ball()
    return ball


def penalty_them(
    sock, endpoint, goalie_id, other_ids, get_pose_by_id, get_ball, *,
    goal_x_m, goal_y_m=0.0, goal_half_width_m=0.5, timeout_s=30.0,
):
    """Defend a penalty: goalie blocks on the goal line, the others wait behind the ball.

    Inputs:
        sock (socket.socket): UDP command socket.
        endpoint (tuple[str, int]): Team address tuple.
        goalie_id (int): Robot that defends the goal.
        other_ids (list[int]): Robots that wait 1 m behind the ball.
        get_pose_by_id (dict): {robot_id: get_pose}; each get_pose returns
            (x_m, y_m, heading_rad) or None. Must contain every robot used.
        get_ball (callable): Returns (x_m, y_m) or None.
        goal_x_m (float): X of our goal line, metres (the goal being defended).
        goal_y_m (float, optional): Y of the goal centre. Default 0.0.
        goal_half_width_m (float, optional): Half the goal width, metres. Division B
            goals are 1.0 m wide (0.5), Division A 1.8 m wide (0.9).
        timeout_s (float, optional): Overall deadline in seconds. Default is 30.0s.

    Outputs:
        bool: True if the shot happened and ended (ball crossed the goal line or
            came to rest), False on timeout.

    Raises:
        ValueError: On invalid arguments or duplicate robot IDs.

    Example:
        >>> penalty_them(sock, endpoint, 0, [1, 2], {0: pose0, 1: pose1, 2: pose2},
        ...              get_ball, goal_x_m=-4.5)
    """
    _b._check_positive("timeout_s", timeout_s)
    _b._check_positive("goal_half_width_m", goal_half_width_m)
    _check_ids(goalie_id, other_ids, get_pose_by_id)

    deadline = time.monotonic() + timeout_s
    ball0 = _wait_for_ball(get_ball, deadline)
    if ball0 is None:
        return False

    positions, hold_heading = _behind_positions(ball0, (goal_x_m, goal_y_m), len(other_ids))
    positions = _assign_slots(other_ids, positions, get_pose_by_id)
    inward_heading = 0.0 if goal_x_m < 0 else math.pi   # goalie faces into the field
    inward_sign = 1.0 if goal_x_m < 0 else -1.0         # +1 when the field is at larger x
    y_limit = max(0.0, goal_half_width_m - ROBOT_RADIUS_M)
    stop = threading.Event()
    goalie_est = _VelocityEstimator()
    get_goalie_pose = get_pose_by_id[goalie_id]

    def goalie_step():
        pose, ball_now = get_goalie_pose(), get_ball()
        if pose is None:
            return None, False
        goalie_est.update(ball_now, time.monotonic())
        vel = goalie_est.velocity()
        y_target = goal_y_m  # centred until the ball is kicked
        if ball_now is not None:
            to_line = goal_x_m - ball_now[0]
            if abs(vel[0]) >= BALL_PREDICT_MIN_MPS and to_line * vel[0] > 0:
                y_target = ball_now[1] + vel[1] * (to_line / vel[0])  # where it crosses
        y_target = max(goal_y_m - y_limit, min(goal_y_m + y_limit, y_target))

        vx = GOALIE_KP * (goal_x_m - pose[0])   # hold the goal line
        vy = GOALIE_KP * (y_target - pose[1])   # slide along it
        norm = math.hypot(vx, vy)
        if norm > GOALIE_MAX_SPEED_MPS:
            vx, vy = vx * GOALIE_MAX_SPEED_MPS / norm, vy * GOALIE_MAX_SPEED_MPS / norm
        w = max(-_b.MAX_ANGULAR_RADPS, min(
            _b.MAX_ANGULAR_RADPS,
            _b.KP_ANGULAR * _b.angle_error(inward_heading, pose[2])))
        return {"vx": vx, "vy": vy, "w": w}, stop.is_set()

    ended = False
    with ThreadPoolExecutor(max_workers=1 + len(other_ids)) as pool:
        futures = [pool.submit(_b._run, sock, endpoint, goalie_id, deadline,
                               goalie_step, settle_cycles=1)]
        for rid, pos in zip(other_ids, positions):
            futures.append(pool.submit(_hold, sock, endpoint, rid, get_pose_by_id[rid],
                                       pos, hold_heading, stop, deadline))
        try:
            # Watch the shot from this thread.
            est = _VelocityEstimator()
            kicked, slow_since = False, None
            while time.monotonic() < deadline:
                ball, now = get_ball(), time.monotonic()
                if ball is not None:
                    est.update(ball, now)
                    if not kicked and math.hypot(ball[0] - ball0[0],
                                                 ball[1] - ball0[1]) >= KICK_DETECT_M:
                        kicked = True
                    if kicked:
                        crossed = (ball[0] - goal_x_m) * inward_sign < 0
                        slow = math.hypot(*est.velocity()) < BALL_STOPPED_MPS
                        slow_since = (slow_since or now) if slow else None
                        if crossed or (slow_since and now - slow_since >= BALL_STOPPED_HOLD_S):
                            ended = True
                            break
                time.sleep(_b.CONTROL_PERIOD_S)
        finally:
            stop.set()
        for f in futures:
            f.result()  # join and surface any error
    return ended


def penalty_us(
    sock, endpoint, shooter_id, other_ids, get_pose_by_id, get_ball, *,
    goal_x_m, goal_y_m=0.0, timeout_s=20.0,
):
    """Prepare our penalty: shooter lines up at the ball (no kick), the others wait behind it.

    Inputs:
        sock (socket.socket): UDP command socket.
        endpoint (tuple[str, int]): Team address tuple.
        shooter_id (int): Robot that will take the penalty.
        other_ids (list[int]): Robots that wait 1 m behind the ball.
        get_pose_by_id (dict): {robot_id: get_pose}; each get_pose returns
            (x_m, y_m, heading_rad) or None. Must contain every robot used.
        get_ball (callable): Returns (x_m, y_m) or None.
        goal_x_m (float): X of the goal we shoot at, metres.
        goal_y_m (float, optional): Y of that goal's centre. Default 0.0.
        timeout_s (float, optional): Overall deadline in seconds. Default is 20.0s.

    Outputs:
        bool: True once the shooter is lined up at the kicker face, False on timeout.
            The robots stop when this returns; the ball has not been kicked.

    Raises:
        ValueError: On invalid arguments or duplicate robot IDs.

    Example:
        >>> penalty_us(sock, endpoint, 0, [1, 2], {0: pose0, 1: pose1, 2: pose2},
        ...            get_ball, goal_x_m=4.5)
    """
    _b._check_positive("timeout_s", timeout_s)
    _check_ids(shooter_id, other_ids, get_pose_by_id)

    deadline = time.monotonic() + timeout_s
    ball0 = _wait_for_ball(get_ball, deadline)
    if ball0 is None:
        return False

    positions, heading = _behind_positions(
        ball0, (goal_x_m, goal_y_m), len(other_ids), lane_m=SHOOTER_LANE_M)
    positions = _assign_slots(other_ids, positions, get_pose_by_id)
    stop = threading.Event()

    def shooter_task():
        try:
            return bool(_b.approach_ball(
                sock, endpoint, shooter_id, get_pose_by_id[shooter_id], get_ball,
                approach_heading_rad=heading,
                timeout_s=max(0.1, deadline - time.monotonic())))
        finally:
            stop.set()  # lets the others stop (also on failure or error)

    with ThreadPoolExecutor(max_workers=1 + len(other_ids)) as pool:
        shooter_future = pool.submit(shooter_task)
        others = [pool.submit(_hold, sock, endpoint, rid, get_pose_by_id[rid],
                              pos, heading, stop, deadline)
                  for rid, pos in zip(other_ids, positions)]
        ready = shooter_future.result()
        for f in others:
            f.result()
    return ready