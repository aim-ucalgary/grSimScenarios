"""Tactic: pass the ball to a teammate so that moving enemy robots cannot intercept it.

Units: metres, seconds, radians. Field frame, counter-clockwise positive.

Like direct_pass, but the kick type and speed are chosen to beat the enemies.
The planner models every enemy conservatively: an enemy can run in any
direction at up to ENEMY_MAX_SPEED_MPS, so it can touch a point q at time t if
|q - enemy| - ENEMY_REACH_RADIUS_M <= speed * t. Because this covers every
direction, it stays valid however the enemies move during the pass.

Candidates are ground passes at several arrival speeds (faster = shorter
exposure) and, optionally, a chip (the ball is above robot height while in the
air, so it can only be touched near the kick and after it lands). A candidate is
"safe" if, at every moment the ball is low, every enemy would arrive later than
the ball by at least SAFETY_MARGIN_S. The fastest safe candidate is flown with
direct_pass; if none is safe, the candidate with the largest margin is used
(or the pass is refused with require_safe=True).

If no candidate is guaranteed safe (typical when an enemy sits near the pass
line: it could always just stand in the way), the planner falls back to a
prediction: each enemy keeps moving at its current velocity (measured by
sample_enemies) and the candidate that stays clear of those predicted positions
is chosen, fastest first. That is what makes it chip over a blocker instead of
shooting straight into it. If even that fails, the candidate with the most
clearance is used (or the pass is refused with require_safe=True).

The plan is made once, just before the pass starts; the passer's approach time
is included in the enemies' running time. The executed pass is direct_pass.
"""

import math
import time

from helpers import behaviors as _b
from skills.kick_to import BALL_DECEL_MPS2
from skills.chip_kick_to import estimate_chip_speed, CHIP_ANGLE_DEG, GRAVITY_MPS2
from tactics.direct_pass import (  # adjust to where direct_pass lives
    direct_pass, pass_kick_speed, MIN_PASS_DISTANCE_M,
    CHIP_LANDING_SHORT_M, MIN_CHIP_LANDING_M)

# Enemy model (worst case). Tune to the opponents / sim.
ENEMY_MAX_SPEED_MPS = 2.0
ENEMY_REACH_RADIUS_M = 0.15   # robot radius (0.09) + ball radius + slack
SAFETY_MARGIN_S = 0.15        # enemy must arrive at least this much after the ball

# Fallback prediction: enemies keep their current velocity (extrapolated at most
# ENEMY_PREDICT_MAX_S) and the ball must stay CLEARANCE_MARGIN_M clear of them.
ENEMY_PREDICT_MAX_S = 1.0
CLEARANCE_MARGIN_M = 0.15
ENEMY_VEL_SAMPLE_S = 0.15     # how long moving_pass watches enemies to get velocities
MIN_ENEMY_SPEED_MPS = 0.1     # slower than this counts as standing still
ENEMY_MATCH_M = 0.5           # max jump when matching an enemy between two samples

# Passer preparation time before the ball leaves (approach + alignment).
PASSER_SPEED_MPS = 1.5
PREP_OVERHEAD_S = 0.6

# Ground candidates: ball speed on arrival at the receiver.
ARRIVAL_SPEEDS_MPS = (0.5, 1.0, 1.5, 2.0, 2.5, 3.0)

# Chip model.
ROBOT_HEIGHT_M = 0.15
CHIP_CLEAR_HEIGHT_M = 0.20    # ball above this is out of reach of robots
CHIP_ROLL_SPEED_FACTOR = 0.5  # ground speed after landing / horizontal flight speed
MAX_CHIP_SPEED_MPS = 6.5

SAMPLE_STEP_M = 0.1
SAMPLE_STEP_S = 0.05


def _ground_samples(dist, arrival_speed, t_prep):
    """[(time_s, distance_along_path_m, is_low)] for a rolling pass, or None."""
    v0 = pass_kick_speed(dist, arrival_speed)
    if v0 * v0 < 2.0 * BALL_DECEL_MPS2 * dist:
        return None  # ball would stop short of the receiver
    n = max(2, int(dist / SAMPLE_STEP_M) + 1)
    samples = []
    for k in range(n):
        s = dist * k / (n - 1)
        t = (v0 - math.sqrt(max(v0 * v0 - 2.0 * BALL_DECEL_MPS2 * s, 0.0))) / BALL_DECEL_MPS2
        samples.append((t_prep + t, s, True))
    return samples


def _chip_samples(dist, t_prep):
    """[(time_s, distance_along_path_m, is_low)] for a chip pass, or None."""
    land_dist = dist - CHIP_LANDING_SHORT_M
    if land_dist < MIN_CHIP_LANDING_M:
        return None
    v = min(MAX_CHIP_SPEED_MPS, estimate_chip_speed(land_dist))
    theta = math.radians(CHIP_ANGLE_DEG)
    vh, vz = v * math.cos(theta), v * math.sin(theta)
    flight = 2.0 * vz / GRAVITY_MPS2
    landed_at = vh * flight
    if landed_at > dist:
        landed_at = dist  # (should not happen: speed is sized for land_dist)
    v_roll = vh * CHIP_ROLL_SPEED_FACTOR
    roll_dist = dist - landed_at
    if v_roll * v_roll < 2.0 * BALL_DECEL_MPS2 * roll_dist:
        return None  # ball would stop short after landing

    samples = []
    t = 0.0
    while t < flight:
        h = vz * t - 0.5 * GRAVITY_MPS2 * t * t
        samples.append((t_prep + t, vh * t, h < CHIP_CLEAR_HEIGHT_M))
        t += SAMPLE_STEP_S
    n = max(2, int(roll_dist / SAMPLE_STEP_M) + 1)
    for k in range(n):
        s = roll_dist * k / (n - 1)
        tr = (v_roll - math.sqrt(max(v_roll * v_roll - 2.0 * BALL_DECEL_MPS2 * s, 0.0))) \
            / BALL_DECEL_MPS2
        samples.append((t_prep + flight + tr, landed_at + s, True))
    return samples


def _margin(samples, ball_xy, u, enemies, enemy_speed):
    """Smallest (enemy arrival time - ball time) over low samples and enemies."""
    margin = math.inf
    for t, s, low in samples:
        if not low:
            continue
        qx, qy = ball_xy[0] + u[0] * s, ball_xy[1] + u[1] * s
        for ex, ey in enemies:
            reach = max(0.0, math.hypot(qx - ex, qy - ey) - ENEMY_REACH_RADIUS_M) / enemy_speed
            margin = min(margin, reach - t)
    return margin


def _clearance(samples, ball_xy, u, enemies):
    """Smallest gap between the low ball and any enemy moving at constant velocity."""
    gap = math.inf
    for t, s_along, low in samples:
        if not low:
            continue
        qx, qy = ball_xy[0] + u[0] * s_along, ball_xy[1] + u[1] * s_along
        tp = min(t, ENEMY_PREDICT_MAX_S)
        for ex, ey, evx, evy in enemies:
            d = math.hypot(qx - (ex + evx * tp), qy - (ey + evy * tp)) - ENEMY_REACH_RADIUS_M
            gap = min(gap, d)
    return gap


def sample_enemies(get_enemies, dt_s=ENEMY_VEL_SAMPLE_S, enemy_max_speed_mps=ENEMY_MAX_SPEED_MPS):
    """Read enemy positions twice, dt_s apart, and return [(x, y, vx, vy), ...]."""
    first, t1 = list(get_enemies() or []), time.monotonic()
    time.sleep(dt_s)
    second, t2 = list(get_enemies() or []), time.monotonic()
    out = []
    for e in second:
        vx = vy = 0.0
        near = min(first, key=lambda f: math.hypot(f[0] - e[0], f[1] - e[1]), default=None)
        if near is not None and math.hypot(near[0] - e[0], near[1] - e[1]) <= ENEMY_MATCH_M:
            vx, vy = (e[0] - near[0]) / (t2 - t1), (e[1] - near[1]) / (t2 - t1)
            speed = math.hypot(vx, vy)
            if speed < MIN_ENEMY_SPEED_MPS:
                vx = vy = 0.0
            elif speed > enemy_max_speed_mps:
                vx, vy = vx * enemy_max_speed_mps / speed, vy * enemy_max_speed_mps / speed
        out.append((e[0], e[1], vx, vy))
    return out


def plan_moving_pass(ball_xy, target_xy, enemies, *, t_prep_s=0.0, allow_chip=True,
                     enemy_max_speed_mps=ENEMY_MAX_SPEED_MPS):
    """Choose the pass type and speed. Pure function (no robot commands).

    Inputs:
        ball_xy (tuple[float, float]): Ball position, metres.
        target_xy (tuple[float, float]): Reception point, metres.
        enemies (list[tuple]): Enemies as (x, y) or (x, y, vx, vy). Without
            velocities, enemies are assumed to stand still in the fallback prediction.
        t_prep_s (float, optional): Seconds before the ball leaves the passer.
        allow_chip (bool, optional): Consider a chip pass.
        enemy_max_speed_mps (float, optional): Worst-case enemy speed.

    Outputs:
        dict: chip (bool), arrival_speed_mps (float, None for chips),
            completion_s (time until the ball reaches the target, from now),
            margin_s (worst-case enemy-arrival minus ball time; inf if no threat),
            clearance_m (smallest predicted gap to an enemy; inf if no threat),
            guaranteed (bool, safe against any enemy motion),
            safe (bool, guaranteed or clear of the predicted enemy motion).

    Raises:
        ValueError: If the target is closer than MIN_PASS_DISTANCE_M or infeasible.
    """
    dist = math.hypot(target_xy[0] - ball_xy[0], target_xy[1] - ball_xy[1])
    if dist < MIN_PASS_DISTANCE_M:
        raise ValueError(f"target is only {dist:.2f} m from the ball (min {MIN_PASS_DISTANCE_M} m)")
    u = ((target_xy[0] - ball_xy[0]) / dist, (target_xy[1] - ball_xy[1]) / dist)
    enemy_xy = [(e[0], e[1]) for e in enemies]
    enemy_xyv = [(e[0], e[1], e[2], e[3]) if len(e) >= 4 else (e[0], e[1], 0.0, 0.0)
                 for e in enemies]

    candidates = []
    for va in ARRIVAL_SPEEDS_MPS:
        samples = _ground_samples(dist, va, t_prep_s)
        if samples:
            candidates.append({"chip": False, "arrival_speed_mps": va, "samples": samples})
    if allow_chip:
        samples = _chip_samples(dist, t_prep_s)
        if samples:
            candidates.append({"chip": True, "arrival_speed_mps": None, "samples": samples})
    if not candidates:
        raise ValueError("no feasible pass to this target (too far for the kicker?)")

    for c in candidates:
        c["margin_s"] = _margin(c["samples"], ball_xy, u, enemy_xy, enemy_max_speed_mps)
        c["clearance_m"] = _clearance(c["samples"], ball_xy, u, enemy_xyv)
        c["completion_s"] = c["samples"][-1][0]
        c["guaranteed"] = c["margin_s"] >= SAFETY_MARGIN_S
        c["safe"] = c["guaranteed"] or c["clearance_m"] >= CLEARANCE_MARGIN_M

    guaranteed = [c for c in candidates if c["guaranteed"]]
    predicted = [c for c in candidates if c["safe"]]
    if guaranteed:
        best = min(guaranteed, key=lambda c: c["completion_s"])
    elif predicted:
        best = min(predicted, key=lambda c: c["completion_s"])
    else:
        best = max(candidates, key=lambda c: c["clearance_m"])
    return {k: best[k] for k in ("chip", "arrival_speed_mps", "completion_s", "margin_s",
                                 "clearance_m", "guaranteed", "safe")}


def moving_pass(
    sock, endpoint, passer_id, receiver_id,
    get_pose_passer, get_pose_receiver, get_ball, get_enemies, *,
    target_x_m=None, target_y_m=None, allow_chip=True, require_safe=True,
    enemy_max_speed_mps=ENEMY_MAX_SPEED_MPS, timeout_s=25.0,
):
    """Pass the ball so that moving enemy robots cannot intercept it.

    Inputs:
        sock (socket.socket): UDP command socket.
        endpoint (tuple[str, int]): Team address tuple (both robots are on one team).
        passer_id (int): ID of the robot that kicks.
        receiver_id (int): ID of the robot that catches.
        get_pose_passer (callable): Returns (x_m, y_m, heading_rad) or None.
        get_pose_receiver (callable): Returns (x_m, y_m, heading_rad) or None.
        get_ball (callable): Returns (x_m, y_m) or None.
        get_enemies (callable): Returns a list of enemy positions, each a tuple
            whose first two items are (x_m, y_m). May be empty.
        target_x_m (float, optional): X where the receiver catches the ball. If
            either target coordinate is None, the receiver's current position.
        target_y_m (float, optional): Y where the receiver catches the ball.
        allow_chip (bool, optional): Allow the planner to choose a chip pass.
        require_safe (bool, optional): If True and no pass is safe (neither
            guaranteed nor clear of the enemies' predicted motion), return False
            without moving the ball.
        enemy_max_speed_mps (float, optional): Worst-case enemy speed in m/s.
        timeout_s (float, optional): Overall deadline in seconds. Default is 25.0s.

    Outputs:
        bool: True if the ball was kicked and caught by the receiver.

    Raises:
        ValueError: On invalid arguments, identical robot IDs, or an infeasible target.

    Example:
        >>> moving_pass(sock, endpoint, 0, 1, pose0, pose1, get_ball, get_enemies,
        ...             target_x_m=2.0, target_y_m=1.0)

    Thread safety: the pose and ball getters are called from several threads at
    once (see direct_pass); get_enemies is called only from the calling thread.
    """
    _b._check_positive("timeout_s", timeout_s)
    _b._check_positive("enemy_max_speed_mps", enemy_max_speed_mps)
    if passer_id == receiver_id:
        raise ValueError("passer_id and receiver_id must differ")

    deadline = time.monotonic() + timeout_s

    # Wait (stationary) for the ball and both robots to be visible.
    while True:
        ball = get_ball()
        passer, receiver = get_pose_passer(), get_pose_receiver()
        if ball is not None and passer is not None and receiver is not None:
            break
        if time.monotonic() >= deadline:
            return False
        time.sleep(_b.CONTROL_PERIOD_S)

    if target_x_m is None or target_y_m is None:
        target_x_m, target_y_m = receiver[0], receiver[1]

    t_prep = math.hypot(passer[0] - ball[0], passer[1] - ball[1]) / PASSER_SPEED_MPS \
        + PREP_OVERHEAD_S
    enemies = sample_enemies(get_enemies, enemy_max_speed_mps=enemy_max_speed_mps)
    plan = plan_moving_pass(
        (ball[0], ball[1]), (target_x_m, target_y_m), enemies,
        t_prep_s=t_prep, allow_chip=allow_chip, enemy_max_speed_mps=enemy_max_speed_mps)
    if require_safe and not plan["safe"]:
        return False

    extra = {} if plan["chip"] else {"arrival_speed_mps": plan["arrival_speed_mps"]}
    return bool(direct_pass(
        sock, endpoint, passer_id, receiver_id,
        get_pose_passer, get_pose_receiver, get_ball,
        target_x_m=target_x_m, target_y_m=target_y_m, chip=plan["chip"],
        timeout_s=max(0.1, deadline - time.monotonic()), **extra))