"""Tactic: pass the ball through an intermediate robot to a final receiver.

Units: metres, seconds, radians. Field frame, counter-clockwise positive.

Leg 1: passer -> middle robot, received at the via point (direct_pass).
Leg 2: middle robot -> receiver, received at the target point (direct_pass).
Either leg can be chipped (chip_leg1 / chip_leg2).

To keep the whole thing fast, the final receiver does not wait for leg 1 to
finish: while leg 1 is in flight it already drives to, and holds, the spot where
its dribbler face is on the target point, facing the via point. Leg 2 then only
has to kick and catch.
"""

import math
import time
from concurrent.futures import ThreadPoolExecutor
import threading

from helpers import behaviors as _b
from skills.go_to_point import go_to_point  # adjust to where go_to_point lives
from skills.receive_ball import CAPTURE_OFFSET_M
from tactics.direct_pass import (  # adjust to where direct_pass lives
    direct_pass, CHIP_LANDING_SHORT_M, MIN_CHIP_LANDING_M)

MIN_LEG_DISTANCE_M = 0.5   # same minimum as direct_pass
POSITION_CHUNK_S = 0.4     # go_to_point is not interruptible: call it in short chunks


def indirect_pass(
    sock, endpoint, passer_id, middle_id, receiver_id,
    get_pose_passer, get_pose_middle, get_pose_receiver, get_ball, *,
    target_x_m, target_y_m, via_x_m=None, via_y_m=None,
    arrival_speed_mps=None, chip_leg1=False, chip_leg2=False, timeout_s=40.0,
):
    """Pass passer -> middle -> receiver so the ball ends at (target_x_m, target_y_m).

    Inputs:
        sock (socket.socket): UDP command socket.
        endpoint (tuple[str, int]): Team address tuple (all robots on one team).
        passer_id (int): ID of the robot that starts with the ball.
        middle_id (int): ID of the intermediate robot (catches, then kicks).
        receiver_id (int): ID of the final receiver.
        get_pose_passer (callable): Returns (x_m, y_m, heading_rad) or None.
        get_pose_middle (callable): Returns (x_m, y_m, heading_rad) or None.
        get_pose_receiver (callable): Returns (x_m, y_m, heading_rad) or None.
        get_ball (callable): Returns (x_m, y_m) or None.
        target_x_m (float): X of the final reception point, metres.
        target_y_m (float): Y of the final reception point, metres.
        via_x_m (float, optional): X where the middle robot receives the ball.
        via_y_m (float, optional): Y where the middle robot receives the ball.
            If either via coordinate is None, the midpoint between the ball and
            the target is used.
        arrival_speed_mps (float, optional): Ball speed wanted at each receiver.
            If None, direct_pass's default is used.
        chip_leg1 (bool, optional): Chip the passer -> middle pass.
        chip_leg2 (bool, optional): Chip the middle -> receiver pass.
        timeout_s (float, optional): Overall deadline in seconds. Default is 40.0s.

    Outputs:
        bool: True if both legs completed.

    Raises:
        ValueError: On invalid arguments, duplicate robot IDs, or a leg shorter
            than MIN_LEG_DISTANCE_M.

    Example:
        >>> indirect_pass(sock, endpoint, 0, 1, 2, pose0, pose1, pose2, get_ball,
        ...               target_x_m=2.0, target_y_m=1.0, via_x_m=0.5, via_y_m=-1.0)

    Thread safety: the pose and ball getters are called from several threads at
    once (see direct_pass), so they must be safe to call concurrently.
    """
    _b._check_positive("timeout_s", timeout_s)
    if len({passer_id, middle_id, receiver_id}) != 3:
        raise ValueError("passer_id, middle_id and receiver_id must all differ")

    deadline = time.monotonic() + timeout_s

    # Wait (stationary) for the ball to be visible.
    ball = get_ball()
    while ball is None:
        if time.monotonic() >= deadline:
            return False
        time.sleep(_b.CONTROL_PERIOD_S)
        ball = get_ball()

    if via_x_m is None or via_y_m is None:
        via_x_m = 0.5 * (ball[0] + target_x_m)
        via_y_m = 0.5 * (ball[1] + target_y_m)

    leg1 = math.hypot(via_x_m - ball[0], via_y_m - ball[1])
    leg2 = math.hypot(target_x_m - via_x_m, target_y_m - via_y_m)
    chip_min = CHIP_LANDING_SHORT_M + MIN_CHIP_LANDING_M
    min1 = chip_min if chip_leg1 else MIN_LEG_DISTANCE_M
    min2 = chip_min if chip_leg2 else MIN_LEG_DISTANCE_M
    if leg1 < min1 or leg2 < min2:
        raise ValueError(
            f"legs are {leg1:.2f} m and {leg2:.2f} m; need >= {min1:.1f} m and >= {min2:.1f} m "
            f"(chip legs need >= {chip_min:.1f} m)")

    # Where the final receiver should wait: dribbler face on the target, facing
    # the via point (where the ball will be kicked from).
    ux, uy = (target_x_m - via_x_m) / leg2, (target_y_m - via_y_m) / leg2
    recv_x = target_x_m + CAPTURE_OFFSET_M * ux
    recv_y = target_y_m + CAPTURE_OFFSET_M * uy
    recv_heading = math.atan2(-uy, -ux)

    extra = {} if arrival_speed_mps is None else {"arrival_speed_mps": arrival_speed_mps}
    leg1_done = threading.Event()

    def prepositioner():
        while not leg1_done.is_set():
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return
            go_to_point(sock, endpoint, receiver_id, get_pose_receiver,
                        x_m=recv_x, y_m=recv_y, heading_rad=recv_heading,
                        timeout_s=min(POSITION_CHUNK_S, remaining))

    with ThreadPoolExecutor(max_workers=1) as pool:
        prep_future = pool.submit(prepositioner)
        try:
            ok1 = direct_pass(
                sock, endpoint, passer_id, middle_id,
                get_pose_passer, get_pose_middle, get_ball,
                target_x_m=via_x_m, target_y_m=via_y_m,
                chip=chip_leg1, timeout_s=max(0.1, deadline - time.monotonic()), **extra)
        finally:
            leg1_done.set()
        prep_future.result()  # join (at most one chunk) and surface any error

    if not ok1:
        return False

    remaining = deadline - time.monotonic()
    if remaining <= 0:
        return False
    # Leg 2: the middle robot is holding the ball; the receiver is already in place.
    return bool(direct_pass(
        sock, endpoint, middle_id, receiver_id,
        get_pose_middle, get_pose_receiver, get_ball,
        target_x_m=target_x_m, target_y_m=target_y_m,
        chip=chip_leg2, timeout_s=remaining, **extra))