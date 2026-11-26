"""Tactic: pass the ball from one robot to another at a chosen field point.

Units: metres, seconds, radians. Field frame, counter-clockwise positive.

Both robots work at the same time, so the pass is as fast as the slower of the
two. The passer drives to the ball and kicks it straight at the target point
with a speed chosen so the ball still arrives there moving at a catchable speed
(a faster kick means a shorter flight). Meanwhile the receiver drives to the
spot where its dribbler face is exactly on the target point, turns toward the
incoming ball, and waits. As soon as the ball has been kicked, the receiver
switches to receive_ball, which intercepts and catches it. If the ball ends up
held by the dribbler, the ball's final position is the target point.

Note: "pass" is a Python keyword, so the function is called pass_ball.
"""

import math
import threading
import time
from concurrent.futures import ThreadPoolExecutor

from helpers import behaviors as _b
from skills.kick_to import kick_to, BALL_DECEL_MPS2, MAX_AUTO_KICK_MPS
from skills.go_to_point import go_to_point  # adjust to where go_to_point lives
from skills.receive_ball import receive_ball, CAPTURE_OFFSET_M

# Speed (m/s) the ball should still have when it reaches the receiver. Higher =
# faster pass but harder to catch. Tune live.
PASS_ARRIVAL_SPEED_MPS = 1.5
MIN_PASS_KICK_MPS = 1.0
MIN_PASS_DISTANCE_M = 0.5
# go_to_point can't be interrupted, so the receiver calls it in short chunks and
# checks for the kick in between. Smaller = reacts faster, but more start/stop.
POSITION_CHUNK_S = 0.4


def pass_kick_speed(distance_m, arrival_speed_mps):
    """Kick speed so a rolling ball covers distance_m and arrives at arrival_speed_mps.

    Uses the same uniform-deceleration model as skills/kick_to.py:
    v0^2 = v_arrival^2 + 2 * a * d.
    """
    v0 = math.sqrt(arrival_speed_mps ** 2 + 2.0 * BALL_DECEL_MPS2 * max(distance_m, 0.0))
    return max(MIN_PASS_KICK_MPS, min(MAX_AUTO_KICK_MPS, v0))


def pass_ball(
    sock, endpoint, passer_id, receiver_id,
    get_pose_passer, get_pose_receiver, get_ball, *,
    target_x_m, target_y_m,
    arrival_speed_mps=PASS_ARRIVAL_SPEED_MPS, timeout_s=25.0,
):
    """Pass the ball from passer to receiver so it ends at (target_x_m, target_y_m).

    Inputs:
        sock (socket.socket): UDP command socket.
        endpoint (tuple[str, int]): Team address tuple (both robots are on one team).
        passer_id (int): ID of the robot that kicks.
        receiver_id (int): ID of the robot that catches.
        get_pose_passer (callable): Returns (x_m, y_m, heading_rad) or None.
        get_pose_receiver (callable): Returns (x_m, y_m, heading_rad) or None.
        get_ball (callable): Returns (x_m, y_m) or None.
        target_x_m (float): X of the point where the ball should be received, metres.
        target_y_m (float): Y of the point where the ball should be received, metres.
        arrival_speed_mps (float, optional): Ball speed wanted at the receiver.
        timeout_s (float, optional): Overall deadline in seconds. Default is 25.0s.

    Outputs:
        bool: True if the ball was kicked and then caught by the receiver.

    Raises:
        ValueError: On invalid arguments, identical robot IDs, or a target too
            close to the ball.

    Example:
        >>> pass_ball(sock, endpoint, 0, 1, pose0, pose1, get_ball,
        ...           target_x_m=2.0, target_y_m=1.0)

    Thread safety: get_pose_*/get_ball are called from two threads at once, so
    they must be safe to call concurrently (e.g. read from a shared tracker that
    one thread updates, not from separate sockets competing for packets).
    """
    _b._check_positive("timeout_s", timeout_s)
    if passer_id == receiver_id:
        raise ValueError("passer_id and receiver_id must differ")
    if not math.isfinite(arrival_speed_mps) or arrival_speed_mps < 0:
        raise ValueError(f"arrival_speed_mps must be >= 0, got {arrival_speed_mps!r}")

    deadline = time.monotonic() + timeout_s

    # Wait (stationary) for the ball to be visible.
    ball = get_ball()
    while ball is None:
        if time.monotonic() >= deadline:
            return False
        time.sleep(_b.CONTROL_PERIOD_S)
        ball = get_ball()

    dist = math.hypot(target_x_m - ball[0], target_y_m - ball[1])
    if dist < MIN_PASS_DISTANCE_M:
        raise ValueError(f"target is only {dist:.2f} m from the ball (min {MIN_PASS_DISTANCE_M} m)")

    # Direction the ball will travel; receiver faces against it, with its centre
    # CAPTURE_OFFSET_M beyond the target so the ball centre lands on the target.
    ux, uy = (target_x_m - ball[0]) / dist, (target_y_m - ball[1]) / dist
    recv_x = target_x_m + CAPTURE_OFFSET_M * ux
    recv_y = target_y_m + CAPTURE_OFFSET_M * uy
    recv_heading = math.atan2(-uy, -ux)
    speed = pass_kick_speed(dist, arrival_speed_mps)

    kicked = threading.Event()
    failed = threading.Event()

    def passer_task():
        ok = False
        try:
            ok = kick_to(sock, endpoint, passer_id, get_pose_passer, get_ball,
                         target_x_m=target_x_m, target_y_m=target_y_m,
                         kick_speed_mps=speed,
                         timeout_s=max(0.1, deadline - time.monotonic()))
            return ok
        finally:
            (kicked if ok else failed).set()

    def receiver_task():
        # Phase 1: get into position (and hold it), facing the incoming ball,
        # until the ball is kicked. go_to_point blocks, so call it in short chunks.
        while not (kicked.is_set() or failed.is_set()):
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return False
            go_to_point(sock, endpoint, receiver_id, get_pose_receiver,
                        x_m=recv_x, y_m=recv_y, heading_rad=recv_heading,
                        timeout_s=min(POSITION_CHUNK_S, remaining))
        if failed.is_set():
            return False

        # Phase 2: the ball is on its way.
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            return False
        return receive_ball(sock, endpoint, receiver_id,
                            get_pose_receiver, get_ball, timeout_s=remaining)

    with ThreadPoolExecutor(max_workers=2) as pool:
        passer_future = pool.submit(passer_task)
        receiver_future = pool.submit(receiver_task)
        passed = passer_future.result()
        received = receiver_future.result()
    return bool(passed and received)