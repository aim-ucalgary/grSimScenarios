"""Feedback-based robot behaviors.

Units: metres, seconds, radians. Field frame, counter-clockwise positive.

Feedback callables:
    get_pose() -> (x_m, y_m, heading_rad), or None when no fresh vision.
    get_ball() -> (x_m, y_m), or None when the ball is not visible.

Every behavior runs a blocking control loop at CONTROL_PERIOD_S, returns
True on success or False on timeout, and sends a zero command before it
returns (also on exceptions such as KeyboardInterrupt). While feedback is
missing, the robot is commanded to stop rather than continue blind.
"""

import math
import time

try:
    import ssl_simulation_robot_control_pb2 as control
except ImportError:
    control = None

from .helpers_communication import send_commands
from .helpers_geometry import angle_error, bearing_to, distance_to

# Control loop
CONTROL_PERIOD_S = 0.02
MAX_SPEED_MPS = 1.0
MAX_ANGULAR_RADPS = 4.0
KP_LINEAR = 2.5          # (m/s) per metre of error
KP_ANGULAR = 5.0         # (rad/s) per radian of error
SETTLE_CYCLES = 3        # consecutive in-tolerance samples to count as done

# Default tolerances
HEADING_TOLERANCE_RAD = 0.05
POSITION_TOLERANCE_M = 0.03

# Geometry (SSL robot / ball)
ROBOT_FRONT_M = 0.08     # robot centre to kicker face
BALL_RADIUS_M = 0.0215
STAGING_DIST_M = 0.30    # line up this far behind the ball
CLEARANCE_M = 0.25       # pass the ball at least this far away
CREEP_SPEED_MPS = 0.3    # final approach speed
CREEP_LATERAL_M = 0.05   # re-stage if the ball drifts this far off line

# Kicking
MAX_KICK_SPEED_MPS = 10.0
KICK_WINDOW_S = 1.0      # how long to keep requesting the kick
KICK_PUSH_MPS = 0.2      # forward speed while requesting the kick
BALL_MOVED_M = 0.10      # ball displacement that counts as kicked


# ---------------------------------------------------------------- utilities

def _check_positive(name, value):
    if not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
        raise ValueError(f"{name} must be a positive number, got {value!r}")


def _send(sock, endpoint, robot_id, vx=0.0, vy=0.0, w=0.0, *,
          kick_speed=0.0, kick_angle=0.0, dribbler_rpm=0.0):
    """Send one full command: global velocity plus kicker and dribbler."""
    if control is None:
        raise RuntimeError("ssl_simulation_robot_control_pb2 is not available")
    message = control.RobotControl()
    command = message.robot_commands.add(id=robot_id)
    velocity = command.move_command.global_velocity
    velocity.x = vx
    velocity.y = vy
    velocity.angular = w
    command.kick_speed = kick_speed
    command.kick_angle = kick_angle
    command.dribbler_speed = dribbler_rpm
    send_commands(sock, endpoint, message)


def _drive_command(pose, x, y, heading, *, max_speed=MAX_SPEED_MPS,
                   slow_down=True):
    """P-controller toward (x, y, heading).

    Returns (command kwargs, distance error, absolute heading error).
    With slow_down=False the robot keeps max_speed until it arrives, which
    suits intermediate waypoints.
    """
    px, py, ph = pose
    ex, ey = x - px, y - py
    dist = math.hypot(ex, ey)
    speed = min(KP_LINEAR * dist, max_speed) if slow_down else max_speed
    if dist > 1e-9:
        vx, vy = ex / dist * speed, ey / dist * speed
    else:
        vx = vy = 0.0
    heading_err = angle_error(heading, ph)
    w = max(-MAX_ANGULAR_RADPS, min(MAX_ANGULAR_RADPS, KP_ANGULAR * heading_err))
    return {"vx": vx, "vy": vy, "w": w}, dist, abs(heading_err)


def _run(sock, endpoint, robot_id, deadline, step, *,
         settle_cycles=SETTLE_CYCLES, stop=True):
    """Call step() every period until it reports success or the deadline."""
    settled = 0
    next_tick = time.monotonic()
    try:
        while time.monotonic() < deadline:
            command, ok = step()
            if command is None:
                _send(sock, endpoint, robot_id)
            else:
                _send(sock, endpoint, robot_id, **command)

            settled = settled + 1 if ok else 0
            if settled >= settle_cycles:
                result = True
                break

            next_tick += CONTROL_PERIOD_S
            delay = next_tick - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                next_tick = time.monotonic()  # fell behind; don't burst
        else:
            result = False
    except BaseException:
        _send(sock, endpoint, robot_id)
        raise
    if stop:
        _send(sock, endpoint, robot_id)
    return result


# ---------------------------------------------------------------- behaviors

def rotate_to(
    sock, endpoint, robot_id, get_pose, *,
    heading_rad, tolerance_rad=0.05, timeout_s=5.0,
):
    """Rotate to a target heading using feedback.

    Inputs:
        sock (socket.socket): UDP command socket.
        endpoint (tuple[str, int]): Target team address tuple.
        robot_id (int): Non-negative robot ID.
        get_pose (callable): Returns (x_m, y_m, heading_rad) or None.
        heading_rad (float): Target orientation in radians.
        tolerance_rad (float, optional): Acceptable error in radians. Default is 0.05 rad.
        timeout_s (float, optional): Maximum execution seconds. Default is 5.0s.

    Outputs:
        bool: True if heading reached within tolerance, False on timeout.

    Example:
        >>> rotate_to(sock, endpoint, 0, get_pose, heading_rad=math.pi / 2)
    """
    _check_positive("tolerance_rad", tolerance_rad)
    _check_positive("timeout_s", timeout_s)

    def step():
        pose = get_pose()
        if pose is None:
            return None, False
        err = angle_error(heading_rad, pose[2])
        w = max(-MAX_ANGULAR_RADPS, min(MAX_ANGULAR_RADPS, KP_ANGULAR * err))
        return {"w": w}, abs(err) <= tolerance_rad

    return _run(sock, endpoint, robot_id, time.monotonic() + timeout_s, step)


def face_point(
    sock, endpoint, robot_id, get_pose, *,
    x_m, y_m, timeout_s=5.0,
):
    """Rotate toward a field position.

    Inputs:
        sock (socket.socket): UDP command socket.
        endpoint (tuple[str, int]): Target team address tuple.
        robot_id (int): Non-negative robot ID.
        get_pose (callable): Returns (x_m, y_m, heading_rad) or None.
        x_m (float): Target X coordinate in metres.
        y_m (float): Target Y coordinate in metres.
        timeout_s (float, optional): Maximum execution seconds. Default is 5.0s.

    Outputs:
        bool: True if aimed within HEADING_TOLERANCE_RAD, False on timeout.

    Example:
        >>> face_point(sock, endpoint, 0, get_pose, x_m=0.0, y_m=0.0)
    """
    _check_positive("timeout_s", timeout_s)

    def step():
        pose = get_pose()
        if pose is None:
            return None, False
        px, py, ph = pose
        if math.hypot(x_m - px, y_m - py) < 1e-3:
            return None, True  # standing on the point: any heading faces it
        err = angle_error(bearing_to((px, py), (x_m, y_m)), ph)
        w = max(-MAX_ANGULAR_RADPS, min(MAX_ANGULAR_RADPS, KP_ANGULAR * err))
        return {"w": w}, abs(err) <= HEADING_TOLERANCE_RAD

    return _run(sock, endpoint, robot_id, time.monotonic() + timeout_s, step)


def hold_pose(
    sock, endpoint, robot_id, get_pose, *,
    x_m, y_m, heading_rad, duration_s,
):
    """Maintain a pose for a bounded duration.

    Inputs:
        sock (socket.socket): UDP command socket.
        endpoint (tuple[str, int]): Target team address tuple.
        robot_id (int): Non-negative robot ID.
        get_pose (callable): Returns (x_m, y_m, heading_rad) or None.
        x_m (float): Target X position in metres.
        y_m (float): Target Y position in metres.
        heading_rad (float): Target orientation in radians.
        duration_s (float): Seconds to maintain position against disturbances.

    Outputs:
        bool: True if within tolerance at the end of the duration.

    Example:
        >>> hold_pose(sock, endpoint, 0, get_pose, x_m=1.0, y_m=0.0, heading_rad=0.0, duration_s=2.0)
    """
    _check_positive("duration_s", duration_s)
    in_tolerance = False

    def step():
        nonlocal in_tolerance
        pose = get_pose()
        if pose is None:
            in_tolerance = False
            return None, False
        command, dist, heading_err = _drive_command(pose, x_m, y_m, heading_rad)
        in_tolerance = (dist <= POSITION_TOLERANCE_M
                        and heading_err <= HEADING_TOLERANCE_RAD)
        return command, False  # never finish early

    _run(sock, endpoint, robot_id, time.monotonic() + duration_s, step)
    return in_tolerance


def follow_waypoints(
    sock, endpoint, robot_id, get_pose, *,
    waypoints, tolerance_m=0.05, timeout_s=30.0,
):
    """Visit (x, y) targets within an overall deadline.

    Inputs:
        sock (socket.socket): UDP command socket.
        endpoint (tuple[str, int]): Target team address tuple.
        robot_id (int): Non-negative robot ID.
        get_pose (callable): Returns (x_m, y_m, heading_rad) or None.
        waypoints (list[tuple[float, float]]): Series of (x, y) target coordinates.
        tolerance_m (float, optional): Distance threshold for waypoints. Default is 0.05m.
        timeout_s (float, optional): Overall deadline in seconds. Default is 30.0s.

    Outputs:
        bool: True if all waypoints were reached before expiry.

    Example:
        >>> pts = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)]
        >>> follow_waypoints(sock, endpoint, 0, get_pose, waypoints=pts)
    """
    waypoints = [(float(x), float(y)) for x, y in waypoints]
    if not waypoints:
        raise ValueError("waypoints must not be empty")
    _check_positive("tolerance_m", tolerance_m)
    _check_positive("timeout_s", timeout_s)

    deadline = time.monotonic() + timeout_s
    held_heading = None

    for index, (wx, wy) in enumerate(waypoints):
        last = index == len(waypoints) - 1

        def step(wx=wx, wy=wy, last=last):
            nonlocal held_heading
            pose = get_pose()
            if pose is None:
                return None, False
            if held_heading is None:
                held_heading = pose[2]
            command, dist, _ = _drive_command(
                pose, wx, wy, held_heading, slow_down=last)
            return command, dist <= tolerance_m

        reached = _run(
            sock, endpoint, robot_id, deadline, step,
            settle_cycles=SETTLE_CYCLES if last else 1, stop=last)
        if not reached:
            _send(sock, endpoint, robot_id)
            return False
    return True


def _segment_distance(px, py, ax, ay, bx, by):
    """Distance from point P to segment AB."""
    abx, aby = bx - ax, by - ay
    length_sq = abx * abx + aby * aby
    t = 0.0 if length_sq == 0 else max(
        0.0, min(1.0, ((px - ax) * abx + (py - ay) * aby) / length_sq))
    return math.hypot(px - (ax + t * abx), py - (ay + t * aby))


def approach_ball(
    sock, endpoint, robot_id, get_pose, get_ball, *,
    approach_heading_rad, timeout_s=10.0,
):
    """Approach the ball from a specified direction.

    Inputs:
        sock (socket.socket): UDP command socket.
        endpoint (tuple[str, int]): Target team address tuple.
        robot_id (int): Non-negative robot ID.
        get_pose (callable): Returns (x_m, y_m, heading_rad) or None.
        get_ball (callable): Returns (x_m, y_m) or None.
        approach_heading_rad (float): Direction the robot faces and travels when contacting the ball.
        timeout_s (float, optional): Maximum execution seconds. Default is 10.0s.

    Outputs:
        bool: True if robot positioned at the ball kicker face, False on timeout.

    Example:
        >>> approach_ball(sock, endpoint, 0, get_pose, get_ball, approach_heading_rad=0.0)
    """
    _check_positive("timeout_s", timeout_s)
    dx, dy = math.cos(approach_heading_rad), math.sin(approach_heading_rad)
    nx, ny = -dy, dx  # left of the approach direction
    contact_offset = ROBOT_FRONT_M + BALL_RADIUS_M
    phase = "stage"

    def step():
        nonlocal phase
        pose, ball = get_pose(), get_ball()
        if pose is None or ball is None:
            return None, False
        px, py, _ = pose
        bx, by = ball
        lateral = (px - bx) * nx + (py - by) * ny

        if phase == "creep" and abs(lateral) > CREEP_LATERAL_M:
            phase = "stage"  # ball moved off line; line up again

        if phase == "stage":
            sx, sy = bx - dx * STAGING_DIST_M, by - dy * STAGING_DIST_M
            tx, ty = sx, sy
            if _segment_distance(bx, by, px, py, sx, sy) < 0.8 * CLEARANCE_M:
                side = 1.0 if lateral >= 0 else -1.0
                tx = bx + side * nx * CLEARANCE_M - dx * 0.5 * CLEARANCE_M
                ty = by + side * ny * CLEARANCE_M - dy * 0.5 * CLEARANCE_M
            command, dist, heading_err = _drive_command(
                pose, tx, ty, approach_heading_rad, slow_down=(tx, ty) == (sx, sy))
            if ((tx, ty) == (sx, sy) and dist <= 0.05
                    and heading_err <= 2 * HEADING_TOLERANCE_RAD):
                phase = "creep"
            return command, False

        cx, cy = bx - dx * contact_offset, by - dy * contact_offset
        command, dist, heading_err = _drive_command(
            pose, cx, cy, approach_heading_rad, max_speed=CREEP_SPEED_MPS)
        return command, (dist <= POSITION_TOLERANCE_M
                         and heading_err <= HEADING_TOLERANCE_RAD)

    return _run(sock, endpoint, robot_id, time.monotonic() + timeout_s, step,
                settle_cycles=2)


def pass_to(
    sock, endpoint, robot_id, get_pose, get_ball, *,
    target_x_m, target_y_m, kick_speed_mps, timeout_s=10.0,
):
    """Coordinate positioning, aiming, and a kick request.

    Inputs:
        sock (socket.socket): UDP command socket.
        endpoint (tuple[str, int]): Target team address tuple.
        robot_id (int): Non-negative robot ID.
        get_pose (callable): Returns (x_m, y_m, heading_rad) or None.
        get_ball (callable): Returns (x_m, y_m) or None.
        target_x_m (float): Destination X coordinate in metres.
        target_y_m (float): Destination Y coordinate in metres.
        kick_speed_mps (float): Kick velocity in (0, 10.0] m/s.
        timeout_s (float, optional): Maximum execution seconds. Default is 10.0s.

    Outputs:
        bool: True once ball displacement confirms a kick, False on timeout.

    Example:
        >>> pass_to(sock, endpoint, 0, get_pose, get_ball,
        ...         target_x_m=2.0, target_y_m=0.0, kick_speed_mps=4.0)
    """
    if not math.isfinite(kick_speed_mps) or not 0 < kick_speed_mps <= MAX_KICK_SPEED_MPS:
        raise ValueError(
            f"kick_speed_mps must be in (0, {MAX_KICK_SPEED_MPS}], got {kick_speed_mps!r}")
    _check_positive("timeout_s", timeout_s)
    deadline = time.monotonic() + timeout_s

    ball = get_ball()
    if ball is None:
        return False
    if math.hypot(target_x_m - ball[0], target_y_m - ball[1]) < 0.1:
        raise ValueError("target is on top of the ball")
    heading = math.atan2(target_y_m - ball[1], target_x_m - ball[0])

    if not approach_ball(sock, endpoint, robot_id, get_pose, get_ball,
                         approach_heading_rad=heading,
                         timeout_s=deadline - time.monotonic()):
        return False

    start_ball = get_ball()
    if start_ball is None:
        return False
    kick_deadline = min(deadline, time.monotonic() + KICK_WINDOW_S)

    def step():
        pose, ball = get_pose(), get_ball()
        if pose is None:
            return None, False
        w = max(-MAX_ANGULAR_RADPS, min(MAX_ANGULAR_RADPS, KP_ANGULAR * angle_error(heading, pose[2])))
        command = {
            "vx": KICK_PUSH_MPS * math.cos(heading),
            "vy": KICK_PUSH_MPS * math.sin(heading),
            "w": w,
            "kick_speed": kick_speed_mps,
        }
        moved = ball is not None and math.hypot(
            ball[0] - start_ball[0], ball[1] - start_ball[1]) >= BALL_MOVED_M
        return command, moved

    return _run(sock, endpoint, robot_id, kick_deadline, step, settle_cycles=1)
