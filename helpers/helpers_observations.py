"""Vision observations and freshness checks.

Normalize positions to metres and headings to radians.
Never substitute a zero pose for missing observations.
"""

import math
import time

from .helpers_geometry import angle_error

try:
    import messages_robocup_ssl_wrapper_pb2 as ssl_wrapper
except ImportError:
    import ssl_vision_wrapper_pb2 as ssl_wrapper


def _check_robot_id(robot_id):
    if isinstance(robot_id, bool) or not isinstance(robot_id, int) or robot_id < 0:
        raise ValueError(f"robot_id must be a non-negative int, got {robot_id!r}")


def _check_positive(name, value):
    if not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
        raise ValueError(f"{name} must be a positive number, got {value!r}")


def _get_timestamp(world_state):
    if isinstance(world_state, dict):
        return (
            world_state.get("last_received_monotonic_s")
            or world_state.get("timestamp")
            or world_state.get("time")
        )
    detection = getattr(world_state, "detection", world_state)
    return getattr(world_state, "last_received_monotonic_s", None) or getattr(
        detection, "t_capture", None
    )


def read_vision(sock, *, timeout_s=1.0):
    """Receive and decode a vision packet from a configured socket.

    Inputs:
        sock (socket.socket): UDP socket bound to vision multicast/unicast port (e.g. 10020).
        timeout_s (float, optional): Maximum seconds to wait for a packet. Default is 1.0s.

    Outputs:
        SSL_WrapperPacket: Decoded protobuf vision packet containing detection/geometry frames.

    Raises:
        TimeoutError: If no datagram is received within timeout_s.
        RuntimeError: If the vision protobuf bindings are missing.

    Example:
        >>> packet = read_vision(vision_sock, timeout_s=0.5)
    """
    _check_positive("timeout_s", timeout_s)
    sock.settimeout(float(timeout_s))
    data, _ = sock.recvfrom(65535)
    packet = ssl_wrapper.SSL_WrapperPacket()
    packet.ParseFromString(data)
    return packet


def get_robot_pose(world_state, team, robot_id, *, max_age_s=0.5):
    """Return a fresh robot pose or raise if unavailable.

    Normalizes field positions from millimetres to metres and headings
    to [-pi, pi] radians. Never substitutes (0, 0, 0) for missing robots.

    Inputs:
        world_state (SSL_WrapperPacket | dict): Current vision packet or tracking dictionary.
        team (str): 'blue' or 'yellow'.
        robot_id (int): Non-negative robot ID.
        max_age_s (float | None, optional): Maximum allowed age in seconds. Default is 0.5s.

    Outputs:
        tuple[float, float, float]: (x_m, y_m, heading_rad) in metres and radians.

    Raises:
        KeyError: If the robot is not detected in the observation.
        TimeoutError: If the observation timestamp is older than max_age_s.

    Example:
        >>> x, y, heading = get_robot_pose(world_state, "blue", 0)
        >>> print(f"Robot 0 at ({x:.2f}m, {y:.2f}m), heading {heading:.2f}rad")
    """
    if world_state is None:
        raise ValueError("world_state cannot be None")
    _check_robot_id(robot_id)

    team_str = str(team).strip().lower()
    if team_str in ("blue", "b"):
        team_name = "blue"
    elif team_str in ("yellow", "y"):
        team_name = "yellow"
    else:
        raise ValueError(f"team must be 'blue' or 'yellow', got {team!r}")

    if max_age_s is not None:
        require_fresh_state(_get_timestamp(world_state), max_age_s=max_age_s)

    # Protobuf packet or detection frame
    detection = getattr(world_state, "detection", world_state)
    robots = getattr(detection, f"robots_{team_name}", None)
    if robots is not None:
        candidates = [r for r in robots if getattr(r, "robot_id", None) == robot_id]
        if candidates:
            r = max(candidates, key=lambda c: getattr(c, "confidence", 1.0))
            return (r.x / 1000.0, r.y / 1000.0, angle_error(getattr(r, "orientation", 0.0), 0.0))
        raise KeyError(f"Robot {robot_id} not found on team {team_name}")

    # Dict world state
    if isinstance(world_state, dict) and team_name in world_state:
        team_robots = world_state[team_name]
        if robot_id in team_robots:
            val = team_robots[robot_id]
            if isinstance(val, dict):
                x, y = val["x"], val["y"]
                h = val.get("orientation", val.get("heading", 0.0))
            else:
                x, y = val[0], val[1]
                h = val[2] if len(val) > 2 else 0.0
            if abs(x) > 25.0 or abs(y) > 25.0:
                x, y = x / 1000.0, y / 1000.0
            return (x, y, angle_error(h, 0.0))

    raise KeyError(f"Robot {robot_id} not found on team {team_name}")


def get_ball_state(world_state, *, max_age_s=0.5):
    """Return a fresh ball estimate or raise if unavailable.

    Normalizes coordinates from millimetres to metres. Never returns (0, 0)
    for missing ball observations.

    Inputs:
        world_state (SSL_WrapperPacket | dict): Current vision packet or tracking dictionary.
        max_age_s (float | None, optional): Maximum allowed age in seconds. Default is 0.5s.

    Outputs:
        tuple[float, float]: (x_m, y_m) coordinates in metres.

    Raises:
        KeyError: If the ball is not detected in the observation.
        TimeoutError: If the observation timestamp is older than max_age_s.

    Example:
        >>> bx, by = get_ball_state(world_state)
        >>> print(f"Ball is at ({bx:.2f}m, {by:.2f}m)")
    """
    if world_state is None:
        raise ValueError("world_state cannot be None")

    if max_age_s is not None:
        require_fresh_state(_get_timestamp(world_state), max_age_s=max_age_s)

    # Protobuf packet or detection frame
    detection = getattr(world_state, "detection", world_state)
    balls = getattr(detection, "balls", None)
    if balls:
        ball = max(balls, key=lambda b: getattr(b, "confidence", 1.0))
        return (ball.x / 1000.0, ball.y / 1000.0)

    # Dict world state
    if isinstance(world_state, dict) and "ball" in world_state:
        val = world_state["ball"]
        if isinstance(val, dict):
            x, y = val["x"], val["y"]
        else:
            x, y = val[0], val[1]
        if abs(x) > 25.0 or abs(y) > 25.0:
            x, y = x / 1000.0, y / 1000.0
        return (x, y)

    raise KeyError("Ball not found in observation")


def require_fresh_state(last_received_monotonic_s, *, max_age_s=0.5):
    """Raise if the observation is missing or too old.

    Inputs:
        last_received_monotonic_s (float | None): time.monotonic() timestamp of reception.
        max_age_s (float): Age threshold in seconds. Default is 0.5s.

    Outputs:
        None

    Raises:
        TimeoutError: If timestamp is None or (now - timestamp) > max_age_s.

    Example:
        >>> require_fresh_state(time.monotonic(), max_age_s=0.5)  # passes
        >>> require_fresh_state(time.monotonic() - 1.0, max_age_s=0.5)  # raises TimeoutError
    """
    if last_received_monotonic_s is None:
        raise TimeoutError("Observation is missing")
    _check_positive("max_age_s", max_age_s)
    age = time.monotonic() - float(last_received_monotonic_s)
    if age > max_age_s:
        raise TimeoutError(f"Observation is too old ({age:.3f}s > {max_age_s}s)")


def estimate_velocity(previous_position, current_position, *, dt_s):
    """Return velocity from positions in metres and a positive interval.

    Inputs:
        previous_position (tuple | float): Starting position (x, y), (x, y, z), or scalar in metres.
        current_position (tuple | float): Final position in metres.
        dt_s (float): Positive elapsed time interval in seconds.

    Outputs:
        tuple[float, ...] | float: Velocity vector (or scalar) in metres per second.

    Example:
        >>> estimate_velocity((1.0, 2.0), (2.0, 4.0), dt_s=0.5)
        (2.0, 4.0)
    """
    _check_positive("dt_s", dt_s)
    dt_s = float(dt_s)
    if isinstance(previous_position, (int, float)) and isinstance(
        current_position, (int, float)
    ):
        return (float(current_position) - float(previous_position)) / dt_s
    return tuple(
        (float(c) - float(p)) / dt_s for p, c in zip(previous_position, current_position)
    )
