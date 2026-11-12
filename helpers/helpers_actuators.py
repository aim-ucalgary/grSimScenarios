"""Kick and dribbler commands.

Kick speed: m/s. Kick elevation: degrees.
"""

import math

import ssl_simulation_robot_control_pb2 as control

from .helpers_communication import send_commands

# grSim treats any positive dribbler speed as "on"; the value is in rpm.
DRIBBLER_ON_RPM = 1000.0

# Above this, grSim kicks are unrealistic (SSL rules cap shots at 6.5 m/s).
MAX_KICK_SPEED_MPS = 10.0
MAX_KICK_ANGLE_DEG = 90.0


def _send(sock, endpoint, robot_id, *, kick_speed=0.0, kick_angle=0.0,
          dribbler_speed=0.0):
    """Build a single-robot RobotControl message and send it as one datagram."""
    if isinstance(robot_id, bool) or not isinstance(robot_id, int) or robot_id < 0:
        raise ValueError(f"robot_id must be a non-negative int, got {robot_id!r}")
    message = control.RobotControl()
    command = message.robot_commands.add(id=robot_id)
    command.kick_speed = kick_speed
    command.kick_angle = kick_angle
    command.dribbler_speed = dribbler_speed
    send_commands(sock, endpoint, message)


def kick(sock, endpoint, robot_id, *, speed_mps, angle_deg=0.0):
    """Send one kick request; delivery and ball contact are not guaranteed.

    Inputs:
        sock (socket.socket): UDP socket for command transmission.
        endpoint (tuple[str, int]): Target (host, port) tuple (e.g. ("127.0.0.1", 10301)).
        robot_id (int): Non-negative robot ID.
        speed_mps (float): Desired ball speed in m/s (0 < speed_mps <= 10.0).
        angle_deg (float, optional): Kick elevation angle in degrees (0.0 <= angle_deg <= 90.0).
            Default is 0.0 (flat kick).

    Outputs:
        None

    Example:
        >>> # Send a flat 4.5 m/s kick command to blue robot 0:
        >>> kick(sock, ("127.0.0.1", 10301), 0, speed_mps=4.5)
        >>> # Send a chipped shot with 30-degree elevation:
        >>> kick(sock, ("127.0.0.1", 10301), 0, speed_mps=3.0, angle_deg=30.0)
    """
    speed_mps = float(speed_mps)
    angle_deg = float(angle_deg)
    if not math.isfinite(speed_mps) or not 0.0 < speed_mps <= MAX_KICK_SPEED_MPS:
        raise ValueError(
            f"speed_mps must be in (0, {MAX_KICK_SPEED_MPS}], got {speed_mps}")
    if not math.isfinite(angle_deg) or not 0.0 <= angle_deg <= MAX_KICK_ANGLE_DEG:
        raise ValueError(
            f"angle_deg must be in [0, {MAX_KICK_ANGLE_DEG}], got {angle_deg}")
    _send(sock, endpoint, robot_id, kick_speed=speed_mps, kick_angle=angle_deg)


def set_dribbler(sock, endpoint, robot_id, *, enabled):
    """Enable or disable the dribbler.

    Inputs:
        sock (socket.socket): UDP socket for command transmission.
        endpoint (tuple[str, int]): Target (host, port) tuple.
        robot_id (int): Non-negative robot ID.
        enabled (bool): True to activate dribbler roller, False to stop it.

    Outputs:
        None

    Example:
        >>> set_dribbler(sock, ("127.0.0.1", 10301), 0, enabled=True)
    """
    _send(sock, endpoint, robot_id,
          dribbler_speed=DRIBBLER_ON_RPM if enabled else 0.0)


def disable_actuators(sock, endpoint, robot_id):
    """Send zero kick/dribbler settings; cannot undo an executed kick.

    Inputs:
        sock (socket.socket): UDP socket for command transmission.
        endpoint (tuple[str, int]): Target (host, port) tuple.
        robot_id (int): Non-negative robot ID.

    Outputs:
        None

    Example:
        >>> disable_actuators(sock, ("127.0.0.1", 10301), 0)
    """
    _send(sock, endpoint, robot_id)
