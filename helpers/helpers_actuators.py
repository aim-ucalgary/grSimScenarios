"""Kick and dribbler commands.

Kick speed: m/s. Kick elevation: degrees.
"""

import math

import ssl_simulation_robot_control_pb2 as control

# grSim treats any positive dribbler speed as "on"; the value is in rpm.
DRIBBLER_ON_RPM = 1000.0

# Above this, grSim kicks are unrealistic (SSL rules cap shots at 6.5 m/s).
MAX_KICK_SPEED_MPS = 10.0
MAX_KICK_ANGLE_DEG = 90.0


def _check_robot_id(robot_id):
    if isinstance(robot_id, bool) or not isinstance(robot_id, int) or robot_id < 0:
        raise ValueError(f"robot_id must be a non-negative int, got {robot_id!r}")


def _send(sock, endpoint, robot_id, *, kick_speed=0.0, kick_angle=0.0,
          dribbler_speed=0.0):
    """Build a single-robot RobotControl message and send it as one datagram."""
    _check_robot_id(robot_id)
    message = control.RobotControl()
    command = message.robot_commands.add(id=robot_id)
    command.kick_speed = kick_speed
    command.kick_angle = kick_angle
    command.dribbler_speed = dribbler_speed
    sock.sendto(message.SerializeToString(), endpoint)


def kick(sock, endpoint, robot_id, *, speed_mps, angle_deg=0.0):
    """Send one kick request; delivery and ball contact are not guaranteed."""
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
    """Enable or disable the dribbler."""
    _send(sock, endpoint, robot_id,
          dribbler_speed=DRIBBLER_ON_RPM if enabled else 0.0)


def disable_actuators(sock, endpoint, robot_id):
    """Send zero kick/dribbler settings; cannot undo an executed kick."""
    _send(sock, endpoint, robot_id)