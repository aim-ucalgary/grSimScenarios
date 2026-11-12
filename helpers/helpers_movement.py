"""Robot velocity commands.

Linear velocity: m/s. Angular velocity: rad/s.
Send functions send once; duration-based functions block.
"""

import math
import time

import ssl_simulation_robot_control_pb2 as control

from .helpers_communication import send_commands


def _check_robot_id(robot_id):
    if isinstance(robot_id, bool) or not isinstance(robot_id, int) or robot_id < 0:
        raise ValueError(f"robot_id must be a non-negative int, got {robot_id!r}")


def _check_positive(name, value):
    if not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
        raise ValueError(f"{name} must be a positive number, got {value!r}")


def send_global_velocity(
    sock, endpoint, robot_id, *,
    vx_mps, vy_mps, omega_radps=0.0,
):
    """Send velocity in field coordinates.

    Inputs:
        sock (socket.socket): UDP socket for command transmission.
        endpoint (tuple[str, int]): Target team port endpoint (e.g. ("127.0.0.1", 10301)).
        robot_id (int): Non-negative robot ID.
        vx_mps (float): Desired velocity along field +X in m/s.
        vy_mps (float): Desired velocity along field +Y in m/s.
        omega_radps (float, optional): Counter-clockwise angular velocity in rad/s. Default is 0.0.

    Outputs:
        None

    Example:
        >>> # Move robot 0 forward along +X at 0.5 m/s without rotating:
        >>> send_global_velocity(sock, ("127.0.0.1", 10301), 0, vx_mps=0.5, vy_mps=0.0)
    """
    _check_robot_id(robot_id)
    message = control.RobotControl()
    cmd = message.robot_commands.add(id=robot_id)
    cmd.move_command.global_velocity.x = float(vx_mps)
    cmd.move_command.global_velocity.y = float(vy_mps)
    cmd.move_command.global_velocity.angular = float(omega_radps)
    send_commands(sock, endpoint, message)


def send_local_velocity(
    sock, endpoint, robot_id, *,
    forward_mps, left_mps, omega_radps=0.0,
):
    """Send velocity in robot coordinates.

    Inputs:
        sock (socket.socket): UDP socket for command transmission.
        endpoint (tuple[str, int]): Target team port endpoint.
        robot_id (int): Non-negative robot ID.
        forward_mps (float): Velocity straight ahead relative to robot face in m/s.
        left_mps (float): Velocity leftward relative to robot face in m/s.
        omega_radps (float, optional): Angular velocity in rad/s. Default is 0.0.

    Outputs:
        None

    Example:
        >>> # Drive robot 0 forward at 1.0 m/s relative to where it faces:
        >>> send_local_velocity(sock, ("127.0.0.1", 10301), 0, forward_mps=1.0, left_mps=0.0)
    """
    _check_robot_id(robot_id)
    message = control.RobotControl()
    cmd = message.robot_commands.add(id=robot_id)
    cmd.move_command.local_velocity.forward = float(forward_mps)
    cmd.move_command.local_velocity.left = float(left_mps)
    cmd.move_command.local_velocity.angular = float(omega_radps)
    send_commands(sock, endpoint, message)


def hold_velocity(
    sock, endpoint, robot_id, *,
    vx_mps, vy_mps, omega_radps=0.0,
    duration_s, rate_hz=50.0,
):
    """Repeat field-relative velocity; caller is responsible for stopping.

    Inputs:
        sock (socket.socket): UDP socket for command transmission.
        endpoint (tuple[str, int]): Target team port endpoint.
        robot_id (int): Non-negative robot ID.
        vx_mps (float): Field velocity along +X in m/s.
        vy_mps (float): Field velocity along +Y in m/s.
        omega_radps (float, optional): Angular velocity in rad/s. Default is 0.0.
        duration_s (float): Number of seconds to maintain the command.
        rate_hz (float, optional): Command repetition frequency. Default is 50.0 Hz.

    Outputs:
        None

    Example:
        >>> # Hold 0.4 m/s in +X for 2.0 seconds:
        >>> hold_velocity(sock, ("127.0.0.1", 10301), 0, vx_mps=0.4, vy_mps=0.0, duration_s=2.0)
    """
    _check_positive("duration_s", duration_s)
    _check_positive("rate_hz", rate_hz)
    dt = 1.0 / rate_hz
    end_time = time.monotonic() + duration_s
    while time.monotonic() < end_time:
        send_global_velocity(sock, endpoint, robot_id, vx_mps=vx_mps, vy_mps=vy_mps, omega_radps=omega_radps)
        time.sleep(dt)


def ramp_velocity(
    sock, endpoint, robot_id, *,
    start_velocity, end_velocity, duration_s, rate_hz=50.0,
):
    """Interpolate field (vx, vy, omega) commands; caller owns stopping.

    Inputs:
        sock (socket.socket): UDP socket for command transmission.
        endpoint (tuple[str, int]): Target team port endpoint.
        robot_id (int): Non-negative robot ID.
        start_velocity (tuple[float, float, float] | tuple[float, float]):
            Initial (vx, vy, [omega]) in m/s and rad/s.
        end_velocity (tuple[float, float, float] | tuple[float, float]):
            Target (vx, vy, [omega]) in m/s and rad/s.
        duration_s (float): Time to interpolate between start and end velocity.
        rate_hz (float, optional): Command frequency. Default is 50.0 Hz.

    Outputs:
        None

    Example:
        >>> # Smoothly ramp from 0 to 1.0 m/s over 1.5 seconds:
        >>> ramp_velocity(sock, ("127.0.0.1", 10301), 0,
        ...               start_velocity=(0.0, 0.0, 0.0),
        ...               end_velocity=(1.0, 0.0, 0.0),
        ...               duration_s=1.5)
    """
    _check_positive("duration_s", duration_s)
    _check_positive("rate_hz", rate_hz)
    dt = 1.0 / rate_hz
    start_time = time.monotonic()
    end_time = start_time + duration_s

    sv = [float(v) for v in start_velocity]
    ev = [float(v) for v in end_velocity]
    sw = sv[2] if len(sv) > 2 else 0.0
    ew = ev[2] if len(ev) > 2 else 0.0

    while True:
        now = time.monotonic()
        if now >= end_time:
            send_global_velocity(sock, endpoint, robot_id, vx_mps=ev[0], vy_mps=ev[1], omega_radps=ew)
            break
        alpha = min(1.0, max(0.0, (now - start_time) / duration_s))
        vx = sv[0] + alpha * (ev[0] - sv[0])
        vy = sv[1] + alpha * (ev[1] - sv[1])
        w = sw + alpha * (ew - sw)
        send_global_velocity(sock, endpoint, robot_id, vx_mps=vx, vy_mps=vy, omega_radps=w)
        time.sleep(dt)


def stop_robot(sock, endpoint, robot_id, *, repeat_for_s=0.25, rate_hz=50.0):
    """Repeat zero translational and angular velocity commands.

    Inputs:
        sock (socket.socket): UDP socket for command transmission.
        endpoint (tuple[str, int]): Target team port endpoint.
        robot_id (int): Non-negative robot ID.
        repeat_for_s (float, optional): Time to keep sending stop commands. Default is 0.25s.
        rate_hz (float, optional): Command frequency. Default is 50.0 Hz.

    Outputs:
        None

    Example:
        >>> stop_robot(sock, ("127.0.0.1", 10301), 0)
    """
    hold_velocity(sock, endpoint, robot_id, vx_mps=0.0, vy_mps=0.0, omega_radps=0.0,
                  duration_s=repeat_for_s, rate_hz=rate_hz)


def stop_robots(sock, endpoint, robot_ids, *, repeat_for_s=0.25, rate_hz=50.0):
    """Stop the specified robots on one team.

    Inputs:
        sock (socket.socket): UDP socket for command transmission.
        endpoint (tuple[str, int]): Target team port endpoint.
        robot_ids (list[int]): Collection of robot IDs to stop.
        repeat_for_s (float, optional): Time to keep sending stop commands. Default is 0.25s.
        rate_hz (float, optional): Command frequency. Default is 50.0 Hz.

    Outputs:
        None

    Example:
        >>> stop_robots(sock, ("127.0.0.1", 10301), [0, 1, 2])
    """
    _check_positive("repeat_for_s", repeat_for_s)
    _check_positive("rate_hz", rate_hz)
    dt = 1.0 / rate_hz
    end_time = time.monotonic() + repeat_for_s
    while time.monotonic() < end_time:
        message = control.RobotControl()
        for rid in robot_ids:
            _check_robot_id(rid)
            cmd = message.robot_commands.add(id=rid)
            cmd.move_command.global_velocity.x = 0.0
            cmd.move_command.global_velocity.y = 0.0
            cmd.move_command.global_velocity.angular = 0.0
        send_commands(sock, endpoint, message)
        time.sleep(dt)
