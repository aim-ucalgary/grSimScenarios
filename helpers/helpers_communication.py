"""UDP communication and simulator responses.

An endpoint is a (host, port) tuple.
Sockets are created explicitly, never during import.
"""

import socket

import ssl_simulation_robot_control_pb2 as control
import ssl_simulation_robot_feedback_pb2 as feedback_pb


def open_socket(*, timeout_s=1.0):
    """Return a configured UDP socket.

    Inputs:
        timeout_s (float, optional): Socket receive timeout in seconds.
            Default is 1.0s.

    Outputs:
        socket.socket: Newly created UDP datagram socket.

    Example:
        >>> sock = open_socket(timeout_s=0.5)
        >>> # ... communicate with grSim ...
        >>> close_socket(sock)
    """
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    if timeout_s is not None:
        sock.settimeout(float(timeout_s))
    return sock


def _populate_robot_command(robot, cmd):
    """Populate a RobotCommand protobuf message from a dictionary."""
    if "global_velocity" in cmd:
        gv = cmd["global_velocity"]
        robot.move_command.global_velocity.x = float(gv.get("x", 0.0))
        robot.move_command.global_velocity.y = float(gv.get("y", 0.0))
        robot.move_command.global_velocity.angular = float(gv.get("angular", 0.0))
    if "local_velocity" in cmd:
        lv = cmd["local_velocity"]
        robot.move_command.local_velocity.forward = float(lv.get("forward", 0.0))
        robot.move_command.local_velocity.left = float(lv.get("left", 0.0))
        robot.move_command.local_velocity.angular = float(lv.get("angular", 0.0))
    if "kick_speed" in cmd:
        robot.kick_speed = float(cmd["kick_speed"])
    if "kick_angle" in cmd:
        robot.kick_angle = float(cmd["kick_angle"])
    if "dribbler_speed" in cmd:
        robot.dribbler_speed = float(cmd["dribbler_speed"])


def send_commands(sock, endpoint, commands):
    """Send one team's commands as a RobotControl message.

    Inputs:
        sock (socket.socket): Active UDP socket.
        endpoint (tuple[str, int]): Target (host, port) address tuple.
        commands (list[dict] | RobotControl | bytes): Control commands.
            Can be a serialized byte string, a protobuf RobotControl message,
            or a list of robot command dictionaries.

    Outputs:
        None

    Example:
        >>> commands = [{
        ...     "id": 0,
        ...     "global_velocity": {"x": 0.5, "y": 0.0, "angular": 0.0},
        ...     "kick_speed": 3.0
        ... }]
        >>> send_commands(sock, ("127.0.0.1", 10301), commands)
    """
    if isinstance(commands, bytes):
        payload = commands
    elif hasattr(commands, "SerializeToString"):
        payload = commands.SerializeToString()
    else:
        message = control.RobotControl()
        for cmd in commands:
            if isinstance(cmd, dict):
                robot = message.robot_commands.add(id=int(cmd["id"]))
                _populate_robot_command(robot, cmd)
            else:
                message.robot_commands.add().CopyFrom(cmd)
        payload = message.SerializeToString()

    sock.sendto(payload, endpoint)


def read_feedback(sock, *, timeout_s=1.0):
    """Return decoded robot feedback; raise on timeout or simulator errors.

    Inputs:
        sock (socket.socket): Socket configured to listen for simulator replies.
        timeout_s (float, optional): Maximum seconds to wait. Default is 1.0s.

    Outputs:
        list[RobotFeedback]: Decoded list of robot feedback messages.

    Raises:
        TimeoutError: If no packet arrives before the timeout expires.
        RuntimeError: If the simulator reports command or execution errors.

    Example:
        >>> feedback_list = read_feedback(sock, timeout_s=0.2)
    """
    if timeout_s is not None:
        sock.settimeout(float(timeout_s))
    try:
        data, _ = sock.recvfrom(65535)
    except (socket.timeout, TimeoutError) as e:
        raise TimeoutError(f"read_feedback timed out after {timeout_s}s") from e


    response = feedback_pb.RobotControlResponse()
    response.ParseFromString(data)

    if getattr(response, "errors", None):
        err_msgs = [getattr(e, "message", str(e)) for e in response.errors]
        raise RuntimeError(f"Simulator errors: {'; '.join(err_msgs)}")

    return list(response.feedback) if hasattr(response, "feedback") else response


def close_socket(sock):
    """Close the supplied socket.

    Inputs:
        sock (socket.socket | None): Socket to close. If None, operation is a no-op.

    Outputs:
        None

    Example:
        >>> close_socket(sock)
    """
    if sock is not None:
        sock.close()
