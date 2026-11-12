"""Scene initialization through the simulation-control endpoint.

Positions: metres. Heading: radians. Team: 'blue' or 'yellow'.
"""

import math

try:
    import ssl_simulation_control_pb2 as sim_control
except ImportError:
    sim_control = None

try:
    import ssl_gc_common_pb2 as gc_common
except ImportError:
    gc_common = None


def _set_robot_id(robot_id_msg, team, robot_id):
    if isinstance(robot_id, bool) or not isinstance(robot_id, int) or robot_id < 0:
        raise ValueError(f"robot_id must be a non-negative int, got {robot_id!r}")
    robot_id_msg.id = robot_id
    team_name = team.strip().lower()
    field = "team" if hasattr(robot_id_msg, "team") else "team_color"
    if team_name in ("blue", "b"):
        val = getattr(gc_common, "TEAM_BLUE", getattr(sim_control, "TEAM_BLUE", 0))
    elif team_name in ("yellow", "y"):
        val = getattr(gc_common, "TEAM_YELLOW", getattr(sim_control, "TEAM_YELLOW", 1))
    else:
        raise ValueError(f"team must be 'blue' or 'yellow', got {team!r}")
    setattr(robot_id_msg, field, val)


def teleport_robot(
    sock, endpoint, team, robot_id, *,
    x_m, y_m, heading_rad=0.0,
    vx_mps=0.0, vy_mps=0.0, omega_radps=0.0,
):
    """Set a robot's pose and initial velocity.

    Inputs:
        sock (socket.socket): UDP socket for simulator control.
        endpoint (tuple[str, int]): Simulator control port endpoint (e.g. ("127.0.0.1", 10300)).
        team (str): 'blue' or 'yellow'.
        robot_id (int): Non-negative robot ID.
        x_m (float): Target position along field +X in metres.
        y_m (float): Target position along field +Y in metres.
        heading_rad (float, optional): Robot orientation in radians. Default is 0.0.
        vx_mps (float, optional): Initial linear velocity along +X in m/s. Default is 0.0.
        vy_mps (float, optional): Initial linear velocity along +Y in m/s. Default is 0.0.
        omega_radps (float, optional): Initial angular velocity in rad/s. Default is 0.0.

    Outputs:
        None

    Example:
        >>> teleport_robot(sock, ("127.0.0.1", 10300), "blue", 0,
        ...                x_m=-1.5, y_m=0.0, heading_rad=0.0)
    """
    if sim_control is None:
        raise RuntimeError("ssl_simulation_control_pb2 is not available")
    cmd = sim_control.SimulatorCommand()
    teleport = cmd.control.teleport_robot.add()
    _set_robot_id(teleport.id, team, robot_id)
    teleport.x = float(x_m)
    teleport.y = float(y_m)
    teleport.orientation = float(heading_rad)
    teleport.v_x = float(vx_mps)
    teleport.v_y = float(vy_mps)
    teleport.v_angular = float(omega_radps)
    sock.sendto(cmd.SerializeToString(), endpoint)


def teleport_ball(
    sock, endpoint, *,
    x_m, y_m, z_m=0.0,
    vx_mps=0.0, vy_mps=0.0, vz_mps=0.0,
):
    """Set the ball's position and initial velocity.

    Inputs:
        sock (socket.socket): UDP socket for simulator control.
        endpoint (tuple[str, int]): Simulator control port endpoint.
        x_m (float): Target position along field +X in metres.
        y_m (float): Target position along field +Y in metres.
        z_m (float, optional): Height above field in metres. Default is 0.0.
        vx_mps (float, optional): Initial linear velocity along +X in m/s. Default is 0.0.
        vy_mps (float, optional): Initial linear velocity along +Y in m/s. Default is 0.0.
        vz_mps (float, optional): Initial linear velocity along +Z in m/s. Default is 0.0.

    Outputs:
        None

    Example:
        >>> # Teleport ball to field center at rest:
        >>> teleport_ball(sock, ("127.0.0.1", 10300), x_m=0.0, y_m=0.0)
    """
    if sim_control is None:
        raise RuntimeError("ssl_simulation_control_pb2 is not available")
    cmd = sim_control.SimulatorCommand()
    teleport = cmd.control.teleport_ball
    teleport.x = float(x_m)
    teleport.y = float(y_m)
    teleport.z = float(z_m)
    teleport.vx = float(vx_mps)
    teleport.vy = float(vy_mps)
    teleport.vz = float(vz_mps)
    sock.sendto(cmd.SerializeToString(), endpoint)


def set_robot_present(sock, endpoint, team, robot_id, *, present):
    """Enable or remove an existing robot slot.

    Inputs:
        sock (socket.socket): UDP socket for simulator control.
        endpoint (tuple[str, int]): Simulator control port endpoint.
        team (str): 'blue' or 'yellow'.
        robot_id (int): Non-negative robot ID.
        present (bool): True to make the robot present on the field; False to remove it.

    Outputs:
        None

    Example:
        >>> # Despawn yellow robot 5:
        >>> set_robot_present(sock, ("127.0.0.1", 10300), "yellow", 5, present=False)
    """
    if sim_control is None:
        raise RuntimeError("ssl_simulation_control_pb2 is not available")
    cmd = sim_control.SimulatorCommand()
    teleport = cmd.control.teleport_robot.add()
    _set_robot_id(teleport.id, team, robot_id)
    teleport.present = bool(present)
    sock.sendto(cmd.SerializeToString(), endpoint)


def apply_initial_state(sock, endpoint, state):
    """Apply a team-defined scene description.

    Inputs:
        sock (socket.socket): UDP socket for simulator control.
        endpoint (tuple[str, int]): Simulator control port endpoint.
        state (dict): Configuration dictionary mapping 'robots' (or 'blue'/'yellow')
            and 'ball' to poses/velocities.

    Outputs:
        None

    Example:
        >>> scene = {
        ...     "blue": {0: (1.0, 0.0, 0.0)},
        ...     "yellow": {0: (-1.0, 0.0, math.pi)},
        ...     "ball": (0.0, 0.0)
        ... }
        >>> apply_initial_state(sock, ("127.0.0.1", 10300), scene)
    """
    if not isinstance(state, dict):
        raise ValueError("state must be a dictionary")

    # Handle robots
    robots = state.get("robots", state)
    for team in ("blue", "yellow"):
        if team in robots and isinstance(robots[team], dict):
            for rid, rdata in robots[team].items():
                if isinstance(rdata, dict):
                    teleport_robot(
                        sock, endpoint, team, rid,
                        x_m=rdata.get("x", 0.0),
                        y_m=rdata.get("y", 0.0),
                        heading_rad=rdata.get("heading", rdata.get("orientation", 0.0)),
                        vx_mps=rdata.get("vx", 0.0),
                        vy_mps=rdata.get("vy", 0.0),
                        omega_radps=rdata.get("omega", 0.0),
                    )
                elif isinstance(rdata, (list, tuple)):
                    teleport_robot(
                        sock, endpoint, team, rid,
                        x_m=rdata[0],
                        y_m=rdata[1],
                        heading_rad=rdata[2] if len(rdata) > 2 else 0.0,
                    )

    # Handle ball
    if "ball" in state:
        b = state["ball"]
        if isinstance(b, dict):
            teleport_ball(
                sock, endpoint,
                x_m=b.get("x", 0.0),
                y_m=b.get("y", 0.0),
                z_m=b.get("z", 0.0),
                vx_mps=b.get("vx", 0.0),
                vy_mps=b.get("vy", 0.0),
                vz_mps=b.get("vz", 0.0),
            )
        elif isinstance(b, (list, tuple)):
            teleport_ball(
                sock, endpoint,
                x_m=b[0],
                y_m=b[1],
                z_m=b[2] if len(b) > 2 else 0.0,
            )


def reset_scene(sock, endpoint, initial_state):
    """Restore the supplied initial scene, including velocities.

    Inputs:
        sock (socket.socket): UDP socket for simulator control.
        endpoint (tuple[str, int]): Simulator control port endpoint.
        initial_state (dict): Configuration dictionary to apply.

    Outputs:
        None

    Example:
        >>> reset_scene(sock, ("127.0.0.1", 10300), initial_scene)
    """
    apply_initial_state(sock, endpoint, initial_state)
