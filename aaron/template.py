"""Demonstrate field-relative movement in grSim.

Purpose:
    Move one robot in four directions using velocity commands.

Parameters:
    Blue robot 0; speed 0.4 m/s; 3 seconds per direction;
    command interval 0.02 seconds.

Setup:
    Run grSim at normal speed. Place the robot near field center
    with a clear 1.2 m square toward +X and +Y.

Actions:
    Move along +X, +Y, -X, and -Y, then stop.

Expected results:
    Approximately 1.2 m per segment, returning near the start.

"""


import socket
import sys
import time
from pathlib import Path

# Allow aaron/template.py to import the repository's helpers package.
REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))

# Generated Protobuf bindings; adjust this path for your environment.
PROTO_DIR = Path("/home/aaron/grsim-pilot/proto")
sys.path.insert(0, str(PROTO_DIR))

import ssl_simulation_robot_control_pb2 as control
import helpers  # Access functions with helpers.movement.*, helpers.setup.*, etc.

# simulation settings
SIMULATOR_IP = "127.0.0.1"


# Robot settings
ROBOT_ID = 0


sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


def move(x, y):
    message = control.RobotControl()
    robot = message.robot_commands.add(id=ROBOT_ID)

    robot.move_command.global_velocity.x = x
    robot.move_command.global_velocity.y = y
    robot.move_command.global_velocity.angular = 0

    sock.sendto(message.SerializeToString(), (SIMULATOR_IP, 10301)) # move blue team
    sock.sendto(b"", (SIMULATOR_IP, 10302)) # keep yellow team stationary


try:
    # Move in each direction at 0.4 m/s for 3s
    for x, y in [(0.4, 0), (0, 0.4), (-0.4, 0), (0, -0.4)]:
        end = time.monotonic() + 3

        while time.monotonic() < end:
            move(x, y)
            time.sleep(0.02)

except KeyboardInterrupt:
    pass

finally:
    # stop
    move(0, 0)
    time.sleep(0.02)

    sock.close()