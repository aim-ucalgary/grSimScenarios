"""Live test: drive a robot in a running grSim.

Usage (from the repo root, grSim already running):
    python tests/live_go_to_point.py --team blue --id 0 --x 1.5 --y -0.5 --heading 1.57
Add --no-teleport to skip resetting the robot to the start pose first.
"""

import argparse
import math
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))
sys.path.insert(0, str(REPO_ROOT / "grSim" / "pb"))   # generated *_pb2.py files

from skills import go_to_point
from skills._compat import load
from skills.vision import make_pose_getter, open_vision_socket

com = load("communication")
setup = load("setup")

CONTROL_PORT = 10300
TEAM_PORTS = {"blue": 10301, "yellow": 10302}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--team", default="blue", choices=["blue", "yellow"])
    ap.add_argument("--id", type=int, default=0)
    ap.add_argument("--x", type=float, default=1.5)
    ap.add_argument("--y", type=float, default=-0.5)
    ap.add_argument("--heading", type=float, default=None)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--no-teleport", action="store_true")
    args = ap.parse_args()

    cmd_endpoint = (args.host, TEAM_PORTS[args.team])
    sock = com.open_socket()
    vision = open_vision_socket()
    get_pose = make_pose_getter(vision, args.team, args.id)

    try:
        if not args.no_teleport:
            setup.teleport_robot(sock, (args.host, CONTROL_PORT), args.team, args.id,
                                 x_m=-1.0, y_m=0.0, heading_rad=0.0)
            time.sleep(0.5)

        # Wait for the first valid pose so we fail clearly if vision isn't arriving.
        deadline = time.monotonic() + 3.0
        while get_pose() is None:
            if time.monotonic() > deadline:
                sys.exit("No vision for this robot. Check grSim vision port/multicast "
                         "and that the robot is on the field.")
        print("start pose:", get_pose())

        ok = go_to_point(sock, cmd_endpoint, args.id, get_pose,
                         x_m=args.x, y_m=args.y, heading_rad=args.heading,
                         timeout_s=15.0)
        time.sleep(0.2)
        final = get_pose()
        print("reached" if ok else "TIMED OUT", "| final pose:", final)
        if final:
            print(f"position error: {math.hypot(final[0]-args.x, final[1]-args.y):.3f} m")
    finally:
        sock.close()
        vision.close()


if __name__ == "__main__":
    main()
