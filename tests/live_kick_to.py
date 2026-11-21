"""Live test: drive to the ball and kick it toward a point in a running grSim.

Usage (from the repo root, grSim already running):
    python tests/live_kick_to.py --team blue --id 0 --tx 2.0 --ty 1.0
Add --no-teleport to use the current robot/ball positions.
"""

import argparse
import math
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))
sys.path.insert(0, str(REPO_ROOT / "grSim" / "pb"))   # generated *_pb2.py files

from helpers import communication as com
from helpers import setup
from skills import kick_to
from skills.vision import VisionTracker, open_vision_socket

CONTROL_PORT = 10300
TEAM_PORTS = {"blue": 10301, "yellow": 10302}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--team", default="blue", choices=["blue", "yellow"])
    ap.add_argument("--id", type=int, default=0)
    ap.add_argument("--tx", type=float, default=0.6)
    ap.add_argument("--ty", type=float, default=0.7)
    ap.add_argument("--speed", type=float, default=None, help="m/s; default = auto")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--no-teleport", action="store_true")
    args = ap.parse_args()

    cmd_endpoint = (args.host, TEAM_PORTS[args.team])
    ctl_endpoint = (args.host, CONTROL_PORT)
    sock = com.open_socket()
    vision = open_vision_socket()
    tracker = VisionTracker(vision, args.team, args.id)

    try:
        if not args.no_teleport:
            setup.teleport_robot(sock, ctl_endpoint, args.team, args.id,
                                 x_m=-1.5, y_m=0.5, heading_rad=0.0)
            setup.teleport_ball(sock, ctl_endpoint, x_m=0.0, y_m=0.0)
            time.sleep(0.7)

        deadline = time.monotonic() + 3.0
        while tracker.get_pose() is None or tracker.get_ball() is None:
            if time.monotonic() > deadline:
                sys.exit("No vision for the robot and/or ball. Check grSim vision "
                         "settings and that both are on the field.")
        print("robot:", tracker.get_pose(), "ball:", tracker.get_ball())

        ok = kick_to(sock, cmd_endpoint, args.id, tracker.get_pose, tracker.get_ball,
                     target_x_m=args.tx, target_y_m=args.ty,
                     #kick_speed_mps=args.speed,
                       timeout_s=25.0)
        print("kick delivered" if ok else "FAILED / timed out")

        # Watch where the ball ends up (to tune BALL_DECEL_MPS2 in skills/kick_to.py).
        end = time.monotonic() + 4.0
        while time.monotonic() < end:
            tracker.get_ball()
            time.sleep(0.05)
        ball = tracker.get_ball()
        if ball:
            err = math.hypot(ball[0] - args.tx, ball[1] - args.ty)
            print(f"ball stopped at ({ball[0]:.2f}, {ball[1]:.2f}); "
                  f"{err:.2f} m from target")
    finally:
        sock.close()
        vision.close()


if __name__ == "__main__":
    main()
