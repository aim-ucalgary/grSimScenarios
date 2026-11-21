"""Live test: drive to the ball and chip-kick it toward a point in a running grSim.

Usage (from the repo root, grSim already running):
    python tests/live_chip_kick_to.py --team blue --id 0 --tx 2.0 --ty 1.0
Add --no-teleport to use the current robot/ball positions.
Add --speed 4.0 to override the auto chip speed (m/s).
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
from skills import chip_kick_to  # make sure skills/__init__.py exports it
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
            time.sleep(0.01)
        print("robot:", tracker.get_pose(), "ball:", tracker.get_ball())

        start_ball = tracker.get_ball()
        target_dist = math.hypot(args.tx - start_ball[0], args.ty - start_ball[1])

        ok = chip_kick_to(sock, cmd_endpoint, args.id, tracker.get_pose, tracker.get_ball,
                          target_x_m=args.tx, target_y_m=args.ty,
                          chip_speed_mps=args.speed,
                          timeout_s=25.0)
        print("chip delivered" if ok else "FAILED / timed out")

        # Watch where the ball ends up. This is the RESTING position after the
        # bounces and roll, not the first landing point, so expect it to overshoot
        # the target; use it to tune CHIP_ANGLE_RAD / speed limits in skills/chip_kick_to.py.
        end = time.monotonic() + 5.0
        while time.monotonic() < end:
            tracker.get_ball()
            time.sleep(0.05)
        ball = tracker.get_ball()
        if ball:
            err = math.hypot(ball[0] - args.tx, ball[1] - args.ty)
            travelled = math.hypot(ball[0] - start_ball[0], ball[1] - start_ball[1])
            print(f"ball stopped at ({ball[0]:.2f}, {ball[1]:.2f}); "
                  f"{err:.2f} m from target; travelled {travelled:.2f} m "
                  f"(target was {target_dist:.2f} m away)")
    finally:
        sock.close()
        vision.close()


if __name__ == "__main__":
    main()