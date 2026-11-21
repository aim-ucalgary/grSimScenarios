"""Live test: go to the ball, hold it, and spin 360 degrees in a running grSim.

Usage (from the repo root, grSim already running):
    python tests/live_turn_with_ball.py --team blue --id 0 --rate 0.5
--rate is the turning speed in rad/s (negative = clockwise). Add --no-teleport to
use the current robot/ball positions.
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
from skills import turn_with_ball
from skills.vision import VisionTracker, open_vision_socket

CONTROL_PORT = 10300
TEAM_PORTS = {"blue": 10301, "yellow": 10302}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--team", default="blue", choices=["blue", "yellow"])
    ap.add_argument("--id", type=int, default=0)
    ap.add_argument("--rate", type=float, default=0.5, help="turn rate, rad/s")
    ap.add_argument("--far", action="store_true",
                    help="start away from the ball so it must drive to it first")
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
            rx = -1.5 if args.far else -0.12
            setup.teleport_robot(sock, ctl_endpoint, args.team, args.id,
                                 x_m=rx, y_m=0.0, heading_rad=0.0)
            setup.teleport_ball(sock, ctl_endpoint, x_m=0.0, y_m=0.0)
            time.sleep(0.7)

        deadline = time.monotonic() + 3.0
        while tracker.get_pose() is None or tracker.get_ball() is None:
            if time.monotonic() > deadline:
                sys.exit("No vision for the robot and/or ball. Check grSim vision "
                         "settings and that both are on the field.")
        start_ball = tracker.get_ball()
        print("start robot:", tracker.get_pose(), "ball:", start_ball)

        t0 = time.monotonic()
        ok = turn_with_ball(sock, cmd_endpoint, args.id, tracker.get_pose, tracker.get_ball, x_m=10, y_m=10,
                            turn_rate_radps=args.rate, timeout_s=30.0)
        print(("done" if ok else "FAILED / timed out") + f" after {time.monotonic() - t0:.1f} s")

        time.sleep(0.5)
        tracker.get_ball()
        ball = tracker.get_ball()
        if ball:
            print(f"ball at ({ball[0]:.2f}, {ball[1]:.2f}); "
                  f"{math.hypot(ball[0] - start_ball[0], ball[1] - start_ball[1]):.3f} m from where it started")
    finally:
        sock.close()
        vision.close()


if __name__ == "__main__":
    main()