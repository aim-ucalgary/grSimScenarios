"""Live test: catch a moving ball with the dribbler in a running grSim.

Usage (from the repo root, grSim already running):
    python tests/live_receive.py --team blue --id 0
    python tests/live_receive.py --bx 2.5 --by 0.6 --bvx -2.5 --bvy -0.3

The ball's initial position and velocity are set with --bx/--by/--bvx/--bvy.
Robot start pose is set with --rx/--ry/--rh.
Add --no-teleport to use the current robot/ball state (the ball must already
be moving, so this is mostly useful with an external kicker).
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
from skills import receive_ball  # make sure skills/__init__.py exports it
from skills.vision import VisionTracker, open_vision_socket

CONTROL_PORT = 10300
TEAM_PORTS = {"blue": 10301, "yellow": 10302}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--team", default="blue", choices=["blue", "yellow"])
    ap.add_argument("--id", type=int, default=0)
    ap.add_argument("--host", default="127.0.0.1")
    # Robot start pose (heading 0 = facing +x).
    ap.add_argument("--rx", type=float, default=0.0)
    ap.add_argument("--ry", type=float, default=0.0)
    ap.add_argument("--rh", type=float, default=0.0, help="heading, rad")
    # Ball initial state.
    ap.add_argument("--bx", type=float, default=2.5)
    ap.add_argument("--by", type=float, default=0.6)
    ap.add_argument("--bvx", type=float, default=-2.5, help="ball vx, m/s")
    ap.add_argument("--bvy", type=float, default=-0.3, help="ball vy, m/s")
    ap.add_argument("--timeout", type=float, default=10.0)
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
                                 x_m=args.rx, y_m=args.ry, heading_rad=args.rh)
            # Ball placed at rest first so vision is live, then launched.
            setup.teleport_ball(sock, ctl_endpoint, x_m=args.bx, y_m=args.by)
            time.sleep(0.7)

        deadline = time.monotonic() + 3.0
        while tracker.get_pose() is None or tracker.get_ball() is None:
            if time.monotonic() > deadline:
                sys.exit("No vision for the robot and/or ball. Check grSim vision "
                         "settings and that both are on the field.")
            time.sleep(0.01)
        print("robot:", tracker.get_pose(), "ball:", tracker.get_ball())

        if not args.no_teleport:
            # Launch the ball. Assumes teleport_ball accepts a velocity.
            try:
                setup.teleport_ball(sock, ctl_endpoint, x_m=args.bx, y_m=args.by,
                                    vx_mps=args.bvx, vy_mps=args.bvy)
            except TypeError:
                sys.exit("setup.teleport_ball does not accept vx_mps/vy_mps yet. "
                         "grSim's ball replacement packet has vx/vy fields; add "
                         "them to the helper.")
        print(f"ball launched at ({args.bx:.2f}, {args.by:.2f}) "
              f"with v=({args.bvx:.2f}, {args.bvy:.2f}) m/s")

        ok = receive_ball(sock, cmd_endpoint, args.id,
                          tracker.get_pose, tracker.get_ball,
                          timeout_s=args.timeout)
        print("ball received" if ok else "FAILED / timed out")

        # Check that the ball stays with the robot after the skill stops it.
        end = time.monotonic() + 2.0
        while time.monotonic() < end:
            tracker.get_ball()
            time.sleep(0.05)
        pose, ball = tracker.get_pose(), tracker.get_ball()
        if pose and ball:
            dx, dy = ball[0] - pose[0], ball[1] - pose[1]
            fwd = dx * math.cos(pose[2]) + dy * math.sin(pose[2])
            lat = -dx * math.sin(pose[2]) + dy * math.cos(pose[2])
            print(f"2 s later, ball in robot frame: forward {fwd:.3f} m, "
                  f"lateral {lat:.3f} m")
    finally:
        sock.close()
        vision.close()


if __name__ == "__main__":
    main()