"""Live test: pass the ball from one robot to another in a running grSim.

Usage (from the repo root, grSim already running):
    python tests/live_pass.py --team blue --passer 0 --receiver 1 --tx 1.5 --ty 0.5
Add --no-teleport to use the current robot/ball positions.
Add --arrival 2.0 to change the wanted ball speed at the receiver (m/s).
"""

import argparse
import math
import sys
import threading
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))
sys.path.insert(0, str(REPO_ROOT / "grSim" / "pb"))   # generated *_pb2.py files

from helpers import communication as com
from helpers import setup
from tactics import direct_pass  # make sure tactics/__init__.py exports it
from skills.vision import VisionTracker, open_vision_socket

CONTROL_PORT = 10300
TEAM_PORTS = {"blue": 10301, "yellow": 10302}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--team", default="blue", choices=["blue", "yellow"])
    ap.add_argument("--passer", type=int, default=0)
    ap.add_argument("--receiver", type=int, default=1)
    ap.add_argument("--tx", type=float, default=1.5, help="pass target X, m")
    ap.add_argument("--ty", type=float, default=0.5, help="pass target Y, m")
    ap.add_argument("--arrival", type=float, default=None,
                    help="wanted ball speed at the receiver, m/s; default = tactic's")
    ap.add_argument("--host", default="127.0.0.1")
    # Start positions (used unless --no-teleport).
    ap.add_argument("--px", type=float, default=-1.5)
    ap.add_argument("--py", type=float, default=0.0)
    ap.add_argument("--rx", type=float, default=1.0)
    ap.add_argument("--ry", type=float, default=1.5)
    ap.add_argument("--bx", type=float, default=-1.0)
    ap.add_argument("--by", type=float, default=0.0)
    ap.add_argument("--timeout", type=float, default=25.0)
    ap.add_argument("--no-teleport", action="store_true")
    args = ap.parse_args()

    cmd_endpoint = (args.host, TEAM_PORTS[args.team])
    ctl_endpoint = (args.host, CONTROL_PORT)
    sock = com.open_socket()

    # Two vision sockets so the two threads inside direct_pass never compete for
    # packets: tracker 1 serves the passer pose AND the ball (always under a
    # lock), tracker 2 serves only the receiver pose (receiver thread only).
    vision1, vision2 = open_vision_socket(), open_vision_socket()
    tracker1 = VisionTracker(vision1, args.team, args.passer)
    tracker2 = VisionTracker(vision2, args.team, args.receiver)
    lock1 = threading.Lock()

    def get_pose_passer():
        with lock1:
            return tracker1.get_pose()

    def get_ball():
        with lock1:
            return tracker1.get_ball()

    def get_pose_receiver():
        return tracker2.get_pose()

    try:
        if not args.no_teleport:
            # Passer faces the target direction; receiver faces the ball.
            passer_heading = math.atan2(args.ty - args.by, args.tx - args.bx)
            receiver_heading = math.atan2(args.by - args.ry, args.bx - args.rx)
            setup.teleport_robot(sock, ctl_endpoint, args.team, args.passer,
                                 x_m=args.px, y_m=args.py, heading_rad=passer_heading)
            setup.teleport_robot(sock, ctl_endpoint, args.team, args.receiver,
                                 x_m=args.rx, y_m=args.ry, heading_rad=receiver_heading)
            setup.teleport_ball(sock, ctl_endpoint, x_m=args.bx, y_m=args.by)
            time.sleep(0.7)

        deadline = time.monotonic() + 3.0
        while (get_pose_passer() is None or get_pose_receiver() is None
               or get_ball() is None):
            if time.monotonic() > deadline:
                sys.exit("No vision for the passer, receiver and/or ball. Check grSim "
                         "vision settings and that all three are on the field.")
            time.sleep(0.01)
        print("passer:", get_pose_passer(), "receiver:", get_pose_receiver(),
              "ball:", get_ball())

        kwargs = {} if args.arrival is None else {"arrival_speed_mps": args.arrival}
        t0 = time.monotonic()
        ok = direct_pass(sock, cmd_endpoint, args.passer, args.receiver,
                       get_pose_passer, get_pose_receiver, get_ball,
                       target_x_m=args.tx, target_y_m=args.ty,
                       timeout_s=args.timeout, **kwargs)
        elapsed = time.monotonic() - t0
        print(f"pass completed in {elapsed:.2f} s" if ok
              else f"FAILED / timed out after {elapsed:.2f} s")

        # Did the ball end up on the target, and is it still on the dribbler?
        time.sleep(1.5)
        ball, rpose = get_ball(), get_pose_receiver()
        if ball:
            err = math.hypot(ball[0] - args.tx, ball[1] - args.ty)
            print(f"ball at ({ball[0]:.2f}, {ball[1]:.2f}); {err:.2f} m from target")
            if rpose:
                dx, dy = ball[0] - rpose[0], ball[1] - rpose[1]
                fwd = dx * math.cos(rpose[2]) + dy * math.sin(rpose[2])
                lat = -dx * math.sin(rpose[2]) + dy * math.cos(rpose[2])
                print(f"ball in receiver frame: forward {fwd:.3f} m, lateral {lat:.3f} m")
    finally:
        sock.close()
        vision1.close()
        vision2.close()


if __name__ == "__main__":
    main()