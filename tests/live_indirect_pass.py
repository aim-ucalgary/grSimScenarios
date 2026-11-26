"""Live test: pass the ball passer -> middle -> receiver in a running grSim.

Usage (from the repo root, grSim already running):
    python tests/live_indirect_pass.py --team blue --passer 0 --middle 1 --receiver 2 \
        --tx 2.0 --ty 0.0 --vx 0.5 --vy 1.0
Add --auto-via to let the tactic use the midpoint between ball and target.
Add --no-teleport to use the current robot/ball positions.
Add --chip1 / --chip2 to chip the first / second pass.
Add --arrival 2.0 to change the wanted ball speed at each receiver (m/s).
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
from tactics import indirect_pass  # make sure tactics/__init__.py exports it
from skills.vision import VisionTracker, open_vision_socket

CONTROL_PORT = 10300
TEAM_PORTS = {"blue": 10301, "yellow": 10302}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--team", default="blue", choices=["blue", "yellow"])
    ap.add_argument("--passer", type=int, default=0)
    ap.add_argument("--middle", type=int, default=1)
    ap.add_argument("--receiver", type=int, default=2)
    ap.add_argument("--tx", type=float, default=2.0, help="final target X, m")
    ap.add_argument("--ty", type=float, default=0.0, help="final target Y, m")
    ap.add_argument("--vx", type=float, default=0.5, help="via point X, m")
    ap.add_argument("--vy", type=float, default=1.0, help="via point Y, m")
    ap.add_argument("--auto-via", action="store_true",
                    help="ignore --vx/--vy; use midpoint of ball and target")
    ap.add_argument("--arrival", type=float, default=None,
                    help="wanted ball speed at each receiver, m/s; default = tactic's")
    ap.add_argument("--chip1", action="store_true", help="chip the passer -> middle pass")
    ap.add_argument("--chip2", action="store_true", help="chip the middle -> receiver pass")
    ap.add_argument("--host", default="127.0.0.1")
    # Start positions (used unless --no-teleport).
    ap.add_argument("--px", type=float, default=-2.0)
    ap.add_argument("--py", type=float, default=0.0)
    ap.add_argument("--mx", type=float, default=0.0)
    ap.add_argument("--my", type=float, default=1.5)
    ap.add_argument("--rx", type=float, default=2.0)
    ap.add_argument("--ry", type=float, default=-1.5)
    ap.add_argument("--bx", type=float, default=-1.5)
    ap.add_argument("--by", type=float, default=0.0)
    ap.add_argument("--timeout", type=float, default=40.0)
    ap.add_argument("--no-teleport", action="store_true")
    args = ap.parse_args()

    cmd_endpoint = (args.host, TEAM_PORTS[args.team])
    ctl_endpoint = (args.host, CONTROL_PORT)
    sock = com.open_socket()

    # One vision socket + tracker per robot so threads never compete for packets.
    # The ball is read through the passer's tracker. Every getter takes its
    # tracker's lock, so no tracker is ever used by two threads at once.
    visions = [open_vision_socket() for _ in range(3)]
    ids = [args.passer, args.middle, args.receiver]
    trackers = [VisionTracker(v, args.team, i) for v, i in zip(visions, ids)]
    locks = [threading.Lock() for _ in trackers]

    def make_pose_getter(k):
        def get():
            with locks[k]:
                return trackers[k].get_pose()
        return get

    def get_ball():
        with locks[0]:
            return trackers[0].get_ball()

    get_pose_passer, get_pose_middle, get_pose_receiver = (
        make_pose_getter(k) for k in range(3))

    via = None if args.auto_via else (args.vx, args.vy)
    first_aim = via if via else (0.5 * (args.bx + args.tx), 0.5 * (args.by + args.ty))

    try:
        if not args.no_teleport:
            # Passer faces the via point, middle faces the ball, receiver faces the via point.
            setup.teleport_robot(sock, ctl_endpoint, args.team, args.passer,
                                 x_m=args.px, y_m=args.py,
                                 heading_rad=math.atan2(first_aim[1] - args.by,
                                                        first_aim[0] - args.bx))
            setup.teleport_robot(sock, ctl_endpoint, args.team, args.middle,
                                 x_m=args.mx, y_m=args.my,
                                 heading_rad=math.atan2(args.by - args.my,
                                                        args.bx - args.mx))
            setup.teleport_robot(sock, ctl_endpoint, args.team, args.receiver,
                                 x_m=args.rx, y_m=args.ry,
                                 heading_rad=math.atan2(first_aim[1] - args.ry,
                                                        first_aim[0] - args.rx))
            setup.teleport_ball(sock, ctl_endpoint, x_m=args.bx, y_m=args.by)
            time.sleep(0.7)

        deadline = time.monotonic() + 3.0
        while (get_pose_passer() is None or get_pose_middle() is None
               or get_pose_receiver() is None or get_ball() is None):
            if time.monotonic() > deadline:
                sys.exit("No vision for all three robots and the ball. Check grSim "
                         "vision settings and that everything is on the field.")
            time.sleep(0.01)
        print("passer:", get_pose_passer(), "middle:", get_pose_middle(),
              "receiver:", get_pose_receiver(), "ball:", get_ball())

        kwargs = {}
        if via:
            kwargs.update(via_x_m=via[0], via_y_m=via[1])
        if args.arrival is not None:
            kwargs["arrival_speed_mps"] = args.arrival

        kwargs.update(chip_leg1=args.chip1, chip_leg2=args.chip2)

        t0 = time.monotonic()
        ok = indirect_pass(sock, cmd_endpoint, args.passer, args.middle, args.receiver,
                           get_pose_passer, get_pose_middle, get_pose_receiver, get_ball,
                           target_x_m=args.tx, target_y_m=args.ty,
                           timeout_s=args.timeout, **kwargs)
        elapsed = time.monotonic() - t0
        print(f"indirect pass completed in {elapsed:.2f} s" if ok
              else f"FAILED / timed out after {elapsed:.2f} s")

        # Did the ball end up on the target, and is it still on the receiver's dribbler?
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
        for v in visions:
            v.close()


if __name__ == "__main__":
    main()