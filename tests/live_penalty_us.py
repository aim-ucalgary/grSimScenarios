"""Live test: prepare our penalty (shooter lines up without kicking, others 1 m behind the ball).

Usage (from the repo root, grSim already running):
    python tests/live_penalty_us.py --team blue --shooter 1 --others 0 2 3 4 5 \
        --goal-x 4.5 --ball-x 2.5
Set --goal-x / --ball-x for your field.
The test passes if the shooter ends at the ball's kicker face, facing the goal,
the ball has not moved, and the others sit 1 m behind the ball.
Add --no-teleport to use the current robot/ball positions.
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
from tactics import penalty_us  # make sure tactics/__init__.py exports it
from skills.vision import VisionTracker, open_vision_socket

CONTROL_PORT = 10300
TEAM_PORTS = {"blue": 10301, "yellow": 10302}
BALL_MOVED_TOL_M = 0.05   # more than this = the shooter pushed/kicked the ball


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--team", default="blue", choices=["blue", "yellow"])
    ap.add_argument("--shooter", type=int, default=1)
    ap.add_argument("--others", type=int, nargs="*", default=[0, 2, 3, 4, 5],
                    help="robots waiting behind the ball (default includes goalie 0)")
    ap.add_argument("--goal-x", type=float, default=4.5, help="goal we shoot at, X in m")
    ap.add_argument("--goal-y", type=float, default=0.0)
    ap.add_argument("--ball-x", type=float, default=2.5, help="penalty mark X, m")
    ap.add_argument("--ball-y", type=float, default=0.0)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--timeout", type=float, default=20.0)
    ap.add_argument("--no-teleport", action="store_true")
    args = ap.parse_args()

    cmd_endpoint = (args.host, TEAM_PORTS[args.team])
    ctl_endpoint = (args.host, CONTROL_PORT)
    sock = com.open_socket()

    # One vision socket + tracker + lock per robot; the ball is read via the shooter's.
    ids = [args.shooter] + args.others
    visions = [open_vision_socket() for _ in ids]
    trackers = {i: VisionTracker(v, args.team, i) for i, v in zip(ids, visions)}
    locks = {i: threading.Lock() for i in ids}

    def make_pose_getter(i):
        def get():
            with locks[i]:
                return trackers[i].get_pose()
        return get

    get_pose_by_id = {i: make_pose_getter(i) for i in ids}

    def get_ball():
        with locks[args.shooter]:
            return trackers[args.shooter].get_ball()

    try:
        if not args.no_teleport:
            back = -1.0 if args.goal_x > 0 else 1.0   # direction away from the goal
            setup.teleport_robot(sock, ctl_endpoint, args.team, args.shooter,
                                 x_m=args.ball_x + back * 2.0, y_m=1.0,
                                 heading_rad=0.0 if args.goal_x > 0 else math.pi)
            for n, rid in enumerate(args.others):
                setup.teleport_robot(sock, ctl_endpoint, args.team, rid,
                                     x_m=args.ball_x + back * 3.0,
                                     y_m=(1.2 + 0.7 * (n // 2)) * (1 if n % 2 else -1),
                                     heading_rad=0.0 if args.goal_x > 0 else math.pi)
            setup.teleport_ball(sock, ctl_endpoint, x_m=args.ball_x, y_m=args.ball_y)
            time.sleep(0.7)

        deadline = time.monotonic() + 3.0
        while any(get_pose_by_id[i]() is None for i in ids) or get_ball() is None:
            if time.monotonic() > deadline:
                sys.exit("No vision for all robots and the ball. Check grSim vision "
                         "settings and that everything is on the field.")
            time.sleep(0.01)
        ball0 = get_ball()
        print("ball:", ball0, {i: get_pose_by_id[i]() for i in ids})

        t0 = time.monotonic()
        ready = penalty_us(sock, cmd_endpoint, args.shooter, args.others,
                           get_pose_by_id, get_ball,
                           goal_x_m=args.goal_x, goal_y_m=args.goal_y,
                           timeout_s=args.timeout)
        elapsed = time.monotonic() - t0
        print(f"shooter ready after {elapsed:.1f} s" if ready
              else f"FAILED / timed out after {elapsed:.1f} s")

        time.sleep(1.0)   # nothing should happen now: check that everything stays put
        ball = get_ball()
        shot_dir = math.atan2(args.goal_y - ball0[1], args.goal_x - ball0[0])
        ax, ay = math.cos(shot_dir), math.sin(shot_dir)

        moved = math.hypot(ball[0] - ball0[0], ball[1] - ball0[1])
        print(f"ball moved {moved:.3f} m: "
              + ("OK, not kicked" if moved <= BALL_MOVED_TOL_M else "FAIL, shooter moved the ball"))

        s = get_pose_by_id[args.shooter]()
        dx, dy = ball0[0] - s[0], ball0[1] - s[1]
        gap = math.hypot(dx, dy)
        lateral = -dx * ay + dy * ax
        head_err = math.degrees(math.atan2(math.sin(shot_dir - s[2]), math.cos(shot_dir - s[2])))
        print(f"shooter: {gap:.3f} m from the ball, {lateral:+.3f} m off the shot line, "
              f"{head_err:+.1f} deg heading error")

        for rid in args.others:
            p = get_pose_by_id[rid]()
            dx, dy = p[0] - ball0[0], p[1] - ball0[1]
            behind = -(dx * ax + dy * ay)   # >0 = on the far side from the goal
            side = -dx * ay + dy * ax
            print(f"robot {rid}: {behind:.2f} m behind the ball, {side:+.2f} m sideways")
    finally:
        sock.close()
        for v in visions:
            v.close()


if __name__ == "__main__":
    main()