"""Live test: defend a penalty (goalie on the goal line, others 1 m behind the ball).

Usage (from the repo root, grSim already running):
    python tests/live_penalty_them.py --team blue --goalie 0 --others 1 2 3 4 5 \
        --goal-x -4.5 --ball-x -2.5 --shoot --aim-y 0.3
Set --goal-x / --goal-half-width / --ball-x for your field (Division B goals are
1.0 m wide -> 0.5, Division A 1.8 m wide -> 0.9).
Without --shoot nobody shoots: the tactic just holds positions until it times out.
With --shoot the ball is launched at the goal after --shot-delay seconds, using
teleport_ball with a velocity.
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
from tactics import penalty_them  # make sure tactics/__init__.py exports it
from skills.vision import VisionTracker, open_vision_socket

CONTROL_PORT = 10300
TEAM_PORTS = {"blue": 10301, "yellow": 10302}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--team", default="blue", choices=["blue", "yellow"])
    ap.add_argument("--goalie", type=int, default=0)
    ap.add_argument("--others", type=int, nargs="*", default=[1, 2, 3, 4, 5])
    ap.add_argument("--goal-x", type=float, default=-4.5, help="our goal line X, m")
    ap.add_argument("--goal-y", type=float, default=0.0)
    ap.add_argument("--goal-half-width", type=float, default=0.5)
    ap.add_argument("--ball-x", type=float, default=-2.5, help="penalty mark X, m")
    ap.add_argument("--ball-y", type=float, default=0.0)
    ap.add_argument("--shoot", action="store_true", help="launch the ball at the goal")
    ap.add_argument("--shot-delay", type=float, default=4.0, help="s after start")
    ap.add_argument("--shot-speed", type=float, default=4.0, help="m/s")
    ap.add_argument("--aim-y", type=float, default=0.3, help="Y on the goal line to shoot at")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--timeout", type=float, default=30.0)
    ap.add_argument("--no-teleport", action="store_true")
    args = ap.parse_args()

    cmd_endpoint = (args.host, TEAM_PORTS[args.team])
    ctl_endpoint = (args.host, CONTROL_PORT)
    sock = com.open_socket()

    # One vision socket + tracker + lock per robot; the ball is read via the goalie's.
    ids = [args.goalie] + args.others
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
        with locks[args.goalie]:
            return trackers[args.goalie].get_ball()

    try:
        if not args.no_teleport:
            inward = 0.0 if args.goal_x < 0 else math.pi
            setup.teleport_robot(sock, ctl_endpoint, args.team, args.goalie,
                                 x_m=args.goal_x + (0.8 if args.goal_x < 0 else -0.8),
                                 y_m=1.0, heading_rad=inward)
            for n, rid in enumerate(args.others):
                setup.teleport_robot(sock, ctl_endpoint, args.team, rid,
                                     x_m=args.ball_x + (0.5 if args.goal_x < 0 else -0.5),
                                     y_m=(1.2 + 0.7 * (n // 2)) * (1 if n % 2 == 0 else -1),
                                     heading_rad=inward)
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

        # Background: launch the shot, and watch the goalie stay on the goal line.
        stats = {"max_line_dev": 0.0}
        stop = threading.Event()

        def watcher():
            t0 = time.monotonic()
            shot_done = not args.shoot
            while not stop.is_set():
                pose = get_pose_by_id[args.goalie]()
                if pose and time.monotonic() - t0 > 2.0:  # after it has arrived
                    stats["max_line_dev"] = max(stats["max_line_dev"],
                                                abs(pose[0] - args.goal_x))
                if not shot_done and time.monotonic() - t0 >= args.shot_delay:
                    dx, dy = args.goal_x - ball0[0], args.aim_y - ball0[1]
                    d = math.hypot(dx, dy)
                    setup.teleport_ball(sock, ctl_endpoint, x_m=ball0[0], y_m=ball0[1],
                                        vx_mps=args.shot_speed * dx / d,
                                        vy_mps=args.shot_speed * dy / d)
                    print(f"shot launched at y={args.aim_y:.2f} with {args.shot_speed} m/s")
                    shot_done = True
                time.sleep(0.03)

        watch = threading.Thread(target=watcher, daemon=True)
        watch.start()

        t0 = time.monotonic()
        ended = penalty_them(sock, cmd_endpoint, args.goalie, args.others,
                             get_pose_by_id, get_ball,
                             goal_x_m=args.goal_x, goal_y_m=args.goal_y,
                             goal_half_width_m=args.goal_half_width,
                             timeout_s=args.timeout)
        elapsed = time.monotonic() - t0
        stop.set()
        watch.join(timeout=1.0)
        print(f"shot over after {elapsed:.1f} s" if ended
              else f"timed out after {elapsed:.1f} s (no shot finished)")

        time.sleep(0.5)
        ball = get_ball()
        if ball and args.shoot:
            inward_sign = 1.0 if args.goal_x < 0 else -1.0
            goal = (ball[0] - args.goal_x) * inward_sign < 0
            print(f"ball at ({ball[0]:.2f}, {ball[1]:.2f}): "
                  + ("GOAL (not blocked)" if goal else "BLOCKED / saved"))
        print(f"goalie max distance from the goal line: {stats['max_line_dev']:.3f} m")
        # Do the others sit 1 m behind the ball, on the far side from our goal?
        for rid in args.others:
            p = get_pose_by_id[rid]()
            if p:
                axis = (ball0[0] - args.goal_x, ball0[1] - args.goal_y)
                n = math.hypot(*axis)
                ax, ay = axis[0] / n, axis[1] / n
                dx, dy = p[0] - ball0[0], p[1] - ball0[1]
                behind = dx * ax + dy * ay   # >0 = further from our goal than the ball
                side = -dx * ay + dy * ax
                print(f"robot {rid}: {behind:.2f} m behind the ball, {side:+.2f} m sideways")
    finally:
        sock.close()
        for v in visions:
            v.close()


if __name__ == "__main__":
    main()