"""Live test: goalie_defend against a series of shots (and a loose-ball clearance) in grSim.

Usage (from the repo root, grSim already running):
    python tests/live_goalie_defend.py --team blue --goalie 0 --goal-x -4.5 --shots 8
    python tests/live_goalie_defend.py --clear-test
Set --goal-x / --goal-half-width for your field (Division B goals are 1.0 m wide
-> 0.5, Division A 1.8 m wide -> 0.9).

Shots are launched with teleport_ball plus a velocity from a random point in front
of the goal, aimed at a random point on or slightly outside the goal. Only shots
aimed clearly inside the posts count as "on target"; the score is saves / on target.
--clear-test places the ball at rest in front of the goal and checks that the goalie
kicks it away without conceding.
"""

import argparse
import math
import random
import sys
import threading
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))
sys.path.insert(0, str(REPO_ROOT / "grSim" / "pb"))   # generated *_pb2.py files

from helpers import communication as com
from helpers import setup
from tactics import goalie_defend  # make sure skills/__init__.py exports it
from skills.vision import VisionTracker, open_vision_socket

CONTROL_PORT = 10300
TEAM_PORTS = {"blue": 10301, "yellow": 10302}
ON_TARGET_MARGIN_M = 0.05   # aim must be this far inside the posts to count
SHOT_WATCH_S = 3.5


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--team", default="blue", choices=["blue", "yellow"])
    ap.add_argument("--goalie", type=int, default=0)
    ap.add_argument("--goal-x", type=float, default=-4.5, help="our goal line X, m")
    ap.add_argument("--goal-y", type=float, default=0.0)
    ap.add_argument("--goal-half-width", type=float, default=0.5)
    ap.add_argument("--shots", type=int, default=6)
    ap.add_argument("--speed-min", type=float, default=2.5, help="shot speed range, m/s")
    ap.add_argument("--speed-max", type=float, default=5.0)
    ap.add_argument("--seed", type=int, default=None)
    ap.add_argument("--clear-test", action="store_true",
                    help="instead of shots, test clearing a ball resting in front of the goal")
    ap.add_argument("--host", default="127.0.0.1")
    args = ap.parse_args()
    if args.seed is not None:
        random.seed(args.seed)

    inward = 1.0 if args.goal_x < 0 else -1.0   # +1 when the field is at larger x
    cmd_endpoint = (args.host, TEAM_PORTS[args.team])
    ctl_endpoint = (args.host, CONTROL_PORT)
    sock = com.open_socket()
    vision = open_vision_socket()
    tracker = VisionTracker(vision, args.team, args.goalie)
    lock = threading.Lock()

    def get_pose():
        with lock:
            return tracker.get_pose()

    def get_ball():
        with lock:
            return tracker.get_ball()

    def in_net(b):
        return (b[0] - args.goal_x) * inward < 0 and abs(b[1] - args.goal_y) <= args.goal_half_width

    stop = threading.Event()
    result = {}

    def goalie_thread():
        result["no_goal"] = goalie_defend(
            sock, cmd_endpoint, args.goalie, get_pose, get_ball,
            goal_x_m=args.goal_x, goal_y_m=args.goal_y,
            goal_half_width_m=args.goal_half_width,
            clear_ball=args.clear_test,   # shots mode: no clearing, the ball is teleported away
            should_stop=stop.is_set, timeout_s=3600.0)

    def watch(seconds):
        """Return (ever_in_net, final_ball) after watching the ball for a while."""
        scored, end = False, time.monotonic() + seconds
        while time.monotonic() < end:
            b = get_ball()
            if b and in_net(b):
                scored = True
            time.sleep(0.02)
        return scored, get_ball()

    try:
        setup.teleport_robot(sock, ctl_endpoint, args.team, args.goalie,
                             x_m=args.goal_x + inward * 0.15, y_m=args.goal_y,
                             heading_rad=0.0 if inward > 0 else math.pi)
        setup.teleport_ball(sock, ctl_endpoint, x_m=args.goal_x + inward * 4.0, y_m=3.0)
        time.sleep(0.7)
        deadline = time.monotonic() + 3.0
        while get_pose() is None or get_ball() is None:
            if time.monotonic() > deadline:
                sys.exit("No vision for the goalie and/or ball. Check grSim vision settings.")
            time.sleep(0.01)

        thread = threading.Thread(target=goalie_thread, daemon=True)
        thread.start()
        time.sleep(1.0)

        if args.clear_test:
            bx, by = args.goal_x + inward * 0.7, args.goal_y + 0.3
            setup.teleport_ball(sock, ctl_endpoint, x_m=bx, y_m=by)
            print(f"ball placed at rest at ({bx:.2f}, {by:.2f}); waiting for the goalie to clear it")
            scored, ball = watch(15.0)
            if ball is None:
                print("FAIL: lost the ball")
            else:
                moved = math.hypot(ball[0] - bx, ball[1] - by)
                out = (ball[0] - args.goal_x) * inward
                ok = (not scored) and moved >= 1.0 and out > 1.0
                print(f"ball ended at ({ball[0]:.2f}, {ball[1]:.2f}), moved {moved:.2f} m, "
                      f"{out:.2f} m from the goal line, own goal={scored}: "
                      + ("PASS" if ok else "FAIL"))
        else:
            on_target = saves = goals = 0
            for n in range(1, args.shots + 1):
                dist = random.uniform(2.0, 3.5)
                sx, sy = args.goal_x + inward * dist, random.uniform(-1.5, 1.5)
                aim_y = args.goal_y + random.uniform(-(args.goal_half_width + 0.2),
                                                     args.goal_half_width + 0.2)
                speed = random.uniform(args.speed_min, args.speed_max)
                dx, dy = args.goal_x - sx, aim_y - sy
                d = math.hypot(dx, dy)
                counts = abs(aim_y - args.goal_y) <= args.goal_half_width - ON_TARGET_MARGIN_M
                setup.teleport_ball(sock, ctl_endpoint, x_m=sx, y_m=sy,
                                    vx_mps=speed * dx / d, vy_mps=speed * dy / d)
                scored, ball = watch(SHOT_WATCH_S)
                if counts:
                    on_target += 1
                    goals += scored
                    saves += not scored
                label = ("GOAL" if scored else "save") if counts else "off target"
                print(f"shot {n}: from ({sx:.1f}, {sy:+.1f}) aimed y={aim_y:+.2f} "
                      f"at {speed:.1f} m/s -> {label}")
                # Park the ball away and give the goalie a moment to reset.
                setup.teleport_ball(sock, ctl_endpoint, x_m=args.goal_x + inward * 4.0, y_m=3.0)
                time.sleep(1.5)
            print(f"saved {saves} of {on_target} on-target shots ({goals} goals conceded)")
    finally:
        stop.set()
        time.sleep(0.2)
        sock.close()
        vision.close()


if __name__ == "__main__":
    main()