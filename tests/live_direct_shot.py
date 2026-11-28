"""Live test: direct_shot_at_goal against an enemy goalie (and optional blockers) in grSim.

Usage (from the repo root, grSim already running):
    python tests/live_direct_shot.py --team blue --shooter 0 --goal-x 4.5 --shots 5
    python tests/live_direct_shot.py --blocker 3.0 0.2 --blocker 2.5 -0.4
Set --goal-x / --goal-half-width for your field (Division B goals are 1.0 m wide
-> 0.5, Division A 1.8 m wide -> 0.9).

The enemy goalie (enemy team, id --enemy-goalie) runs goalie_defend. Blockers are
extra enemy robots (ids 1, 2, ...) that stand still at the given positions. Each
shot is planned from the live enemy positions, printed, then taken; the result is
GOAL if the ball ends up in the net, otherwise no goal (saved / blocked / missed).
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
from tactics import goalie_defend  # make sure skills/__init__.py exports it
from tactics import direct_shot_at_goal  # make sure tactics/__init__.py exports it
from tactics.direct_shot_at_goal import plan_goal_shot
from skills.vision import VisionTracker, open_vision_socket

CONTROL_PORT = 10300
TEAM_PORTS = {"blue": 10301, "yellow": 10302}
WATCH_S = 4.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--team", default="blue", choices=["blue", "yellow"])
    ap.add_argument("--shooter", type=int, default=0)
    ap.add_argument("--enemy-goalie", type=int, default=0)
    ap.add_argument("--goal-x", type=float, default=4.5, help="goal we shoot at, X in m")
    ap.add_argument("--goal-y", type=float, default=0.0)
    ap.add_argument("--goal-half-width", type=float, default=0.5)
    ap.add_argument("--ball-x", type=float, default=1.5)
    ap.add_argument("--ball-y", type=float, default=0.3)
    ap.add_argument("--blocker", type=float, nargs=2, action="append", default=[],
                    metavar=("X", "Y"), help="static enemy blocker (repeatable)")
    ap.add_argument("--shots", type=int, default=3)
    ap.add_argument("--require-clear", action="store_true",
                    help="refuse to shoot when no lane is open")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--timeout", type=float, default=25.0)
    args = ap.parse_args()

    enemy_team = "yellow" if args.team == "blue" else "blue"
    inward = 1.0 if args.goal_x < 0 else -1.0   # +1 when the field is at larger x
    cmd_endpoint = (args.host, TEAM_PORTS[args.team])
    enemy_endpoint = (args.host, TEAM_PORTS[enemy_team])
    ctl_endpoint = (args.host, CONTROL_PORT)
    sock = com.open_socket()

    # Enemy robot ids: the goalie plus one id per blocker (1, 2, ...; skipping the goalie's).
    blocker_ids, nxt = [], 0
    for _ in args.blocker:
        nxt += 1
        while nxt == args.enemy_goalie:
            nxt += 1
        blocker_ids.append(nxt)
    enemy_ids = [args.enemy_goalie] + blocker_ids

    # One vision socket + tracker + lock per robot; the ball is read via the shooter's.
    visions = [open_vision_socket() for _ in range(1 + len(enemy_ids))]
    shooter_tracker = VisionTracker(visions[0], args.team, args.shooter)
    enemy_trackers = {i: VisionTracker(v, enemy_team, i) for i, v in zip(enemy_ids, visions[1:])}
    shooter_lock = threading.Lock()
    enemy_locks = {i: threading.Lock() for i in enemy_ids}

    def get_pose():
        with shooter_lock:
            return shooter_tracker.get_pose()

    def get_ball():
        with shooter_lock:
            return shooter_tracker.get_ball()

    def make_enemy_pose_getter(i):
        def get():
            with enemy_locks[i]:
                return enemy_trackers[i].get_pose()
        return get

    enemy_pose = {i: make_enemy_pose_getter(i) for i in enemy_ids}

    def get_enemies():
        poses = [enemy_pose[i]() for i in enemy_ids]
        return [(p[0], p[1]) for p in poses if p]

    def in_net(b):
        return (b[0] - args.goal_x) * inward < 0 and abs(b[1] - args.goal_y) <= args.goal_half_width

    stop = threading.Event()

    def enemy_goalie_thread():
        goalie_defend(sock, enemy_endpoint, args.enemy_goalie, enemy_pose[args.enemy_goalie],
                      get_ball, goal_x_m=args.goal_x, goal_y_m=args.goal_y,
                      goal_half_width_m=args.goal_half_width,
                      clear_ball=False, should_stop=stop.is_set, timeout_s=3600.0)

    try:
        heading_to_goal = 0.0 if inward < 0 else math.pi
        for (bx, by), rid in zip(args.blocker, blocker_ids):
            setup.teleport_robot(sock, ctl_endpoint, enemy_team, rid, x_m=bx, y_m=by,
                                 heading_rad=heading_to_goal + math.pi)
        setup.teleport_robot(sock, ctl_endpoint, enemy_team, args.enemy_goalie,
                             x_m=args.goal_x + inward * 0.15, y_m=args.goal_y,
                             heading_rad=heading_to_goal + math.pi)
        thread = None
        goals = taken = 0

        for n in range(1, args.shots + 1):
            # Reset the shooter and ball for each attempt.
            setup.teleport_robot(sock, ctl_endpoint, args.team, args.shooter,
                                 x_m=args.ball_x + inward * 1.5,   # 1.5 m behind the ball, away from the goal
                                 y_m=args.ball_y - 0.8, heading_rad=heading_to_goal)
            setup.teleport_ball(sock, ctl_endpoint, x_m=args.ball_x, y_m=args.ball_y)
            time.sleep(0.7)

            deadline = time.monotonic() + 3.0
            while (get_pose() is None or get_ball() is None
                   or any(enemy_pose[i]() is None for i in enemy_ids)):
                if time.monotonic() > deadline:
                    sys.exit("No vision for every robot and the ball. Check grSim vision settings.")
                time.sleep(0.01)
            if thread is None:   # start the enemy goalie once everything is visible
                thread = threading.Thread(target=enemy_goalie_thread, daemon=True)
                thread.start()
                time.sleep(1.0)

            ball = get_ball()
            plan = plan_goal_shot(ball, args.goal_x, args.goal_y, args.goal_half_width,
                                  get_enemies())
            print(f"shot {n}: aim y={plan['aim_y']:+.2f}, open lane={plan['clear']}, "
                  f"open angle {math.degrees(plan['free_angle_rad']):.1f} deg, "
                  f"clearance {plan['clearance_m']:.2f} m")

            ok = direct_shot_at_goal(
                sock, cmd_endpoint, args.shooter, get_pose, get_ball, get_enemies,
                goal_x_m=args.goal_x, goal_y_m=args.goal_y,
                goal_half_width_m=args.goal_half_width,
                require_clear=args.require_clear, timeout_s=args.timeout)
            if not ok:
                print("   no kick delivered (refused / failed / timed out)")
                continue
            taken += 1

            scored, end = False, time.monotonic() + WATCH_S
            while time.monotonic() < end:
                b = get_ball()
                if b and in_net(b):
                    scored = True
                time.sleep(0.02)
            goals += scored
            b = get_ball()
            print("   GOAL" if scored else
                  f"   no goal (ball at ({b[0]:.2f}, {b[1]:.2f}))" if b else "   no goal")
        print(f"scored {goals} of {taken} shots taken")
    finally:
        stop.set()
        time.sleep(0.2)
        sock.close()
        for v in visions:
            v.close()


if __name__ == "__main__":
    main()