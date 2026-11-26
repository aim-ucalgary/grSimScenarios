"""Live test: pass the ball while an enemy robot moves, in a running grSim.

Usage (from the repo root, grSim already running):
    python tests/live_moving_pass.py --team blue --passer 0 --receiver 1 \
        --tx 2.0 --ty 0.0 --enemy-mode patrol
Enemy modes:
    static  - enemy stays at its start position (--ex, --ey)
    patrol  - enemy runs back and forth between (--ex, --ey) and (--ex2, --ey2)
    block   - enemy drives to the pass line (at --block-frac of its length) and holds
Add --no-chip to forbid chip passes, --require-safe to refuse unsafe passes.
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
from skills.go_to_point import go_to_point
from tactics import moving_pass  # make sure tactics/__init__.py exports it
from tactics.moving_pass import (
    plan_moving_pass, sample_enemies, PASSER_SPEED_MPS, PREP_OVERHEAD_S)
from skills.vision import VisionTracker, open_vision_socket

CONTROL_PORT = 10300
TEAM_PORTS = {"blue": 10301, "yellow": 10302}
ENEMY_CHUNK_S = 0.5      # go_to_point call length, so the enemy thread can be stopped
TOUCH_DISTANCE_M = 0.12  # enemy-to-ball distance counted as an interception


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--team", default="blue", choices=["blue", "yellow"])
    ap.add_argument("--passer", type=int, default=0)
    ap.add_argument("--receiver", type=int, default=1)
    ap.add_argument("--enemy-id", type=int, default=0)
    ap.add_argument("--tx", type=float, default=2.0, help="pass target X, m")
    ap.add_argument("--ty", type=float, default=0.0, help="pass target Y, m")
    ap.add_argument("--enemy-mode", default="patrol", choices=["static", "patrol", "block"])
    ap.add_argument("--block-frac", type=float, default=0.5,
                    help="block mode: position along the pass line, 0 = ball, 1 = target")
    ap.add_argument("--no-chip", action="store_true", help="forbid chip passes")
    ap.add_argument("--require-safe", action="store_true",
                    help="refuse the pass if no safe option exists")
    ap.add_argument("--host", default="127.0.0.1")
    # Start positions (used unless --no-teleport).
    ap.add_argument("--px", type=float, default=-2.0)
    ap.add_argument("--py", type=float, default=0.0)
    ap.add_argument("--rx", type=float, default=1.0)
    ap.add_argument("--ry", type=float, default=1.5)
    ap.add_argument("--bx", type=float, default=-1.5)
    ap.add_argument("--by", type=float, default=0.0)
    ap.add_argument("--ex", type=float, default=0.0, help="enemy start X")
    ap.add_argument("--ey", type=float, default=1.5, help="enemy start Y")
    ap.add_argument("--ex2", type=float, default=0.0, help="patrol: second point X")
    ap.add_argument("--ey2", type=float, default=-1.5, help="patrol: second point Y")
    ap.add_argument("--timeout", type=float, default=25.0)
    ap.add_argument("--no-teleport", action="store_true")
    args = ap.parse_args()

    enemy_team = "yellow" if args.team == "blue" else "blue"
    cmd_endpoint = (args.host, TEAM_PORTS[args.team])
    enemy_endpoint = (args.host, TEAM_PORTS[enemy_team])
    ctl_endpoint = (args.host, CONTROL_PORT)
    sock = com.open_socket()

    # One vision socket + tracker per robot; every getter takes its tracker's lock.
    # The ball is read through the passer's tracker.
    visions = [open_vision_socket() for _ in range(3)]
    trackers = [VisionTracker(visions[0], args.team, args.passer),
                VisionTracker(visions[1], args.team, args.receiver),
                VisionTracker(visions[2], enemy_team, args.enemy_id)]
    locks = [threading.Lock() for _ in trackers]

    def make_pose_getter(k):
        def get():
            with locks[k]:
                return trackers[k].get_pose()
        return get

    def get_ball():
        with locks[0]:
            return trackers[0].get_ball()

    get_pose_passer, get_pose_receiver, get_pose_enemy = (make_pose_getter(k) for k in range(3))

    def get_enemies():
        pose = get_pose_enemy()
        return [] if pose is None else [(pose[0], pose[1])]

    stop = threading.Event()

    def enemy_task():
        # Drives the enemy according to the chosen mode until stop is set.
        if args.enemy_mode == "static":
            return
        if args.enemy_mode == "block":
            ball = get_ball() or (args.bx, args.by)
            goal = (ball[0] + args.block_frac * (args.tx - ball[0]),
                    ball[1] + args.block_frac * (args.ty - ball[1]))
            while not stop.is_set():
                go_to_point(sock, enemy_endpoint, args.enemy_id, get_pose_enemy,
                            x_m=goal[0], y_m=goal[1], timeout_s=ENEMY_CHUNK_S)
            return
        points = [(args.ex2, args.ey2), (args.ex, args.ey)]
        k = 0
        while not stop.is_set():
            # Re-issue the same goal in chunks until reached, then switch ends.
            reached = go_to_point(sock, enemy_endpoint, args.enemy_id, get_pose_enemy,
                                  x_m=points[k][0], y_m=points[k][1],
                                  timeout_s=ENEMY_CHUNK_S)
            if reached:
                k = 1 - k

    closest = {"d": math.inf, "t": None}

    def monitor_task():
        # Logs how close the enemy gets to the ball during the pass.
        t0 = time.monotonic()
        while not stop.is_set():
            ball, enemy = get_ball(), get_pose_enemy()
            if ball and enemy:
                d = math.hypot(ball[0] - enemy[0], ball[1] - enemy[1])
                if d < closest["d"]:
                    closest["d"], closest["t"] = d, time.monotonic() - t0
            time.sleep(0.03)

    enemy_thread = threading.Thread(target=enemy_task, daemon=True)
    monitor_thread = threading.Thread(target=monitor_task, daemon=True)

    try:
        if not args.no_teleport:
            passer_heading = math.atan2(args.ty - args.by, args.tx - args.bx)
            receiver_heading = math.atan2(args.by - args.ry, args.bx - args.rx)
            setup.teleport_robot(sock, ctl_endpoint, args.team, args.passer,
                                 x_m=args.px, y_m=args.py, heading_rad=passer_heading)
            setup.teleport_robot(sock, ctl_endpoint, args.team, args.receiver,
                                 x_m=args.rx, y_m=args.ry, heading_rad=receiver_heading)
            setup.teleport_robot(sock, ctl_endpoint, enemy_team, args.enemy_id,
                                 x_m=args.ex, y_m=args.ey, heading_rad=0.0)
            setup.teleport_ball(sock, ctl_endpoint, x_m=args.bx, y_m=args.by)
            time.sleep(0.7)

        deadline = time.monotonic() + 3.0
        while (get_pose_passer() is None or get_pose_receiver() is None
               or get_pose_enemy() is None or get_ball() is None):
            if time.monotonic() > deadline:
                sys.exit("No vision for the passer, receiver, enemy and/or ball. Check "
                         "grSim vision settings and that everything is on the field.")
            time.sleep(0.01)
        print("passer:", get_pose_passer(), "receiver:", get_pose_receiver(),
              "enemy:", get_pose_enemy(), "ball:", get_ball())

        # Start the enemy moving, let it get going, then show what the planner thinks.
        enemy_thread.start()
        monitor_thread.start()
        time.sleep(0.5)
        ball, passer = get_ball(), get_pose_passer()
        t_prep = math.hypot(passer[0] - ball[0], passer[1] - ball[1]) / PASSER_SPEED_MPS \
            + PREP_OVERHEAD_S
        plan = plan_moving_pass(ball, (args.tx, args.ty), sample_enemies(get_enemies),
                                t_prep_s=t_prep, allow_chip=not args.no_chip)
        kind = "chip" if plan["chip"] else f"ground, arrival {plan['arrival_speed_mps']} m/s"
        print(f"plan: {kind}; completion {plan['completion_s']:.2f} s; "
              f"worst-case margin {plan['margin_s']:.2f} s; "
              f"predicted clearance {plan['clearance_m']:.2f} m; "
              f"guaranteed={plan['guaranteed']}, safe={plan['safe']}")

        t0 = time.monotonic()
        ok = moving_pass(sock, cmd_endpoint, args.passer, args.receiver,
                         get_pose_passer, get_pose_receiver, get_ball, get_enemies,
                         target_x_m=args.tx, target_y_m=args.ty,
                         allow_chip=not args.no_chip, require_safe=args.require_safe,
                         timeout_s=args.timeout)
        elapsed = time.monotonic() - t0
        print(f"pass completed in {elapsed:.2f} s" if ok
              else f"FAILED / refused / timed out after {elapsed:.2f} s")

        time.sleep(1.5)
        stop.set()
        enemy_thread.join(timeout=2.0)
        monitor_thread.join(timeout=2.0)

        print(f"closest enemy-to-ball distance: {closest['d']:.3f} m "
              + ("(INTERCEPTED?)" if closest["d"] < TOUCH_DISTANCE_M else "(clear)"))
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
        stop.set()
        sock.close()
        for v in visions:
            v.close()


if __name__ == "__main__":
    main()