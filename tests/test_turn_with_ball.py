"""Closed-loop test for skills.turn_with_ball (approach, hold at coordinates, 360 turn)."""

import math
import sys
from pathlib import Path
from unittest.mock import MagicMock

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

for _mod in [
    "ssl_simulation_robot_control_pb2",
    "ssl_simulation_robot_feedback_pb2",
    "ssl_simulation_control_pb2",
    "ssl_gc_common_pb2",
    "ssl_vision_wrapper_pb2",
    "messages_robocup_ssl_wrapper_pb2",
]:
    if _mod not in sys.modules:
        sys.modules[_mod] = MagicMock()

from helpers import behaviors
from skills import turn_with_ball

mod = sys.modules["skills.turn_with_ball"]

DT = 0.02  # simulated seconds per command
CONTACT = behaviors.ROBOT_FRONT_M + behaviors.BALL_RADIUS_M
TWO_PI = 2 * math.pi


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class World:
    """Point-mass robot; the ball sticks to the kicker face while the dribbler is on."""

    def __init__(self, robot, ball, drop_after_rad=None, can_hold=True):
        self.robot, self.ball = list(robot), list(ball)
        self.ball_v = [0.0, 0.0]
        self.carried = False
        self.can_hold = can_hold
        self.turned = 0.0            # accumulated |w| * dt while the dribbler is on
        self.drop_after_rad, self.dropped = drop_after_rad, False
        self.sent = []               # (vx, vy, w, dribbler_rpm)
        self.turns = []              # one record per _turn_with_ball call

    def pose(self):
        return (self.robot[0], self.robot[1], wrap(self.robot[2]))  # like real vision

    def send(self, sock, endpoint, robot_id, vx=0.0, vy=0.0, w=0.0, *,
             kick_speed=0.0, kick_angle=0.0, dribbler_rpm=0.0):
        self.sent.append((vx, vy, w, dribbler_rpm))
        r, b = self.robot, self.ball
        r[0] += vx * DT; r[1] += vy * DT; r[2] += w * DT
        if dribbler_rpm > 0:
            self.turned += abs(w) * DT
        c, s = math.cos(r[2]), math.sin(r[2])
        if dribbler_rpm <= 0:
            self.carried = False

        f = (b[0] - r[0]) * c + (b[1] - r[1]) * s
        l = -(b[0] - r[0]) * s + (b[1] - r[1]) * c
        if (self.can_hold and dribbler_rpm > 0 and not self.carried
                and math.hypot(*self.ball_v) < 0.05 and 0 <= f <= 0.13 and abs(l) <= 0.04):
            self.carried = True

        if (self.carried and self.drop_after_rad is not None and not self.dropped
                and self.turned >= self.drop_after_rad):
            self.carried, self.dropped = False, True
            self.ball_v = [1.2 * c, 1.2 * s]  # ball pops out ahead

        if self.carried:
            b[0], b[1] = r[0] + CONTACT * c, r[1] + CONTACT * s
            self.ball_v = [0.0, 0.0]
        else:
            b[0] += self.ball_v[0] * DT
            b[1] += self.ball_v[1] * DT
            speed = math.hypot(*self.ball_v)
            if speed > 0:
                k = max(0.0, speed - 1.0 * DT) / speed  # 1 m/s^2 rolling decel
                self.ball_v = [self.ball_v[0] * k, self.ball_v[1] * k]


def _spy_on_turn(world):
    """Wrap _turn_with_ball to record what the 360-degree turn did."""
    original = mod._turn_with_ball

    def wrapper(*a, **k):
        # cycles just before the turn with the dribbler on and no translation
        # (skipping the short stop burst that _run sends between phases)
        history = list(world.sent)
        while history and history[-1][3] <= 0:
            history.pop()
        still = 0
        for vx, vy, _, rpm in reversed(history):
            if rpm > 0 and vx == 0.0 and vy == 0.0:
                still += 1
            else:
                break
        start = len(world.sent)
        pos, heading = tuple(world.robot[:2]), world.robot[2]
        result = original(*a, **k)
        world.turns.append({
            "result": result,
            "net_turn": world.robot[2] - heading,
            "translation": math.hypot(world.robot[0] - pos[0], world.robot[1] - pos[1]),
            "peak_w": max((abs(x[2]) for x in world.sent[start:]), default=0.0),
            "dribbler_on": all(x[3] > 0 for x in world.sent[start:-1]),
            "still_before": still,
        })
        return result

    mod._turn_with_ball = wrapper
    return original


def _run(world, **kw):
    behaviors._send = world.send
    return turn_with_ball(None, None, 0, world.pose, lambda: tuple(world.ball), **kw)


def dist(world, point):
    return math.hypot(world.ball[0] - point[0], world.ball[1] - point[1])


def test_turn_with_ball():
    print("Testing skills.turn_with_ball...")
    orig = (behaviors._send, behaviors.CONTROL_PERIOD_S, mod._turn_with_ball,
            mod.TURN_ACCEL_RADPS2, behaviors.approach_ball)
    behaviors.CONTROL_PERIOD_S = 0.0005  # run fast
    spinup_cycles = math.ceil(mod.DRIBBLER_SPINUP_S / 0.0005)
    try:
        # Normal run: drives to the ball, grips it, spins 360 in place
        approach_calls = []
        real_approach = behaviors.approach_ball

        def counting_approach(*a, **k):
            approach_calls.append(1)
            return real_approach(*a, **k)

        behaviors.approach_ball = counting_approach
        _spy_on_turn(World((0, 0, 0), (0, 0)))
        w = World((-1.0, 0.5, 0.0), (0.0, 0.0))
        _spy_on_turn(w)
        start_robot = tuple(w.robot[:2])
        assert _run(w, timeout_s=60.0) is True
        assert approach_calls, "approach_ball must always run"
        assert math.hypot(w.robot[0] - start_robot[0], w.robot[1] - start_robot[1]) > 0.5, \
            "robot must actually drive to the ball (regression: it used to return at once)"
        t = w.turns[-1]
        assert t["result"] and TWO_PI <= t["net_turn"] < TWO_PI + 0.3, t["net_turn"]
        assert t["translation"] < 1e-9, "turn must be in place"
        assert t["dribbler_on"]
        assert t["still_before"] >= spinup_cycles - 5, \
            "must stand still with the dribbler on before turning"
        assert dist(w, (0.0, 0.0)) <= 0.06, "ball must stay at its original coordinates"
        mod._turn_with_ball = orig[2]

        # Regression: ball ~2 cm short of the kicker still gets approached and gripped
        w = World((-0.12, 0.0, 0.0), (0.0, 0.0))
        _spy_on_turn(w)
        assert _run(w, timeout_s=60.0) is True
        assert w.turns[-1]["still_before"] >= spinup_cycles - 5
        mod._turn_with_ball = orig[2]

        # Robot on the far side of the ball: still works
        w = World((1.0, 0.0, math.pi), (0.0, 0.0))
        _spy_on_turn(w)
        assert _run(w, timeout_s=60.0) is True
        assert TWO_PI <= w.turns[-1]["net_turn"] < TWO_PI + 0.3
        assert dist(w, (0.0, 0.0)) <= 0.06
        mod._turn_with_ball = orig[2]

        # Turn rate parameter and direction (ramp removed so the cap is reached)
        mod.TURN_ACCEL_RADPS2 = 1e6
        for rate in (0.3, 1.0):
            w = World((-0.5, 0.0, 0.0), (0.0, 0.0))
            _spy_on_turn(w)
            assert _run(w, turn_rate_radps=rate, timeout_s=60.0) is True
            assert abs(w.turns[-1]["peak_w"] - rate) < 1e-6, (rate, w.turns[-1]["peak_w"])
            mod._turn_with_ball = orig[2]
        w = World((-0.5, 0.0, 0.0), (0.0, 0.0))
        _spy_on_turn(w)
        assert _run(w, turn_rate_radps=-0.5, timeout_s=60.0) is True
        assert -TWO_PI - 0.3 < w.turns[-1]["net_turn"] <= -TWO_PI, "negative rate = clockwise"
        mod._turn_with_ball = orig[2]
        mod.TURN_ACCEL_RADPS2 = orig[3]

        # Ball pops out mid-turn: re-acquires, carries it back to its ORIGINAL
        # coordinates, and completes a full turn
        w = World((-0.5, 0.0, 0.0), (0.0, 0.0), drop_after_rad=3.0)
        _spy_on_turn(w)
        assert _run(w, timeout_s=90.0) is True
        assert w.dropped
        assert [t["result"] for t in w.turns] == [False, True], w.turns
        assert dist(w, (0.0, 0.0)) <= 0.06
        mod._turn_with_ball = orig[2]

        # Dribbler cannot hold the ball: gives up, does not hang
        w = World((-0.5, 0.0, 0.0), (0.0, 0.0), can_hold=False)
        assert _run(w, max_reacquire=1, timeout_s=15.0) is False

        # No vision of the ball, or of the robot: False after the deadline (no crash)
        w = World((0.0, 0.0, 0.0), (0.0, 0.0))
        behaviors._send = w.send
        assert turn_with_ball(None, None, 0, w.pose, lambda: None, timeout_s=0.2) is False
        assert turn_with_ball(None, None, 0, lambda: None, lambda: (0.0, 0.0),
                              timeout_s=0.2) is False
        assert not w.sent
    finally:
        (behaviors._send, behaviors.CONTROL_PERIOD_S, mod._turn_with_ball,
         mod.TURN_ACCEL_RADPS2, behaviors.approach_ball) = orig

    for bad in ({"turn_rate_radps": 0}, {"turn_rate_radps": float("nan")},
                {"tolerance_m": -1}, {"max_reacquire": -1}, {"timeout_s": 0},
                {"dribbler_rpm": 0}):
        try:
            turn_with_ball(None, None, 0, lambda: None, lambda: None, **bad)
            assert False, f"expected ValueError for {bad}"
        except ValueError:
            pass
    print("  -> turn_with_ball tests passed.")


if __name__ == "__main__":
    test_turn_with_ball()