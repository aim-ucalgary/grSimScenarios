"""Closed-loop test for skills.dribble_to with a simulated robot, ball and dribbler."""

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
from skills import dribble_to

DT = 0.02  # simulated seconds per command
CONTACT = behaviors.ROBOT_FRONT_M + behaviors.BALL_RADIUS_M


class World:
    """Point-mass robot; the ball sticks to the kicker face while the dribbler is on."""

    def __init__(self, robot, ball, drop_at_x=None, can_hold=True):
        self.robot, self.ball = list(robot), list(ball)
        self.ball_v = [0.0, 0.0]
        self.carried = False
        self.drop_at_x, self.dropped = drop_at_x, False
        self.can_hold = can_hold
        self.dribbler_seen_on = False

    def send(self, sock, endpoint, robot_id, vx=0.0, vy=0.0, w=0.0, *,
             kick_speed=0.0, kick_angle=0.0, dribbler_rpm=0.0):
        r, b = self.robot, self.ball
        r[0] += vx * DT; r[1] += vy * DT; r[2] += w * DT
        c, s = math.cos(r[2]), math.sin(r[2])
        if dribbler_rpm > 0:
            self.dribbler_seen_on = True
        else:
            self.carried = False

        f = (b[0] - r[0]) * c + (b[1] - r[1]) * s
        l = -(b[0] - r[0]) * s + (b[1] - r[1]) * c
        if (self.can_hold and dribbler_rpm > 0 and not self.carried
                and math.hypot(*self.ball_v) < 0.05 and 0 <= f <= 0.13 and abs(l) <= 0.04):
            self.carried = True

        if (self.carried and self.drop_at_x is not None and not self.dropped
                and r[0] >= self.drop_at_x):
            self.dropped, self.carried = True, False
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


def _patched(world):
    orig = behaviors._send, behaviors.CONTROL_PERIOD_S
    behaviors._send, behaviors.CONTROL_PERIOD_S = world.send, 0.0005
    return orig


def _run(world, target, **kw):
    behaviors._send = world.send
    return dribble_to(None, None, 0, lambda: tuple(world.robot), lambda: tuple(world.ball),
                      x_m=target[0], y_m=target[1], **kw)


def test_dribble_to():
    print("Testing skills.dribble_to...")
    orig = _patched(World((0, 0, 0), (0, 0)))
    try:
        # Normal dribble: robot behind the ball, target ahead and to the side
        w = World((-1.0, 0.5, 0.0), (0.0, 0.0))
        assert _run(w, (1.0, 0.5), timeout_s=60.0) is True
        assert math.hypot(w.ball[0] - 1.0, w.ball[1] - 0.5) <= 0.06
        assert w.dribbler_seen_on

        # Robot on the far side of the ball: must go around it
        w = World((1.0, 0.0, math.pi), (0.0, 0.0))
        assert _run(w, (-1.0, 0.0), timeout_s=60.0) is True
        assert math.hypot(w.ball[0] + 1.0, w.ball[1]) <= 0.06

        # Ball pops out mid-way: must notice, re-acquire and finish
        w = World((-1.0, 0.0, 0.0), (0.0, 0.0), drop_at_x=0.4)
        assert _run(w, (2.0, 0.0), timeout_s=90.0) is True
        assert w.dropped
        assert math.hypot(w.ball[0] - 2.0, w.ball[1]) <= 0.06

        # Dribbler cannot hold the ball: gives up, does not hang
        w = World((-1.0, 0.0, 0.0), (0.0, 0.0), can_hold=False)
        assert _run(w, (2.0, 0.0), max_reacquire=1, timeout_s=15.0) is False

        # Already at the target: nothing to do
        w = World((-1.0, 0.0, 0.0), (1.0, 1.0))
        assert _run(w, (1.0, 1.0), timeout_s=5.0) is True
        assert w.robot == [-1.0, 0.0, 0.0]

        # Ball not visible
        assert dribble_to(None, None, 0, lambda: (0.0, 0.0, 0.0), lambda: None,
                          x_m=1.0, y_m=0.0, timeout_s=0.2) is False
    finally:
        behaviors._send, behaviors.CONTROL_PERIOD_S = orig

    for bad in ({"speed_mps": 0}, {"tolerance_m": -1}, {"max_reacquire": -1},
                {"timeout_s": 0}, {"dribbler_rpm": 0}):
        try:
            dribble_to(None, None, 0, lambda: None, lambda: None, x_m=1.0, y_m=0.0, **bad)
            assert False, f"expected ValueError for {bad}"
        except ValueError:
            pass
    print("  -> dribble_to tests passed.")


if __name__ == "__main__":
    test_dribble_to()
