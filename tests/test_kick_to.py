"""Closed-loop test for skills.kick_to with a simulated robot and ball."""

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
from skills import kick_to
from skills.kick_to import estimate_kick_speed

DT = 0.02  # simulated seconds per command


class World:
    def __init__(self, robot, ball, kicker_works=True):
        self.robot, self.ball = list(robot), list(ball)
        self.ball_v = [0.0, 0.0]
        self.kicker_works = kicker_works
        self.kicks = 0

    def send(self, sock, endpoint, robot_id, vx=0.0, vy=0.0, w=0.0, *,
             kick_speed=0.0, **_):
        r, b = self.robot, self.ball
        r[0] += vx * DT; r[1] += vy * DT; r[2] += w * DT
        # kicker fires when the ball is on the kicker face
        face = (r[0] + math.cos(r[2]) * (behaviors.ROBOT_FRONT_M + behaviors.BALL_RADIUS_M),
                r[1] + math.sin(r[2]) * (behaviors.ROBOT_FRONT_M + behaviors.BALL_RADIUS_M))
        if kick_speed > 0 and self.kicker_works and math.hypot(b[0]-face[0], b[1]-face[1]) < 0.04:
            self.ball_v = [kick_speed * math.cos(r[2]), kick_speed * math.sin(r[2])]
            self.kicks += 1
        b[0] += self.ball_v[0] * DT
        b[1] += self.ball_v[1] * DT
        self.ball_v = [v * 0.995 for v in self.ball_v]


def _patched(world):
    orig = behaviors._send, behaviors.CONTROL_PERIOD_S
    behaviors._send, behaviors.CONTROL_PERIOD_S = world.send, 0.0005
    return orig


def test_kick_to():
    print("Testing skills.kick_to...")

    # Normal kick: robot behind the ball relative to the target
    w = World(robot=(-1.5, 0.5, 0.0), ball=(0.0, 0.0))
    orig = _patched(w)
    try:
        ok = kick_to(None, None, 0, lambda: tuple(w.robot), lambda: tuple(w.ball),
                     target_x_m=2.0, target_y_m=1.0, timeout_s=20.0)
        assert ok is True and w.kicks >= 1
        # ball should be heading toward the target
        want = math.atan2(1.0, 2.0)
        got = math.atan2(w.ball_v[1], w.ball_v[0])
        assert abs(got - want) < 0.15, (got, want)

        # Robot on the wrong side of the ball: must go around, not through it
        w2 = World(robot=(1.0, 0.0, math.pi), ball=(0.0, 0.0))
        behaviors._send = w2.send
        ok = kick_to(None, None, 0, lambda: tuple(w2.robot), lambda: tuple(w2.ball),
                     target_x_m=2.0, target_y_m=0.0, timeout_s=30.0)
        assert ok is True and w2.ball_v[0] > 0

        # Broken kicker: must give up and return False, not hang
        w3 = World(robot=(-1.0, 0.0, 0.0), ball=(0.0, 0.0), kicker_works=False)
        behaviors._send = w3.send
        ok = kick_to(None, None, 0, lambda: tuple(w3.robot), lambda: tuple(w3.ball),
                     target_x_m=2.0, target_y_m=0.0, max_attempts=2, timeout_s=8.0)
        assert ok is False and w3.kicks == 0

        # Ball not visible: False after the deadline
        assert kick_to(None, None, 0, lambda: (0.0, 0.0, 0.0), lambda: None,
                       target_x_m=1.0, target_y_m=0.0, timeout_s=0.2) is False
    finally:
        behaviors._send, behaviors.CONTROL_PERIOD_S = orig

    # Validation
    for bad in ({"kick_speed_mps": 0}, {"kick_speed_mps": 11}, {"max_attempts": 0}, {"timeout_s": -1}):
        try:
            kick_to(None, None, 0, lambda: None, lambda: None,
                    target_x_m=1.0, target_y_m=0.0, **bad)
            assert False, f"expected ValueError for {bad}"
        except ValueError:
            pass

    assert 1.5 <= estimate_kick_speed(0.5) <= estimate_kick_speed(5.0) <= 6.5
    print("  -> kick_to tests passed.")


if __name__ == "__main__":
    test_kick_to()
