"""Closed-loop test for skills.go_to_point using a simulated point-mass robot."""

import math
import sys
from pathlib import Path
from unittest.mock import MagicMock

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

# Mock protobuf modules when they are not installed
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

from helpers import behaviors, validation
from skills import go_to_point


def test_go_to_point():
    print("Testing skills.go_to_point...")
    pose = [0.0, 0.0, 0.0]

    def fake_send(sock, endpoint, robot_id, vx=0.0, vy=0.0, w=0.0, **_):
        dt = 0.02  # simulated time per command
        pose[0] += vx * dt
        pose[1] += vy * dt
        pose[2] += w * dt

    orig_send, orig_period = behaviors._send, behaviors.CONTROL_PERIOD_S
    behaviors._send, behaviors.CONTROL_PERIOD_S = fake_send, 0.0005  # run fast
    try:
        ok = go_to_point(None, None, 0, lambda: tuple(pose),
                         x_m=1.5, y_m=-0.5, heading_rad=math.pi / 2,
                         timeout_s=5.0)
        assert ok is True
        validation.assert_near_position((pose[0], pose[1]), (1.5, -0.5), tolerance_m=0.05)
        validation.assert_heading(pose[2], math.pi / 2, tolerance_rad=0.08)

        # Missing vision must time out, not succeed
        assert go_to_point(None, None, 0, lambda: None,
                           x_m=1.0, y_m=0.0, timeout_s=0.2) is False

        # Input validation
        try:
            go_to_point(None, None, 0, lambda: None, x_m=0, y_m=0, timeout_s=-1)
            assert False, "Expected ValueError"
        except ValueError:
            pass
    finally:
        behaviors._send, behaviors.CONTROL_PERIOD_S = orig_send, orig_period
    print("  -> go_to_point tests passed.")


if __name__ == "__main__":
    test_go_to_point()
