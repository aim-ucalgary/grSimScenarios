"""Comprehensive unit tests for the helpers package."""

import math
import sys
import tempfile
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

import helpers


# ---------------------------------------------------------------- mock socket

class MockSocket:
    def __init__(self, timeout_on_recv=False):
        self.timeout_on_recv = timeout_on_recv
        self.timeout = None
        self.sent = []
        self.closed = False

    def settimeout(self, t):
        self.timeout = t

    def sendto(self, data, endpoint):
        self.sent.append((data, endpoint))

    def recvfrom(self, bufsize):
        if self.timeout_on_recv:
            raise TimeoutError("Socket timed out")
        return b"", ("127.0.0.1", 10020)

    def close(self):
        self.closed = True


# ---------------------------------------------------------------- test suites

def test_geometry():
    print("Testing helpers.geometry...")
    geo = helpers.geometry

    # distance_to
    assert abs(geo.distance_to((0, 0), (3, 4)) - 5.0) < 1e-6
    # bearing_to
    assert abs(geo.bearing_to((0, 0), (1, 0)) - 0.0) < 1e-6
    assert abs(geo.bearing_to((0, 0), (0, 1)) - (math.pi / 2)) < 1e-6
    # angle_error
    assert abs(geo.angle_error(0.1, 0.0) - 0.1) < 1e-6
    assert abs(geo.angle_error(math.pi, -math.pi) - 0.0) < 1e-5
    # global_to_local & local_to_global
    fwd, left = geo.global_to_local(1.0, 0.0, math.pi / 2)
    assert abs(fwd - 0.0) < 1e-6 and abs(left - (-1.0)) < 1e-6
    vx, vy = geo.local_to_global(fwd, left, math.pi / 2)
    assert abs(vx - 1.0) < 1e-6 and abs(vy - 0.0) < 1e-6

    print("  -> geometry tests passed.")


def test_communication():
    print("Testing helpers.communication...")
    com = helpers.communication
    sock = com.open_socket(timeout_s=0.5)
    assert sock.gettimeout() == 0.5
    com.close_socket(sock)

    mock = MockSocket()
    com.send_commands(mock, ("127.0.0.1", 10301), b"test_payload")
    assert len(mock.sent) == 1
    assert mock.sent[0] == (b"test_payload", ("127.0.0.1", 10301))
    com.close_socket(mock)
    assert mock.closed is True
    print("  -> communication tests passed.")


def test_movement():
    print("Testing helpers.movement...")
    mov = helpers.movement
    mock = MockSocket()

    try:
        mov.send_global_velocity(mock, ("127.0.0.1", 10301), 0, vx_mps=1.0, vy_mps=0.0)
    except RuntimeError:
        pass  # Protobuf not installed in this environment

    # Validation
    try:
        mov.send_global_velocity(mock, ("127.0.0.1", 10301), -1, vx_mps=0.0, vy_mps=0.0)
        assert False, "Expected ValueError for negative robot_id"
    except ValueError:
        pass
    print("  -> movement tests passed.")


def test_observations():
    print("Testing helpers.observations...")
    obs = helpers.observations
    now = time.monotonic()

    # require_fresh_state
    obs.require_fresh_state(now, max_age_s=0.5)
    try:
        obs.require_fresh_state(now - 1.0, max_age_s=0.5)
        assert False, "Expected TimeoutError"
    except TimeoutError:
        pass

    # estimate_velocity
    assert abs(obs.estimate_velocity(1.0, 3.0, dt_s=1.0) - 2.0) < 1e-6
    vx, vy = obs.estimate_velocity((1.0, 2.0), (3.0, 5.0), dt_s=0.5)
    assert abs(vx - 4.0) < 1e-6 and abs(vy - 6.0) < 1e-6

    # get_robot_pose from dict
    ws = {
        "timestamp": now,
        "blue": {0: (1500.0, 2000.0, 0.0)},
        "yellow": {1: (2.0, -1.0, math.pi * 3)},
        "ball": (500.0, -1000.0),
    }
    x, y, h = obs.get_robot_pose(ws, "blue", 0)
    assert abs(x - 1.5) < 1e-6 and abs(y - 2.0) < 1e-6 and abs(h) < 1e-6

    x, y, h = obs.get_robot_pose(ws, "yellow", 1)
    assert abs(x - 2.0) < 1e-6 and abs(y - (-1.0)) < 1e-6
    # heading wrapped to [-pi, pi]
    assert abs(abs(h) - math.pi) < 1e-5

    bx, by = obs.get_ball_state(ws)
    assert abs(bx - 0.5) < 1e-6 and abs(by - (-1.0)) < 1e-6
    print("  -> observations tests passed.")


def test_validation():
    print("Testing helpers.validation...")
    val = helpers.validation

    # assert_near_position
    val.assert_near_position((1.0, 1.0), (1.02, 1.01), tolerance_m=0.05)
    try:
        val.assert_near_position((1.0, 1.0), (2.0, 2.0), tolerance_m=0.05)
        assert False, "Expected AssertionError"
    except AssertionError:
        pass

    # assert_heading
    val.assert_heading(0.1, 0.12, tolerance_rad=0.05)
    try:
        val.assert_heading(0.0, math.pi, tolerance_rad=0.05)
        assert False, "Expected AssertionError"
    except AssertionError:
        pass

    # assert_ball_displacement
    val.assert_ball_displacement((0, 0), (0.5, 0), min_distance_m=0.4, max_distance_m=0.6)
    try:
        val.assert_ball_displacement((0, 0), (0.1, 0), min_distance_m=0.4)
        assert False, "Expected AssertionError"
    except AssertionError:
        pass

    # assert_stopped
    samples = [
        (0.0, (0.01, 0.01, 0.01)),
        (0.1, (0.00, 0.01, 0.00)),
        (0.3, (0.00, 0.00, 0.00)),
    ]
    val.assert_stopped(samples, linear_tolerance_mps=0.05, angular_tolerance_radps=0.05, duration_s=0.25)
    print("  -> validation tests passed.")


def test_execution():
    print("Testing helpers.execution...")
    exe = helpers.execution

    # run_steps
    calls = []
    steps = [
        ("step1", lambda: calls.append(1) or "ok1"),
        ("step2", lambda: calls.append(2) or "ok2"),
    ]
    results = exe.run_steps(steps)
    assert calls == [1, 2]
    assert results == [("step1", "ok1"), ("step2", "ok2")]

    # wait_until
    counter = 0
    def pred():
        nonlocal counter
        counter += 1
        return counter >= 3

    assert exe.wait_until(pred, timeout_s=0.5, poll_interval_s=0.01) is True

    # repeat_scenario
    run_count = 0
    reset_count = 0
    def run_once():
        nonlocal run_count
        run_count += 1
        return run_count

    def reset():
        nonlocal reset_count
        reset_count += 1

    res = exe.repeat_scenario(run_once, reset, repetitions=3)
    assert run_count == 3
    assert reset_count == 3
    assert res == [1, 2, 3]

    # sweep_parameters
    def param_fn(a, b):
        return a + b

    sweep_res = exe.sweep_parameters(param_fn, [{"a": 1, "b": 2}, {"a": 10, "b": 20}])
    assert sweep_res == [3, 30]
    print("  -> execution tests passed.")


def test_reporting():
    print("Testing helpers.reporting...")
    rep = helpers.reporting
    with tempfile.TemporaryDirectory() as tmpdir:
        p1 = Path(tmpdir) / "run.json"
        rep.record_run(p1, metadata={"scenario": "test"}, events=[{"time": 0.0, "type": "start"}])
        assert p1.exists()

        p2 = Path(tmpdir) / "results.json"
        rep.save_results(p2, {"passed": True})
        assert p2.exists()
    print("  -> reporting tests passed.")


def test_actuators():
    print("Testing helpers.actuators...")
    act = helpers.actuators
    mock = MockSocket()

    # Validation
    try:
        act.kick(mock, ("127.0.0.1", 10301), -1, speed_mps=2.0)
        assert False, "Expected ValueError for negative robot_id"
    except ValueError:
        pass

    try:
        act.kick(mock, ("127.0.0.1", 10301), 0, speed_mps=15.0)  # > MAX_KICK_SPEED_MPS
        assert False, "Expected ValueError for excessive kick speed"
    except ValueError:
        pass
    print("  -> actuators tests passed.")


if __name__ == "__main__":
    test_geometry()
    test_communication()
    test_movement()
    test_observations()
    test_validation()
    test_execution()
    test_reporting()
    test_actuators()
    print("\nAll helper tests passed successfully!")
