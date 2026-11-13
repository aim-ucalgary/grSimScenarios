"""Vision feedback for live runs: turns grSim's vision stream into a get_pose() callable."""

import socket
import struct
import time

from ._compat import load


def open_vision_socket(group="224.5.23.2", port=10020):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("", port))
    mreq = struct.pack("4sl", socket.inet_aton(group), socket.INADDR_ANY)
    sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
    return sock


def make_pose_getter(vision_sock, team, robot_id, max_age_s=0.2):
    """Return get_pose() -> (x_m, y_m, heading_rad) or None when vision is stale."""
    obs = load("observations")
    last = {"pose": None, "t": 0.0}

    def get_pose():
        # Drain everything queued; keep the newest packet that contains the robot.
        while True:
            try:
                packet = obs.read_vision(vision_sock, timeout_s=0.005)
            except (socket.timeout, TimeoutError, BlockingIOError):
                break
            try:
                # Freshness is tracked here: t_capture is wall-clock, not monotonic.
                last["pose"] = obs.get_robot_pose(packet, team, robot_id, max_age_s=None)
                last["t"] = time.monotonic()
            except KeyError:
                pass  # this camera frame did not contain the robot
        if last["pose"] is None or time.monotonic() - last["t"] > max_age_s:
            return None
        return last["pose"]

    return get_pose
