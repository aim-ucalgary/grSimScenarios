"""Vision feedback for live runs: turns grSim's vision stream into get_pose()/get_ball()."""

import socket
import struct
import time

from helpers import observations as obs


def open_vision_socket(group="224.5.23.2", port=10020):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("", port))
    mreq = struct.pack("4sl", socket.inet_aton(group), socket.INADDR_ANY)
    sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
    return sock


class VisionTracker:
    """Reads one vision socket and serves both robot pose and ball position.

    Use one tracker per socket: two getters reading the same socket would
    steal packets from each other. Freshness is tracked here because the
    packet's t_capture is wall-clock time, not time.monotonic().
    """

    def __init__(self, vision_sock, team, robot_id, max_age_s=0.2):
        self.sock = vision_sock
        self.team = team
        self.robot_id = robot_id
        self.max_age_s = max_age_s
        self._pose = self._ball = None
        self._pose_t = self._ball_t = 0.0

    def _update(self):
        while True:  # drain everything queued, keep the newest data
            try:
                packet = obs.read_vision(self.sock, timeout_s=0.005)
            except (socket.timeout, TimeoutError, BlockingIOError):
                return
            now = time.monotonic()
            try:
                self._pose = obs.get_robot_pose(
                    packet, self.team, self.robot_id, max_age_s=None)
                self._pose_t = now
            except KeyError:
                pass  # this camera frame did not contain the robot
            try:
                self._ball = obs.get_ball_state(packet, max_age_s=None)
                self._ball_t = now
            except KeyError:
                pass

    def get_pose(self):
        """(x_m, y_m, heading_rad), or None when vision is missing or stale."""
        self._update()
        if self._pose is None or time.monotonic() - self._pose_t > self.max_age_s:
            return None
        return self._pose

    def get_ball(self):
        """(x_m, y_m), or None when the ball is missing or stale."""
        self._update()
        if self._ball is None or time.monotonic() - self._ball_t > self.max_age_s:
            return None
        return self._ball


def make_pose_getter(vision_sock, team, robot_id, max_age_s=0.2):
    """Return get_pose() for a socket used only for the robot pose."""
    return VisionTracker(vision_sock, team, robot_id, max_age_s).get_pose