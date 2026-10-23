"""Kick and dribbler commands.

Kick speed: m/s. Kick elevation: degrees.
"""


def kick(sock, endpoint, robot_id, *, speed_mps, angle_deg=0.0):
    """Send one kick request; delivery and ball contact are not guaranteed."""
    raise NotImplementedError


def set_dribbler(sock, endpoint, robot_id, *, enabled):
    """Enable or disable the dribbler."""
    raise NotImplementedError


def disable_actuators(sock, endpoint, robot_id):
    """Send zero kick/dribbler settings; cannot undo an executed kick."""
    raise NotImplementedError