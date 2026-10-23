"""UDP communication and simulator responses.

An endpoint is a (host, port) tuple.
Sockets are created explicitly, never during import.
"""


def open_socket(*, timeout_s=1.0):
    """Return a configured UDP socket."""
    raise NotImplementedError


def send_commands(sock, endpoint, commands):
    """Send one team's commands as a RobotControl message."""
    raise NotImplementedError


def read_feedback(sock, *, timeout_s=1.0):
    """Return decoded robot feedback; raise on timeout or simulator errors."""
    raise NotImplementedError


def close_socket(sock):
    """Close the supplied socket."""
    raise NotImplementedError