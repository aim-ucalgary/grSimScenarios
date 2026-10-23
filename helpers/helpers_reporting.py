"""Scenario logs, results, and plots.

Keep optional plotting dependencies inside functions that need them.
"""


def record_run(path, *, metadata, events):
    """Write run metadata and timestamped events."""
    raise NotImplementedError


def save_results(path, results):
    """Export measurements and validation outcomes."""
    raise NotImplementedError


def plot_trajectory(robot_positions, *, ball_positions=None, output_path=None):
    """Plot observed field positions; optionally save the figure."""
    raise NotImplementedError