"""Scenario logs, results, and plots.

Keep optional plotting dependencies inside functions that need them.
"""

import json
from pathlib import Path


def record_run(path, *, metadata, events):
    """Write run metadata and timestamped events.

    Inputs:
        path (str | Path): File path to save JSON log file.
        metadata (dict): Metadata attributes describing run parameters.
        events (list[dict]): Timestamped sequence of recorded event entries.

    Outputs:
        None

    Example:
        >>> record_run("runs/trial1.json",
        ...            metadata={"robot_id": 0, "scenario": "velocity_test"},
        ...            events=[{"time": 0.0, "event": "start"}, {"time": 1.0, "vx": 0.5}])
    """
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "metadata": metadata,
        "events": events,
    }
    with open(path, "w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2, default=str)


def save_results(path, results):
    """Export measurements and validation outcomes.

    Inputs:
        path (str | Path): File path to save output JSON.
        results (dict | list): Test outcomes, error metrics, and validation summaries.

    Outputs:
        None

    Example:
        >>> save_results("results/summary.json", {"passed": True, "error_m": 0.015})
    """
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        json.dump(results, f, indent=2, default=str)


def plot_trajectory(robot_positions, *, ball_positions=None, output_path=None):
    """Plot observed field positions; optionally save the figure.

    Inputs:
        robot_positions (list[tuple[float, float]]): Series of (x, y) robot positions in metres.
        ball_positions (list[tuple[float, float]], optional): Optional series of (x, y) ball positions.
        output_path (str | Path, optional): Path to export figure (.png/.pdf). If None, calls plt.show().

    Outputs:
        None

    Example:
        >>> r_path = [(0.0, 0.0), (0.5, 0.0), (1.0, 0.0)]
        >>> plot_trajectory(r_path, output_path="plots/trajectory.png")
    """
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots()

    # Robot trajectory
    rx = [p[0] for p in robot_positions]
    ry = [p[1] for p in robot_positions]
    ax.plot(rx, ry, label="Robot", color="blue", marker="o", markersize=3)

    # Ball trajectory
    if ball_positions:
        bx = [p[0] for p in ball_positions]
        by = [p[1] for p in ball_positions]
        ax.plot(bx, by, label="Ball", color="orange", linestyle="--")

    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_title("Trajectory")
    ax.grid(True)
    ax.axis("equal")
    ax.legend()

    if output_path is not None:
        out = Path(output_path)
        out.parent.mkdir(parents=True, exist_ok=True)
        plt.savefig(out)
        plt.close(fig)
    else:
        plt.show()
