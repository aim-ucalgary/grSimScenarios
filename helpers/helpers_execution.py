"""Scenario scheduling.
"""


def run_steps(steps):
    """Execute an iterable of (name, callable) pairs in order."""
    raise NotImplementedError


def wait_until(predicate, *, timeout_s, poll_interval_s=0.02):
    """Wait until predicate() is true; raise TimeoutError on expiry."""
    raise NotImplementedError


def repeat_scenario(run_once, reset, *, repetitions):
    """Reset before each run and collect returned results."""
    raise NotImplementedError


def sweep_parameters(run_once, parameter_sets):
    """Call run_once(**parameters) for each parameter dictionary."""
    raise NotImplementedError