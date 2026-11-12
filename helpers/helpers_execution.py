"""Scenario scheduling.
"""

import time


def run_steps(steps):
    """Execute an iterable of (name, callable) pairs in order.

    Inputs:
        steps (iterable[tuple[str, callable]]): Ordered sequence of (step_name, step_callable).

    Outputs:
        list[tuple[str, Any]]: List of (step_name, return_value) pairs.

    Example:
        >>> steps = [
        ...     ("approach", lambda: approach_ball()),
        ...     ("kick", lambda: kick_ball())
        ... ]
        >>> results = run_steps(steps)
    """
    results = []
    for name, action in steps:
        res = action()
        results.append((name, res))
    return results


def wait_until(predicate, *, timeout_s, poll_interval_s=0.02):
    """Wait until predicate() is true; raise TimeoutError on expiry.

    Inputs:
        predicate (callable): Function taking no arguments and returning truthy/falsy.
        timeout_s (float): Maximum seconds to wait before failing.
        poll_interval_s (float, optional): Sleep time between checks. Default is 0.02s.

    Outputs:
        Any: Truthy return value from the successful predicate() call.

    Raises:
        TimeoutError: If predicate() does not evaluate to truthy within timeout_s.

    Example:
        >>> wait_until(lambda: get_ball_state(ws)[0] > 1.0, timeout_s=3.0)
    """
    deadline = time.monotonic() + float(timeout_s)
    while time.monotonic() < deadline:
        res = predicate()
        if res:
            return res
        time.sleep(float(poll_interval_s))

    res = predicate()
    if res:
        return res
    raise TimeoutError(f"wait_until timed out after {timeout_s}s")


def repeat_scenario(run_once, reset, *, repetitions):
    """Reset before each run and collect returned results.

    Inputs:
        run_once (callable): Function executing a single trial run.
        reset (callable | None): Function restoring the initial scene between trials.
        repetitions (int): Number of trials to run.

    Outputs:
        list[Any]: List of return values from each run_once() execution.

    Example:
        >>> outcomes = repeat_scenario(run_trial, reset_field, repetitions=5)
    """
    results = []
    for _ in range(int(repetitions)):
        if reset is not None:
            reset()
        res = run_once()
        results.append(res)
    return results


def sweep_parameters(run_once, parameter_sets):
    """Call run_once(**parameters) for each parameter dictionary.

    Inputs:
        run_once (callable): Function accepting keyword arguments.
        parameter_sets (iterable[dict]): Sequence of parameter dictionaries.

    Outputs:
        list[Any]: Collected return values from each trial run.

    Example:
        >>> configs = [{"speed": 1.0}, {"speed": 2.0}, {"speed": 3.0}]
        >>> results = sweep_parameters(run_shot_trial, configs)
    """
    results = []
    for params in parameter_sets:
        res = run_once(**params)
        results.append(res)
    return results
