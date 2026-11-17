"""Failures that the CLI, notebooks, and evaluator report consistently."""


class PlanningFailure(RuntimeError):
    code = "planner_error"


class IncompatibleState(PlanningFailure):
    code = "incompatible_state"


class NoPathFound(PlanningFailure):
    code = "no_path"


class InvalidPlan(PlanningFailure):
    code = "invalid_output"


def failure_cause(error: Exception) -> Exception:
    """SUMO wraps planner errors; retain their specific failure status and message."""
    while not hasattr(error, "code") and error.__cause__ is not None:
        error = error.__cause__
    return error
