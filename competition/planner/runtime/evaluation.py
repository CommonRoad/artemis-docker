"""Evaluation records are produced even when a submission cannot return a plan."""

from dataclasses import dataclass
from collections.abc import Iterable
from pathlib import Path
import time

from .execution import RunResult, run_submission
from ..common.errors import failure_cause


@dataclass
class ScenarioScore:
    scenario_name: str
    passed: bool
    score: float | None
    status: str
    message: str
    elapsed_seconds: float


def describe_result(result: RunResult) -> dict[str, bool | int | float | str | None]:
    solution = result.solution.planning_problem_solutions[0]
    problem = result.planning_problems.planning_problem_dict[solution.planning_problem_id]
    return {
        "goal_reached": bool(problem.goal_reached(solution.trajectory)[0]),
        "feasible": result.validation.feasible,
        "valid": result.valid,
        "validation_message": result.validation.message,
        "scoring_error": result.scoring_error,
        solution.cost_function.name: result.cost,
        "planning_calls": len(result.diagnostics),
        "elapsed_seconds": round(result.elapsed_seconds, 2),
    }


def evaluate_scenario(
    path: str | Path,
    submission_directory: str | Path,
) -> ScenarioScore:
    started = time.monotonic()
    try:
        result = run_submission(path, submission_directory)
        status = "invalid_solution"
        if result.valid:
            status = "scoring_error" if result.scoring_error is not None else "passed"
        return ScenarioScore(
            Path(path).stem,
            status == "passed",
            result.cost,
            status,
            result.scoring_error or result.validation.message,
            time.monotonic() - started,
        )
    except Exception as error:
        error = failure_cause(error)
        return ScenarioScore(
            Path(path).stem,
            False,
            None,
            getattr(error, "code", "planner_error"),
            str(error),
            time.monotonic() - started,
        )


def leaderboard_payload(username: str, results: Iterable[ScenarioScore]) -> dict[str, object]:
    # Keep the existing leaderboard schema. Detailed failures remain in results.json.
    return {
        "username": username,
        "results": [
            {
                "scenario_name": result.scenario_name,
                "passed": result.passed,
                "score": result.score if result.passed else 0.0,
            }
            for result in results
        ],
    }
