"""Summarize upstream solution checks for completed drives and saved XML pairs."""

from dataclasses import dataclass

from commonroad.common.solution import Solution
from commonroad.planning.planning_problem import PlanningProblemSet
from commonroad.scenario.scenario import Scenario

from simulation.solution_check import SolutionCheckerException, valid_solution


@dataclass(frozen=True)
class SolutionValidation:
    valid: bool
    feasible: bool | None
    message: str


def validate_solution(
    scenario: Scenario, planning_problems: PlanningProblemSet, solution: Solution
) -> SolutionValidation:
    """Run the unchanged upstream checker once and summarize its decision.

    ``feasible=None`` means validation stopped without returning feasibility
    results. Expected checker failures become invalid results; unexpected errors
    propagate. No extra checks, tolerances or coordinate transformations apply.
    """
    try:
        valid, details = valid_solution(scenario, planning_problems, solution)
    except SolutionCheckerException as error:
        return SolutionValidation(False, None, str(error) or type(error).__name__)

    failed = [str(problem_id) for problem_id, result in details.items() if not result[0]]
    if failed:
        message = f"Dynamics feasibility failed for planning problem(s): {', '.join(failed)}."
    else:
        message = "Solution is valid." if valid else "Solution is invalid."
    return SolutionValidation(bool(valid), not failed, message)
