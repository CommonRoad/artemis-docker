"""Upstream CommonRoad solution checks, also used by the search bonus exercise.

``valid_solution`` returns ``(valid, feasibility_details)`` and can raise
``SolutionCheckerException``. Use ``planner.common.validation.validate_solution``
for the runner/tutorial's concise result summary. Interactive validation requires
the simulated traffic Scenario from the same run as the ego Solution.
"""

from commonroad_dc.feasibility.solution_checker import (
    CollisionException,
    GoalNotReachedException,
    MissingSolutionException,
    SolutionCheckerException,
    boundary_collision,
    ego_collision,
    goal_reached,
    obstacle_collision,
    solution_feasible,
    solved_all_problems,
    starts_at_correct_state,
    valid_solution,
)

__all__ = [
    "CollisionException",
    "GoalNotReachedException",
    "MissingSolutionException",
    "SolutionCheckerException",
    "boundary_collision",
    "ego_collision",
    "goal_reached",
    "obstacle_collision",
    "solution_feasible",
    "solved_all_problems",
    "starts_at_correct_state",
    "valid_solution",
]
