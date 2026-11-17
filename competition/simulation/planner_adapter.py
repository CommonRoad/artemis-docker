"""Prepare observations and validate planner trajectories before execution."""

from copy import deepcopy
from collections.abc import Iterable

import numpy as np
from commonroad.planning.planning_problem import PlanningProblem
from commonroad.scenario.obstacle import StaticObstacle
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.trajectory import Trajectory

from .traffic import remove_initial_state_from_predictions


class PlannerOutputError(ValueError):
    """The planner returned output that cannot be executed."""

    code = "invalid_output"


def restore_static_obstacles(scenario: Scenario, obstacles: Iterable[StaticObstacle]) -> Scenario:
    """Restore original fixed geometry omitted by CommonRoad-SUMO 2025.1.4."""
    existing = {obstacle.obstacle_id for obstacle in scenario.obstacles}
    static = {obstacle.obstacle_id for obstacle in scenario.static_obstacles}
    for obstacle in obstacles:
        if obstacle.obstacle_id not in existing:
            scenario.add_objects(deepcopy(obstacle))
        elif obstacle.obstacle_id not in static:
            raise PlannerOutputError(f"SUMO reused static obstacle ID {obstacle.obstacle_id}")
    return scenario


def future_trajectory(trajectory: Trajectory, problem: PlanningProblem) -> Trajectory:
    """Check the output format and copy the future states for execution.
    """
    label = (
        f"Planning problem {problem.planning_problem_id} at step {problem.initial_state.time_step}"
    )
    if not isinstance(trajectory, Trajectory):
        raise PlannerOutputError(f"{label}: plan must return a CommonRoad Trajectory")
    states = trajectory.state_list
    current = problem.initial_state.time_step
    # Planners may include the current state; execution starts at the next step.
    if states and states[0].time_step == current:
        states = states[1:]
    if not states:
        raise PlannerOutputError(f"{label}: trajectory must contain at least one future state")
    # Require absolute simulation steps with no gaps and finite, usable state values.
    for expected, state in enumerate(states, current + 1):
        if state.time_step != expected:
            raise PlannerOutputError(
                f"{label}: expected consecutive absolute time step {expected}, got {state.time_step}"
            )
        for attr in ("position", "orientation", "velocity"):
            value = getattr(state, attr, None)
            if value is None or not np.all(np.isfinite(value)):
                raise PlannerOutputError(f"{label}: state {expected} needs finite {attr}")
        if np.asarray(state.position).shape != (2,):
            raise PlannerOutputError(f"{label}: state {expected} needs a 2D position")
    # Keep execution's state objects independent of any trajectory cached by the planner.
    return Trajectory(current + 1, deepcopy(states))


def _observation(
    scenario: Scenario,
    problem: PlanningProblem,
    interactive: bool,
    static_obstacles: Iterable[StaticObstacle],
) -> Scenario:
    """Copy the scenario and expose the traffic information allowed for this call.

    Interactive calls retain available history through the current step.
    Non-interactive calls also retain recorded future traffic for offline planning.
    """
    # Isolate planner edits and restore fixed obstacles omitted by the SUMO conversion.
    observation = restore_static_obstacles(deepcopy(scenario), static_obstacles)
    if interactive:
        current = problem.initial_state.time_step
        for obstacle in list(observation.dynamic_obstacles):
            # Future arrivals must remain unknown until they appear in the simulation.
            if obstacle.initial_state.time_step > current:
                observation.remove_obstacle(obstacle)
            elif obstacle.prediction is not None:
                # CommonRoad's prediction field may contain history as well as future states.
                states = [
                    state
                    for state in obstacle.prediction.trajectory.state_list
                    if state.time_step <= current
                ]
                if states:
                    obstacle.prediction.trajectory = Trajectory(states[0].time_step, states)
                else:
                    obstacle.prediction = None
        # Store initial occupancy only in initial_state to keep collision timestamps aligned.
        remove_initial_state_from_predictions(observation)
    return observation


class PlannerAdapter:
    """Wrap an object with plan(scenario, problem) -> Trajectory; no inheritance required."""

    def __init__(self, planner, *, interactive=True, static_obstacles=()):
        if not callable(getattr(planner, "plan", None)):
            raise TypeError("Provide a planner with plan(scenario, problem) -> Trajectory")
        self.planner = planner
        self.interactive = interactive
        self.static_obstacles = static_obstacles

    def plan(self, scenario: Scenario, planning_problem: PlanningProblem) -> Trajectory:
        observation = _observation(
            scenario, planning_problem, self.interactive, self.static_obstacles
        )
        trajectory = self.planner.plan(observation, deepcopy(planning_problem))
        return future_trajectory(trajectory, planning_problem)
