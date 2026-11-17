"""Reuse an SMP search class on repeated, current-observation planning problems."""

import time
from collections.abc import Sequence
from copy import deepcopy
from dataclasses import dataclass
from importlib import import_module

import numpy as np
from commonroad.common.solution import VehicleType
from commonroad.planning.planning_problem import PlanningProblem
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.state import State
from commonroad.scenario.trajectory import Trajectory

from ..common.trajectory import assert_same_state, center_state, recover_steering, validate_prefix
from ..common.errors import IncompatibleState, InvalidPlan, NoPathFound
from ..common.prediction import predict_observations, relative_problem, search_horizon
from ..common.scenarios import primitive_path
from .smp_automaton import ExactStateAutomaton, PRIMITIVE_STEPS


@dataclass(frozen=True)
class SMPConfig:
    speed_spacing: int | None = None  # None selects the scenario's default spacing.


def center_trajectory(
    paths: Sequence[Sequence[State]], rear_axle_distance: float, time_offset: int
) -> Trajectory:
    """Flatten paths without relying on a fixed number of states in each segment."""
    states = []
    for path in paths:
        for state in path:
            converted = center_state(state, rear_axle_distance, time_offset)
            step = converted.time_step
            if states and step == states[-1].time_step:
                assert_same_state(states[-1], converted)
                continue
            if states and step != states[-1].time_step + 1:
                raise InvalidPlan("SMP returned a gap or reversed timestamp")
            states.append(converted)
    if len(states) < 2:
        raise NoPathFound("SMP did not return a future state")
    return Trajectory(states[0].time_step, states)


class SMPPlanner:
    def __init__(
        self,
        planner_class: (
            type | str
        ) = "SMP.motion_planner.search_algorithms.best_first_search:AStarSearch",
        config: SMPConfig | None = None,
    ):
        self.planner_class = planner_class
        self.config = config or SMPConfig()
        self._automata = {}
        self.reset()

    def reset(self):
        self._endpoints = {}
        self.diagnostics = []

    def _automaton(self, scenario_id):
        path = primitive_path(scenario_id, self.config.speed_spacing)
        if path not in self._automata:
            # Resolve SMP after the submission directory is selected, so a student
            # source tree takes precedence over the bundled framework.
            module = import_module("SMP.maneuver_automaton.maneuver_automaton")
            self._automata[path] = module.ManeuverAutomaton.generate_automaton(str(path))
        return self._automata[path]

    def plan(self, scenario: Scenario, planning_problem: PlanningProblem) -> Trajectory:
        started = time.monotonic()
        if not np.isclose(scenario.dt, 0.1, rtol=0, atol=1e-12):
            raise IncompatibleState("SMP primitives require scenario.dt = 0.1 seconds")
        current = planning_problem.initial_state
        key = (str(scenario.scenario_id), planning_problem.planning_problem_id)
        previous = self._endpoints.get(key)
        steering = recover_steering(current, previous)
        horizon = search_horizon(planning_problem)
        predicted = predict_observations(scenario, current.time_step, horizon)
        problem = relative_problem(planning_problem, steering)
        automaton = ExactStateAutomaton(self._automaton(scenario.scenario_id), horizon)
        planner_class = self.planner_class
        if isinstance(planner_class, str):
            module, name = planner_class.split(":")
            planner_class = getattr(import_module(module), name)
        plot_config_class = import_module("SMP.motion_planner.plot_config").StudentScriptPlotConfig
        plot_config = plot_config_class(DO_PLOT=False)
        # Older submitted SMP trees accept only DO_PLOT in their constructor.
        plot_config.RECORD_HISTORY = False
        search = planner_class(predicted, problem, automaton, plot_config=plot_config)
        paths, _, _ = search.execute_search()
        if not paths:
            raise NoPathFound("SMP found no path to the original goal under the current prediction")
        trajectory = center_trajectory(paths, search.rear_ax_dist, current.time_step)
        assert_same_state(trajectory.state_list[0], current, steering=steering)
        if not planning_problem.goal.is_reached(trajectory.final_state):
            raise InvalidPlan("SMP returned a trajectory that does not reach the original goal")
        # A shorter path already ends at the goal checked above.
        states = trajectory.state_list[: PRIMITIVE_STEPS + 1]
        prefix = Trajectory(current.time_step, states)
        validate_prefix(
            prefix, scenario.dt, VehicleType.BMW_320i, search.shape_ego, search.collision_checker
        )
        self._endpoints[key] = deepcopy(states[-1])
        self.diagnostics.append(
            {
                "time_step": current.time_step,
                "horizon_steps": horizon,
                "executed_steps": len(states) - 1,
                "planning_seconds": time.monotonic() - started,
            }
        )
        return Trajectory(current.time_step + 1, states[1:])
