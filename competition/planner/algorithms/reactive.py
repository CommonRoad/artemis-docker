"""Adapt the reactive planner to current observations and the trajectory interface."""

from dataclasses import dataclass
import time

import numpy as np
from commonroad.common.solution import VehicleType
from commonroad.geometry.shape import Rectangle
from commonroad.planning.planning_problem import PlanningProblem
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.trajectory import Trajectory
from commonroad_clcs.config import CLCSParams, ProcessingOption, ResamplingOption
from commonroad_clcs.pycrccosy import (
    CurvilinearProjectionDomainLateralError,
    CurvilinearProjectionDomainLongitudinalError,
)
from commonroad_dc.feasibility.vehicle_dynamics import VehicleParameterMapping
from commonroad_rp.reactive_planner import ReactivePlanner
from commonroad_rp.state import ReactivePlannerState
from commonroad_rp.utility.config import PlanningConfiguration, ReactivePlannerConfiguration
from commonroad_rp.utility.utils_coordinate_system import (
    CoordinateSystem,
    create_initial_ref_path,
)

from ..common.trajectory import assert_same_state, center_state, recover_steering, validate_prefix
from ..common.errors import IncompatibleState, InvalidPlan, NoPathFound
from ..common.prediction import predict_observations, relative_problem


class ReactiveCoordinateSystem(CoordinateSystem):
    def convert_to_cartesian_coords(
        self, s: float, l: float, check_proj_domain: bool = True
    ) -> np.ndarray | None:
        try:
            return super().convert_to_cartesian_coords(s, l, check_proj_domain)
        except (
            CurvilinearProjectionDomainLongitudinalError,
            CurvilinearProjectionDomainLateralError,
        ):
            # The reactive core rejects out-of-domain samples when conversion returns None.
            # CLCS 2025.2 raises typed errors instead; restore that existing rejection branch.
            return None


def reference_coordinates(scenario: Scenario, problem: PlanningProblem) -> CoordinateSystem:
    params = CLCSParams(processing_option=ProcessingOption.SPLINE_SMOOTHING)
    params.spline.smoothing_factor = 2.0
    # Fixed spacing handles straight segments without computing infinite curvature radii.
    params.resampling.option = ResamplingOption.FIXED
    params.resampling.fixed_step = 0.5
    reference = create_initial_ref_path(scenario.lanelet_network, problem)
    return ReactiveCoordinateSystem(
        reference=reference,
        preprocess_reference=True,
        clcs_params=params,
    )


@dataclass(frozen=True)
class ReactiveConfig:
    horizon_steps: int = 20
    replanning_interval: int = 3

    def __post_init__(self):
        if self.horizon_steps < 2 or not 1 <= self.replanning_interval <= self.horizon_steps:
            raise ValueError(
                "Use a horizon of at least two steps and a replanning interval within it"
            )


class ReactivePlannerAdapter:
    def __init__(self, config: ReactiveConfig | None = None):
        self.config = config or ReactiveConfig()
        self.reset()

    def reset(self):
        self._planners = {}
        self._endpoints = {}
        self.diagnostics = []

    def _planner(
        self, key: tuple[str, int], scenario: Scenario, problem: PlanningProblem
    ) -> ReactivePlanner:
        if key not in self._planners:
            config = ReactivePlannerConfiguration()
            config.vehicle.id_type_vehicle = VehicleType.BMW_320i.value
            # Load dimensions, axle distances and constraints from the vehicle model.
            config.vehicle.update_vehicle_config({})
            config.planning = PlanningConfiguration(
                dt=scenario.dt,
                time_steps_computation=self.config.horizon_steps,
                replanning_frequency=self.config.replanning_interval,
                standstill_lookahead=min(10, self.config.horizon_steps - 1),
            )
            config.sampling.t_min = 0.2
            # Keep planning in the calling process for notebook debugging.
            config.debug.multiproc = False
            config.debug.show_plots = False
            config.debug.save_plots = False
            config.debug.show_evaluation_plots = False
            config.update(scenario, problem)
            planner = ReactivePlanner(config)
            planner.set_reference_path(coordinate_system=reference_coordinates(scenario, problem))
            self._planners[key] = planner
        planner = self._planners[key]
        planner.config.update(scenario, problem)
        return planner

    def plan(self, scenario: Scenario, planning_problem: PlanningProblem) -> Trajectory:
        started = time.monotonic()
        if not np.isclose(scenario.dt, 0.1, rtol=0, atol=1e-12):
            raise IncompatibleState("The reactive example requires scenario.dt = 0.1 seconds")
        current = planning_problem.initial_state
        if current.velocity < 0:
            raise IncompatibleState("The reactive example supports forward motion only")
        deadline = max(goal.time_step.end for goal in planning_problem.goal.state_list)
        steps = int(min(self.config.replanning_interval, deadline - current.time_step))
        if steps < 1:
            raise NoPathFound("The original goal deadline has expired")

        key = (str(scenario.scenario_id), planning_problem.planning_problem_id)
        previous = self._endpoints.get(key)
        # BMW axle distances come from the same CommonRoad vehicle model as validation.
        vehicle = VehicleParameterMapping.from_vehicle_type(VehicleType.BMW_320i)
        steering = recover_steering(current, previous, wheelbase=vehicle.a + vehicle.b)
        predicted = predict_observations(scenario, current.time_step, self.config.horizon_steps)
        problem = relative_problem(planning_problem, steering)
        rear = center_state(problem.initial_state, -vehicle.b, 0)
        initial = ReactivePlannerState(
            time_step=0,
            position=rear.position,
            orientation=rear.orientation,
            velocity=rear.velocity,
            steering_angle=steering,
            acceleration=(
                previous.acceleration if previous is not None else (current.acceleration or 0.0)
            ),
            yaw_rate=current.velocity * np.tan(steering) / (vehicle.a + vehicle.b),
        )
        planner = self._planner(key, predicted, problem)
        # Rebuild collision checking from this observation; reset also recomputes
        # curvilinear state at relative time zero using the cached reference path.
        planner.reset(initial_state_cart=initial, coordinate_system=planner.coordinate_system)
        planner.set_desired_velocity(current_speed=current.velocity)
        optimal = planner.plan()
        if optimal is None:
            raise NoPathFound(
                "Reactive planner found no feasible trajectory under the current prediction"
            )

        states = [
            center_state(state, vehicle.b, current.time_step) for state in optimal[0].state_list
        ]
        timestamps = list(range(current.time_step, current.time_step + len(states)))
        if len(states) <= steps or [state.time_step for state in states] != timestamps:
            raise InvalidPlan(
                "Reactive planner returned a short trajectory or nonconsecutive timestamps"
            )
        assert_same_state(states[0], current, steering=steering)
        states = states[: steps + 1]
        for index, state in enumerate(states[1:], 1):
            if planning_problem.goal.is_reached(state):
                states = states[: index + 1]
                break
        prefix = Trajectory(current.time_step, states)
        validate_prefix(
            prefix,
            scenario.dt,
            VehicleType.BMW_320i,
            Rectangle(vehicle.l, vehicle.w),
            planner.collision_checker,
        )
        endpoint = ReactivePlannerState.shift_state_to_center(
            optimal[0].state_list[len(states) - 1], vehicle.b
        )
        endpoint.time_step += current.time_step
        # Preserve acceleration and steering omitted by SUMO's InitialState.
        self._endpoints[key] = endpoint
        self.diagnostics.append(
            {
                "time_step": current.time_step,
                "horizon_steps": self.config.horizon_steps,
                "executed_steps": len(states) - 1,
                "planning_seconds": time.monotonic() - started,
            }
        )
        return Trajectory(current.time_step + 1, states[1:])
