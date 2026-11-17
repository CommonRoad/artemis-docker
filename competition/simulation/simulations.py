"""Convert standalone CommonRoad XMLs and run the public CommonRoad-SUMO API."""

from copy import deepcopy
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Self

import numpy as np
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.common.solution import Solution, VehicleType
from commonroad.planning.planning_problem import PlanningProblem, PlanningProblemSet
from commonroad.scenario.obstacle import DynamicObstacle
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.trajectory import Trajectory
from commonroad_dc.feasibility.vehicle_dynamics import VehicleParameterMapping
from commonroad_sumo.cr2sumo import (
    ResimulationTrafficGeneratorConfig,
)
from commonroad_sumo.errors import SumoInterfaceError
from commonroad_sumo.simulation.abstract_sumo_simulation import SumoSimulationConfig
from commonroad_sumo.simulation.interactive_simulation import (
    InteractiveSumoSimulationResult,
    InteractiveSumoSimulationWithMotionPlannerConfig,
)
from commonroad_sumo.simulation.non_interactive_simulation import NonInteractiveSumoSimulation
from commonroad_sumo.sumolib.sumo_project import SumoFileType

from planner.runtime.submission import DEFAULT_SUBMISSION, load_submission

from .planner_adapter import (
    PlannerAdapter,
    PlannerOutputError,
    restore_static_obstacles,
)
from .video import create_video
from .conversion import StableMapConverter
from .network_geometry import prepare_traffic_geometry
from .interactive import InteractiveSimulation as InteractiveSumoSimulationWithMotionPlanner
from .traffic import (
    TerminalAwareTrafficGenerator,
    preserve_vehicle_dimensions,
    remove_initial_state_from_predictions,
)


@dataclass(frozen=True)
class SimulationSettings:
    """Simulation horizon, replanning interval, seed and vehicle dimensions."""

    simulation_steps: int = 500
    replanning_interval: int = 1
    random_seed: int = 1234
    lateral_resolution: float = 1.0
    # Match the default PlannerSpec vehicle type.
    ego_veh_length: float = VehicleParameterMapping.FORD_ESCORT.value.l
    ego_veh_width: float = VehicleParameterMapping.FORD_ESCORT.value.w

    @classmethod
    def for_vehicle(cls, vehicle_type: VehicleType, **settings) -> Self:
        """Use the vehicle model's dimensions with the requested simulation settings."""
        vehicle = VehicleParameterMapping.from_vehicle_type(vehicle_type)
        return cls(ego_veh_length=vehicle.l, ego_veh_width=vehicle.w, **settings)

    def __post_init__(self):
        if self.simulation_steps < 1 or self.replanning_interval < 1:
            raise ValueError("Simulation steps and replanning interval must be positive")
        if self.random_seed < 0:
            raise ValueError("random_seed must be nonnegative")
        for name in ("lateral_resolution", "ego_veh_length", "ego_veh_width"):
            value = getattr(self, name)
            if not np.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be finite and positive")


def _settings(settings: SimulationSettings | None, num_of_steps: int | None) -> SimulationSettings:
    settings = settings or SimulationSettings()
    return settings if num_of_steps is None else replace(settings, simulation_steps=num_of_steps)


def _read(path: str | Path, use_sumo_manager=False) -> tuple[Scenario, PlanningProblemSet]:
    if use_sumo_manager:
        raise ValueError("SUMO Manager is unsupported; the runner uses local libsumo")
    path = Path(path)
    if not path.is_file() or path.suffix != ".xml":
        raise ValueError("Provide a standalone CommonRoad XML file")
    return CommonRoadFileReader(str(path)).open()


def run_simulation(
    scenario: Scenario,
    planning_problem_set: PlanningProblemSet,
    planner,
    *,
    settings: SimulationSettings | None = None,
    num_of_steps: int | None = None,
) -> InteractiveSumoSimulationResult:
    settings = _settings(settings, num_of_steps)
    if not np.isfinite(scenario.dt) or scenario.dt <= 0:
        raise ValueError("Scenario dt must be finite and positive")
    config = InteractiveSumoSimulationWithMotionPlannerConfig(
        random_seed=settings.random_seed,
        lateral_resolution=settings.lateral_resolution,
        ego_veh_length=settings.ego_veh_length,
        ego_veh_width=settings.ego_veh_width,
    )
    runner = InteractiveSumoSimulationWithMotionPlanner.from_scenario(
        scenario, planning_problem_set, simulation_config=config
    )
    preserve_vehicle_dimensions(
        scenario, runner._sumo_project.get_file_path(SumoFileType.VEHICLE_ROUTES)
    )
    result = runner.run(
        PlannerAdapter(planner, static_obstacles=scenario.static_obstacles),
        reevaluation_interval=settings.replanning_interval,
        max_simulation_steps=settings.simulation_steps,
    )
    restore_static_obstacles(result.scenario, scenario.static_obstacles)
    remove_initial_state_from_predictions(result.scenario)
    return result


def simulate_with_planner(
    interactive_scenario_path: str | Path,
    output_folder_path=None,
    create_video=False,
    use_sumo_manager=False,
    create_ego_obstacle=False,
    *,
    num_of_steps: int | None = None,
    planner=None,
    settings: SimulationSettings | None = None,
) -> tuple[Scenario, PlanningProblemSet, dict[int, DynamicObstacle]]:
    """Return (simulated scenario, original problems, egos keyed by problem ID)."""
    scenario, problems = _read(interactive_scenario_path, use_sumo_manager)
    if planner is None:
        with load_submission(DEFAULT_SUBMISSION) as spec:
            spec.settings = settings or spec.settings
            spec.validate()
            result = run_simulation(
                scenario, problems, spec.planner, settings=spec.settings, num_of_steps=num_of_steps
            )
    else:
        result = run_simulation(
            scenario, problems, planner, settings=settings, num_of_steps=num_of_steps
        )
    if create_video:
        create_video_for_simulation(
            result.scenario, output_folder_path, problems, result.ego_vehicles, "_planner"
        )
    if create_ego_obstacle:
        result.scenario.add_objects(deepcopy(list(result.ego_vehicles.values())))
    return result.scenario, problems, result.ego_vehicles


def simulate_without_ego(
    interactive_scenario_path: str | Path,
    output_folder_path=None,
    create_video=False,
    use_sumo_manager=False,
    num_of_steps: int | None = None,
    *,
    settings: SimulationSettings | None = None,
) -> tuple[Scenario, PlanningProblemSet]:
    scenario, problems = _read(interactive_scenario_path, use_sumo_manager)
    settings = _settings(settings, num_of_steps)
    traffic = TerminalAwareTrafficGenerator(
        config=ResimulationTrafficGeneratorConfig(
            extend_routes=True,
            extend_routes_downstream_junction=True,
            extend_routes_upstream_junction=True,
        )
    )
    project = StableMapConverter(scenario).create_sumo_files()
    if project is None:
        raise SumoInterfaceError(f"Failed to convert {scenario.scenario_id} to SUMO!")
    if not traffic.generate_traffic(scenario, project):
        raise SumoInterfaceError(f"Failed to generate traffic for {scenario.scenario_id}!")
    geometry_adjustments = prepare_traffic_geometry(scenario, project)
    runner = NonInteractiveSumoSimulation(
        scenario,
        project,
        simulation_config=SumoSimulationConfig(
            random_seed=settings.random_seed, lateral_resolution=settings.lateral_resolution
        ),
    )
    runner.traffic_geometry_adjustments = geometry_adjustments
    runner.traffic_route_diagnostics = traffic.route_matching_diagnostics
    preserve_vehicle_dimensions(
        scenario, runner._sumo_project.get_file_path(SumoFileType.VEHICLE_ROUTES)
    )
    result = runner.run(settings.simulation_steps + 1).scenario
    restore_static_obstacles(result, scenario.static_obstacles)
    remove_initial_state_from_predictions(result)
    if create_video:
        create_video_for_simulation(result, output_folder_path, problems, {}, "_without_ego")
    return result, problems


def simulate_with_solution(
    interactive_scenario_path: str | Path,
    output_folder_path=None,
    solution: Solution | None = None,
    create_video=False,
    use_sumo_manager=False,
    create_ego_obstacle=False,
    *,
    num_of_steps: int | None = None,
    settings: SimulationSettings | None = None,
) -> tuple[Scenario, PlanningProblemSet, dict[int, DynamicObstacle]]:
    if not isinstance(solution, Solution):
        raise PlannerOutputError("A CommonRoad Solution is required for replay")
    scenario, problems = _read(interactive_scenario_path, use_sumo_manager)
    if solution.scenario_id != scenario.scenario_id:
        raise PlannerOutputError("Replay solution scenario ID does not match")
    if set(solution.planning_problem_ids) != set(problems.planning_problem_dict):
        raise PlannerOutputError("Replay solution planning problem IDs do not match the scenario")
    settings = _settings(settings, num_of_steps)
    if any(
        s.trajectory.final_state.time_step < settings.simulation_steps
        for s in solution.planning_problem_solutions
    ):
        raise PlannerOutputError(
            f"Replay requires a trajectory through step {settings.simulation_steps}"
        )

    class ReplayPlanner:
        def plan(self, observation: Scenario, problem: PlanningProblem) -> Trajectory:
            selected = next(
                s
                for s in solution.planning_problem_solutions
                if s.planning_problem_id == problem.planning_problem_id
            )
            states = [
                s
                for s in selected.trajectory.state_list
                if s.time_step > problem.initial_state.time_step
            ]
            if not states:
                raise PlannerOutputError("Replay trajectory ended before simulation")
            return Trajectory(states[0].time_step, deepcopy(states))

    return simulate_with_planner(
        interactive_scenario_path,
        output_folder_path,
        create_video,
        use_sumo_manager,
        create_ego_obstacle,
        planner=ReplayPlanner(),
        settings=settings,
    )


def create_video_for_simulation(
    scenario: Scenario,
    output_folder_path: str | Path,
    planning_problem_set: PlanningProblemSet,
    ego_vehicles: dict[int, DynamicObstacle],
    suffix,
    follow_ego=True,
) -> str:
    if output_folder_path is None:
        raise ValueError("An output folder is required when create_video=True")
    return create_video(
        scenario,
        output_folder_path,
        planning_problem_set,
        ego_vehicles,
        follow_ego=follow_ego,
        suffix=suffix,
    )
