"""Keep planner commands, SUMO observations and exported histories on one clock."""

from copy import deepcopy
from itertools import combinations
from typing import Self
import xml.etree.ElementTree as ET

import libsumo
import numpy as np
from commonroad.geometry.shape import Circle, Rectangle
from commonroad.planning.planning_problem import PlanningProblemSet
from commonroad.prediction.prediction import TrajectoryPrediction
from commonroad.scenario.obstacle import DynamicObstacle, ObstacleType
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.state import InitialState, State
from commonroad.scenario.trajectory import Trajectory
from commonroad_sumo import InteractiveSumoSimulationWithMotionPlanner
from commonroad_sumo.cr2sumo import (
    ResimulationTrafficGeneratorConfig,
)
from commonroad_sumo.errors import SumoInterfaceError
from commonroad_sumo.interface.util import calculate_angle_from_orientation
from commonroad_sumo.sumolib.sumo_project import SumoFileType
from commonroad_sumo.utils import convert_state_to_state_type

from .conversion import StableMapConverter
from .ego_routes import EgoRoutePreloader
from .network_geometry import prepare_traffic_geometry
from .traffic import TerminalAwareTrafficGenerator


def _append_state(obstacle: DynamicObstacle, state: State) -> None:
    if obstacle.prediction is None:
        obstacle.prediction = TrajectoryPrediction(
            Trajectory(state.time_step, [deepcopy(state)]), obstacle.obstacle_shape
        )
    else:
        obstacle.prediction.trajectory.append_state(deepcopy(state))


def _place_background_vehicle(obstacle: DynamicObstacle, state: State) -> None:
    """Place the exact initial pose while retaining the assigned traffic route."""
    shape = obstacle.obstacle_shape
    if isinstance(shape, Rectangle):
        length = shape.length
    elif isinstance(shape, Circle):
        length = 2 * shape.radius
    else:
        raise ValueError(f"Unsupported traffic shape for obstacle {obstacle.obstacle_id}: {shape}")
    front = state.position + 0.5 * length * np.array(
        [np.cos(state.orientation), np.sin(state.orientation)]
    )
    identifier = str(obstacle.obstacle_id)
    libsumo.vehicle.moveToXY(
        identifier,
        "dummy",
        -1,
        float(front[0]),
        float(front[1]),
        calculate_angle_from_orientation(state.orientation),
        # Keep the assigned route. The birth-step check rejects any pose change.
        keepRoute=1,
    )
    if state.has_value("velocity"):
        libsumo.vehicle.setSpeed(identifier, state.velocity)


class InteractiveSimulation(InteractiveSumoSimulationWithMotionPlanner):
    """Use SUMO for traffic and execute the planner's validated ego states exactly.

    One initialization step places the complete CommonRoad state zero. Subsequent
    SUMO steps execute transitions; raw SUMO time is therefore (step + 1) * dt.
    """

    @classmethod
    def from_scenario(
        cls, scenario: Scenario, planning_problem_set: PlanningProblemSet, **kwargs
    ) -> Self:
        project = StableMapConverter(scenario).create_sumo_files()
        if project is None:
            raise SumoInterfaceError(f"Failed to convert {scenario.scenario_id} to SUMO!")
        traffic = TerminalAwareTrafficGenerator(
            config=ResimulationTrafficGeneratorConfig(
                extend_routes_downstream_junction=True,
                extend_routes_upstream_junction=True,
                extend_routes=True,
            )
        )
        if not traffic.generate_traffic(scenario, project):
            raise SumoInterfaceError(f"Failed to generate traffic for {scenario.scenario_id}!")
        geometry_adjustments = prepare_traffic_geometry(scenario, project)
        runner = cls(scenario, planning_problem_set, project, **kwargs)
        runner.traffic_geometry_adjustments = geometry_adjustments
        runner.traffic_route_diagnostics = traffic.route_matching_diagnostics
        return runner

    def simulation_loop(self) -> None:
        self._ego_route_preloader = EgoRoutePreloader(
            self._sumo_project.get_file_path(SumoFileType.NET)
        )
        config = self._sumo_project.get_file_path(SumoFileType.CONFIG)
        tree = ET.parse(config)
        processing = tree.getroot().find("processing")
        if processing is None:
            processing = ET.SubElement(tree.getroot(), "processing")
        ET.SubElement(processing, "collision.mingap-factor", value="0")
        tree.write(config, encoding="utf-8", xml_declaration=True)
        self._pending = {key: [] for key in self._ego_vehicles}
        self._ego_ids = {ego.obstacle_id for ego in self._ego_vehicles.values()}
        self.collision_events = []
        self._simulation_backend.start(config)
        try:
            self._place_arrivals(0)
            self._simulation_backend.simulation_step()
            self._finish_arrivals(0)
            self._post_simulation_step_hook(0)
            step = 0
            while not self._reached_simulation_end_condition(step):
                self._pre_simulation_step_hook(step)
                self._sumo_interface.simulate_step()
                step += 1
                self._finish_arrivals(step)
                for key, ego in self._ego_vehicles.items():
                    if ego.initial_state.time_step < step:
                        _append_state(ego, self._pending[key].pop(0))
                self._post_simulation_step_hook(step)
        finally:
            self._simulation_backend.stop()

    def _place(self, obstacle: DynamicObstacle, state: State) -> None:
        if (
            obstacle.obstacle_id not in self._ego_ids
            and obstacle.obstacle_type != ObstacleType.PEDESTRIAN
        ):
            _place_background_vehicle(obstacle, state)
            return
        interface = (
            self._sumo_interface.pedestrians
            if obstacle.obstacle_type == ObstacleType.PEDESTRIAN
            else self._sumo_interface.vehicles
        )
        sumo_id = str(obstacle.obstacle_id)
        if obstacle.obstacle_id in self._ego_ids:
            if self._id_mapper.cr2sumo(obstacle.obstacle_id) is None:
                self._id_mapper.insert_mapping(sumo_id, obstacle.obstacle_id)
                interface._add_new_dynamic_obstacle_in_sumo_simulation(sumo_id, obstacle)
                # Planner feasibility and collision validation govern controlled egos.
                libsumo.vehicle.setSpeedMode(sumo_id, 96)
                libsumo.vehicle.setLaneChangeMode(sumo_id, 0)
        interface._sync_dynamic_obstacle_state_to_sumo_simulation(sumo_id, state, obstacle)

    def _place_arrivals(self, step: int) -> None:
        for actor in [*self._scenario.dynamic_obstacles, *self._ego_vehicles.values()]:
            if actor.initial_state.time_step == step:
                self._place(actor, actor.initial_state)

    def _finish_arrivals(self, step: int) -> None:
        for actor in [*self._scenario.dynamic_obstacles, *self._ego_vehicles.values()]:
            if actor.initial_state.time_step == step:
                sumo_id = str(actor.obstacle_id)
                if actor.obstacle_type == ObstacleType.PEDESTRIAN:
                    self._check_state(actor, actor.initial_state)
                    continue
                libsumo.vehicle.setPreviousSpeed(sumo_id, actor.initial_state.velocity)
                if actor.obstacle_id not in self._ego_ids:
                    # Calibration sets a temporary speed; traffic resumes its own driving model.
                    libsumo.vehicle.setSpeed(sumo_id, -1)
                self._check_state(actor, actor.initial_state)

    def _pre_simulation_step_hook(self, step: int) -> None:
        self._place_arrivals(step + 1)
        for key, ego in self._ego_vehicles.items():
            current = ego.state_at_time(step)
            if current is None:
                continue
            if not self._pending[key]:
                problem = self._planning_problem_set.planning_problem_dict[key]
                problem.initial_state = convert_state_to_state_type(current, InitialState)
                self._pending[key] = self._execute_motion_planner(problem).state_list
            self._ego_route_preloader.prepare(ego, self._pending[key])
            libsumo.vehicle.setPreviousSpeed(str(ego.obstacle_id), current.velocity)
            self._place(ego, self._pending[key][0])

    def _post_simulation_step_hook(self, step: int) -> None:
        for actor in self._scenario.dynamic_obstacles:
            if actor.initial_state.time_step < step:
                self.sync_from_sumo_simulation(actor)
        for ego in self._ego_vehicles.values():
            state = ego.state_at_time(step)
            if state is not None:
                self._check_state(ego, state)
        self._record_collisions(step)

    def _check_state(self, actor: DynamicObstacle, state: State) -> None:
        interface = (
            self._sumo_interface.pedestrians
            if actor.obstacle_type == ObstacleType.PEDESTRIAN
            else self._sumo_interface.vehicles
        )
        actual = interface._fetch_dynamic_obstacle_state_params_from_sumo_simulation(
            str(actor.obstacle_id)
        )
        heading_error = np.arctan2(
            np.sin(actual["orientation"] - state.orientation),
            np.cos(actual["orientation"] - state.orientation),
        )
        if (
            np.linalg.norm(actual["position"] - state.position) > 1e-6
            or abs(heading_error) > 1e-6
            or abs(actual["velocity"] - state.velocity) > 1e-6
        ):
            raise RuntimeError(
                f"SUMO actor {actor.obstacle_id} does not match state {state.time_step}"
            )

    def _record_collisions(self, step: int) -> None:
        native = {
            tuple(sorted((int(c.collider), int(c.victim))))
            for c in libsumo.simulation.getCollisions()
        }
        shapes = {}
        for actor in [*self._scenario.dynamic_obstacles, *self._ego_vehicles.values()]:
            state = actor.state_at_time(step)
            if state is not None:
                shapes[actor.obstacle_id] = actor.obstacle_shape.rotate_translate_local(
                    state.position, state.orientation
                ).shapely_object
        contacts = {
            pair
            for pair in combinations(sorted(shapes), 2)
            if self._ego_ids.intersection(pair) and shapes[pair[0]].intersects(shapes[pair[1]])
        }
        physical = contacts | {pair for pair in native if not self._ego_ids.intersection(pair)}
        if native or physical:
            self.collision_events.append(
                dict(
                    step=step,
                    sumo_pairs=sorted(native),
                    physical_pairs=sorted(physical),
                    geometry_disagreements=sorted(native - physical),
                )
            )

    def _reached_simulation_end_condition(self, step: int) -> bool:
        if step >= self._max_simulation_steps:
            return True
        return all(
            (state := self._ego_vehicles[key].state_at_time(step)) is not None
            and problem.goal.is_reached(state)
            for key, problem in self._planning_problem_set.planning_problem_dict.items()
        )
