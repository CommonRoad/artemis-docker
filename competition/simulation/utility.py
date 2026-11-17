"""Visualization and solution export for CommonRoad DynamicObstacle ego results."""

from copy import deepcopy
from pathlib import Path

from commonroad.common.solution import (
    CommonRoadSolutionWriter,
    CostFunction,
    PlanningProblemSolution,
    Solution,
    VehicleModel,
    VehicleType,
)
from commonroad.planning.planning_problem import PlanningProblemSet
from commonroad.scenario.obstacle import DynamicObstacle
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.trajectory import Trajectory
from commonroad.visualization.draw_params import MPDrawParams
from commonroad.visualization.mp_renderer import MPRenderer
from commonroad_sumo.utils import get_full_state_list_of_obstacle
from IPython import display


def visualize_scenario_with_trajectory(
    scenario, planning_problem_set, ego_vehicles=None, discrete_time_step=False, num_time_steps=None
):
    egos = list((ego_vehicles or {}).values())
    if num_time_steps is None:
        num_time_steps = max(
            (e.prediction.final_time_step + 1 for e in egos if e.prediction), default=50
        )
    for step in range(num_time_steps):
        if not discrete_time_step:
            display.clear_output(wait=True)
        renderer = MPRenderer(figsize=(12, 12))
        renderer.draw_params.time_begin = step
        scenario.draw(renderer)
        planning_problem_set.draw(renderer)
        renderer.draw_params.dynamic_obstacle.vehicle_shape.occupancy.shape.facecolor = "green"
        renderer.draw_list(egos)
        renderer.render(show=True)


def make_solution(
    scenario: Scenario,
    planning_problem_set: PlanningProblemSet,
    ego_vehicles: dict[int, DynamicObstacle],
    vehicle_type: VehicleType,
    vehicle_model: VehicleModel,
    cost_function: CostFunction,
) -> Solution:
    """Export initial state and driven states once, without modifying the results."""
    if set(ego_vehicles) != set(planning_problem_set.planning_problem_dict):
        raise ValueError("Ego vehicle IDs must match the planning problem IDs")
    solutions = []
    for problem_id, ego in ego_vehicles.items():
        states = deepcopy(get_full_state_list_of_obstacle(ego))
        initial = planning_problem_set.planning_problem_dict[problem_id].initial_state
        if states[0].time_step != initial.time_step:
            raise ValueError("Ego trajectory does not start at the original initial time")
        solutions.append(
            PlanningProblemSolution(
                problem_id,
                vehicle_model,
                vehicle_type,
                cost_function,
                Trajectory(states[0].time_step, states),
            )
        )
    return Solution(scenario.scenario_id, solutions)


def save_solution(
    scenario: Scenario,
    planning_problem_set: PlanningProblemSet,
    ego_vehicles: dict[int, DynamicObstacle],
    vehicle_type: VehicleType,
    vehicle_model: VehicleModel,
    cost_function: CostFunction,
    output_path: str | Path = "./",
    overwrite=False,
) -> Solution:
    solution = make_solution(
        scenario, planning_problem_set, ego_vehicles, vehicle_type, vehicle_model, cost_function
    )
    Path(output_path).mkdir(parents=True, exist_ok=True)
    CommonRoadSolutionWriter(solution).write_to_file(
        output_path=str(output_path), overwrite=overwrite
    )
    return solution


def visualize_scenario_with_solution(
    scenario: Scenario,
    planning_problem_set: PlanningProblemSet,
    solution: Solution,
    discrete_time_step: bool = False,
):
    ego_vehicles = list(solution.create_dynamic_obstacle().values())
    start_time_step = min(ego_vehicle.initial_state.time_step for ego_vehicle in ego_vehicles)
    end_time_step = max(ego_vehicle.prediction.final_time_step for ego_vehicle in ego_vehicles)

    # visualize scenario
    for i in range(start_time_step, end_time_step):
        if not discrete_time_step:
            display.clear_output(wait=True)
        rnd = MPRenderer(figsize=(20, 20))
        rnd.draw_params.time_begin = i
        scenario.draw(rnd)
        planning_problem_set.draw(rnd)

        rnd.draw_params.dynamic_obstacle.vehicle_shape.occupancy.shape.facecolor = "green"
        rnd.draw_list(ego_vehicles)

        rnd.render(show=True)


def create_video_for_scenario_with_solution(
    scenario: Scenario,
    planning_problem_set: PlanningProblemSet,
    solution: Solution,
    output_file: Path | str,
) -> None:
    ego_vehicles = list(solution.create_dynamic_obstacle().values())
    start_time_step = min(ego_vehicle.initial_state.time_step for ego_vehicle in ego_vehicles)
    end_time_step = max(ego_vehicle.prediction.final_time_step for ego_vehicle in ego_vehicles)

    draw_params = MPDrawParams()
    draw_params.time_begin = start_time_step
    draw_params.time_end = end_time_step
    draw_params.dynamic_obstacle.show_label = False
    draw_params.dynamic_obstacle.draw_icon = True
    draw_params.dynamic_obstacle.draw_shape = True
    draw_params.dynamic_obstacle.trajectory.draw_trajectory = True

    rnd = MPRenderer()

    rnd.create_video(
        [scenario, planning_problem_set] + ego_vehicles,
        file_path=str(output_file),
        draw_params=draw_params,
    )
