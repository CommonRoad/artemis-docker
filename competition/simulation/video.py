"""Render simulation results without a dependency on the legacy sumocr EgoVehicle."""

from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from commonroad.planning.planning_problem import PlanningProblemSet
from commonroad.prediction.prediction import TrajectoryPrediction
from commonroad.scenario.obstacle import DynamicObstacle, ObstacleType
from commonroad.scenario.scenario import Scenario
from commonroad.visualization.mp_renderer import MPRenderer
from commonroad.visualization.draw_params import MPDrawParams


def create_video(
    scenario: Scenario,
    output_folder: str | Path,
    planning_problem_set: PlanningProblemSet | None = None,
    trajectory_pred: dict[int, DynamicObstacle] | list[TrajectoryPrediction] | None = None,
    follow_ego=False,
    suffix="",
    file_type="mp4",
) -> str:
    if file_type not in ("mp4", "gif"):
        raise ValueError("file_type must be mp4 or gif")
    if isinstance(trajectory_pred, dict):
        egos = list(trajectory_pred.values())
    else:
        # Keep the previous list-of-TrajectoryPrediction convenience interface.
        egos = [
            DynamicObstacle(
                scenario.generate_object_id(),
                ObstacleType.CAR,
                prediction.shape,
                prediction.trajectory.state_list[0],
                prediction,
            )
            for prediction in (trajectory_pred or [])
        ]
    # The public runner may retain obstacles scheduled to enter after the run.
    # Bound an ego video by the driven trajectory, not those future departures.
    all_obstacles = egos or scenario.dynamic_obstacles
    final_step = max(
        (
            o.prediction.final_time_step if o.prediction else o.initial_state.time_step
            for o in all_obstacles
        ),
        default=0,
    )
    renderer = MPRenderer(figsize=(6, 6))
    params = MPDrawParams()
    params.axis_visible = False

    def draw(step):
        renderer.clear(keep_static_artists=False)
        params.time_begin = step
        scenario.draw(renderer, draw_params=params)
        if planning_problem_set is not None:
            planning_problem_set.draw(renderer)
        ego_params = MPDrawParams()
        ego_params.time_begin = step
        ego_params.dynamic_obstacle.vehicle_shape.occupancy.shape.facecolor = "green"
        renderer.draw_list(egos, draw_params=ego_params)
        if follow_ego and egos:
            state = egos[0].state_at_time(step)
            if state is not None:
                x, y = state.position
                renderer.plot_limits = [x - 60, x + 60, y - 60, y + 60]
        renderer.render()
        return ()

    output = Path(output_folder)
    output.mkdir(parents=True, exist_ok=True)
    filename = f"{scenario.scenario_id}{suffix}.{file_type}"
    animation = FuncAnimation(
        renderer.f, draw, frames=range(final_step + 1), interval=1000 * scenario.dt, blit=False
    )
    try:
        animation.save(
            str(output / filename), writer="ffmpeg" if file_type == "mp4" else "pillow", dpi=100
        )
    finally:
        plt.close(renderer.f)
    return filename
