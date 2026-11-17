"""Video rendering for saved CommonRoad solutions with highlighted ego vehicles."""

from pathlib import Path
from typing import Callable, List, Literal, Optional, Union

import matplotlib.axes
import matplotlib.figure
import matplotlib.pyplot as plt
from commonroad.planning.planning_problem import PlanningProblemSet
from commonroad.scenario.obstacle import DynamicObstacle
from commonroad.scenario.scenario import Scenario
from commonroad.visualization.draw_params import BaseParam, MPDrawParams
from commonroad.visualization.drawable import IDrawable
from commonroad.visualization.mp_renderer import MPRenderer
from commonroad.visualization.util import approximate_bounding_box_dyn_obstacles
from commonroad_sumo.errors import SumoInterfaceError
from matplotlib.animation import FuncAnimation
from tqdm import tqdm

MAGENTA = "#FF00FF"
MAGENTA_DARK = "#990099"
ORANGE = "#FF8C00"
ORANGE_DARK = "#995400"
RED = "#FF0000"
RED_DARK = "#990000"
BLACK = "#000000"
BLACK_LIGHT = "#4D4D4D"

_DEFAULT_VIDEO_FIG_SIZE = [16, 9]
_DEFAULT_VIDEO_DPI = 120


class SpecificMPRenderer(MPRenderer):
    def create_specific_video(self,
                              obj_lists: List[IDrawable],
                              ego_vehicles: List[IDrawable],
                              end_time_step,
                              file_path: str,
                              delta_time_steps: int = 1,
                              plotting_horizon: int = 0,
                              draw_params: Union[List[Optional[BaseParam]], BaseParam, None] = None,

                              fig_size: Union[list, None] = None,
                              dt: Optional[int] = None,
                              dpi: int = 120,
                              progress: bool = True,
                              callback: Optional[
                                  Callable[[matplotlib.figure.Figure, matplotlib.axes.Axes, int], None]] = None,
                              ):

        ego_draw_params = MPDrawParams()
        ego_draw_params.time_end = end_time_step
        ego_draw_params.dynamic_obstacle.show_label = False
        ego_draw_params.dynamic_obstacle.draw_icon = True
        ego_draw_params.dynamic_obstacle.draw_shape = True
        ego_draw_params.dynamic_obstacle.trajectory.draw_trajectory = True
        ego_draw_params.dynamic_obstacle.vehicle_shape.occupancy.shape.facecolor = ORANGE
        ego_draw_params.dynamic_obstacle.vehicle_shape.occupancy.shape.edgecolor = ORANGE_DARK

        if not isinstance(draw_params, list):
            draw_params = [draw_params] * len(obj_lists)
        for i, params in enumerate(draw_params):
            draw_params[i] = params or self.draw_params

        ego_draw_params = [ego_draw_params] * len(ego_vehicles)

        time_begin = draw_params[0].time_begin
        time_end = draw_params[0].time_end
        assert time_begin < time_end, (
                "<video/create_scenario_video> "
                "time_begin=%i needs to smaller than "
                "time_end=%i." % (time_begin, time_end)
        )

        if fig_size is None:
            fig_size = _DEFAULT_VIDEO_FIG_SIZE

        self.ax.clear()
        self.f.set_size_inches(*fig_size)
        self.ax.set_aspect("equal")

        def init_frame():
            self.draw_list(obj_lists, draw_params=draw_params)
            self.draw_list(ego_vehicles, draw_params=ego_draw_params)
            self.render_static()

        def update(frame=0):
            current_time_step = time_begin + delta_time_steps * frame
            for params_iter in draw_params:
                params_iter.time_begin = current_time_step
                params_iter.time_end = time_begin + min(frame_count, delta_time_steps * frame + plotting_horizon)
            for ego_params_iter in ego_draw_params:
                ego_params_iter.time_begin = current_time_step
                ego_params_iter.time_end = time_begin + min(frame_count, delta_time_steps * frame + plotting_horizon)
            self.remove_dynamic()
            self.clear()
            self.draw_list(obj_lists, draw_params=draw_params)
            self.draw_list(ego_vehicles, draw_params=ego_draw_params)
            artists = self.render_dynamic()
            if callback is not None:
                callback(self.f, self.ax, frame)

            if hasattr(self, 'focus_obstacle_id') and self.focus_obstacle_id is not None and len(ego_vehicles) > 0:
                ego_state = ego_vehicles[0].state_at_time(current_time_step)
                if ego_state is not None:
                    ego_x = ego_state.position[0]
                    ego_y = ego_state.position[1]

                    # Adjust these values to zoom in or out (meters)
                    window_width = 80.0
                    window_height = 80.0

                    self.ax.set_xlim(ego_x - (window_width / 2), ego_x + (window_width / 2))
                    self.ax.set_ylim(ego_y - (window_height / 2), ego_y + (window_height / 2))
                else:
                    self.ax.autoscale()

            else:
                if self.plot_limits is None:
                    self.ax.autoscale()
                elif self.plot_limits == "auto":
                    limits = approximate_bounding_box_dyn_obstacles(obj_lists, current_time_step)
                    if limits is not None:
                        self.ax.set_xlim(limits[0][0] - 10, limits[0][1] + 10)
                        self.ax.set_ylim(limits[1][0] - 10, limits[1][1] + 10)
                    else:
                        self.ax.autoscale()
                else:
                    self.ax.set_xlim(self.plot_limits_focused[0], self.plot_limits_focused[1])
                    self.ax.set_ylim(self.plot_limits_focused[2], self.plot_limits_focused[3])

            if not self.draw_params.axis_visible:
                self.ax.axes.xaxis.set_visible(False)
                self.ax.axes.yaxis.set_visible(False)
                self.ax.axis("off")

            return artists

            # Min frame rate is 1 fps

        if dt is None:
            # Try to infer from a scenario
            for obj in obj_lists:
                if isinstance(obj, Scenario):
                    dt = obj.dt * 1000.0
                    break
            else:
                dt = 100.0
        dt = min(1000.0, dt)
        frame_count = (time_end - time_begin) // delta_time_steps
        plt.ioff()
        ax = self.f.gca()
        ax.set_aspect('equal')

        self.f.subplots_adjust(top=0.98, bottom=0.02)

        anim = FuncAnimation(self.f, update, frames=frame_count, init_func=init_frame, blit=False, interval=dt)

        if not any([file_path.endswith(".mp4"), file_path.endswith(".gif"), file_path.endswith(".avi")]):
            file_path += ".mp4"
        interval_seconds = dt / 1000.0
        with tqdm(total=frame_count, unit="frame", disable=not progress) as t:
            anim.save(
                file_path,
                dpi=dpi,
                writer="ffmpeg",
                extra_args=["-g", "1", "-keyint_min", str(interval_seconds)],
                progress_callback=lambda *_: t.update(1),

            )
        self.clear()
        self.ax.clear()

    def create_video_with_ego_vehicles(self,
            output_folder: Path | str,
            scenario: Scenario,
            ego_vehicle_obstacles: list[DynamicObstacle],
            planning_problem_set: PlanningProblemSet | None = None,
            follow_ego: bool = True,
            video_file_type: Literal["mp4", "gif"] = "gif",
    ):
        # The renderer does not automatically determine the time horizon, so it must be done manually.
        start_time_step = min(ego_vehicle.initial_state.time_step for ego_vehicle in ego_vehicle_obstacles)
        end_time_step = max(ego_vehicle.prediction.final_time_step for ego_vehicle in ego_vehicle_obstacles)
        draw_params = MPDrawParams(
            time_begin=start_time_step,
            time_end=end_time_step,
            # If `follow_ego` is active the axis would stutter during the movement of the ego vehicle.
            axis_visible=not follow_ego,
        )
        draw_params.dynamic_obstacle.show_label = False
        draw_params.dynamic_obstacle.draw_icon = True
        draw_params.dynamic_obstacle.draw_shape = True
        draw_params.dynamic_obstacle.trajectory.draw_trajectory = True

        # If follow_ego is enabled the renderer should focus on the single ego vehicle.
        if follow_ego:
            if len(ego_vehicle_obstacles) != 1:
                raise SumoInterfaceError(
                    f"Cannot create video for scenario {scenario.scenario_id}: If `follow_ego` is enabled only one ego vehicle must be given, but {len(ego_vehicle_obstacles)} ego vehicles are given (ego vehicles: {', '.join([str(obstacle.obstacle_id) for obstacle in ego_vehicle_obstacles])})"
                )

            self.focus_obstacle_id = ego_vehicle_obstacles[0].obstacle_id

        rnd_obj_list: list[IDrawable] = [scenario]
        rnd_ego_list: list[IDrawable] = []


        for ego_vehicle_obstacle in ego_vehicle_obstacles:
            rnd_ego_list.append(ego_vehicle_obstacle)

        # If the planning problem set is given it should also be rendered.
        if planning_problem_set is not None:
            rnd_obj_list.append(planning_problem_set)

        # Determine the video file location.
        if not isinstance(output_folder, Path):
            output_folder = Path(output_folder)
        file_path = (output_folder / str(scenario.scenario_id)).with_suffix(
            f".{video_file_type}"
        )

        self.create_specific_video(
            obj_lists=rnd_obj_list,
            ego_vehicles=rnd_ego_list,
            end_time_step=end_time_step,
            file_path=str(file_path),
            dpi=_DEFAULT_VIDEO_DPI,
            dt=scenario.dt * 1000.0,
            fig_size=_DEFAULT_VIDEO_FIG_SIZE,
            draw_params=draw_params,

        )

        return file_path
