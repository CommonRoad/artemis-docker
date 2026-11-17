"""Browser and optional desktop views of CommonRoad scenarios and solutions."""

import copy
import io
import os
import sys

import ipywidgets as widgets
import matplotlib.pyplot as plt
import numpy as np
from commonroad.common.solution import Solution
from commonroad.geometry.shape import Rectangle
from commonroad.planning.planning_problem import PlanningProblemSet
from commonroad.scenario.scenario import Scenario
from commonroad.visualization.draw_params import MPDrawParams, ShapeParams
from commonroad.visualization.mp_renderer import MPRenderer
from commonroad_route_planner.fast_api.fast_api import (
    generate_reference_path_from_scenario_and_planning_problem,
)
from IPython import get_ipython
from IPython.display import HTML, Image as NotebookImage, display
from matplotlib.animation import FuncAnimation
from matplotlib.figure import Figure
from PIL import Image

from .CR_Toolbox import CRToolbox
from .specific_mp_renderer import SpecificMPRenderer


def _show_figure(fig: Figure):
    """Display a notebook PNG even when Matplotlib uses the Agg backend."""
    if get_ipython() is None:
        plt.show(block=True)
    else:
        image = io.BytesIO()
        fig.savefig(image, format="png", bbox_inches="tight")
        display(NotebookImage(data=image.getvalue()))


def _load_desktop_tools():
    # Desktop widgets are optional; importing this module must work in a container.
    global tk, ttk, FigureCanvasTkAgg, ImageTk
    import tkinter as tk
    from tkinter import ttk
    from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
    from PIL import ImageTk


class CRVisualizer:
    def __init__(self, root=None):
        self.toolbox = CRToolbox(root)
        self.path_root = self.toolbox.path_root

    @staticmethod
    def animate_scenario(
        scenario: Scenario, problems: PlanningProblemSet, solution: Solution
    ) -> HTML:
        """Return a browser animation without a desktop window or video player."""

        egos = list(solution.create_dynamic_obstacle().values())
        last = max(s.trajectory.final_state.time_step for s in solution.planning_problem_solutions)
        vertices = np.vstack([lane.center_vertices for lane in scenario.lanelet_network.lanelets])
        bounds = [
            vertices[:, 0].min() - 5,
            vertices[:, 0].max() + 5,
            vertices[:, 1].min() - 5,
            vertices[:, 1].max() + 5,
        ]
        fig, ax = plt.subplots(figsize=(10, 4))

        def draw(step):
            ax.clear()
            params = MPDrawParams(time_begin=step, time_end=step)
            params.dynamic_obstacle.trajectory.draw_trajectory = False
            params.planning_problem.initial_state.state.draw_arrow = False
            params.planning_problem.initial_state.state.radius = 0.0
            params.planning_problem.goal_region.shape.opacity = 0.25
            renderer = MPRenderer(ax=ax, plot_limits=bounds, draw_params=params)
            scenario.draw(renderer)
            problems.draw(renderer)
            ego_params = copy.deepcopy(params)
            ego_params.dynamic_obstacle.vehicle_shape.occupancy.shape.facecolor = "green"
            renderer.draw_list(egos, draw_params=ego_params)
            renderer.render()
            ax.set_title(f"{scenario.scenario_id} — {step * scenario.dt:.1f} s")

        animation = FuncAnimation(fig, draw, frames=range(last + 1), interval=1000 * scenario.dt)
        html = animation.to_jshtml().replace(
            '<link rel="stylesheet"\n'
            'href="https://maxcdn.bootstrapcdn.com/font-awesome/4.4.0/css/font-awesome.min.css">',
            "",
        )
        # Matplotlib's control icons need a remote font; text also works offline.
        for icon, label in {
            "minus": "−",
            "fast-backward": "First",
            "step-backward": "Prev",
            "play fa-flip-horizontal": "Back",
            "pause": "Pause",
            "play": "Play",
            "step-forward": "Next",
            "fast-forward": "Last",
            "plus": "+",
        }.items():
            html = html.replace(f'<i class="fa fa-{icon}"></i>', label)
        plt.close(fig)
        return HTML(html)

    def visualize_benchmark(
        self,
        path_scenario,
        show_ref_path=False,
        figsize=(15, 5),
        ref_path_zorder=100,
        ref_path_linewidth=2,
        ref_path_color="green",
    ):
        """Display a benchmark map, planning problems and optional reference path."""
        target_scenario, planning_problem_set = self.toolbox.extract_scenario(path_scenario)

        fig, ax = plt.subplots(figsize=figsize)
        fig.canvas.manager.set_window_title(target_scenario.scenario_id)
        renderer = MPRenderer(ax=ax)
        target_scenario.draw(renderer=renderer)
        planning_problem_set.draw(renderer=renderer)
        renderer.render()
        if show_ref_path:
            planning_problem = self.toolbox.extract_planning_problem(planning_problem_set)
            reference_path = generate_reference_path_from_scenario_and_planning_problem(
                target_scenario, planning_problem
            ).reference_path
            ax.plot(
                reference_path[:, 0],
                reference_path[:, 1],
                zorder=ref_path_zorder,
                linewidth=ref_path_linewidth,
                color=ref_path_color,
            )
        _show_figure(fig)
        plt.close(fig)

    def visualize_driving_states_others(
        self, path_scenario, max_nr, start_at=0, in_jupyter_notebook=True
    ):
        """
        Visualizes the driving states of other vehicles in a given scenario.

        This method extracts a driving scenario from a given file path and allows visualizing
        the driving states of other vehicles using a slideshow. The visualization can be displayed
        either inside a Jupyter Notebook or as a standalone application.

        Parameters:
        - path_scenario (str): The file path to the scenario to be visualized.
        - max_nr (int): The maximum number of items to visualize in the slideshow.
        - start_at (int, optional): The starting point in the slideshow. Defaults to 0.
        - in_jupyter_notebook (bool, optional): If True, shows the visualization
          inside a Jupyter Notebook. If False, launches a standalone application. Defaults to True.
        """

        target_scenario, planning_problem_set = self.toolbox.extract_scenario(path_scenario)

        if not in_jupyter_notebook:
            Slideshow(
                max_nr,
                target_scenario,
                planning_problem_set,
                start_at=start_at,
                in_jupyter_notebook=False,
            )
        else:

            def show_frame(step):
                fig, ax = plt.subplots(figsize=(15, 6))
                renderer = MPRenderer(ax=ax)
                renderer.draw_params.time_begin = step
                target_scenario.draw(renderer)
                planning_problem_set.draw(renderer)
                renderer.render()
                _show_figure(fig)
                plt.close(fig)

            control = widgets.interactive(
                show_frame, step=widgets.IntSlider(min=start_at, max=max_nr, value=start_at)
            )
            display(control)
            return control

    def visualize_solution(
        self,
        path_solution,
        path_scenario,
        path_output,
        create_gif=False,
        follow_ego=True,
        create_gif_slideshow=False,
    ):
        """
        Visualizes the solution for a specific planning scenario and generates a video or GIF output.

        Detailed summary:
        This function extracts a planning scenario and solution from provided file paths, and generates a
        visualization that showcases the movement of ego vehicles and other dynamic elements. The output
        can be saved in either MP4 or GIF format. Optionally, it allows the creation of a GIF slideshow.
        The visualization can also be configured to follow the ego vehicle throughout the scenario.

        Parameters:
            path_solution: str
                Path to the file containing the planned solution to be visualized.
            path_scenario: str
                Path to the file defining the planning scenario for the solution.
            path_output: str
                Directory path where the generated visualization output will be saved.
            create_gif: bool, optional
                Flag indicating whether to generate the output as a GIF file. Defaults to False.
            follow_ego: bool, optional
                Flag indicating whether to follow the ego vehicle during the visualization. Defaults to True.
            create_gif_slideshow: bool, optional
                Flag to create a slideshow for the GIF output. Only applicable if 'create_gif' is True.
                Defaults to False.
        """
        original_backend = plt.get_backend()
        plt.switch_backend("Agg")
        try:
            scenario, planning_problem_set = self.toolbox.extract_scenario(path_scenario)
            solution = self.toolbox.extract_solution(path_solution)
            ending = ".mp4" if not create_gif else ".gif"
            ego_obstacles = list(solution.create_dynamic_obstacle().values())
            renderer = SpecificMPRenderer()
            video_file_type = "mp4" if not create_gif else "gif"
            renderer.create_video_with_ego_vehicles(
                output_folder=path_output,
                scenario=scenario,
                ego_vehicle_obstacles=ego_obstacles,
                planning_problem_set=planning_problem_set,
                follow_ego=follow_ego,
                video_file_type=video_file_type,
            )
            output_file_name = os.path.join(path_output, f"{scenario.scenario_id}{ending}")
            if create_gif and create_gif_slideshow:
                display(NotebookImage(filename=output_file_name))
        finally:
            plt.switch_backend(original_backend)

    def plot_trajectory_within_scenario(self, scenario: Scenario, planning_problem_set, solution:Solution=None,
                                        solution_path=None):
        """
        Plots the trajectory of a dynamic obstacle (ego vehicle) within a specified scenario.

        Parameters:
            scenario (Scenario): The scenario in which the trajectory will be visualized.
            planning_problem_set: The set of planning problems associated with the scenario.
            solution (Solution, optional): The solution containing the trajectory to visualize.
            solution_path (str, optional): The file path of the solution to extract and visualize.

        Raises:
            ValueError: If neither `solution` nor `solution_path` is provided, or if both are provided.

        Notes:
            - At least one of `solution` or `solution_path` must be supplied to provide trajectory data.
            - The drawing parameters for various entities in the scenario are customized to render
              meaningful visualizations, focusing on the dynamic movement of the ego vehicle.
        """

        if solution is None and solution_path is None:
            raise ValueError("Either solution or solution_path must be provided")
        if solution is not None and solution_path is not None:
            raise ValueError("Only one of solution or solution_path can be provided")
        if solution_path is not None:
            solution = self.toolbox.extract_solution(solution_path)

        fig, ax = plt.subplots(figsize=(15, 5))
        fig.canvas.manager.set_window_title(str(scenario.scenario_id))
        renderer = MPRenderer(ax=ax)
        renderer.draw_params.dynamic_obstacle.draw_direction = False
        renderer.draw_params.planning_problem.initial_state.state.draw_arrow = False
        renderer.draw_params.dynamic_obstacle.trajectory.draw_trajectory = False
        renderer.draw_params.planning_problem.initial_state.state.radius = 0.0
        orange = "#FF8C00"
        orange_dark = "#995400"

        planning_problem = self.toolbox.extract_planning_problem(planning_problem_set)

        scenario.draw(renderer=renderer)
        planning_problem.draw(renderer=renderer)

        ego_vehicle = list(solution.create_dynamic_obstacle().values())[0]
        state_list = solution.planning_problem_solutions[0].trajectory.state_list
        ego_shape = ego_vehicle.obstacle_shape

        length = ego_shape.length
        width = ego_shape.width

        renderer.draw_params.time_begin = 0

        rect_params = ShapeParams()
        rect_params.facecolor = orange
        rect_params.edgecolor = orange_dark
        rect_params.opacity = 0.2
        rect_params.zorder = 20

        for state in state_list[1:]:
            rect = Rectangle(length=length, width=width,
                             center=state.position, orientation=state.orientation)

            rect.draw(renderer=renderer, draw_params=rect_params)

        ego_params = MPDrawParams()
        ego_params.dynamic_obstacle.draw_icon = True
        ego_params.dynamic_obstacle.draw_shape = False
        ego_params.dynamic_obstacle.show_label = False
        ego_params.dynamic_obstacle.trajectory.draw_trajectory = False

        ego_params.dynamic_obstacle.vehicle_shape.occupancy.shape.facecolor = orange
        ego_params.dynamic_obstacle.vehicle_shape.occupancy.shape.edgecolor = orange_dark

        ego_vehicle.draw(renderer=renderer, draw_params=ego_params)
        ax.axis("off")
        renderer.render()

        _show_figure(fig)
        plt.close(fig)


class Slideshow:
    """
    A class for rendering and displaying a slideshow of frames in a Tkinter-based GUI.

    The Slideshow class is designed to visualize specific frames of a scenario in a
    CommonRoad environment, along with its planning problem set. It provides a Tkinter-based
    GUI where users can navigate through frames using a slider, and the frame rendering
    is updated dynamically. This tool is particularly useful for visually analyzing
    scenarios and their planning problem sets in simulation environments.

    Attributes:
        end_at (int): The last frame number to be displayed in the slideshow.
        scenario: The scenario object that represents the target scenario to be visualized.
        planning_problem_set: The set of planning problems to be rendered alongside the scenario.
        root (tk.Tk): The root Tkinter window for the slideshow GUI.
        fig (Figure): The matplotlib figure used for rendering frames.
        ax: The axis of the matplotlib figure for drawing the scenario and planning problem set.
        canvas (FigureCanvasTkAgg): The canvas for embedding the matplotlib figure in the Tkinter GUI.
        canvas_widget: The Tkinter widget associated with the canvas.
        slider (ttk.Scale): A horizontal slider for navigating through the frames.
        label (ttk.Label): A label displaying the current frame number.

    Methods:
        on_slider_move(value: float) -> None:
            Handles slider movement events and updates the displayed frame.

        draw_frame(frame_number: int) -> None:
            Renders the given frame number on the matplotlib canvas using the renderer.

        on_closing(in_jupyter_notebook: bool = True) -> None:
            Handles the closing event of the slideshow window, ensuring proper resource cleanup.
    """
    def __init__(self, end_at, target_scenario, planning_problem_set, start_at=0, in_jupyter_notebook=True,
                 figsize=(15, 6)):
        self.end_at = end_at
        self.scenario = target_scenario
        self.planning_problem_set = planning_problem_set

        _load_desktop_tools()
        self.root = tk.Tk()
        self.root.title(f"CommonRoad Frame Slideshow: {target_scenario.scenario_id}")

        self.root.protocol("WM_DELETE_WINDOW", lambda: self.on_closing(in_jupyter_notebook))

        self.fig = Figure(figsize=figsize)
        self.ax = self.fig.add_subplot(111)

        self.canvas = FigureCanvasTkAgg(self.fig, master=self.root)
        self.canvas_widget = self.canvas.get_tk_widget()
        self.canvas_widget.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        self.slider = ttk.Scale(
            self.root,
            from_=start_at,
            to=self.end_at,
            orient='horizontal',
            command=self.on_slider_move
        )
        self.slider.pack(side=tk.BOTTOM, fill=tk.X, padx=15, pady=15)

        self.label = ttk.Label(self.root, text=f"Frame: {start_at}")
        self.label.pack(side=tk.BOTTOM, pady=2)

        self.draw_frame(0)
        self.root.mainloop()

    def on_slider_move(self, value):
        frame_number = int(float(value))
        self.label.config(text=f"Frame: {frame_number}")
        self.draw_frame(frame_number)

    def draw_frame(self, frame_number):
        self.ax.cla()

        rnd = MPRenderer(ax=self.ax)
        rnd.draw_params.time_begin = frame_number

        self.scenario.draw(renderer=rnd)
        self.planning_problem_set.draw(renderer=rnd)
        rnd.render(show=False)

        self.canvas.draw_idle()

    def on_closing(self, in_jupyter_notebook=True):
        plt.close(self.fig)
        self.root.quit()
        self.root.destroy()
        if not in_jupyter_notebook:
            sys.exit()


class SolutionSlideshow:
    """
    Manage and display a GUI slideshow of GIF frames with interactive features.

    This class provides a tkinter-based GUI application for viewing and interacting
    with individual frames of a GIF file. Frames can be navigated, zoomed, and
    selected for custom view adjustments. The application supports both mouse and
    slider interactions, and allows responsive frame rendering with zoom resets
    and scroll alignment.

    Attributes:
        gif_path (str): Path to the GIF file to be loaded.
        scenario (Scenario): Scenario object containing metadata for the slideshow.
        start_frame (int): The starting frame index to load from the GIF.
        end_frame (int or None): The ending frame index to stop loading, or None to load to the end.
        step (int): The step size for loading frames, allowing skipping of frames.
        in_jupyter_notebook (bool): Whether the application is running inside a Jupyter Notebook.
    """
    def __init__(self, gif_path, scenario, start_frame=0, end_frame=None, step=1, in_jupyter_notebook=True):
        self.gif_path = gif_path
        self.start_frame = start_frame
        self.end_frame = end_frame
        self.step = step

        self.zoom_factor = 1.0

        self.raw_frames = []
        self.actual_frame_numbers = []
        self.current_tk_image = None
        self.canvas_image_id = None

        self.rect = None
        self.start_x = 0
        self.start_y = 0

        _load_desktop_tools()
        self.root = tk.Tk()
        self.root.title(f"Solution Slideshow: {scenario.scenario_id}")

        self.root.geometry("800x600")

        self.root.resizable(False, False)

        self.root.protocol("WM_DELETE_WINDOW", lambda: self.on_closing(in_jupyter_notebook))

        print(f"Loading frames {start_frame} to {end_frame or 'End'} (Step: {step})...")
        self.load_frames()

        if not self.raw_frames:
            print("Error: Could not load any frames.")
            self.root.destroy()
            return

        self.slider_max = len(self.raw_frames) - 1

        self.image_frame = tk.Frame(self.root)
        self.image_frame.pack(side=tk.TOP, fill=tk.BOTH, expand=True, padx=10, pady=10)

        self.h_scroll = ttk.Scrollbar(self.image_frame, orient=tk.HORIZONTAL)
        self.h_scroll.pack(side=tk.BOTTOM, fill=tk.X)

        self.canvas = tk.Canvas(
            self.image_frame,
            xscrollcommand=self.h_scroll.set,
            bg="white",
            cursor="crosshair"
        )
        self.canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        self.h_scroll.config(command=self.canvas.xview)

        self.canvas.bind("<ButtonPress-1>", self.on_mouse_down)
        self.canvas.bind("<B1-Motion>", self.on_mouse_drag)
        self.canvas.bind("<ButtonRelease-1>", self.on_mouse_up)

        self.canvas.bind("<MouseWheel>", self.on_mouse_wheel)  # Windows / macOS
        self.canvas.bind("<Button-4>", self.on_mouse_wheel)  # Linux scroll up
        self.canvas.bind("<Button-5>", self.on_mouse_wheel)  # Linux scroll down

        self.controls_frame = tk.Frame(self.root)
        self.controls_frame.pack(side=tk.BOTTOM, fill=tk.X, pady=5)

        self.btn_zoom_out = ttk.Button(self.controls_frame, text="Zoom Out (-)", command=self.zoom_out)
        self.btn_zoom_out.pack(side=tk.LEFT, padx=10)

        self.zoom_label = ttk.Label(self.controls_frame, text="Zoom: 100%")
        self.zoom_label.pack(side=tk.LEFT, padx=10)

        self.btn_zoom_in = ttk.Button(self.controls_frame, text="Zoom In (+)", command=self.zoom_in)
        self.btn_zoom_in.pack(side=tk.LEFT, padx=10)

        self.btn_reset = ttk.Button(self.controls_frame, text="Reset", command=self.reset_zoom)
        self.btn_reset.pack(side=tk.LEFT, padx=10)

        self.slider = ttk.Scale(
            self.root, from_=0, to=self.slider_max, orient='horizontal', command=self.on_slider_move
        )
        self.slider.pack(side=tk.BOTTOM, fill=tk.X, padx=15, pady=5)

        self.frame_label = ttk.Label(self.root, text=f"Frame: {self.actual_frame_numbers[0]}")
        self.frame_label.pack(side=tk.BOTTOM, pady=2)

        self.root.update_idletasks()
        self.draw_frame(0)
        self.center_scrollbars()

        self.root.mainloop()

    def load_frames(self):
        try:
            gif_image = Image.open(self.gif_path)
        except Exception as e:
            print(f"Error opening GIF: {e}")
            return

        current_index = self.start_frame
        while True:
            if self.end_frame is not None and current_index > self.end_frame:
                break
            try:
                gif_image.seek(current_index)
                frame_snapshot = gif_image.copy().convert('RGBA')
                self.raw_frames.append(frame_snapshot)
                self.actual_frame_numbers.append(current_index)
                current_index += self.step
            except EOFError:
                break
        print(f"Successfully loaded {len(self.raw_frames)} frames.")

    def on_mouse_wheel(self, event):
        if event.num == 4:
            self.canvas.yview_scroll(-1, "units")
        elif event.num == 5:
            self.canvas.yview_scroll(1, "units")
        else:
            delta = -1 if event.delta > 0 else 1
            self.canvas.yview_scroll(delta, "units")

    def on_mouse_down(self, event):
        self.start_x = self.canvas.canvasx(event.x)
        self.start_y = self.canvas.canvasy(event.y)

        if self.rect:
            self.canvas.delete(self.rect)
        self.rect = self.canvas.create_rectangle(
            self.start_x, self.start_y, self.start_x, self.start_y,
            outline='red', width=2, dash=(4, 4)
        )

    def on_mouse_drag(self, event):
        cur_x = self.canvas.canvasx(event.x)
        cur_y = self.canvas.canvasy(event.y)
        if self.rect:
            self.canvas.coords(self.rect, self.start_x, self.start_y, cur_x, cur_y)

    def on_mouse_up(self, event):
        end_x = self.canvas.canvasx(event.x)
        end_y = self.canvas.canvasy(event.y)

        if abs(end_x - self.start_x) < 10 or abs(end_y - self.start_y) < 10:
            if self.rect:
                self.canvas.delete(self.rect)
                self.rect = None
            return

        x0, x1 = sorted([self.start_x, end_x])
        y0, y1 = sorted([self.start_y, end_y])

        orig_x0 = x0 / self.zoom_factor
        orig_y0 = y0 / self.zoom_factor
        orig_w = (x1 - x0) / self.zoom_factor
        orig_h = (y1 - y0) / self.zoom_factor

        cw = self.canvas.winfo_width()
        ch = self.canvas.winfo_height()

        if orig_w > 0 and orig_h > 0:
            zoom_x = cw / orig_w
            zoom_y = ch / orig_h
            self.zoom_factor = min(zoom_x, zoom_y)

        self.update_zoom_label()

        current_slider_index = int(float(self.slider.get()))
        self.draw_frame(current_slider_index)

        self.canvas.update_idletasks()

        img = self.raw_frames[current_slider_index]
        new_total_width = img.width * self.zoom_factor
        new_total_height = img.height * self.zoom_factor

        scroll_x = (orig_x0 * self.zoom_factor) / new_total_width
        scroll_y = (orig_y0 * self.zoom_factor) / new_total_height

        self.canvas.xview_moveto(scroll_x)
        self.canvas.yview_moveto(scroll_y)

        if self.rect:
            self.canvas.delete(self.rect)
            self.rect = None

    def zoom_in(self):
        self.zoom_factor *= 1.25
        self.update_zoom_label()
        self.draw_frame(int(float(self.slider.get())))

    def zoom_out(self):
        self.zoom_factor /= 1.25
        self.update_zoom_label()
        self.draw_frame(int(float(self.slider.get())))

    def reset_zoom(self):
        self.zoom_factor = 1.0
        self.update_zoom_label()
        self.draw_frame(int(float(self.slider.get())))
        self.center_scrollbars()

    def update_zoom_label(self):
        zoom_percent = int(self.zoom_factor * 100)
        self.zoom_label.config(text=f"Zoom: {zoom_percent}%")

    def on_slider_move(self, value):
        slider_index = int(float(value))
        real_frame = self.actual_frame_numbers[slider_index]
        self.frame_label.config(text=f"Frame: {real_frame}")
        self.draw_frame(slider_index)

    def draw_frame(self, slider_index):
        raw_img = self.raw_frames[slider_index]
        new_width = max(1, int(raw_img.width * self.zoom_factor))
        new_height = max(1, int(raw_img.height * self.zoom_factor))

        resized_img = raw_img.resize((new_width, new_height),
                                     Image.Resampling.NEAREST if hasattr(Image, 'Resampling') else Image.NEAREST)
        self.current_tk_image = ImageTk.PhotoImage(resized_img, master=self.root)

        canvas_w = self.canvas.winfo_width()
        canvas_h = self.canvas.winfo_height()

        x_pos = max(0, (canvas_w - new_width) // 2) if canvas_w > 1 else 0
        y_pos = max(0, (canvas_h - new_height) // 2) if canvas_h > 1 else 0

        if self.canvas_image_id is None:
            self.canvas_image_id = self.canvas.create_image(x_pos, y_pos, anchor=tk.NW, image=self.current_tk_image)
        else:
            self.canvas.coords(self.canvas_image_id, x_pos, y_pos)
            self.canvas.itemconfig(self.canvas_image_id, image=self.current_tk_image)

        self.canvas.config(scrollregion=self.canvas.bbox(tk.ALL))

    def center_scrollbars(self):
        self.root.update_idletasks()

        canvas_w = self.canvas.winfo_width()
        canvas_h = self.canvas.winfo_height()

        slider_index = int(float(self.slider.get()))
        raw_img = self.raw_frames[slider_index]
        img_w = raw_img.width * self.zoom_factor
        img_h = raw_img.height * self.zoom_factor

        if img_w > canvas_w:
            x_fraction = ((img_w - canvas_w) / 2) / img_w
            self.canvas.xview_moveto(x_fraction)

        if img_h > canvas_h:
            y_fraction = ((img_h - canvas_h) / 2) / img_h
            self.canvas.yview_moveto(y_fraction)

    def on_closing(self, in_jupyter_notebook=True):
        self.root.quit()
        self.root.destroy()
        if not in_jupyter_notebook:
            sys.exit()
