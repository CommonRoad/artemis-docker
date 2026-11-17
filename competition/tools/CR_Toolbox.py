
from pathlib import Path

import numpy as np

from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.common.file_writer import CommonRoadFileWriter, OverwriteExistingFile
from commonroad.common.solution import CommonRoadSolutionReader, CommonRoadSolutionWriter, Solution
from commonroad.geometry.shape import Circle, Rectangle
from commonroad.scenario.obstacle import ObstacleType, StaticObstacle
from commonroad.scenario.state import InitialState

from planner.common.scoring import CostFunctionEvaluator

class CRToolbox:
    """
    Scenario I/O and saved-solution inspection helpers.

    Validate with simulation.solution_check.valid_solution() and export interactive
    runs with planner.runtime.artifacts.export_result().
    """
    def __init__(self, root=None):
        self.path_root = str(Path(root).resolve() if root is not None else Path(__file__).resolve().parents[1])

    def _path(self, path):
        return str(Path(self.path_root) / path)

    def extract_scenario(self, path_benchmark):
        """
        Extracts a scenario and planning problem set from a benchmark file.

        This method combines the root path with the provided benchmark path to
        locate the specific benchmark file. It then parses the file to extract
        the scenario and the planning problem set.

        Parameters:
        path_benchmark : str
            Relative path to the benchmark file from the root directory.

        Returns:
        tuple
            A tuple containing the scenario and the planning problem set.
        """

        benchmark_path = self._path(path_benchmark)
        scenario, planning_problem_set = CommonRoadFileReader(benchmark_path).open()
        return scenario, planning_problem_set

    @staticmethod
    def extract_planning_problem(planning_problem_set):
        """
        Extracts the first planning problem from a given planning problem set.

        This method retrieves the first planning problem from a set of planning problems
        by accessing the dictionary of planning problems associated with the set.

        Args:
            planning_problem_set: The set of planning problems containing a dictionary
                of planning problems.

        Returns:
            The first planning problem from the set.

        """
        return list(planning_problem_set.planning_problem_dict.values())[0]

    def extract_solution(self, path_solution):
        """
        Extracts a solution from the specified file path.

        This method constructs the absolute path to a solution file and uses the
        CommonRoadSolutionReader to open the file and extract its solution.

        Parameters:
        path_solution: str
            Relative path to the solution file from the root path.

        Returns:
        CommonRoadSolution
            An object representing the solution extracted from the file.
        """
        solution_path = self._path(path_solution)
        return CommonRoadSolutionReader.open(solution_path)

    @staticmethod
    def save_non_interactive_solution(solution: Solution, path: str):
        """
        Saves the non-interactive solution to a specified file.

        This method writes the solution to the provided file path as a CommonRoad solution.


        Args:
            solution (Solution): The solution object to be saved.
            path (str): The file path where the solution will be saved.

        Returns:
            Any: The result of the file writing operation.
        """
        to_be_returned = CommonRoadSolutionWriter(solution).write_to_file(
            output_path=path, overwrite=True, pretty=True
        )
        print("Wrote the non-interactive solution successfully")
        return to_be_returned

    def evaluate_cost(self, path_benchmark, path_solution):
        """Print and return a saved solution's cost; read and scoring errors propagate."""
        scenario, planning_problem_set = self.extract_scenario(path_benchmark)
        solution = self.extract_solution(path_solution)
        cost_evaluator = CostFunctionEvaluator.init_from_solution(solution)
        cost_result = cost_evaluator.evaluate_solution(scenario, planning_problem_set, solution)
        print(cost_result)
        return cost_result

    def add_static_rectangle_obstacle_to_benchmark(self, path_benchmark, obstacle_type: ObstacleType,
                                                   width, length, pos_x, pos_y,
                                                   orientation):
        """
        Adds a static rectangular obstacle to a benchmark scenario.

        This method modifies the scenario specified in the given benchmark by adding
        a static obstacle with a rectangular shape. It initializes the obstacle with
        the given dimensions, position, and orientation, and adds it to the scenario.

        Arguments:
            path_benchmark: str
                Path to the benchmark scenario.
            obstacle_type: ObstacleType
                Type of the static obstacle being added.
            width: float
                Width of the rectangular obstacle.
            length: float
                Length of the rectangular obstacle.
            pos_x: float
                X-coordinate of the obstacle's position.
            pos_y: float
                Y-coordinate of the obstacle's position.
            orientation: float
                Orientation of the obstacle in radians.

        Returns:
            tuple
                A tuple containing the modified scenario and the planning problem set.
        """
        scenario, planning_problem_set = self.extract_scenario(path_benchmark)
        static_obstacle_id = scenario.generate_object_id()
        static_obstacle_type = obstacle_type
        static_obstacle_shape = Rectangle(width=width, length=length)

        static_obstacle_initial_state = InitialState(position=np.array([pos_x, pos_y]), orientation=orientation,
                                                     time_step=0)
        static_obstacle = StaticObstacle(static_obstacle_id, static_obstacle_type, static_obstacle_shape,
                                         static_obstacle_initial_state)
        scenario.add_objects(static_obstacle)
        return scenario, planning_problem_set

    def add_static_circle_obstacle_to_benchmark(self, path_benchmark, obstacle_type: ObstacleType, radius, center_x,
                                                center_y,
                                                pos_x, pos_y, orientation):
        """
        Adds a static circular obstacle to the given benchmark scenario.

        This function extracts the scenario and planning problem set from the benchmark data,
        generates a unique ID for the static obstacle, and defines its type, shape, and initial
        state using the provided parameters. The newly created static obstacle is then added
        to the scenario, and the updated scenario and planning problem set are returned.

        Arguments:
            path_benchmark: The path to the benchmark file. This contains the scenario
                and planning problem set data.
            obstacle_type: The type of the obstacle represented as an ObstacleType value.
            radius: The radius of the circular obstacle.
            center_x: The X-coordinate of the circle's center in its local frame.
            center_y: The Y-coordinate of the circle's center in its local frame.
            pos_x: The X-coordinate of the obstacle's position in the world frame.
            pos_y: The Y-coordinate of the obstacle's position in the world frame.
            orientation: The orientation angle of the obstacle in radians.

        Returns:
            A tuple consisting of:
                scenario: The updated scenario object containing the newly added static obstacle.
                planning_problem_set: The planning problem set extracted from the
                benchmark data.
        """
        scenario, planning_problem_set = self.extract_scenario(path_benchmark)
        static_obstacle_id = scenario.generate_object_id()
        static_obstacle_type = obstacle_type
        static_obstacle_shape = Circle(radius=radius, center=np.array([center_x, center_y]))
        static_obstacle_initial_state = InitialState(position=np.array([pos_x, pos_y]), orientation=orientation,
                                                     time_step=0)
        static_obstacle = StaticObstacle(static_obstacle_id, static_obstacle_type, static_obstacle_shape,
                                         static_obstacle_initial_state)
        scenario.add_objects(static_obstacle)
        return scenario, planning_problem_set

    def write_to_benchmark(self, scenario, planning_problem_set, new_file_name, author=None, affiliation=None, source=None, tags=None):
        """
        Writes the given scenario and planning problems to a CommonRoad benchmark file.

        This function generates a CommonRoad file at the specified location using the provided
        scenario and planning problem data. Additional metadata such as author, affiliation,
        source, and tags may also be included in the benchmark file.

        Arguments:
            scenario: The scenario to be written to the benchmark file.
            planning_problem_set: The set of planning problems to be associated with the
                scenario in the file.
            new_file_name: The name of the file to be created.
            author: Optional; the author of the benchmark file.
            affiliation: Optional; the author's affiliation.
            source: Optional; the source or origin of the scenario or planning problems.
            tags: Optional; a list of tags to categorize or describe the benchmark file.

        Returns:
            None
        """

        new_path = self._path(new_file_name)
        fw = CommonRoadFileWriter(scenario, planning_problem_set, author, affiliation, source, tags)
        fw.write_to_file(new_path, OverwriteExistingFile.ALWAYS)
