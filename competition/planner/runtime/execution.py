"""Execute planners, build Solutions, and validate and score interactive drives."""

import math
import time
from copy import deepcopy
from dataclasses import dataclass, replace
from pathlib import Path

from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.common.solution import (
    CostFunction,
    PlanningProblemSolution,
    Solution,
    VehicleModel,
    VehicleType,
)
from commonroad.planning.planning_problem import PlanningProblemSet
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.trajectory import Trajectory
from commonroad_sumo.utils import convert_state_to_state_type

from simulation.planner_adapter import PlannerAdapter
from simulation.simulations import SimulationSettings, run_simulation
from simulation.utility import make_solution

from ..common.scenarios import BENCHMARK_RANDOM_SEED, BENCHMARK_SIMULATION_STEPS
from ..common.scoring import CostFunctionEvaluator
from ..common.validation import SolutionValidation, validate_solution
from ..spec import PlannerSpec
from .artifacts import run_metadata, submission_metadata
from .submission import DEFAULT_SUBMISSION, load_submission


@dataclass
class RunResult:
    scenario: Scenario
    planning_problems: PlanningProblemSet
    solution: Solution
    validation: SolutionValidation
    cost: float | None
    diagnostics: list[dict[str, object]]
    elapsed_seconds: float
    metadata: dict[str, object]
    scoring_error: str | None = None

    @property
    def valid(self) -> bool:
        return self.validation.valid


def run_spec(scenario_path: str | Path, spec: PlannerSpec) -> RunResult:
    """Run an in-memory planner with its settings for notebook experiments and debugging."""
    spec.validate()
    metadata = run_metadata(scenario_path, spec)
    started = time.monotonic()
    scenario, problems = CommonRoadFileReader(str(scenario_path)).open()
    if getattr(spec.planner, "reset", None) is not None:
        spec.planner.reset()
    simulated, solution = run_motion_planner_interactive(
        spec.planner,
        scenario,
        problems,
        settings=spec.settings,
        vehicle_model=spec.vehicle_model,
        vehicle_type=spec.vehicle_type,
        cost_function=spec.cost_function,
    )
    validation = validate_solution(simulated, problems, solution)
    cost = None
    scoring_error = None
    if validation.valid:
        try:
            cost = float(
                CostFunctionEvaluator.init_from_solution(solution)
                .evaluate_solution(simulated, problems, solution)
                .total_costs
            )
            if not math.isfinite(cost):
                raise ValueError(f"Scorer returned a non-finite cost: {cost}")
        except Exception as error:
            # The completed ego/traffic pair remains useful for independent checks.
            cost = None
            scoring_error = f"{type(error).__name__}: {error}"
    return RunResult(
        simulated,
        problems,
        solution,
        validation,
        cost,
        list(getattr(spec.planner, "diagnostics", [])),
        time.monotonic() - started,
        metadata,
        scoring_error,
    )


def run_submission(scenario_path: str | Path, submission_directory: str | Path) -> RunResult:
    """Load a submission and evaluate with SM1, seed 1234, and a 200-step allowance."""
    source = submission_metadata(submission_directory)
    with load_submission(submission_directory) as spec:
        spec.cost_function = CostFunction.SM1
        spec.settings = replace(
            spec.settings,
            simulation_steps=BENCHMARK_SIMULATION_STEPS,
            random_seed=BENCHMARK_RANDOM_SEED,
        )
        result = run_spec(scenario_path, spec)
        result.metadata["source"] = source
        return result


def run_motion_planner_non_interactive(
    motion_planner,
    scenario: Scenario,
    planning_problem_set: PlanningProblemSet,
    *,
    vehicle_model=VehicleModel.KS,
    vehicle_type=VehicleType.FORD_ESCORT,
    cost_function=CostFunction.SM1,
) -> Solution:
    # Recorded future traffic is available for the non-interactive task.
    planner = PlannerAdapter(motion_planner, interactive=False)
    solutions = []
    for problem in planning_problem_set.planning_problem_dict.values():
        future = planner.plan(scenario, problem)
        initial = deepcopy(
            convert_state_to_state_type(problem.initial_state, type(future.state_list[0]))
        )
        trajectory = Trajectory(initial.time_step, [initial, *future.state_list])
        solutions.append(
            PlanningProblemSolution(
                problem.planning_problem_id, vehicle_model, vehicle_type, cost_function, trajectory
            )
        )
    return Solution(scenario.scenario_id, solutions)


def run_motion_planner_interactive(
    motion_planner,
    scenario: Scenario,
    planning_problem_set: PlanningProblemSet,
    *,
    settings: SimulationSettings | None = None,
    num_of_steps: int | None = None,
    vehicle_model=VehicleModel.KS,
    vehicle_type=VehicleType.FORD_ESCORT,
    cost_function=CostFunction.SM1,
) -> tuple[Scenario, Solution]:
    """Return simulated traffic and the driven Solution; preserve exception causes."""
    result = run_simulation(
        scenario, planning_problem_set, motion_planner, settings=settings, num_of_steps=num_of_steps
    )
    solution = make_solution(
        result.scenario,
        planning_problem_set,
        result.ego_vehicles,
        vehicle_type,
        vehicle_model,
        cost_function,
    )
    return result.scenario, solution


def motion_planner(
    scenario: Scenario, solution_dir, planning_problem_set: PlanningProblemSet
) -> Solution:
    """Run the student factory's planner against recorded traffic."""
    with load_submission(DEFAULT_SUBMISSION) as spec:
        spec.validate()
        return run_motion_planner_non_interactive(
            spec.planner,
            scenario,
            planning_problem_set,
            vehicle_model=spec.vehicle_model,
            vehicle_type=spec.vehicle_type,
            cost_function=spec.cost_function,
        )
