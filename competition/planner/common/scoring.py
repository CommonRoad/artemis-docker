"""Upstream cost functions with CLCS projection-exception compatibility."""

from commonroad_dc.costs.evaluation import (
    CostFunctionEvaluator as UpstreamCostFunctionEvaluator,
    PartialCostFunctionMapping,
    PlanningProblemCostResult,
)
from commonroad.planning.planning_problem import PlanningProblem
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.trajectory import Trajectory

from .vendor.route_matcher import LaneletRouteMatcher


class CostFunctionEvaluator(UpstreamCostFunctionEvaluator):
    """Keep upstream scores while allowing its intended projection fallback paths."""

    def evaluate_pp_solution(
        self,
        cr_scenario: Scenario,
        cr_pproblem: PlanningProblem,
        trajectory: Trajectory,
        draw_lanelet_path: bool = False,
        debug_plot: bool = False,
    ) -> PlanningProblemCostResult:
        result = PlanningProblemCostResult(self.cost_function_id, cr_pproblem.planning_problem_id)
        matcher = LaneletRouteMatcher(cr_scenario, self.vehicle_type)
        transformed, _, properties = matcher.compute_curvilinear_coordinates(
            trajectory,
            required_properties=self.required_properties,
            draw_lanelet_path=draw_lanelet_path,
            debug_plot=debug_plot,
        )
        for partial, weight in self.partial_cost_funcs:
            cost = PartialCostFunctionMapping[partial](
                cr_scenario, cr_pproblem, transformed, properties
            )
            result.add_partial_costs(partial, cost, weight)
        return result
