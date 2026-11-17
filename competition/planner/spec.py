"""A planner and the execution settings needed to drive and export its motion."""

from dataclasses import dataclass, field

from commonroad.common.solution import CostFunction, VehicleModel, VehicleType
from commonroad_dc.feasibility.vehicle_dynamics import VehicleParameterMapping

from simulation.simulations import SimulationSettings

from .algorithms.reactive import ReactiveConfig, ReactivePlannerAdapter
from .algorithms.smp import SMPConfig, SMPPlanner
from .algorithms.smp_automaton import PRIMITIVE_STEPS
from .common.scenarios import BENCHMARK_SIMULATION_STEPS


@dataclass
class PlannerSpec:
    """A planner with its simulation settings and solution vehicle metadata."""

    planner: object  # An object with plan(scenario, planning_problem) -> Trajectory
    # SUMO duration, replanning interval, seed, and ego dimensions
    settings: SimulationSettings = field(default_factory=SimulationSettings)
    # Vehicle metadata for solution export and feasibility checks
    vehicle_model: VehicleModel = VehicleModel.KS
    vehicle_type: VehicleType = VehicleType.FORD_ESCORT
    # Match the search exercise's final score; planners choose their own search costs.
    cost_function: CostFunction = CostFunction.SM1

    def validate(self) -> None:
        """Ensure built-in adapters and simulation agree on the vehicle and execution interval."""
        if isinstance(self.planner, (SMPPlanner, ReactivePlannerAdapter)):
            vehicle = VehicleParameterMapping.from_vehicle_type(VehicleType.BMW_320i)
            interval = (
                PRIMITIVE_STEPS
                if isinstance(self.planner, SMPPlanner)
                else self.planner.config.replanning_interval
            )
            if (
                self.vehicle_model != VehicleModel.KS
                or self.vehicle_type != VehicleType.BMW_320i
                or self.settings.replanning_interval != interval
                or abs(self.settings.ego_veh_length - vehicle.l) > 1e-6
                or abs(self.settings.ego_veh_width - vehicle.w) > 1e-6
            ):
                raise ValueError(
                    f"This planner requires KS/BMW_320i, dimensions {vehicle.l:g} x {vehicle.w:g} m, "
                    f"and interval {interval}"
                )


def smp_spec(
    planner_class: (
        type | str
    ) = "SMP.motion_planner.search_algorithms.best_first_search:AStarSearch",
    *,
    config: SMPConfig | None = None,
) -> PlannerSpec:
    """Build the standard SMP profile; choose a student class without editing it."""
    vehicle_type = VehicleType.BMW_320i
    return PlannerSpec(
        SMPPlanner(planner_class, config),
        settings=SimulationSettings.for_vehicle(
            vehicle_type,
            simulation_steps=BENCHMARK_SIMULATION_STEPS,
            replanning_interval=PRIMITIVE_STEPS,
        ),
        vehicle_type=vehicle_type,
    )


def reactive_spec(*, config: ReactiveConfig | None = None) -> PlannerSpec:
    """Use the copied reactive core through the same runner and validation as SMP."""
    planner = ReactivePlannerAdapter(config)
    vehicle_type = VehicleType.BMW_320i
    return PlannerSpec(
        planner,
        settings=SimulationSettings.for_vehicle(
            vehicle_type,
            simulation_steps=BENCHMARK_SIMULATION_STEPS,
            replanning_interval=planner.config.replanning_interval,
        ),
        vehicle_type=vehicle_type,
    )
