"""Choose one approach below; leave exactly one create_planner() uncommented.

To switch approaches, comment out the active option's imports and factory,
then uncomment the option you want. Submit this file with any helper modules.
"""

# Option 1: Search planner (default). Replace AStarSearch with your exercise class.
from SMP.motion_planner.search_algorithms.best_first_search import AStarSearch
from planner.spec import PlannerSpec, smp_spec


def create_planner() -> PlannerSpec:
    return smp_spec(AStarSearch)


# Option 2: Reactive planner.
# from planner.spec import PlannerSpec, reactive_spec
#
#
# def create_planner() -> PlannerSpec:
#     return reactive_spec()


# Option 3: Your own planner. Implement StudentPlanner.plan() in custom_planner.py.
# from commonroad.common.solution import VehicleType
# from planner.spec import PlannerSpec
# from simulation.simulations import SimulationSettings
# from custom_planner import StudentPlanner
#
#
# def create_planner() -> PlannerSpec:
#     vehicle_type = VehicleType.BMW_320i
#     return PlannerSpec(
#         StudentPlanner(),
#         vehicle_type=vehicle_type,
#         settings=SimulationSettings.for_vehicle(
#             vehicle_type,
#             simulation_steps=200,
#             # The dummy plan returns five future states per call.
#             replanning_interval=5,
#         ),
#     )
