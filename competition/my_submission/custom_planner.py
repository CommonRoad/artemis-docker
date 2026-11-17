"""Custom planner template with a route-following example and optional helpers."""

import numpy as np
from commonroad.common.solution import VehicleType
from commonroad.planning.planning_problem import PlanningProblem
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.state import KSState
from commonroad.scenario.trajectory import Trajectory
from commonroad_route_planner.reference_path_planner import ReferencePathPlanner
from commonroad_route_planner.route_planner import RoutePlanner
from commonroad_dc.feasibility.vehicle_dynamics import VehicleParameterMapping

from planner.common.prediction import predict_observations


class StudentPlanner:
    """Here is a motion planner template. Only plan() is required.

    The other helpers are examples: reuse, modify, or replace them with your own
    approaches. They may also live in separate modules or classes.
    """

    def __init__(self, horizon_steps: int = 5) -> None:
        """Set the number of future states; the submission example executes five."""
        if horizon_steps < 1:
            raise ValueError("horizon_steps must be positive")
        self.horizon_steps = horizon_steps

    def plan(self, scenario: Scenario, problem: PlanningProblem) -> Trajectory:
        """
        Motion planning is to generate a trajectory for the ego vehicle to execute over
        a future planning horizon.

        A trajectory is a sequence of vehicle states indexed by discrete time steps.
        We use the KS (kinematic single-track) model. Each returned KSState contains:

        - position: vehicle-center coordinates [x, y] in the scenario's map frame, in meters.
        - orientation: heading in radians, measured counterclockwise from the positive x-axis.
        - velocity: longitudinal speed in meters per second.
        - steering_angle: front-wheel steering angle in radians.

        Consecutive states are scenario.dt seconds apart. The current ego state is
        problem.initial_state; its time_step is the current simulation step.

        Return vehicle-center states with consecutive absolute time steps,
        starting at the current step or the next step and including future motion.
        Recorded future obstacle trajectories are unavailable in interactive runs.

        The example below samples a route at the initial speed. It preserves the
        initial position offset from that route and stops at its endpoint. Its
        steering estimate does not make sharp turns or stops dynamically feasible.
        """
        # DUMMY EXAMPLE: advance along the route at the current speed.
        # This is geometric sampling, not a dynamically feasible controller. It
        # does not react to obstacles or check road boundaries or the goal.
        state = problem.initial_state
        route = np.asarray(self.plan_route(scenario, problem), dtype=float)
        if route.ndim != 2 or route.shape[1] != 2 or not np.all(np.isfinite(route)):
            raise ValueError("The reference path must contain finite [x, y] points")

        # Drop repeated points so each segment has a usable direction.
        vectors = np.diff(route, axis=0)
        lengths = np.linalg.norm(vectors, axis=1)
        keep = lengths > 1e-9
        starts, vectors, lengths = route[:-1][keep], vectors[keep], lengths[keep]
        if len(lengths) == 0:
            raise ValueError("The reference path needs at least one nonzero segment")
        distances = np.r_[0.0, np.cumsum(lengths)]

        # Find the closest point on the route to the current vehicle center.
        fractions = np.clip(
            np.sum((state.position - starts) * vectors, axis=1) / lengths**2, 0.0, 1.0
        )
        projections = starts + fractions[:, None] * vectors
        nearest = int(np.argmin(np.linalg.norm(projections - state.position, axis=1)))
        progress = distances[nearest] + fractions[nearest] * lengths[nearest]
        start_offset = state.position - projections[nearest]

        # The translated sample path begins at the actual vehicle center. This
        # avoids an instantaneous jump if that center is slightly off the route.
        # It may leave the road if the initial offset or route curvature is large.
        wheelbase = VehicleParameterMapping.from_vehicle_type(VehicleType.BMW_320i)
        wheelbase = wheelbase.a + wheelbase.b  # Match the example in student_planner.py.
        previous_heading = state.orientation
        previous_progress = progress
        states = []
        for step in range(1, self.horizon_steps + 1):
            position_along_route = min(
                progress + step * scenario.dt * max(state.velocity, 0.0), distances[-1]
            )
            segment = min(
                int(np.searchsorted(distances, position_along_route, side="right")) - 1,
                len(lengths) - 1,
            )
            fraction = (position_along_route - distances[segment]) / lengths[segment]
            position = starts[segment] + fraction * vectors[segment] + start_offset
            heading = (
                float(np.arctan2(vectors[segment, 1], vectors[segment, 0]))
                if position_along_route > previous_progress + 1e-9
                else previous_heading
            )
            moving = state.velocity > 0.0 and position_along_route < distances[-1]
            heading_change = np.arctan2(
                np.sin(heading - previous_heading), np.cos(heading - previous_heading)
            )
            steering = (
                float(np.arctan(wheelbase * heading_change / (state.velocity * scenario.dt)))
                if moving
                else 0.0
            )
            states.append(
                KSState(
                    position=position,
                    orientation=heading,
                    velocity=state.velocity if moving else 0.0,
                    steering_angle=steering,
                    time_step=state.time_step + step,
                )
            )
            previous_heading = heading
            previous_progress = position_along_route
        return Trajectory(states[0].time_step, states)

    def plan_route(self, scenario: Scenario, problem: PlanningProblem) -> np.ndarray:
        """
        A motion planner usually needs a positional reference path to track, which is an (N, 2) array of map coordinates.

        Here we provide an example using the CommonRoad Route Planner.
        You can replace this with your own route-planning approach.
        """
        # Find lanelet routes from the initial state toward the goal.
        routes = RoutePlanner(scenario.lanelet_network, problem).plan_routes()

        # Convert the lanelet routes into reference-path geometry.
        reference_planner = ReferencePathPlanner(
            scenario.lanelet_network,
            problem,
            routes,
        )
        return reference_planner.plan_shortest_reference_path().reference_path

    def predict_obstacles(self, scenario: Scenario, current: int, horizon: int) -> Scenario:
        """Example lane-following predictor; you can use your own approach.

        Keeps current longitudinal speed and lateral offset along the lane.
        Uses straight-line prediction if lane matching or projection fails.

        current: absolute simulation step.
        horizon: number of future steps to predict.

        The returned scenario copy preserves the map and static obstacles.
        Actor times are relative: 0 is now, followed by steps 1 through horizon.
        Align ego times with this origin for collision checks.
        """
        return predict_observations(scenario, current, horizon)

    def is_trajectory_valid(
        self, trajectory: Trajectory, prediction: Scenario, problem: PlanningProblem
    ) -> bool:
        """Suggested: check dynamics, road boundaries, and collisions with predicted traffic.

        Include static obstacles and use the same time origin for ego and obstacle states.
        """
        pass
