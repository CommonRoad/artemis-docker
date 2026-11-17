"""Build current-observation predictions and planning problems at relative time zero."""

from copy import deepcopy
from math import ceil

import numpy as np
from commonroad.common.util import Interval
from commonroad.planning.planning_problem import PlanningProblem
from commonroad.prediction.prediction import TrajectoryPrediction
from commonroad.scenario.lanelet import LaneletNetwork
from commonroad.scenario.obstacle import DynamicObstacle
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.state import InitialState, CustomState, State
from commonroad.scenario.trajectory import Trajectory
from commonroad_clcs.pycrccosy import (
    CartesianProjectionDomainError,
    CurvilinearCoordinateSystem,
    CurvilinearProjectionDomainLateralError,
    CurvilinearProjectionDomainLongitudinalError,
)
from commonroad_clcs.util import resample_polyline

from .errors import IncompatibleState, NoPathFound
from ..algorithms.smp_automaton import PRIMITIVE_STEPS


def search_horizon(problem: PlanningProblem) -> int:
    """Cover the last complete primitive, even if the goal ends partway through it."""
    if len(problem.goal.state_list) != 1:
        raise IncompatibleState("The SMP starter supports one goal region")
    deadline = problem.goal.state_list[0].time_step.end
    current = problem.initial_state.time_step
    if not np.isfinite(deadline):
        raise IncompatibleState("SMP needs a finite goal deadline")
    remaining = deadline - current
    if remaining <= 0:
        raise NoPathFound("The original goal deadline has expired")
    return int(ceil(remaining / PRIMITIVE_STEPS) * PRIMITIVE_STEPS)


def predict_observations(scenario: Scenario, current: int, horizon: int) -> Scenario:
    """Keep the map/static obstacles and predict only actors observed at `current`.

    Follow an aligned lane centerline at the current longitudinal speed, keeping
    the observed lateral offset. Follow unique successors; at forks or lane ends,
    extend the last tangent. Fall back to constant speed and heading if lane
    matching or Frenet projection fails. Stationary actors remain exactly still.

    The planner sees time zero as now. Neither recorded future states nor the
    simulator's routes are consulted. An actor without a current state has left
    the observation and is not extrapolated from stale history.
    """
    predicted = deepcopy(scenario)
    for obstacle in list(predicted.dynamic_obstacles):
        predicted.remove_obstacle(obstacle)
    for obstacle in scenario.dynamic_obstacles:
        state = obstacle.state_at_time(current)
        if state is None:
            continue
        values = [*state.position, state.orientation, state.velocity]
        if not np.all(np.isfinite(values)) or state.velocity < 0:
            raise IncompatibleState(
                f"Obstacle {obstacle.obstacle_id} has an unusable current state"
            )
        initial = InitialState(
            position=deepcopy(state.position),
            orientation=state.orientation,
            velocity=state.velocity,
            acceleration=0.0,
            yaw_rate=0.0,
            slip_angle=0.0,
            time_step=0,
        )
        states = _predict_states(scenario, state, horizon)
        shape = deepcopy(obstacle.obstacle_shape)
        predicted.add_objects(
            DynamicObstacle(
                obstacle.obstacle_id,
                obstacle.obstacle_type,
                shape,
                initial,
                TrajectoryPrediction(Trajectory(1, states), shape),
            )
        )
    return predicted


def _straight_states(state: State, dt: float, horizon: int) -> list[CustomState]:
    """Original constant-speed, constant-heading forecast, also used as fallback."""
    direction = np.array([np.cos(state.orientation), np.sin(state.orientation)])
    return [
        CustomState(
            position=state.position + direction * state.velocity * dt * step,
            orientation=state.orientation,
            velocity=state.velocity,
            time_step=step,
        )
        for step in range(1, horizon + 1)
    ]


def _centerline(vertices: np.ndarray) -> np.ndarray:
    """Remove repeated vertices before projection and interpolation."""
    keep = np.r_[True, np.linalg.norm(np.diff(vertices, axis=0), axis=1) > 1e-6]
    return vertices[keep]


def _lane_reference(network: LaneletNetwork, state: State, distance: float) -> np.ndarray | None:
    """Match position/heading and build a local reference without guessing a route."""
    candidates = []
    for lane_id in network.find_lanelet_by_position([state.position])[0]:
        lane = network.find_lanelet_by_id(lane_id)
        path = _centerline(lane.center_vertices - state.position)
        if len(path) < 2 or not np.all(np.isfinite(path)):
            continue
        segments = np.diff(path, axis=0)
        lengths = np.linalg.norm(segments, axis=1)
        fractions = np.clip(-np.sum(path[:-1] * segments, axis=1) / lengths**2, 0, 1)
        offsets = path[:-1] + fractions[:, None] * segments
        index = np.argmin(np.linalg.norm(offsets, axis=1))
        heading = np.arctan2(segments[index, 1], segments[index, 0])
        error = abs(
            np.arctan2(np.sin(heading - state.orientation), np.cos(heading - state.orientation))
        )
        # A crossing or oncoming actor should not be redirected along this lane.
        if error <= np.pi / 4:
            candidates.append((error, np.linalg.norm(offsets[index]), lane_id, path))
    if not candidates:
        return None
    _, _, lane_id, path = min(candidates, key=lambda item: item[:3])
    lane = network.find_lanelet_by_id(lane_id)
    visited = {lane_id}
    remaining = distance
    while len(lane.successor) == 1 and remaining > 0:
        next_id = lane.successor[0]
        if next_id in visited:
            break
        successor = network.find_lanelet_by_id(next_id)
        extension = _centerline(successor.center_vertices - state.position)
        if (
            len(extension) < 2
            or not np.all(np.isfinite(extension))
            or np.linalg.norm(path[-1] - extension[0]) > 1e-3
        ):
            break
        path = np.vstack((path, extension[1:]))
        remaining -= np.linalg.norm(np.diff(extension, axis=0), axis=1).sum()
        visited.add(next_id)
        lane = successor

    # Extend at both ends so actors near a lane endpoint can still be projected.
    # The forward extension is also the straight continuation at an unknown fork.
    start = path[0] - (path[1] - path[0]) / np.linalg.norm(path[1] - path[0])
    tangent = (path[-1] - path[-2]) / np.linalg.norm(path[-1] - path[-2])
    end = path[-1] + (distance + 1.0) * tangent
    return resample_polyline(np.vstack((start, path, end)), step=0.5)


def _predict_states(scenario: Scenario, state: State, horizon: int) -> list[CustomState]:
    if state.velocity == 0:
        return _straight_states(state, scenario.dt, horizon)
    reference = _lane_reference(
        scenario.lanelet_network, state, state.velocity * scenario.dt * horizon
    )
    if reference is not None:
        try:
            frame = CurvilinearCoordinateSystem(reference)
            # The reference is translated by the observed position for numerical stability.
            longitudinal, lateral = frame.convert_to_curvilinear_coords(0.0, 0.0)
            states = []
            for step in range(1, horizon + 1):
                s = longitudinal + step * scenario.dt * state.velocity
                position = frame.convert_to_cartesian_coords(s, lateral) + state.position
                tangent = frame.tangent(s)
                states.append(
                    CustomState(
                        position=position,
                        orientation=float(np.arctan2(tangent[1], tangent[0])),
                        velocity=state.velocity,
                        time_step=step,
                    )
                )
            return states
        except (
            CartesianProjectionDomainError,
            CurvilinearProjectionDomainLateralError,
            CurvilinearProjectionDomainLongitudinalError,
            ValueError,
            RuntimeError,
        ):
            # Some lane geometries cannot form a usable Frenet projection domain.
            pass
    return _straight_states(state, scenario.dt, horizon)


def relative_problem(problem: PlanningProblem, steering: float) -> PlanningProblem:
    """Translate the time origin, leaving the physical initial state and goal intact."""
    result = deepcopy(problem)
    current = problem.initial_state.time_step
    result.initial_state.time_step = 0
    result.initial_state.steering_angle = steering
    for goal in result.goal.state_list:
        goal.time_step = Interval(
            max(0, goal.time_step.start - current), goal.time_step.end - current
        )
    return result
