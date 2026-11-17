"""State conversion and executable-prefix checks shared by the built-in planners."""

from copy import deepcopy

import numpy as np
from commonroad.common.solution import VehicleType
from commonroad.geometry.shape import Shape
from commonroad.prediction.prediction import TrajectoryPrediction
from commonroad.scenario.state import KSState, State
from commonroad.scenario.trajectory import Trajectory
from commonroad_dc.collision.collision_detection.pycrcc_collision_dispatch import (
    create_collision_object,
)
from commonroad_dc.feasibility.feasibility_checker import trajectory_feasibility
from commonroad_dc.feasibility.vehicle_dynamics import VehicleDynamics
from commonroad_dc.pycrcc import CollisionChecker

from .errors import IncompatibleState, InvalidPlan

STATE_TOLERANCE = 1e-6


def assert_same_state(expected: State, actual: State, *, steering: float | None = None) -> None:
    angle_error = np.arctan2(
        np.sin(expected.orientation - actual.orientation),
        np.cos(expected.orientation - actual.orientation),
    )
    if (
        expected.time_step != actual.time_step
        or not np.allclose(expected.position, actual.position, rtol=0, atol=STATE_TOLERANCE)
        or not np.isclose(expected.velocity, actual.velocity, rtol=0, atol=STATE_TOLERANCE)
        or not np.isclose(angle_error, 0.0, rtol=0, atol=STATE_TOLERANCE)
    ):
        raise IncompatibleState("The current state differs from the last executed endpoint")
    steering = getattr(actual, "steering_angle", None) if steering is None else steering
    if steering is not None and not np.isclose(
        expected.steering_angle, steering, rtol=0, atol=STATE_TOLERANCE
    ):
        raise IncompatibleState("The current steering differs from the executed endpoint")


def recover_steering(
    current: State, previous: State | None, *, wheelbase: float | None = None
) -> float:
    """SUMO omits steering. Recover the executed angle; RP can infer its first angle from yaw rate."""
    if previous is not None:
        assert_same_state(previous, current)
        return previous.steering_angle
    steering = getattr(current, "steering_angle", None)
    yaw_rate = getattr(current, "yaw_rate", 0.0) or 0.0
    if steering is None:
        steering = 0.0
        if abs(yaw_rate) > STATE_TOLERANCE:
            if wheelbase is None or current.velocity <= 0:
                raise IncompatibleState(
                    "A turning initial state needs an explicit compatible steering angle"
                )
            steering = np.arctan(wheelbase * yaw_rate / current.velocity)
    if wheelbase is None and abs(yaw_rate) > STATE_TOLERANCE and steering == 0:
        raise IncompatibleState(
            "A turning initial state needs an explicit compatible steering angle"
        )
    if not np.isfinite(steering):
        raise IncompatibleState("Initial steering must be finite")
    return steering


def center_state(state: State, rear_axle_distance: float, time_offset: int) -> KSState:
    """Convert a planner's rear-axle state to a center-coordinate KS state once."""
    if not all(
        np.all(np.isfinite(getattr(state, name, np.nan)))
        for name in ("position", "orientation", "velocity", "steering_angle", "time_step")
    ):
        raise InvalidPlan("Planner returned a non-finite or incomplete state")
    position = state.position + rear_axle_distance * np.array(
        [np.cos(state.orientation), np.sin(state.orientation)]
    )
    return KSState(
        position=position,
        orientation=state.orientation,
        velocity=state.velocity,
        steering_angle=state.steering_angle,
        time_step=state.time_step + time_offset,
    )


def validate_prefix(
    trajectory: Trajectory,
    dt: float,
    vehicle_type: VehicleType,
    shape: Shape,
    collision_checker: CollisionChecker,
) -> None:
    """Check center-coordinate states, including the real initial transition."""
    states = deepcopy(trajectory.state_list)
    for state in states:
        state.time_step -= trajectory.initial_time_step
    relative = Trajectory(0, states)
    try:
        feasible = trajectory_feasibility(relative, VehicleDynamics.KS(vehicle_type), dt)[0]
    except Exception as error:
        raise InvalidPlan(f"Could not validate trajectory dynamics: {error}") from error
    if not feasible:
        raise InvalidPlan("The executable trajectory is not dynamically feasible")
    occupancy = create_collision_object(TrajectoryPrediction(relative, shape))
    if collision_checker.collide(occupancy):
        raise InvalidPlan(
            "The executable trajectory collides with predicted traffic or the road boundary"
        )
