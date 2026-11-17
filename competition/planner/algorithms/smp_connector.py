"""Temporary first primitives that join the actual speed to the SMP speed grid."""

from bisect import bisect_left
from importlib import import_module

import numpy as np
from commonroad.scenario.state import InputState, KSState
from commonroad.scenario.trajectory import Trajectory
from commonroad_dc.feasibility.vehicle_dynamics import VehicleDynamics

from ..common.trajectory import STATE_TOLERANCE
from ..common.errors import IncompatibleState


def speed_connectors(automaton, speed: float, steering: float, steps: int) -> list:
    """Offer the neighboring grid speeds without changing the real starting state."""
    if not np.isfinite(speed) or speed < 0:
        raise IncompatibleState("The SMP speed connector requires a finite, nonnegative speed")

    compatible = [
        p
        for p in automaton.list_primitives
        if abs(p.state_initial.steering_angle - steering) <= STATE_TOLERANCE
    ]
    if not compatible:
        raise IncompatibleState(f"No primitive starts at steering {steering:g} rad")

    speeds = sorted({p.state_initial.velocity for p in compatible})
    index = bisect_left(speeds, speed)
    # Try the lower and upper neighbors; outside the grid, only its nearest edge exists.
    targets = speeds[max(0, index - 1) : index + 1]
    dynamics = VehicleDynamics.KS(automaton.type_vehicle)
    next_id = max(p.id for p in automaton.list_primitives) + 1
    connectors = []
    for target in targets:
        connector = _speed_connector(speed, target, steering, steps, dynamics)
        if connector is None:
            continue
        connector.id = next_id + len(connectors)
        connector.list_successors = [
            p for p in compatible if abs(p.state_initial.velocity - target) <= STATE_TOLERANCE
        ]
        connector.list_ids_successors = [p.id for p in connector.list_successors]
        connectors.append(connector)

    if not connectors:
        raise IncompatibleState(
            f"No feasible {steps * 0.1:g}-second speed connector from {speed:g} m/s "
            f"at steering {steering:g} rad to the neighboring primitive speeds {targets}"
        )
    return connectors


def _speed_connector(speed, target, steering, steps, dynamics):
    """Simulate constant acceleration and fixed steering using the library's KS model."""
    dt = 0.1
    rear_axle_distance = dynamics.parameters.b
    # The dynamics API uses vehicle centers; SMP stores primitives at the rear axle.
    # Start the center ahead of a local rear-axle origin, then convert back below.
    states = [
        KSState(
            position=np.array([rear_axle_distance, 0.0]),
            velocity=speed,
            steering_angle=steering,
            orientation=0.0,
            time_step=0,
        )
    ]
    control = InputState(
        acceleration=(target - speed) / (steps * dt),
        steering_angle_speed=0.0,
        time_step=0,
    )
    for _ in range(steps):
        state = dynamics.simulate_next_state(states[-1], control, dt, throw=False)
        if state is None:
            return None
        states.append(state)

    # Acceleration may be limited by the model. Never snap a missed endpoint to the grid.
    if abs(states[-1].velocity - target) > STATE_TOLERANCE:
        return None
    for state in states:
        state.position -= rear_axle_distance * np.array(
            [np.cos(state.orientation), np.sin(state.orientation)]
        )

    # Resolve the class after submission loading so a student's SMP tree is respected.
    primitive_class = import_module("SMP.maneuver_automaton.motion_primitive").MotionPrimitive
    initial, final = [
        primitive_class.PrimitiveState(
            x=state.position[0],
            y=state.position[1],
            steering_angle=state.steering_angle,
            velocity=state.velocity,
            orientation=state.orientation,
            time_step=state.time_step,
        )
        for state in (states[0], states[-1])
    ]
    return primitive_class(initial, final, Trajectory(0, states), dt)
