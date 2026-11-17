"""Exact-state and finite-horizon views of the unmodified SMP automaton."""

from copy import copy

from commonroad.planning.planning_problem import PlanningProblem

from ..common.trajectory import STATE_TOLERANCE
from .smp_connector import speed_connectors

PRIMITIVE_STEPS = 5


class PrimitiveAtDepth:
    """Delegate motion to the original primitive, but stop expanding at the horizon."""

    def __init__(self, primitive, depth, automaton):
        self.primitive = primitive
        self.depth = depth
        self.automaton = automaton

    def __getattr__(self, name):
        return getattr(self.primitive, name)

    @property
    def list_successors(self) -> list["PrimitiveAtDepth"]:
        if self.depth >= self.automaton.max_depth:
            return []
        return [self.automaton.at_depth(p, self.depth + 1) for p in self.primitive.list_successors]


class ExactStateAutomaton:
    """An adapter view; neither the source automaton nor SMP code is modified."""

    def __init__(self, automaton, horizon):
        self.source = automaton
        self.max_depth = horizon // PRIMITIVE_STEPS
        self._views = {}

    def __getattr__(self, name):
        return getattr(self.source, name)

    def at_depth(self, primitive, depth: int) -> PrimitiveAtDepth:
        key = (primitive.id, depth)
        if key not in self._views:
            self._views[key] = PrimitiveAtDepth(primitive, depth, self)
        return self._views[key]

    def create_initial_motion_primitive(
        self, planning_problem: PlanningProblem, initial_steering_angle: float = 0.0
    ):
        state = planning_problem.initial_state
        steering = getattr(state, "steering_angle", initial_steering_angle)
        matches = [
            p
            for p in self.source.list_primitives
            if abs(p.state_initial.velocity - state.velocity) <= STATE_TOLERANCE
            and abs(p.state_initial.steering_angle - steering) <= STATE_TOLERANCE
        ]
        if not matches:
            # For a continuous starting speed, search a five-step transition
            # to a neighboring grid speed at fixed steering. It counts toward the
            # same horizon and is collision-checked by SMP like any other primitive.
            matches = speed_connectors(self.source, state.velocity, steering, PRIMITIVE_STEPS)
        # SMP only reads this dummy primitive's successors. A shallow copy avoids
        # altering a real primitive or invoking SMP's speed-snapping initializer.
        root = copy(matches[0])
        root.list_successors = [self.at_depth(p, 1) for p in matches]
        return root
