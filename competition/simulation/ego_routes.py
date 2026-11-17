"""Prepare a legal SUMO route before executing an exact ego command."""

from types import SimpleNamespace

import libsumo
import numpy as np
import sumolib

from .route_matching import FullGuideMapMatcher


class _EgoRouteMatcher(FullGuideMapMatcher):
    """Anchor to the actual lane and associate commands by geometric distance.

    Traffic guides need a heading cost to resolve their complete route intent.
    An ego command already specifies its exact pose; a heading cost can otherwise
    prefer the old route endpoint over a point just inside the next junction.
    The shared matcher still requires forward headings and legal connectivity.
    """

    def match_from_lane(self, states, lane):
        self._initial_sample = states[0]
        self._initial_node = lane.getID() if lane.getEdge().isSpecial() else lane.getEdge().getID()
        return self.match(states)

    def _candidates(self, state):
        candidates = super()._candidates(state)
        if state is self._initial_sample:
            return (
                {self._initial_node: candidates[self._initial_node]}
                if self._initial_node in candidates
                else {}
            )
        return candidates

    def _candidate_cost(self, distance, heading):
        return distance**2


class EgoRoutePreloader:
    """Retain native traffic reaction while avoiding stale ego route endpoints."""

    def __init__(self, network_file, delta=5.0):
        self._network = sumolib.net.readNet(str(network_file), withInternal=True)
        self._delta = delta
        self._matchers = {}

    def prepare(self, ego, future):
        identifier = str(ego.obstacle_id)
        lane_id = libsumo.vehicle.getLaneID(identifier)
        if not lane_id:
            # Remote-controlled ego poses may lie outside the SUMO lane geometry.
            # Route association does not add a restriction to planner commands.
            return
        vehicle_class = libsumo.vehicle.getVehicleClass(identifier)
        if vehicle_class not in self._matchers:
            self._matchers[vehicle_class] = _EgoRouteMatcher(
                self._network, vehicle_class, self._delta
            )
        samples = [
            SimpleNamespace(
                position=np.array(libsumo.vehicle.getPosition(identifier)),
                orientation=np.deg2rad(90 - libsumo.vehicle.getAngle(identifier)),
            )
        ]
        half_length = ego.obstacle_shape.length / 2
        samples.extend(
            SimpleNamespace(
                position=state.position
                + half_length * np.array([np.cos(state.orientation), np.sin(state.orientation)]),
                orientation=state.orientation,
            )
            for state in future
        )
        try:
            matched = self._matchers[vehicle_class].match_from_lane(
                samples, self._network.getLane(lane_id)
            )
        except ValueError:
            # Keep executing the original exact command when lane geometry cannot
            # represent its path; the existing execution/validation checks govern it.
            return
        route = list(matched.edges)
        current = list(libsumo.vehicle.getRoute(identifier))
        remaining = current[libsumo.vehicle.getRouteIndex(identifier) :]
        if route != remaining[: len(route)]:
            libsumo.vehicle.setRoute(identifier, route)
