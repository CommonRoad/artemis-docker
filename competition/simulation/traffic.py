"""Preserve vehicle geometry and timestamps at the SUMO integration boundary."""

from copy import deepcopy
from dataclasses import asdict
from pathlib import Path
import xml.etree.ElementTree as ET
import sumolib

from commonroad.geometry.shape import Rectangle
from commonroad.prediction.prediction import TrajectoryPrediction
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.trajectory import Trajectory
from commonroad_sumo.cr2sumo import UnsafeResimulationTrafficGenerator
from commonroad_sumo.cr2sumo.map_converter.util import get_state_list_of_dynamic_obstacle
from commonroad_sumo.cr2sumo.map_converter.mapping import VEHICLE_TYPE_CR2SUMO
from commonroad_sumo.cr2sumo.traffic_generator.sumo_map_matcher import SumoMapMatcher
from commonroad_sumo.cr2sumo.traffic_generator.trajectory_traffic_generator import (
    _get_likely_downstream_edge_from_junction,
    _get_likely_upstream_edge_from_junction,
)
from commonroad_sumo.sumolib.sumo_project import SumoFileType

from .route_matching import FullGuideMapMatcher


class TerminalAwareTrafficGenerator(UnsafeResimulationTrafficGenerator):
    """Match complete guides and extend only where the requested road exists.

    A source or sink junction's polygon can enclose a vehicle on its adjoining
    road. The pinned upstream helpers assume nonempty incoming/outgoing edges.
    Retain mapped roads at real map endpoints. Internal guide endpoints already
    include their adjoining roads. Configuration and library globals are unchanged.
    """

    def generate_traffic(self, scenario, sumo_project):
        self._matching_network = sumolib.net.readNet(
            str(sumo_project.get_file_path(SumoFileType.NET)), withInternal=True
        )
        self._route_matchers = {}
        self.route_matching_diagnostics = {}
        generated = super().generate_traffic(scenario, sumo_project)
        if not generated:
            return False
        self.route_position_adjustments = preserve_route_positions(
            scenario,
            sumo_project.get_file_path(SumoFileType.NET),
            sumo_project.get_file_path(SumoFileType.VEHICLE_ROUTES),
            extend_routes=self._config.extend_routes,
        )
        return generated

    def _get_route_for_dynamic_obstacle(self, dynamic_obstacle, sumo_net):
        states = get_state_list_of_dynamic_obstacle(dynamic_obstacle)
        positions = [tuple(state.position) for state in states]
        vehicle_class = VEHICLE_TYPE_CR2SUMO[dynamic_obstacle.obstacle_type].value
        if vehicle_class not in self._route_matchers:
            self._route_matchers[vehicle_class] = FullGuideMapMatcher(
                self._matching_network, vehicle_class, self._config.map_matching_delta
            )
        route_matcher = self._route_matchers[vehicle_class]
        matched = route_matcher.match(states)
        self.route_matching_diagnostics[dynamic_obstacle.obstacle_id] = {
            "vehicle_class": vehicle_class,
            **asdict(matched),
        }
        edges = [sumo_net.getEdge(identifier) for identifier in matched.edges]
        matcher = SumoMapMatcher(sumo_net, self._config.map_matching_delta)
        next_edge = edges[0] if edges else None
        if self._config.extend_routes_upstream_junction and not matched.starts_inside_junction:
            junction = matcher.junction_at_position(positions[0], outgoing_edge=next_edge)
            if junction is not None and junction.getIncoming():
                upstream = _get_likely_upstream_edge_from_junction(
                    junction, positions[0], states[0].orientation, next_edge
                )
                if (
                    upstream is not None
                    and edges[0].getID() != upstream.getID()
                    and route_matcher.permits_connection(upstream.getID(), edges[0].getID())
                ):
                    edges.insert(0, upstream)
        if self._config.extend_routes_downstream_junction and not matched.ends_inside_junction:
            previous_edge = edges[-1] if edges else None
            junction = matcher.junction_at_position(positions[-1], incoming_edge=previous_edge)
            if junction is not None and junction.getOutgoing():
                # Preserve the pinned helper's existing next-edge hint policy.
                downstream = _get_likely_downstream_edge_from_junction(
                    junction, positions[-1], states[-1].orientation, next_edge
                )
                if (
                    downstream is not None
                    and edges[-1].getID() != downstream.getID()
                    and route_matcher.permits_connection(edges[-1].getID(), downstream.getID())
                ):
                    edges.append(downstream)
        if not all(
            route_matcher.permits_connection(a.getID(), b.getID()) for a, b in zip(edges, edges[1:])
        ):
            raise ValueError("Matched guide contains a class-forbidden or disconnected movement")
        self.route_matching_diagnostics[dynamic_obstacle.obstacle_id]["final_edges"] = [
            edge.getID() for edge in edges
        ]
        return edges


def preserve_route_positions(scenario, network_path, routes_path, *, extend_routes):
    """Restore full positions after duarouter rounds them to two decimals.

    Use each vehicle's selected lane, whose declared length can differ from lane
    zero. Extended arrivals retain the upstream one-millimetre endpoint margin;
    other arrivals retain the projected guide endpoint. Routes remain unchanged.
    """
    network = sumolib.net.readNet(str(network_path))
    tree = ET.parse(routes_path)
    actors = {str(actor.obstacle_id): actor for actor in scenario.dynamic_obstacles}
    adjustments = []
    for vehicle in tree.getroot().findall("vehicle"):
        actor = actors[vehicle.get("id")]
        edges = vehicle.find("route").get("edges").split()
        endpoints = (
            ("departPos", edges[0], "departLane", actor.initial_state.position),
            (
                "arrivalPos",
                edges[-1],
                "arrivalLane",
                actor.prediction.trajectory.final_state.position,
            ),
        )
        for attribute, edge_id, lane_attribute, point in endpoints:
            lane = network.getEdge(edge_id).getLane(int(vehicle.get(lane_attribute)))
            length = lane.getLength()
            offset, _ = lane.getClosestLanePosAndDist(tuple(point))
            position = (
                max(0.0, length - 0.001)
                if attribute == "arrivalPos" and extend_routes
                else min(length, max(0.0, offset))
            )
            previous = float(vehicle.get(attribute))
            vehicle.set(attribute, format(position, ".17g"))
            if previous != position:
                adjustments.append(
                    {
                        "actor_id": actor.obstacle_id,
                        "attribute": attribute,
                        "lane_id": lane.getID(),
                        "declared_length_m": length,
                        "previous_m": previous,
                        "restored_m": position,
                    }
                )
    tree.write(routes_path, encoding="utf-8", xml_declaration=True)
    return adjustments


def remove_initial_state_from_predictions(scenario: Scenario) -> None:
    """Keep a SUMO actor's initial occupancy only in its initial_state.

    SUMO histories can repeat that state at the start of the prediction. The
    collision converter prepends initial_state itself, so predictions must start
    one step later. Preserve absolute timestamps and actors with no later states.
    """
    for actor in scenario.dynamic_obstacles:
        prediction = actor.prediction
        if not isinstance(prediction, TrajectoryPrediction):
            continue
        if prediction.initial_time_step != actor.initial_state.time_step:
            continue
        states = prediction.trajectory.state_list[1:]
        if states:
            # The setter also invalidates previously cached occupancies.
            prediction.trajectory = Trajectory(states[0].time_step, states)
        else:
            actor.prediction = None


def preserve_vehicle_dimensions(scenario: Scenario, route_path: Path) -> None:
    """Give each routed vehicle its source length/width before SUMO calibration.

    The upstream converter emits shared type defaults. Its position calibration
    uses the source length, so a different SUMO length shifts the observed center.
    Copy the generated type to retain its driving model and vehicle class.
    """
    tree = ET.parse(route_path)
    root = tree.getroot()
    types = {node.get("id"): node for node in root.findall("vType")}
    actors = {str(actor.obstacle_id): actor for actor in scenario.dynamic_obstacles}
    for vehicle in root.findall("vehicle"):
        actor = actors.get(vehicle.get("id"))
        if actor is None or not isinstance(actor.obstacle_shape, Rectangle):
            continue
        name = f"cr_dimensions_{actor.obstacle_id}"
        if name not in types:
            original = types.get(vehicle.get("type"))
            definition = deepcopy(original) if original is not None else ET.Element("vType")
            definition.set("id", name)
            root.insert(0, definition)
            types[name] = definition
        definition = types[name]
        definition.set("length", str(actor.obstacle_shape.length))
        definition.set("width", str(actor.obstacle_shape.width))
        vehicle.set("type", name)
    tree.write(route_path, encoding="utf-8", xml_declaration=True)
