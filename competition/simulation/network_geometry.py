"""Preprocess generated SUMO traffic geometry using source road corridors."""

import math
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
import shapely
from scipy.optimize import minimize_scalar
from shapely.ops import unary_union
from commonroad.geometry.shape import Rectangle
from commonroad.scenario.obstacle import ObstacleType
from commonroad.scenario.scenario import Scenario
from commonroad_sumo.sumolib.sumo_project import SumoFileType, SumoProject
from commonroad_sumo.cr2sumo.map_converter import partition_lanelet_network_into_edges_and_lanes
from .conversion import _connection_gaps


def _unique_vertices(points):
    points = np.asarray(points)
    return points[np.r_[True, np.linalg.norm(np.diff(points, axis=0), axis=1) > 1e-08]]


def _stations(points):
    return np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(points, axis=0), axis=1))]


def _read_network(path):
    root = ET.parse(path).getroot()
    lanes = {l.get("id"): l for l in root.findall("./edge/lane")}
    links = root.findall("connection")
    out = {}
    for c in links:
        out.setdefault(c.get("from") + "_" + c.get("fromLane"), []).append(c)
    return (root, lanes, links, out)


def _lane_chain(c, out):
    source = c.get("from") + "_" + c.get("fromLane")
    target = c.get("to") + "_" + c.get("toLane")
    result = [source]
    next_id = c.get("via") or target
    while next_id != target:
        if next_id in result:
            raise ValueError("loop")
        result.append(next_id)
        (link,) = out[next_id]
        next_id = link.get("via") or link.get("to") + "_" + link.get("toLane")
    result.append(target)
    return result


def _lane_shape(lane):
    return np.asarray([[float(v) for v in pair.split(",")] for pair in lane.get("shape").split()])


def _path_model(lanes, identifiers, overrides=None):
    """Map SUMO's declared lane positions to their polyline coordinates."""
    coordinates, route_stations = [], []
    route_offset = 0.0
    for identifier in identifiers:
        lane = lanes[identifier]
        changed = overrides is not None and identifier in overrides
        lane_coordinates = overrides[identifier] if changed else _lane_shape(lane)
        geometric_stations = _stations(lane_coordinates)
        declared_length = geometric_stations[-1] if changed else float(lane.get("length"))
        coordinates.extend(lane_coordinates)
        route_stations.extend(
            geometric_stations / geometric_stations[-1] * declared_length + route_offset
        )
        route_offset += declared_length
    coordinates = np.asarray(coordinates)
    route_stations = np.asarray(route_stations)
    # Keep one coordinate at each shared lane endpoint.
    keep = np.r_[np.diff(route_stations) > 1e-8, True]
    return coordinates[keep], route_stations[keep]


def _footprint_area(
    lanes,
    identifiers,
    road,
    length,
    width,
    lateral=0.0,
    overrides=None,
    spacing=0.1,
    interval=None,
):
    """Measure complete rectangles using SUMO's front/back heading model.

    The rear reference follows the route by the vehicle's declared length;
    its chord defines orientation. The physical rear is one vehicle length
    behind the front along that orientation. Lateral alignment offsets both
    reference points without changing the vehicle's dimensions.
    """
    coordinates, route_stations = _path_model(lanes, identifiers, overrides)
    first_length = float(lanes[identifiers[0]].get("length"))
    last_length = float(lanes[identifiers[-1]].get("length"))
    start = first_length
    end = route_stations[-1] - last_length + min(length, last_length)
    if len(identifiers) == 1:
        start, end = length, route_stations[-1]
    if interval is not None:
        start, end = max(start, interval[0]), min(end, interval[1])
    if end < start:
        return 0.0, start
    front_stations = np.arange(start, end + 1e-5, spacing)

    def positions(samples):
        points = np.column_stack(
            [np.interp(samples, route_stations, coordinates[:, axis]) for axis in range(2)]
        )
        if lateral:
            segments = np.clip(
                np.searchsorted(route_stations, samples, side="right") - 1,
                0,
                len(coordinates) - 2,
            )
            directions = coordinates[segments + 1] - coordinates[segments]
            directions /= np.linalg.norm(directions, axis=1)[:, None]
            normals = np.column_stack([-directions[:, 1], directions[:, 0]])
            points += lateral * normals
        return points

    front_points = positions(front_stations)
    back_references = positions(front_stations - length)
    forward_vectors = front_points - back_references
    forward_vectors /= np.linalg.norm(forward_vectors, axis=1)[:, None]
    half_width_offsets = (
        width
        / 2
        * np.column_stack(
            [
                -forward_vectors[:, 1],
                forward_vectors[:, 0],
            ]
        )
    )
    rear_points = front_points - length * forward_vectors
    footprints = shapely.polygons(
        np.stack(
            [
                front_points + half_width_offsets,
                front_points - half_width_offsets,
                rear_points - half_width_offsets,
                rear_points + half_width_offsets,
            ],
            axis=1,
        )
    )
    outside_areas = shapely.area(shapely.difference(footprints, road))
    return float(np.max(outside_areas)), float(front_stations[np.argmax(outside_areas)])


def _set_lane_shape(lane, p):
    lane.set("shape", " ".join((",".join((f"{x:.12f}" for x in row)) for row in p)))
    lane.set("length", f"{_stations(p)[-1]:.12f}")


def _route_dimensions(scenario, route_root):
    """Associate source dimensions with generated logical edge routes."""
    actors = {str(a.obstacle_id): a for a in scenario.dynamic_obstacles}
    pairs = {}
    bike_routes = []
    for vehicle in route_root.findall("vehicle"):
        actor = actors.get(vehicle.get("id"))
        if actor is None or not isinstance(actor.obstacle_shape, Rectangle):
            continue
        edges = vehicle.find("route").get("edges").split()
        dim = (actor.obstacle_shape.length, actor.obstacle_shape.width)
        for a, b in zip(edges, edges[1:]):
            pairs.setdefault((a, b), set()).add(dim)
        if actor.obstacle_type == ObstacleType.BICYCLE:
            bike_routes.append(
                (
                    vehicle.get("id"),
                    edges,
                    dim,
                    float(vehicle.get("departPos", "0")),
                    float(vehicle.get("arrivalPos", "inf")),
                )
            )
    return (pairs, bike_routes)


def _fit_turns(lanes, links, outgoing, pairs, road):
    """Fit each routed connector with a small fixed-endpoint normal offset."""
    rows = []
    for link in links:
        if link.get("from").startswith(":") or not link.get("via"):
            continue
        dims = sorted(pairs.get((link.get("from"), link.get("to")), ()))
        if not dims:
            continue
        ids = _lane_chain(link, outgoing)

        def evaluate(overrides=None, spacing=0.1):
            return max(
                (
                    _footprint_area(lanes, ids, road, *dim, overrides=overrides, spacing=spacing)[0]
                    for dim in dims
                )
            )

        initial = evaluate()
        if initial <= 1e-07:
            continue
        for identifier in ids[1:-1]:
            base = _unique_vertices(_lane_shape(lanes[identifier]))
            d = _stations(base)
            if d[-1] < 1:
                continue
            tangent = np.gradient(base, axis=0)
            tangent /= np.linalg.norm(tangent, axis=1)[:, None]
            normal = np.column_stack([-tangent[:, 1], tangent[:, 0]])
            blend = np.sin(np.pi * d / d[-1]) ** 2

            def candidate(alpha):
                return base + alpha * blend[:, None] * normal

            result = minimize_scalar(
                lambda a: evaluate({identifier: candidate(a)}),
                bounds=(-0.3, 0.3),
                method="bounded",
                options={"xatol": 0.0001},
            )
            best = evaluate({identifier: candidate(result.x)})
            if best > 1e-07:
                continue
            lo, hi = (0.0, abs(float(result.x)))
            sign = np.sign(result.x)
            for _ in range(18):
                mid = (lo + hi) / 2
                if evaluate({identifier: candidate(sign * mid)}) < 1e-07:
                    hi = mid
                else:
                    lo = mid
            alpha = sign * (hi + 0.005)
            new = candidate(alpha)
            # The extra 5 mm is accepted only after checking the opposite side
            # as well, at 1 cm stations over the complete front/rear path.
            dense = evaluate({identifier: new}, spacing=0.01)
            if dense > 1e-07:
                continue
            old_length = float(lanes[identifier].get("length"))
            _set_lane_shape(lanes[identifier], new)
            rows.append(
                {
                    "kind": "turn",
                    "lane": identifier,
                    "connection": [link.get("from"), link.get("to")],
                    "dimensions": dims,
                    "before_area_m2": initial,
                    "after_dense_area_m2": dense,
                    "offset_parameter_m": alpha,
                    "maximum_vertex_displacement_m": float(
                        np.max(np.linalg.norm(new - base, axis=1))
                    ),
                    "old_length_m": old_length,
                    "new_length_m": float(lanes[identifier].get("length")),
                }
            )
            if evaluate(spacing=0.01) < 1e-07:
                break
        if evaluate(spacing=0.01) > 1e-07:
            rows.append(
                {
                    "kind": "unfitted-turn",
                    "connection": [link.get("from"), link.get("to")],
                    "area_m2": evaluate(spacing=0.01),
                    "dimensions": dims,
                }
            )
        elif link.get("shape") is not None:
            link.set(
                "shape",
                " ".join(
                    (
                        " ".join(
                            (",".join((f"{x:.12f}" for x in p)) for p in _lane_shape(lanes[i]))
                        )
                        for i in ids[1:-1]
                    )
                ),
            )
    return rows


def _terminal_lane_caps(scenario):
    """Index genuine longitudinal source endcaps by converted external lane."""
    caps = {}
    groups = partition_lanelet_network_into_edges_and_lanes(scenario.lanelet_network)
    for edge_id, lanelet_ids in groups.items():
        for index, lanelet_id in enumerate(lanelet_ids):
            lanelet = scenario.lanelet_network.find_lanelet_by_id(lanelet_id)
            if not lanelet.polygon.shapely_object.is_valid:
                continue
            boundaries = {}
            if not lanelet.predecessor:
                direction = lanelet.center_vertices[0] - lanelet.center_vertices[1]
                boundaries["entry"] = (
                    lanelet.left_vertices[0],
                    lanelet.right_vertices[0],
                    direction / np.linalg.norm(direction),
                )
            if not lanelet.successor:
                direction = lanelet.center_vertices[-1] - lanelet.center_vertices[-2]
                boundaries["exit"] = (
                    lanelet.left_vertices[-1],
                    lanelet.right_vertices[-1],
                    direction / np.linalg.norm(direction),
                )
            if boundaries:
                caps[f"{edge_id}_{index}"] = boundaries
    return caps


def _fit_bicycle_widths(root, lanes, links, outgoing, bicycle_routes, road, terminal_caps=None):
    """Cap only individual road lanes/turn chains that fail a full-body check."""
    changes = []
    for actor_id, edges, dimensions, depart_position, arrival_position in bicycle_routes:
        route_pairs = set(zip(edges, edges[1:]))
        constraints = []
        for link in links:
            if (link.get("from"), link.get("to")) in route_pairs:
                constraints.append((_lane_chain(link, outgoing), None, []))
        for edge in root.findall("edge"):
            if edge.get("id") not in edges:
                continue
            # SUMO's right-aligned bicycle uses the rightmost road lane.
            # Required turning lanes are included in the connection chains.
            lane = edge.find("lane[@index='0']")
            if lane is None or float(lane.get("length")) <= dimensions[0] + 0.1:
                continue
            interval = (
                depart_position if edge.get("id") == edges[0] else 0.0,
                arrival_position if edge.get("id") == edges[-1] else float("inf"),
            )
            lane_id = lane.get("id")
            boundaries = (terminal_caps or {}).get(lane_id, {})
            matched_caps = []
            if edge.get("id") == edges[0] and "entry" in boundaries:
                matched_caps.append(boundaries["entry"])
            if edge.get("id") == edges[-1] and "exit" in boundaries:
                matched_caps.append(boundaries["exit"])
            constraints.append(([lane_id], interval, matched_caps))

        for identifiers, interval, matched_caps in constraints:
            # A genuine route-terminal crossing can extend longitudinally,
            # within the exact source cap's lateral bounds. It must not block
            # an otherwise valid interior width fit. Actual admission separately
            # requires the observed arrival/entry and matching endcap evidence.
            constraint_road = road
            if matched_caps:
                extension = dimensions[0] + dimensions[1]
                cap_corridors = [
                    shapely.Polygon(
                        [
                            left,
                            right,
                            right + extension * direction,
                            left + extension * direction,
                        ]
                    )
                    for left, right, direction in matched_caps
                ]
                constraint_road = unary_union([road, *cap_corridors])
            # Adjacent roads are checked with a turn because a vehicle's rear
            # can still occupy them while its front enters/leaves the turn.
            current_offset = max(
                max(0.0, (float(lanes[identifier].get("width", "3.2")) - dimensions[1]) / 2)
                for identifier in identifiers
            )

            def outside_area(offset, spacing=0.1):
                return _footprint_area(
                    lanes,
                    identifiers,
                    constraint_road,
                    *dimensions,
                    lateral=-offset,
                    spacing=spacing,
                    interval=interval,
                )[0]

            initial_area = outside_area(current_offset)
            if initial_area <= 1e-7:
                continue
            if outside_area(0.0) > 1e-7:
                changes.append(
                    {
                        "kind": "unfitted-width",
                        "actor": actor_id,
                        "constraint_lane_ids": identifiers,
                        "dimensions": dimensions,
                        "area_m2": initial_area,
                    }
                )
                continue
            lower, upper = 0.0, current_offset
            for _ in range(18):
                middle = (lower + upper) / 2
                if outside_area(middle) < 1e-7:
                    lower = middle
                else:
                    upper = middle
            # A local geometric cap, rounded down rather than widening the
            # physical corridor through decimal serialization.
            safe_width = math.floor((dimensions[1] + 2 * lower) * 100) / 100
            while (
                safe_width >= dimensions[1]
                and outside_area((safe_width - dimensions[1]) / 2, spacing=0.01) > 1e-7
            ):
                safe_width -= 0.01
            if safe_width < dimensions[1]:
                continue
            dense_area = outside_area((safe_width - dimensions[1]) / 2, spacing=0.01)
            for identifier in identifiers:
                old_width = float(lanes[identifier].get("width", "3.2"))
                if safe_width >= old_width:
                    continue
                lanes[identifier].set("width", f"{safe_width:.12f}")
                changes.append(
                    {
                        "kind": "width",
                        "lane": identifier,
                        "actor": actor_id,
                        "dimensions": dimensions,
                        "old_width_m": old_width,
                        "new_width_m": safe_width,
                        "constraint_lane_ids": identifiers,
                        "before_area_m2": initial_area,
                        "after_dense_area_m2": dense_area,
                    }
                )
    return changes


def prepare_traffic_geometry(scenario: Scenario, project: SumoProject) -> list[dict]:
    """Fit routed background rectangles inside the unchanged CommonRoad road.

    Call after traffic generation and before constructing/starting SUMO. Only
    generated internal lane shapes, their lengths, and conservative lane widths
    may change. Route IDs, connection endpoints, priorities, signals, vehicle
    dimensions and lateral driving models remain intact. Returned records
    describe changes or bounded turn fits that could not be completed; benchmark
    qualification still checks actual full bodies, entries/exits and collisions.
    """
    netpath = project.get_file_path(SumoFileType.NET)
    root, lanes, links, outgoing = _read_network(netpath)
    road = unary_union(
        [
            lane.polygon.shapely_object
            for lane in scenario.lanelet_network.lanelets
            if lane.polygon.shapely_object.is_valid
        ]
    )
    route_root = ET.parse(project.get_file_path(SumoFileType.VEHICLE_ROUTES)).getroot()
    pairs, bicycle_routes = _route_dimensions(scenario, route_root)
    rows = _fit_turns(lanes, links, outgoing, pairs, road)
    rows.extend(
        _fit_bicycle_widths(
            root,
            lanes,
            links,
            outgoing,
            bicycle_routes,
            road,
            _terminal_lane_caps(scenario),
        )
    )
    if rows:
        tree = ET.ElementTree(root)
        with tempfile.NamedTemporaryFile(
            dir=Path(netpath).parent, suffix=".net.xml", delete=False
        ) as stream:
            staged_path = Path(stream.name)
            tree.write(stream, encoding="utf-8", xml_declaration=True)
        try:
            if _connection_gaps(staged_path):
                raise ValueError("Geometry preprocessing changed connection endpoints")
            staged_path.replace(netpath)
        finally:
            staged_path.unlink(missing_ok=True)
    return rows
