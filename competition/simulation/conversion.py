"""Preserve source geometry and stable traffic signs in generated SUMO maps."""

import logging
import xml.etree.ElementTree as ET

import numpy as np
from commonroad_sumo.helpers import SumoApplication, execute_sumo_application
from commonroad_sumo.sumolib.sumo_project import SumoFileType, SumoIntermediateFileType, SumoProject

from commonroad_sumo.cr2sumo import CR2SumoMapConverter
from commonroad_sumo.cr2sumo.map_converter.traffic_sign import TrafficSignEncoder


_LOGGER = logging.getLogger(__name__)


class StableTrafficSignEncoder(TrafficSignEncoder):
    """Keep sign-boundary traversal identities stable while preserving upstream rules."""

    def encode(self) -> None:
        # The inherited dispatcher only iterates its original edge/sign assignments.
        # Traversal must not look them up by Edge after changing its type_id/hash.
        self.edge_traffic_signs = {
            edge: tuple(
                sorted(
                    elements,
                    key=lambda item: (
                        item.traffic_sign_element_id.name,
                        tuple(item.additional_values),
                    ),
                )
            )
            for edge, elements in self.edge_traffic_signs.items()
        }
        self._sign_types_by_edge_id = {
            edge.id: {element.traffic_sign_element_id for element in elements}
            for edge, elements in self.edge_traffic_signs.items()
        }
        super().encode()

    def _bfs_until(self, start, element):
        queue = [start]
        visited = set()
        while queue:
            edge = queue.pop()
            if edge.id in visited:
                continue
            visited.add(edge.id)
            if (
                edge.id != start.id
                and element.traffic_sign_element_id in self._sign_types_by_edge_id.get(edge.id, ())
            ):
                continue
            queue.extend(edge.outgoing)
            yield edge


class StableMapConverter(CR2SumoMapConverter):
    """Preserve source lane endpoints and stable traffic-sign traversal identities."""

    def _compute_node_coords(self, lanelets, index):
        # Source ordering must not affect a half-micrometre rounding tie in
        # the intermediate XML's node coordinates.
        return np.mean(
            [
                lanelet.center_vertices[index]
                for lanelet in sorted(lanelets, key=lambda lanelet: lanelet.lanelet_id)
            ],
            axis=0,
        )

    def _merge_junctions_intersecting_lanelets(self):
        nodes, merged, replaced = super()._merge_junctions_intersecting_lanelets()
        # Upstream averages a set of Node objects, whose order depends on the
        # process hash seed. Sum the same source nodes in a stable ID order.
        for node_id, original_ids in merged.items():
            nodes[node_id].coord = np.mean(
                [self.nodes[original_id].coord for original_id in sorted(original_ids)], axis=0
            )
        return nodes, merged, replaced

    def _create_sumo_edges_and_lanes(self) -> None:
        super()._create_sumo_edges_and_lanes()
        for edge in self.edges.values():
            # netconvert uses the edge reference shape to compute junction cuts,
            # even when every lane has an explicit custom shape. Without it the
            # merged junction coordinates replace the source edge endpoints.
            shapes = [np.asarray(lane.shape) for lane in edge.lanes]
            stations = []
            for shape in shapes:
                distances = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(shape, axis=0), axis=1))]
                stations.append(distances / distances[-1])
            samples = np.unique(np.concatenate(stations))
            edge.shape = np.mean(
                [
                    np.column_stack(
                        [np.interp(samples, station, shape[:, axis]) for axis in range(2)]
                    )
                    for shape, station in zip(shapes, stations)
                ],
                axis=0,
            )

    def merge_intermediate_files(self, sumo_intermediate_project, cleanup):
        # Custom connection shapes follow the CommonRoad intersection lanelets.
        # Keep their adjoining road endpoints when computing junction outlines.
        project = SumoProject.from_intermediate_sumo_project(sumo_intermediate_project)
        arguments = [
            "--no-turnarounds=true",
            "--precision=12",
            "--junctions.internal-link-detail=20",
            "--junctions.endpoint-shape=true",
            "--geometry.avoid-overlap=true",
            "--geometry.remove.keep-edges.explicit=true",
            "--geometry.remove.min-length=0.0",
            "--tls.guess-signals=true",
            "--tls.group-signals=true",
            "--tls.green.time=50",
            "--tls.red.time=50",
            "--tls.yellow.time=10",
            "--tls.allred.time=50",
            "--tls.left-green.time=50",
            "--tls.crossing-min.time=50",
            "--tls.crossing-clearance.time=50",
            "--offset.disable-normalization=true",
        ]
        for option, kind in (
            ("node-files", SumoIntermediateFileType.NODES),
            ("edge-files", SumoIntermediateFileType.EDGES),
            ("connection-files", SumoIntermediateFileType.CONNECTIONS),
            ("tllogic-files", SumoIntermediateFileType.TLLOGICS),
            ("type-files", SumoIntermediateFileType.TYPES),
        ):
            arguments.append(f"--{option}={sumo_intermediate_project.get_file_path(kind)}")
        arguments.extend(
            [
                f"--output-file={project.get_file_path(SumoFileType.NET)}",
                f"--seed={self._conf.random_seed}",
            ]
        )
        output = execute_sumo_application(SumoApplication.NETCONVERT, arguments)
        if output is None:
            _LOGGER.error("Failed to merge intermediate files: netconvert failed")
            return None
        for line in output.splitlines():
            _LOGGER.debug("netconvert output: %s", line)
        disconnected = _connection_gaps(project.get_file_path(SumoFileType.NET))
        if disconnected:
            # Preserve mapped turns with small endpoint discrepancies. A standard
            # replacement spline can cut across the road even for a 4 mm gap.
            # Only materially trimmed custom turns require a complete rebuild.
            keys = {key for key, gap in disconnected if gap > 0.10}
            if keys:
                connections_path = sumo_intermediate_project.get_file_path(
                    SumoIntermediateFileType.CONNECTIONS
                )
                tree = ET.parse(connections_path)
                repaired = set()
                for connection in tree.getroot().findall("connection"):
                    key = tuple(
                        connection.get(name) for name in ("from", "to", "fromLane", "toLane")
                    )
                    if key in keys and "shape" in connection.attrib:
                        del connection.attrib["shape"]
                        repaired.add(key)
                if repaired != keys:
                    raise ValueError(
                        f"Disconnected SUMO lane connections lack custom source shapes: {keys - repaired}"
                    )
                tree.write(connections_path, encoding="utf-8", xml_declaration=True)
                output = execute_sumo_application(SumoApplication.NETCONVERT, arguments)
                if output is None:
                    _LOGGER.error("Failed to rebuild disconnected SUMO lane connections")
                    return None
                _LOGGER.info("Rebuilt %s materially trimmed SUMO connections", len(repaired))
        self.endpoint_stitches = _stitch_small_connection_gaps(
            project.get_file_path(SumoFileType.NET)
        )
        remaining = _connection_gaps(project.get_file_path(SumoFileType.NET))
        if remaining:
            raise ValueError(f"Disconnected SUMO lane connections after repair: {remaining}")
        if cleanup:
            sumo_intermediate_project.cleanup()
        return project

    def _encode_traffic_signs(self) -> None:
        encoder = StableTrafficSignEncoder(self.edge_types)
        signs = {sign.traffic_sign_id: sign for sign in self._lanelet_network.traffic_signs}
        for lanelet in self._lanelet_network.lanelets:
            if not lanelet.traffic_signs:
                continue
            edge_id = self.lanelet_id2edge_id[lanelet.lanelet_id]
            if edge_id not in self.new_edges:
                _LOGGER.warning(
                    "Merged Edge %s with traffic signs %s. These Traffic signs will not be converted.",
                    edge_id,
                    lanelet.traffic_signs,
                )
                continue
            edge = self.new_edges[edge_id]
            for sign_id in lanelet.traffic_signs:
                encoder.apply(signs[sign_id], edge)
        encoder.encode()


def _connection_gaps(network_path):
    """Find gaps in complete lane chains, following internal via lanes too.

    The converter writes twelve decimal places. The 1e-8m threshold admits
    numerical serialization noise only; actual full vehicle containment
    remains a separate geometric check.
    """
    root = ET.parse(network_path).getroot()
    shapes = {
        lane.get("id"): np.asarray(
            [[float(value) for value in point.split(",")] for point in lane.get("shape").split()]
        )
        for lane in root.findall("./edge/lane")
    }
    links = root.findall("connection")
    outgoing = {}
    for link in links:
        source = f"{link.get('from')}_{link.get('fromLane')}"
        outgoing.setdefault(source, []).append(link)
    gaps = []
    for link in links:
        if link.get("from").startswith(":"):
            continue
        key = tuple(link.get(name) for name in ("from", "to", "fromLane", "toLane"))
        target = f"{link.get('to')}_{link.get('toLane')}"
        previous = f"{link.get('from')}_{link.get('fromLane')}"
        following = link.get("via") or target
        visited = set()
        maximum = 0.0
        while True:
            if previous not in shapes or following not in shapes:
                raise ValueError(
                    f"SUMO connection references a missing lane: {previous}, {following}"
                )
            maximum = max(
                maximum, float(np.linalg.norm(shapes[previous][-1] - shapes[following][0]))
            )
            if following == target:
                break
            if following in visited:
                raise ValueError(f"Cyclic internal SUMO connection chain: {key}")
            visited.add(following)
            successors = outgoing.get(following, [])
            if len(successors) != 1:
                raise ValueError(
                    f"Ambiguous or incomplete internal SUMO connection chain: {following}"
                )
            previous = following
            following = (
                successors[0].get("via")
                or f"{successors[0].get('to')}_{successors[0].get('toLane')}"
            )
        if maximum > 1e-8:
            gaps.append((key, maximum))
    return sorted(gaps)


def _stitch_small_connection_gaps(network_path, maximum_gap=0.10):
    """Retain mapped turn interiors and join small endpoint gaps exactly.

    This threshold chooses a repair method, never an acceptance tolerance.
    Every final connection still has to satisfy the 1e-8m continuity check.
    External road geometry and all logical/control attributes remain intact.
    """
    tree = ET.parse(network_path)
    root = tree.getroot()
    lanes = {lane.get("id"): lane for lane in root.findall("./edge/lane")}
    shapes = {
        identifier: np.asarray(
            [[float(value) for value in point.split(",")] for point in lane.get("shape").split()]
        )
        for identifier, lane in lanes.items()
    }
    changed = set()
    edits = []
    for connection in root.findall("connection"):
        previous = f"{connection.get('from')}_{connection.get('fromLane')}"
        following = connection.get("via") or f"{connection.get('to')}_{connection.get('toLane')}"
        gap = float(np.linalg.norm(shapes[previous][-1] - shapes[following][0]))
        if gap <= 1e-8 or gap > maximum_gap:
            continue
        if following.startswith(":"):
            identifier, endpoint, target = following, 0, shapes[previous][-1]
        elif previous.startswith(":"):
            identifier, endpoint, target = previous, -1, shapes[following][0]
        else:
            # Joining two external roads would alter the source map; let the
            # final continuity check reject that unsupported conversion.
            continue
        shapes[identifier][endpoint] = target.copy()
        changed.add(identifier)
        edits.append({"lane": identifier, "endpoint": endpoint, "gap_m": gap})
    for identifier in sorted(changed):
        coordinates = shapes[identifier]
        lane = lanes[identifier]
        lane.set(
            "shape", " ".join(",".join(f"{value:.12f}" for value in point) for point in coordinates)
        )
        geometric_length = float(np.sum(np.linalg.norm(np.diff(coordinates, axis=0), axis=1)))
        # SUMO represents coincident junction points using a short declared
        # lane length. Preserve that convention for a zero-length geometry.
        lane.set("length", f"{max(geometric_length, 0.01):.12f}")
    if changed:
        tree.write(network_path, encoding="utf-8", xml_declaration=True)
    return edits
