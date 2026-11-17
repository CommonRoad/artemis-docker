"""Match every guide state to a forward, connected, class-permitted SUMO path."""

from collections import defaultdict, deque
from dataclasses import dataclass
import math

from shapely.geometry import LineString, Point
from shapely.strtree import STRtree


@dataclass(frozen=True)
class MatchedRoute:
    edges: tuple[str, ...]
    node_path: tuple[str, ...]
    starts_inside_junction: bool
    ends_inside_junction: bool
    samples: int
    maximum_distance_m: float
    initial_heading_error_rad: float


class FullGuideMapMatcher:
    """Use full-trace costs; branch decisions remain open until later evidence.

    Normal edges group their permitted lanes, allowing ordinary lane changes.
    Internal lanes remain separate to retain each legal junction movement.
    Every state contributes a distance and heading cost. An unmatched state or
    disconnected guide fails explicitly rather than silently shortening a route.
    """

    def __init__(self, network, vehicle_class: str, delta: float):
        self.delta = delta
        self.normal = set()
        self.graph = defaultdict(set)
        self.reverse = defaultdict(set)
        self.keys, self.lines = [], []
        allowed_lanes = {
            lane.getID(): lane
            for edge in network.getEdges()
            for lane in edge.getLanes()
            if lane.allows(vehicle_class)
        }
        for identifier, lane in sorted(allowed_lanes.items()):
            edge = lane.getEdge()
            key = identifier if edge.isSpecial() else edge.getID()
            self.keys.append(key)
            self.lines.append(LineString(lane.getShape()))
            if not edge.isSpecial():
                self.normal.add(key)
        known = set(self.keys)
        for identifier, lane in sorted(allowed_lanes.items()):
            source = identifier if lane.getEdge().isSpecial() else lane.getEdge().getID()
            for connection in lane.getOutgoing():
                target_lane = connection.getToLane()
                via = connection.getViaLaneID()
                if target_lane.getID() not in allowed_lanes or (via and via not in allowed_lanes):
                    continue
                target = via or (
                    target_lane.getID()
                    if target_lane.getEdge().isSpecial()
                    else target_lane.getEdge().getID()
                )
                if target in known:
                    self.graph[source].add(target)
                    self.reverse[target].add(source)
        self.tree = STRtree(self.lines)

    def _candidate_cost(self, distance, heading):
        return distance**2 + self.delta**2 * (1 - math.cos(heading))

    def _candidates(self, state):
        point, candidates = Point(state.position), {}
        for index in sorted(self.tree.query(point, predicate="dwithin", distance=self.delta)):
            line, key = self.lines[index], self.keys[index]
            distance, offset = line.distance(point), line.project(point)
            before = line.interpolate(max(0, offset - 0.1))
            after = line.interpolate(min(line.length, offset + 0.1))
            angle = math.atan2(after.y - before.y, after.x - before.x) - state.orientation
            heading = abs(math.atan2(math.sin(angle), math.cos(angle)))
            cost = self._candidate_cost(distance, heading)
            value = (cost, distance, heading, offset)
            if key not in candidates or value < candidates[key]:
                candidates[key] = value
        return candidates

    def _junction_end(self, start, reverse=False):
        graph = self.reverse if reverse else self.graph
        queue, seen, options = deque([(start, ())]), set(), []
        while queue:
            node, path = queue.popleft()
            if node in seen:
                continue
            seen.add(node)
            for neighbor in sorted(graph[node]):
                extension = (neighbor,) + path if reverse else path + (neighbor,)
                if neighbor in self.normal:
                    options.append(extension)
                else:
                    queue.append((neighbor, extension))
        if not options:
            raise ValueError("Internal guide endpoint has no class-permitted adjoining road")
        return min(options, key=lambda path: (len(path), path))

    def permits_connection(self, source: str, target: str) -> bool:
        """Check a normal-edge movement through permitted internal lanes."""
        queue, seen = deque(sorted(self.graph[source])), set()
        while queue:
            node = queue.popleft()
            if node == target:
                return True
            if node in seen or node in self.normal:
                continue
            seen.add(node)
            queue.extend(sorted(self.graph[node]))
        return False

    def match(self, states) -> MatchedRoute:
        # Values are (full cost, node path, offset, initial heading, maximum distance).
        paths = {
            node: (cost, (node,), offset, heading, distance)
            for node, (cost, distance, heading, offset) in self._candidates(states[0]).items()
            if heading < math.pi / 2
        }
        if not paths:
            raise ValueError("No class-permitted forward route at the initial guide state")
        for index, state in enumerate(states[1:], 1):
            candidates, following = self._candidates(state), {}
            for previous, (cost, path, last_offset, initial_heading, maximum) in paths.items():
                # A dense sample may cross several short internal lane segments.
                # Traverse only nodes close to that sample, retaining connectivity.
                extensions, queue = {previous: ()}, deque([(previous, ())])
                while queue:
                    node, extension = queue.popleft()
                    for target in sorted(self.graph[node]):
                        if target in extensions or target not in candidates:
                            continue
                        extension2 = extension + (target,)
                        extensions[target] = extension2
                        queue.append((target, extension2))
                for target, extension in extensions.items():
                    if target not in candidates:
                        continue
                    emission, distance, heading, offset = candidates[target]
                    if heading >= math.pi / 2:
                        continue
                    backward = max(0, last_offset - offset) if target == previous else 0
                    value = (
                        cost + emission + backward**2,
                        path + extension,
                        offset,
                        initial_heading,
                        max(maximum, distance),
                    )
                    if target not in following or value[:2] < following[target][:2]:
                        following[target] = value
            if not following:
                raise ValueError(f"No connected forward route at guide state {index}")
            paths = following
        _, path, _, heading, maximum = min(paths.values(), key=lambda value: value[:2])
        starts_internal, ends_internal = path[0] not in self.normal, path[-1] not in self.normal
        if starts_internal:
            path = self._junction_end(path[0], reverse=True) + path
        if ends_internal:
            path += self._junction_end(path[-1])
        return MatchedRoute(
            tuple(node for node in path if node in self.normal),
            path,
            starts_internal,
            ends_internal,
            len(states),
            maximum,
            heading,
        )
