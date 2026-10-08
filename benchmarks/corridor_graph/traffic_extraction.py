"""Robot-independent profile intervals and separate local junction regions."""

from dataclasses import asdict, dataclass
import math

import numpy as np
from shapely.geometry import LineString, Point, Polygon

from .geometry import FreeSpace
from .traffic_model import ResourceInterval, ResourcePort, TrafficResource, TrafficResourceGraph
from .width_audit import WidthAuditor, continuous_minimum
from .geometric_intervals import WidthClassIntervals
from .traffic_geometry import (cache_routing_geometry, detailed_intervals, geometry_hash,
                               resource_relationships)


@dataclass(frozen=True)
class TrafficConfig:
    narrow_width: float = 3.0  # illustrative map-unit class, not a robot diameter
    detect_narrow: bool = True
    detect_junctions: bool = True
    junction_max_radius: float = 1.5
    junction_clearance_fraction: float = .75
    junction_min_angle_degrees: float = 15.0
    tolerance: float = 1e-7
    audit_tolerance: float = 1e-5
    width_hysteresis: float = 1e-4  # map-unit class envelope; does not smooth or clamp clearance

    def __post_init__(self):
        for name in ("narrow_width", "junction_max_radius", "tolerance", "audit_tolerance"):
            if not math.isfinite(getattr(self, name)) or getattr(self, name) <= 0:
                raise ValueError(f"{name} must be finite and positive")
        if not 0 < self.junction_clearance_fraction < 1:
            raise ValueError("junction_clearance_fraction must lie strictly between zero and one")
        if not 0 < self.junction_min_angle_degrees < 180:
            raise ValueError("junction_min_angle_degrees must lie between zero and 180")
        if not math.isfinite(self.width_hysteresis) or self.width_hysteresis < 0:
            raise ValueError("width_hysteresis must be finite and nonnegative")


def reference_profile(edge, detailed):
    points, widths = [], []
    for span in edge.source_spans:
        samples = [detailed.edges[span.edge_id].width_profile[i] for i in span.indices()]
        if points:
            if points[-1] != samples[0].position:
                raise ValueError("discontinuous reference spans")
            samples = samples[1:]
        points.extend(s.position for s in samples)
        widths.extend(s.width for s in samples)
    distances = np.r_[0., np.cumsum(np.linalg.norm(np.diff(points, axis=0), axis=1))]
    if not math.isclose(distances[-1], edge.length, abs_tol=1e-7, rel_tol=1e-9):
        raise ValueError("routing length disagrees with reference geometry")
    return distances, np.asarray(points), np.asarray(widths)


def slice_profile(profile, start, end):
    distances, points, _ = profile
    def at(distance):
        return tuple(float(np.interp(distance, distances, points[:, axis])) for axis in (0, 1))
    inside = points[(distances > start + 1e-10) & (distances < end - 1e-10)]
    return (at(start), *map(tuple, inside), at(end))


def _port(edge, distance, position, tolerance):
    node = edge.u if abs(distance) <= tolerance else edge.v if abs(distance - edge.length) <= tolerance else None
    return ResourcePort(edge.id, float(distance), tuple(map(float, position)), node,
                        boundary_kind="routing_node" if node is not None else "routing_edge_cut")


def _weighted_median(parts, space):
    values, weights = [], []
    for points in parts:
        points = np.asarray(points)
        lengths = np.linalg.norm(np.diff(points, axis=0), axis=1)
        values.extend(2 * space.clearances((points[:-1] + points[1:]) / 2))
        weights.extend(lengths)
    order = np.argsort(values)
    cumulative = np.cumsum(np.asarray(weights)[order])
    return float(np.asarray(values)[order][np.searchsorted(cumulative, cumulative[-1] / 2)])


def _make_resource(resource_id, kind, parts, intervals, ports, detailed, routing, space, auditor,
                   region=(), closed=False, junction_node_id=None):
    minimum, position = min((continuous_minimum(part, space) for part in parts), key=lambda v: v[0])
    edge_ids = tuple(sorted({i.edge_id for i in intervals}))
    nodes = tuple(sorted({p.node_id for i in intervals for p in (i.start_port, i.end_port) if p.node_id is not None}))
    detailed_nodes = tuple(sorted({s for n in nodes for s in routing.nodes[n].source_node_ids}))
    detailed_edges = tuple(sorted({s.edge_id for i in intervals for s in i.source_intervals}))
    trim = detailed.metadata.get("effective_boundary_trim") or detailed.metadata.get("geometry_config", {}).get("boundary_trim", .0001)
    terminations = tuple({"routing_node_id": n, **auditor.boundary_limit(routing.nodes[n].position, trim)}
                         for n in nodes if routing.nodes[n].kind == "terminal" and
                         abs(routing.nodes[n].local_width / 2 - trim) <= max(auditor.precision, trim * 1e-4))
    classification = "junction_candidate" if kind == "junction_region" else "constrained_passage"
    if kind == "narrow_corridor" and terminations:
        touching = any(p["relation"] == "touching_boundaries" for t in terminations for p in t["feature_pairs"])
        classification = "boundary_taper" if touching else "trimmed_boundary_endpoint"
    locations = [position]
    locations.extend(routing.nodes[n].position for n in nodes if routing.nodes[n].kind == "bottleneck")
    witnesses = tuple(auditor.witness(p) for p in dict.fromkeys(locations))
    representative = _weighted_median(parts, space)
    connections = tuple((i, j) for i in range(len(ports)) for j in range(len(ports))) if region else ()
    return TrafficResource(id=resource_id, kind=kind, source_node_ids=nodes, source_edge_ids=edge_ids,
                           detailed_node_ids=detailed_nodes, detailed_edge_ids=detailed_edges, geometry=tuple(parts),
                           region=tuple(region), length=float(sum(LineString(p).length for p in parts)),
                           min_width=float(minimum), min_width_position=position, representative_width=representative,
                           entrances=tuple(ports), exits=tuple(ports), min_clearance=float(minimum / 2),
                           representative_clearance=representative / 2, geometric_classification=classification,
                           local_minima=witnesses, boundary_terminations=terminations, possible_connections=connections,
                           junction_node_id=junction_node_id, closed=closed, width_witness=auditor.witness(position))


def _prefix_inside_region(points, region):
    """First connected arm clipped to the actual stored polygon, not its circle."""
    length = 0.
    for a, b in zip(points[:-1], points[1:]):
        segment_length = np.linalg.norm(b - a)
        if not region.covers(Point(b)):
            chord = LineString([a, b])
            clipped = chord.intersection(region)
            if clipped.is_empty:
                return float(length)
            coordinates = list(clipped.coords)
            return float(length + max(chord.project(Point(p)) for p in coordinates))
        length += segment_length
    return float(length)


def extract_traffic_resources(detailed, routing, world, config=None, validate=True):
    """Overlay resources without modifying either graph or its connectivity."""
    config = config or TrafficConfig()
    space = world if isinstance(world, FreeSpace) else FreeSpace(world)
    auditor = WidthAuditor(space, config.audit_tolerance)
    detector = WidthClassIntervals(auditor)
    profiles = {e.id: reference_profile(e, detailed) for e in routing.edges.values()}
    traffic = TrafficResourceGraph(edge_intervals={e: () for e in routing.edges},
                                   metadata={"units": "map units", "config": asdict(config),
                                             "width_metric": "continuous reference clearance diameter",
                                             "junction_exclusions": [], "schema_version": 2,
                                             "stage": "offline_robot_independent_geometry",
                                             "map_sha256": geometry_hash(space.world),
                                             "routing_sha256": geometry_hash(routing.to_dict()),
                                             "detailed_sha256": geometry_hash(detailed.to_dict()),
                                             "width_class_envelope": config.narrow_width + config.width_hysteresis},
                                   map_geometry=space.world, boundary_features=tuple(auditor.features),
                                   routing_geometry=cache_routing_geometry(detailed, routing, profiles, auditor))
    adjacency = routing.adjacency()
    pieces = []
    if config.detect_narrow:
        for edge in routing.edges.values():
            for start, end in detector.extract(profiles[edge.id], config.narrow_width + config.width_hysteresis):
                geometry = slice_profile(profiles[edge.id], start, end)
                pieces.append(ResourceInterval(-1, edge.id, start, end, 1, geometry,
                                               _port(edge, start, geometry[0], config.tolerance),
                                               _port(edge, end, geometry[-1], config.tolerance),
                                               detailed_intervals(edge, detailed, start, end)))
    # Build interval incidence. Only two-arm routing nodes may couple regions.
    endpoints = {}
    for i, piece in enumerate(pieces):
        for end, port in enumerate((piece.start_port, piece.end_port)):
            if port.node_id is not None:
                endpoints.setdefault(port.node_id, []).append((i, end))
    neighbors = {}
    pure_cycles = set()
    for component in routing.connected_components():
        if component and all(len(adjacency[n]) == 2 for n in component):
            pure_cycles.update(component)
    for node, ends in endpoints.items():
        if len(adjacency[node]) != 2 or len(ends) != 2 or node in pure_cycles:
            continue
        (a, ea), (b, eb) = ends
        if a == b:
            continue
        ra, rb = (routing.edges[pieces[k].edge_id] for k in (a, b))
        if ra.id != rb.id and {ra.u, ra.v} == {rb.u, rb.v}:
            continue  # explicit parallel alternatives, even in a small component
        neighbors[a, ea], neighbors[b, eb] = (b, eb), (a, ea)
    unused = set(range(len(pieces)))
    from dataclasses import replace
    while unused:
        seed = min(unused)
        # Find one endpoint of this nonbranching chain.
        index, entrance, visited = seed, 0, set()
        while (index, entrance) in neighbors and index not in visited:
            visited.add(index)
            index, other_end = neighbors[index, entrance]
            entrance = 1 - other_end
        chain, merged = [], []
        resource_id = len(traffic.resources)
        while index in unused:
            unused.remove(index)
            piece = pieces[index]
            direction = 1 if entrance == 0 else -1
            oriented = piece.geometry if direction == 1 else tuple(reversed(piece.geometry))
            if chain:
                assert math.dist(chain[-1], oriented[0]) <= config.tolerance
                chain.extend(oriented[1:])
            else:
                chain.extend(oriented)
                first_port = piece.start_port if direction == 1 else piece.end_port
            merged.append(replace(piece, resource_id=resource_id, direction=direction))
            last_port = piece.end_port if direction == 1 else piece.start_port
            exit_end = 1 - entrance
            if (index, exit_end) not in neighbors:
                break
            index, entrance = neighbors[index, exit_end]
        closed = math.dist(chain[0], chain[-1]) <= config.tolerance
        ports = (first_port, last_port)  # closed cycles retain an explicit traversal seam
        resource = _make_resource(resource_id, "narrow_corridor", [tuple(chain)], merged, ports,
                                  detailed, routing, space, auditor, closed=closed)
        if resource.min_width >= config.narrow_width - auditor.precision:
            continue  # class envelope alone cannot introduce a constrained passage
        traffic.resources[resource_id] = resource
        for interval in merged:
            traffic.edge_intervals[interval.edge_id] += (interval,)
    if config.detect_junctions:
        for node in routing.nodes.values():
            if len(adjacency[node.id]) < 3:
                continue
            clearance = float(space.clearances([node.position])[0])
            radius = min(config.junction_max_radius, config.junction_clearance_fraction * clearance)
            polygon = Point(node.position).buffer(radius, quad_segs=16)
            arms, vectors, ports = [], [], []
            for edge_id in sorted({a[1] for a in adjacency[node.id]}):
                edge = routing.edges[edge_id]
                profile = profiles[edge_id]
                # A self-loop contributes two distinct arms.
                for reverse in ([False, True] if edge.u == edge.v == node.id else [edge.v == node.id]):
                    points = profile[1][::-1] if reverse else profile[1]
                    length = _prefix_inside_region(points, polygon)
                    if edge.u == edge.v:
                        length = min(length, edge.length / 2)  # arms partition a small loop; never count it twice
                    if length <= config.tolerance:
                        continue
                    start, end = (edge.length - length, edge.length) if reverse else (0., length)
                    geometry = slice_profile(profile, start, end)
                    first = _port(edge, start, geometry[0], config.tolerance)
                    last = _port(edge, end, geometry[-1], config.tolerance)
                    port = first if reverse else last
                    boundary_kind = ("junction_boundary" if polygon.boundary.distance(Point(port.position)) <= auditor.precision * 8
                                     else "internal_connection")
                    if reverse:
                        first = replace(first, arm="v", boundary_kind=boundary_kind)
                    else:
                        last = replace(last, arm="u", boundary_kind=boundary_kind)
                    port = first if reverse else last
                    vector = np.asarray(port.position) - node.position
                    if np.linalg.norm(vector) <= config.tolerance:
                        vector = max(points - node.position, key=np.linalg.norm)
                    if np.linalg.norm(vector) <= config.tolerance:
                        continue
                    vectors.append(vector / np.linalg.norm(vector))
                    ports.append(port)
                    arms.append(ResourceInterval(-1, edge_id, start, end, 0, geometry, first, last,
                                                 detailed_intervals(edge, detailed, start, end)))
            separated = any(np.dot(a, b) < math.cos(math.radians(config.junction_min_angle_degrees))
                            for i, a in enumerate(vectors) for b in vectors[:i])
            if len(arms) < 3 or not separated:
                traffic.metadata["junction_exclusions"].append({"node_id": node.id, "reason": "insufficient_distinct_geometric_arms"})
                continue
            resource_id = len(traffic.resources)
            arms = [replace(i, resource_id=resource_id) for i in arms]
            region = tuple(polygon.exterior.coords)
            resource = _make_resource(resource_id, "junction_region", [a.geometry for a in arms], arms,
                                      ports, detailed, routing, space, auditor, region=region, junction_node_id=node.id)
            traffic.resources[resource_id] = resource
            for arm in arms:
                traffic.edge_intervals[arm.edge_id] += (arm,)
    traffic.edge_intervals = {k: tuple(sorted(v, key=lambda i: (i.start, i.end, i.resource_id)))
                              for k, v in traffic.edge_intervals.items()}
    traffic.relationships = resource_relationships(traffic)
    traffic.metadata["geometry_cache_sha256"] = geometry_hash({"map_geometry": traffic.map_geometry,
                                                               "boundary_features": traffic.boundary_features,
                                                               "routing_geometry": traffic.routing_geometry})
    if validate:
        validate_traffic_resources(traffic, detailed, routing, space, config)
    return traffic


def validate_traffic_resources(traffic, detailed, routing, space, config=None):
    """Validate full chords, metrics, reversible incidence and source provenance."""
    config = config or TrafficConfig(**traffic.metadata.get("config", {}))
    assert set(traffic.edge_intervals) == set(routing.edges)
    precision = WidthAuditor(space, config.audit_tolerance).precision
    usage = {}
    for edge_id, intervals in traffic.edge_intervals.items():
        edge = routing.edges[edge_id]
        profile = reference_profile(edge, detailed)
        for interval in intervals:
            assert interval.edge_id == edge_id and interval.resource_id in traffic.resources
            assert -config.tolerance <= interval.start < interval.end <= edge.length + config.tolerance
            assert interval.geometry == slice_profile(profile, interval.start, interval.end)
            assert interval.source_intervals == detailed_intervals(edge, detailed, interval.start, interval.end)
            assert space.line_is_free(interval.geometry)
            assert interval.start_port.position == interval.geometry[0]
            assert interval.end_port.position == interval.geometry[-1]
            for port, distance in ((interval.start_port, interval.start), (interval.end_port, interval.end)):
                assert port.edge_id == edge_id and port.distance == distance
                if port.node_id is not None:
                    assert math.dist(port.position, routing.nodes[port.node_id].position) <= config.tolerance
            usage.setdefault(interval.resource_id, []).append(interval)
        forward = traffic.directed_mapping(edge_id, 1)
        backward = traffic.directed_mapping(edge_id, -1)
        assert set(backward) == {(r, end, start, -d) for r, start, end, d in forward}
        assert [p.distance for _, p, _, _ in backward] == sorted((i.end for i in intervals), reverse=True)
    for resource in traffic.resources.values():
        intervals = usage[resource.id]
        assert set(resource.source_edge_ids) == {i.edge_id for i in intervals}
        assert set(resource.detailed_edge_ids) == {s.edge_id for i in intervals for s in i.source_intervals}
        assert resource.entrances == resource.exits and resource.entrances
        for port in resource.entrances:
            assert any(port == p for i in intervals for p in (i.start_port, i.end_port))
        assert all(space.line_is_free(part) for part in resource.geometry)
        if resource.region:
            assert space.geometry.contains(Polygon(resource.region))
            polygon = Polygon(resource.region).buffer(8 * precision)
            assert all(polygon.covers(LineString(p)) for p in resource.geometry)
        minimum = min(continuous_minimum(part, space)[0] for part in resource.geometry)
        assert abs(minimum - resource.min_width) <= config.tolerance
        assert resource.min_clearance == resource.min_width / 2
        assert resource.representative_clearance == resource.representative_width / 2
        assert math.isclose(resource.length, sum(i.end - i.start for i in intervals), abs_tol=config.tolerance)
        assert math.isclose(resource.length, sum(LineString(p).length for p in resource.geometry), abs_tol=config.tolerance)
        if resource.kind == "narrow_corridor":
            assert resource.min_width < config.narrow_width
            internal = set(resource.source_node_ids) - {p.node_id for p in resource.entrances}
            assert all(len(routing.adjacency()[n]) == 2 for n in internal)
            remaining, chain = list(intervals), []
            current = resource.entrances[0].position
            while remaining:
                choices = [(i, p.geometry if p.direction == 1 else tuple(reversed(p.geometry)))
                           for i, p in enumerate(remaining)]
                choices = [(i, g) for i, g in choices if math.dist(g[0], current) <= config.tolerance]
                assert len(choices) == 1, "narrow resource has ambiguous or disconnected oriented provenance"
                index, geometry = choices[0]
                remaining.pop(index)
                chain.extend(geometry[1:] if chain else geometry)
                current = geometry[-1]
            assert tuple(chain) == resource.geometry[0]
            assert resource.entrances[0].position == chain[0]
            assert resource.entrances[-1].position == chain[-1]
        else:
            assert sorted(resource.geometry) == sorted(i.geometry for i in intervals)
            assert resource.kind == "junction_region"
            assert resource.junction_node_id in routing.nodes
            assert set(resource.possible_connections) == {(i, j) for i in range(len(resource.entrances))
                                                          for j in range(len(resource.exits))}
    assert traffic.metadata["routing_sha256"] == geometry_hash(routing.to_dict())
    assert traffic.metadata["detailed_sha256"] == geometry_hash(detailed.to_dict())
    assert traffic.metadata["map_sha256"] == geometry_hash(space.world)
    assert geometry_hash(traffic.relationships) == geometry_hash(resource_relationships(traffic))
