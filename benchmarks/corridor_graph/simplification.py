"""Conservative topology reductions, followed by independent geometry reduction."""

from dataclasses import asdict, dataclass, replace
import math

import numpy as np
from shapely.geometry import LineString, Polygon

from .bottlenecks import BottleneckConfig, detect_bottlenecks
from .geometry import FreeSpace
from .routing_model import (RoutingCorridorGraph, RoutingEdge, RoutingNode, SourceSpan,
                            geometry_from_spans, metrics_from_spans)


@dataclass(frozen=True)
class SimplificationConfig:
    prune_branches: bool = True
    branch_max_length: float = 5.0
    branch_length_clearance_ratio: float = 1.5
    branch_region_growth_ratio: float = 1.10
    branch_tip_width_ratio: float = 0.15
    junction_max_extent: float = 1.0
    junction_extent_clearance_ratio: float = 1.0
    junction_max_width_ratio: float = 1.5
    width_transition_ratio: float = 2.0
    width_transition_window: float = 1.0
    geometry_tolerance: float = 0.05
    protected_node_ids: tuple[int, ...] = ()
    validation_pairs: int = 200
    validation_seed: int = 42
    distortion_relative_warning: float = 0.25
    distortion_absolute_warning: float = 0.5

    def __post_init__(self):
        object.__setattr__(self, "protected_node_ids", tuple(self.protected_node_ids))
        if any(not isinstance(node, int) or node < 0 for node in self.protected_node_ids):
            raise ValueError("protected_node_ids must contain nonnegative integers")
        for name in ("branch_max_length", "branch_length_clearance_ratio",
                     "junction_max_extent", "junction_extent_clearance_ratio",
                     "width_transition_window", "geometry_tolerance",
                     "distortion_relative_warning", "distortion_absolute_warning"):
            value = getattr(self, name)
            if not math.isfinite(value) or value < 0:
                raise ValueError(f"{name} must be finite and nonnegative")
        for name in ("branch_region_growth_ratio", "junction_max_width_ratio", "width_transition_ratio"):
            if not math.isfinite(getattr(self, name)) or getattr(self, name) < 1:
                raise ValueError(f"{name} must be finite and at least one")
        if not 0 <= self.branch_tip_width_ratio <= 1:
            raise ValueError("branch_tip_width_ratio must be in [0,1]")
        if not isinstance(self.validation_pairs, int) or self.validation_pairs < 0:
            raise ValueError("validation_pairs must be a nonnegative integer")
        if not isinstance(self.validation_seed, int) or self.validation_seed < 0:
            raise ValueError("validation_seed must be a nonnegative integer")


@dataclass(frozen=True)
class _WorkingEdge:
    id: int
    u: int
    v: int
    spans: tuple[SourceSpan, ...]


def _adjacency(nodes, edges):
    result = {node: [] for node in nodes}
    for edge in edges.values():
        result[edge.u].append(edge.id)
        result[edge.v].append(edge.id)
    return result


def _oriented(edge, start):
    if start == edge.u:
        return edge.spans, edge.v
    return tuple(span.reversed() for span in edge.spans[::-1]), edge.u


def _working_graph(detailed, config):
    """Also normalize mark-only bottlenecks into protected nodes, without ECM edits."""
    nodes = {n.id: RoutingNode(n.id, n.position, n.kind, (n.id,), protected_reasons=
                              ("bottleneck",) if n.kind == "bottleneck" else ())
             for n in detailed.nodes.values()}
    unknown = set(config.protected_node_ids) - nodes.keys()
    if unknown:
        raise ValueError(f"unknown protected detailed nodes: {sorted(unknown)}")
    for node in config.protected_node_ids:
        value = nodes[node]
        nodes[node] = replace(value, kind=value.kind if value.kind == "bottleneck" else "region",
                              protected_reasons=tuple(dict.fromkeys((*value.protected_reasons, "access"))))
    edges, next_node = {}, max(nodes, default=-1) + 1
    detection = replace(BottleneckConfig(**detailed.metadata.get("bottleneck_config", {})), enabled=True)
    for original in detailed.edges.values():
        cuts = {0: original.u, len(original.geometry) - 1: original.v}
        minima = detect_bottlenecks(original, detection)
        positions = tuple(dict.fromkeys((*original.bottleneck_positions,
                                        *(original.geometry[i] for i in minima))))
        for position in positions:
            indices = [i for i, point in enumerate(original.geometry) if point == position]
            if not indices:
                raise ValueError(f"bottleneck on edge {original.id} has no geometry sample")
            index = indices[0]
            if index not in cuts:
                cuts[index] = next_node
                nodes[next_node] = RoutingNode(next_node, position, "bottleneck", (),
                                               ((original.id, index),), ("bottleneck",))
                next_node += 1
            else:
                node = cuts[index]
                reasons = tuple(dict.fromkeys((*nodes[node].protected_reasons, "bottleneck")))
                nodes[node] = replace(nodes[node], kind="bottleneck", protected_reasons=reasons)
        indices = sorted(cuts)
        for first, last in zip(indices[:-1], indices[1:]):
            edge_id = len(edges)
            edges[edge_id] = _WorkingEdge(edge_id, cuts[first], cuts[last],
                                         (SourceSpan(original.id, first, last),))
    # A large change in representative width across a degree-two node is an
    # explicit region transition, even when it is not a local minimum.
    adjacency = _adjacency(nodes, edges)
    for node, incident in adjacency.items():
        if len(incident) != 2 or nodes[node].protected_reasons:
            continue
        widths = []
        for edge_id in incident:
            spans, _ = _oriented(edges[edge_id], node)
            samples, distance = [], 0.
            for span in spans:
                profile = detailed.edges[span.edge_id].width_profile
                previous = None
                for index in span.indices():
                    sample = profile[index]
                    if previous is not None:
                        distance += abs(sample.distance - previous.distance)
                    if distance <= config.width_transition_window:
                        samples.append(sample.width)
                    previous = sample
            widths.append(float(np.median(samples)))
        if min(widths) > 0 and max(widths) / min(widths) >= config.width_transition_ratio:
            nodes[node] = replace(nodes[node], kind="transition", protected_reasons=("width_transition",))
    return nodes, edges


def _prune_branches(nodes, edges, detailed, space, config):
    """Only prune fading boundary tips in a bounded, directly visible local region.

    Never prune a protected path, a clearance-growing room, or a positive-width
    terminal. Record everything removed and its attachment for off-graph access.
    """
    records = []
    if not config.prune_branches:
        return records
    while True:
        adjacency = _adjacency(nodes, edges)
        pending, scheduled_nodes, attachments = [], set(), set()
        for leaf in sorted(nodes):
            if len(adjacency[leaf]) != 1 or nodes[leaf].protected_reasons:
                continue
            current, previous, branch_nodes, branch_edges, spans = leaf, None, [leaf], [], []
            while True:
                candidates = [i for i in adjacency[current] if i != previous]
                if len(candidates) != 1:
                    break
                edge_id = candidates[0]
                edge_spans, other = _oriented(edges[edge_id], current)
                branch_edges.append(edge_id)
                spans.extend(edge_spans)
                previous, current = edge_id, other
                if len(adjacency[current]) != 2 or nodes[current].protected_reasons:
                    break
                branch_nodes.append(current)
            if current == leaf or nodes[current].protected_reasons:
                continue
            if set(branch_nodes) & (scheduled_nodes | attachments) or current in scheduled_nodes:
                continue
            points = geometry_from_spans(spans, detailed)
            length, _, _ = metrics_from_spans(spans, detailed)
            clearance = float(space.clearances([nodes[current].position])[0])
            if clearance <= 0 or length > config.branch_max_length:
                continue
            widths = space.clearances(points) * 2
            if (length > config.branch_length_clearance_ratio * clearance
                    or widths.max() > 2 * clearance * config.branch_region_growth_ratio
                    or widths[0] > 2 * clearance * config.branch_tip_width_ratio):
                continue
            # The attachment sees the entire branch. A wall or a turn into
            # another room prevents pruning even if the branch is short.
            attachment = nodes[current].position
            if not all(point == attachment or space.line_is_free([attachment, point]) for point in points):
                continue
            # Certify visibility between samples too: a thin obstacle between
            # two individually clear rays must not hide a meaningful branch.
            fans = [Polygon([attachment, a, b]) for a, b in zip(points[:-1], points[1:])]
            if any(fan.area > 0 and (not space.workspace.covers(fan)
                   or space.obstacles.intersects(fan) or space.workspace.boundary.intersects(fan))
                   for fan in fans):
                continue
            record = {"attachment": current, "node_ids": branch_nodes,
                      "spans": [asdict(span) for span in spans], "length": length,
                      "attachment_clearance": clearance}
            pending.append((record, branch_edges))
            scheduled_nodes.update(branch_nodes)
            attachments.add(current)
        if not pending:
            return records
        # Evaluate disjoint tips against the SAME attachment graph. Removing
        # one tip must not turn its equally insignificant sibling into a long
        # composite branch with an arbitrary terminal corner.
        for record, branch_edges in pending:
            records.append(record)
            for edge_id in branch_edges:
                del edges[edge_id]
            for node in record["node_ids"]:
                del nodes[node]


def _junction_clusters(nodes, edges, detailed, space, config):
    """Merge only connected, induced trees inside a small unobstructed region.

    No cycles or protected vertices/edges can be swallowed. Complete cluster
    diameter, local clearance and width range constrain transitive merging.
    """
    adjacency = _adjacency(nodes, edges)
    eligible = {node for node in nodes if len(adjacency[node]) >= 3
                and not nodes[node].protected_reasons}
    membership = {node: node for node in nodes}
    groups = {node: {node} for node in nodes}
    node_clearance = {node: float(space.clearances([n.position])[0]) for node, n in nodes.items()}
    candidates = sorted(edges.values(), key=lambda e: (metrics_from_spans(e.spans, detailed)[0], e.id))
    for edge in candidates:
        if edge.u not in eligible or edge.v not in eligible:
            continue
        a, b = membership[edge.u], membership[edge.v]
        if a == b:
            continue
        group = groups[a] | groups[b]
        internal = [e for e in edges.values() if e.u in group and e.v in group]
        if len(internal) != len(group) - 1:  # induced connected tree, including parallel edges
            continue
        positions = np.array([nodes[n].position for n in sorted(group)])
        diameter = float(np.linalg.norm(positions[:, None] - positions[None, :], axis=2).max())
        local_clearance = min(node_clearance[n] for n in group)
        if (diameter > config.junction_max_extent
                or diameter > config.junction_extent_clearance_ratio * local_clearance):
            continue
        profile_widths = [detailed.edges[s.edge_id].width_profile[i].width
                          for e in internal for s in e.spans for i in s.indices()]
        if min(profile_widths) <= 0 or max(profile_widths) / min(profile_widths) > config.junction_max_width_ratio:
            continue
        # All junctions must share a directly visible local decision region;
        # close nodes on opposite sides of an obstacle cannot be merged.
        if any(not space.line_is_free([nodes[x].position, nodes[y].position])
               for i, x in enumerate(sorted(group)) for y in sorted(group)[i + 1:]):
            continue
        root = min(a, b)
        groups[root] = group
        del groups[max(a, b)]
        for node in group:
            membership[node] = root
    return list(groups.values())


def _merge_clusters(nodes, edges, groups, detailed):
    new_nodes, membership, paths, records = {}, {}, {}, []
    for group in groups:
        internal = [e for e in edges.values() if e.u in group and e.v in group]
        adjacency = _adjacency({n: nodes[n] for n in group}, {e.id: e for e in internal})
        # A real original junction is the representative, avoiding arbitrary
        # centroids inside obstacles. Choose the least-total-tree-distance root.
        def tree_paths(root):
            found, stack = {root: ()}, [root]
            while stack:
                node = stack.pop()
                for edge_id in adjacency[node]:
                    spans, other = _oriented(edges[edge_id], node)
                    if other not in found:
                        found[other] = found[node] + spans
                        stack.append(other)
            return found
        options = [(sum(metrics_from_spans(s, detailed)[0] for s in tree_paths(root).values()), root)
                   for root in sorted(group)]
        _, anchor = min(options)
        paths.update(tree_paths(anchor))
        originals = tuple(sorted({source for n in group for source in nodes[n].source_node_ids}))
        new_nodes[anchor] = replace(nodes[anchor], source_node_ids=originals)
        for node in group:
            membership[node] = anchor
        if len(group) > 1:
            records.append({"routing_node": anchor, "working_nodes": sorted(group),
                            "internal_edge_ids": [e.id for e in internal]})
    new_edges = {}
    for edge in edges.values():
        u, v = membership[edge.u], membership[edge.v]
        if u == v and edge.u != edge.v:
            continue
        prefix, suffix = paths[edge.u], paths[edge.v]
        spans = prefix + edge.spans + tuple(s.reversed() for s in suffix[::-1])
        new_edges[edge.id] = _WorkingEdge(edge.id, u, v, spans)
    return new_nodes, new_edges, membership, records


def _collapse_chains(nodes, edges):
    adjacency = _adjacency(nodes, edges)
    significant = {n for n in nodes if len(adjacency[n]) != 2 or nodes[n].protected_reasons}
    visited, result_nodes, result_edges, node_map = set(), {}, {}, {}

    def walk(start, first):
        current, edge_id, spans, chain_nodes = start, first, [], [start]
        while True:
            visited.add(edge_id)
            oriented, other = _oriented(edges[edge_id], current)
            spans.extend(oriented)
            current = other
            if current in significant or current == start:
                break
            chain_nodes.append(current)
            edge_id = next(i for i in adjacency[current] if i != edge_id)
        result_nodes[start] = nodes[start]
        result_nodes[current] = nodes[current]
        node_map[start], node_map[current] = start, current
        # Geometric degree-two nodes remain recoverable from the edge provenance.
        for node in chain_nodes[1:]:
            node_map[node] = start
        edge_id = len(result_edges)
        result_edges[edge_id] = _WorkingEdge(edge_id, start, current, tuple(spans))

    for node in sorted(significant):
        if not adjacency[node]:
            result_nodes[node] = replace(nodes[node], kind="region")
            node_map[node] = node
        for edge_id in adjacency[node]:
            if edge_id not in visited:
                walk(node, edge_id)
    for edge in edges.values():
        if edge.id not in visited:
            anchor = min(edge.u, edge.v)
            nodes[anchor] = replace(nodes[anchor], kind="cycle_anchor")
            walk(anchor, edge.id)
    return result_nodes, result_edges, node_map


def simplify_polyline(points, space, tolerance, pinned=()):
    """Bounded-error DP with full chord collision, corridor and clearance checks.

    Topology has already been fixed. Pins (including minimum-width positions)
    cannot be removed. Each shortcut must stay in the tolerance tube of its
    ORIGINAL subchain and retain the reference walk's minimum continuous
    clearance. Thus it cannot jump between nearby obstacle-separated corridors.
    """
    points = tuple(points)
    if tolerance == 0 or len(points) <= 2:
        return points
    reference = LineString(points)
    minimum_clearance = reference.distance(space.boundary)
    pinned = set(pinned)
    cuts = sorted({0, len(points) - 1, *[i for i, p in enumerate(points) if p in pinned]})
    retained = set(cuts)
    stack = list(zip(cuts[:-1], cuts[1:]))
    while stack:
        first, last = stack.pop()
        if last <= first + 1:
            continue
        chord = LineString([points[first], points[last]])
        array = np.asarray(points[first:last + 1])
        delta = array[-1] - array[0]
        norm = float(delta @ delta)
        if norm == 0:
            split = (first + last) // 2
            valid = False
        else:
            projection = np.clip(((array - array[0]) @ delta) / norm, 0, 1)
            errors = np.linalg.norm(array - array[0] - projection[:, None] * delta, axis=1)
            split = first + int(np.argmax(errors))
            valid = (errors.max() <= tolerance
                     and space.line_is_free([points[first], points[last]])
                     and chord.distance(space.boundary) >= minimum_clearance - 1e-9
                     and LineString(array).buffer(tolerance).covers(chord))
        if not valid:
            if split in (first, last):
                split = (first + last) // 2
            retained.add(split)
            stack.extend([(first, split), (split, last)])
    return tuple(points[i] for i in sorted(retained))


def simplify_corridor_graph(detailed, world, config=None, validate=True):
    """Derive an independent routing graph; never mutate the detailed ECM."""
    config = config or SimplificationConfig()
    space = world if isinstance(world, FreeSpace) else FreeSpace(world)
    working_nodes, working_edges = _working_graph(detailed, config)
    # Keep the normalized comparison graph before any simplification.
    original_nodes, original_edges = dict(working_nodes), dict(working_edges)
    pruned = _prune_branches(working_nodes, working_edges, detailed, space, config)
    groups = _junction_clusters(working_nodes, working_edges, detailed, space, config)
    nodes, edges, membership, clusters = _merge_clusters(working_nodes, working_edges, groups, detailed)
    nodes, edges, collapsed_map = _collapse_chains(nodes, edges)
    graph = RoutingCorridorGraph(nodes=nodes)
    for edge in edges.values():
        reference = geometry_from_spans(edge.spans, detailed)
        length, width, location = metrics_from_spans(edge.spans, detailed)
        bottlenecks = tuple(n for n in dict.fromkeys((edge.u, edge.v)) if "bottleneck" in nodes[n].protected_reasons)
        pins = (location, *(nodes[n].position for n in bottlenecks))
        geometry = simplify_polyline(reference, space, config.geometry_tolerance, pins)
        graph.edges[edge.id] = RoutingEdge(edge.id, edge.u, edge.v, geometry, length, width,
                                          location, edge.spans, bottlenecks)
    adjacency = graph.adjacency()
    for node, value in list(graph.nodes.items()):
        value = replace(value, local_width=float(space.clearances([value.position])[0] * 2))
        graph.nodes[node] = value
        if not value.protected_reasons and value.kind != "cycle_anchor":
            kind = "junction" if len(adjacency[node]) >= 3 else "terminal" if len(adjacency[node]) == 1 else "region"
            graph.nodes[node] = replace(value, kind=kind)
    # All detailed vertices map to a routing decision or an explicitly pruned
    # local access region. Collapsed edge-interior vertices also retain spans.
    mapping = {n: collapsed_map[membership[n]] for n in working_nodes}
    for record in reversed(pruned):
        target = mapping[record["attachment"]]
        for node in record["node_ids"]:
            mapping[node] = target
    graph.metadata.update(simplification_config=asdict(config), pruned_branches=pruned,
                          junction_clusters=clusters, working_to_routing=mapping,
                          detailed_nodes=len(detailed.nodes), detailed_edges=len(detailed.edges),
                          bounds=list(space.bounds))
    if validate:
        from .routing_validation import validate_routing_graph
        graph.metadata["validation"] = validate_routing_graph(
            detailed, graph, space, config, original_nodes, original_edges)
    return graph
