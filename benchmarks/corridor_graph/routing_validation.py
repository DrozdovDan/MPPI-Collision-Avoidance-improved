"""Provenance, connectivity, protected cuts, cycle-space and distance checks."""

import heapq
import itertools
import math

import numpy as np
from shapely.geometry import LineString, Point

from .routing_model import metrics_from_spans


def _neighbors(nodes, edges, excluded=None):
    result = {n: [] for n in nodes if n != excluded}
    for edge in edges.values():
        if edge.u == excluded or edge.v == excluded:
            continue
        result[edge.u].append((edge.v, edge.id))
        result[edge.v].append((edge.u, edge.id))
    return result


def _component_labels(nodes, edges, excluded=None):
    adjacency = _neighbors(nodes, edges, excluded)
    labels = {}
    for root in sorted(adjacency):
        if root in labels:
            continue
        stack, label = [root], len(set(labels.values()))
        while stack:
            node = stack.pop()
            if node in labels:
                continue
            labels[node] = label
            stack.extend(other for other, _ in adjacency[node] if other not in labels)
    return labels


def _edge_vector(edge, detailed, columns):
    vector = np.zeros(len(columns))
    spans = edge.source_spans if hasattr(edge, "source_spans") else edge.spans
    for span in spans:
        original = detailed.edges[span.edge_id]
        distance = original.width_profile[span.last].distance - original.width_profile[span.first].distance
        vector[columns[span.edge_id]] += distance / original.length
    return vector


def _cycle_vectors(nodes, edges, detailed):
    """Fundamental cycles expanded into oriented detailed-edge coordinates.

    Comparing these spaces checks WHICH detailed loops survive, not only how
    many. It catches lost obstacle-separated alternatives and duplicate loops.
    Subspans cancel on shared junction access paths and sum over split edges.
    """
    columns = {edge_id: i for i, edge_id in enumerate(sorted(detailed.edges))}
    adjacency = _neighbors(nodes, edges)
    potentials, tree_edges = {}, set()
    vectors = {edge.id: _edge_vector(edge, detailed, columns) for edge in edges.values()}
    for root in sorted(nodes):
        if root in potentials:
            continue
        potentials[root] = np.zeros(len(columns))
        stack = [root]
        while stack:
            node = stack.pop()
            for other, edge_id in adjacency[node]:
                if other in potentials:
                    continue
                edge = edges[edge_id]
                vector = vectors[edge_id] if edge.u == node else -vectors[edge_id]
                potentials[other] = potentials[node] + vector
                tree_edges.add(edge_id)
                stack.append(other)
    cycles = [potentials[e.u] + vectors[e.id] - potentials[e.v]
              for e in edges.values() if e.id not in tree_edges]
    return np.asarray(cycles).reshape(len(cycles), len(columns))


def _distances(nodes, edges, root, detailed=None):
    adjacency = _neighbors(nodes, edges)
    weights = {e.id: e.length if hasattr(e, "length") else metrics_from_spans(e.spans, detailed)[0]
               for e in edges.values()}
    distances, queue = {root: 0.}, [(0., root)]
    while queue:
        distance, node = heapq.heappop(queue)
        if distance != distances[node]:
            continue
        for other, edge_id in adjacency[node]:
            candidate = distance + weights[edge_id]
            if candidate < distances.get(other, float("inf")):
                distances[other] = candidate
                heapq.heappush(queue, (candidate, other))
    return distances


def validate_routing_graph(detailed, routing, space, config, original_nodes=None, original_edges=None):
    if original_nodes is None:
        from .simplification import _working_graph
        original_nodes, original_edges = _working_graph(detailed, config)
    for node in routing.nodes.values():
        assert space.contains_points([node.position]).all(), f"routing node {node.id} outside free space"
        assert node.id in original_nodes and node.position == original_nodes[node.id].position
        assert math.isclose(node.local_width, float(space.clearances([node.position])[0] * 2), rel_tol=1e-9)
    used_ranges = {edge_id: [] for edge_id in detailed.edges}
    for edge in routing.edges.values():
        assert edge.source_spans, f"routing edge {edge.id} has no provenance"
        for span in edge.source_spans:
            assert span.edge_id in detailed.edges, "unknown source edge"
            assert 0 <= span.first < len(detailed.edges[span.edge_id].geometry)
            assert 0 <= span.last < len(detailed.edges[span.edge_id].geometry)
            assert span.first != span.last
            used_ranges[span.edge_id].append(tuple(sorted((span.first, span.last))))
        reference = edge.reference_geometry(detailed)
        assert reference[0] == routing.nodes[edge.u].position
        assert reference[-1] == routing.nodes[edge.v].position
        assert edge.geometry[0] == reference[0] and edge.geometry[-1] == reference[-1]
        assert space.line_is_free(reference), f"source walk for edge {edge.id} collides"
        assert space.line_is_free(edge.geometry), f"routing edge {edge.id} collides"
        length, width, location = metrics_from_spans(edge.source_spans, detailed)
        assert math.isclose(edge.length, length, rel_tol=1e-9, abs_tol=1e-9)
        assert math.isclose(edge.min_width, width, rel_tol=1e-9, abs_tol=1e-9)
        assert edge.min_width_position == location and location in edge.geometry
        assert LineString(edge.geometry).distance(space.boundary) >= LineString(reference).distance(space.boundary) - 1e-9
        # Geometry is an ordered reference subsequence. Verify EACH shortcut in
        # its own source subchain, preventing jumps between parallel corridors.
        cursor = 0
        for point in edge.geometry[1:]:
            last = next(i for i in range(cursor + 1, len(reference)) if reference[i] == point)
            if last > cursor + 1:
                subchain = LineString(reference[cursor:last + 1])
                chord = LineString([reference[cursor], point])
                assert subchain.buffer(config.geometry_tolerance + 1e-9).covers(chord)
                assert max(Point(p).distance(chord) for p in reference[cursor:last + 1]) <= config.geometry_tolerance + 1e-9
            cursor = last
        assert cursor == len(reference) - 1
        expected_bottlenecks = tuple(n for n in dict.fromkeys((edge.u, edge.v))
                                     if "bottleneck" in routing.nodes[n].protected_reasons)
        assert edge.bottleneck_node_ids == expected_bottlenecks
    # Every original segment must survive in a routing walk, a merged-tree
    # access path, or an explicitly recorded pruned branch. No silent drops.
    for record in routing.metadata["pruned_branches"]:
        for span in record["spans"]:
            used_ranges[span["edge_id"]].append(tuple(sorted((span["first"], span["last"]))))
    for edge_id, ranges in used_ranges.items():
        cursor = 0
        for first, last in sorted(ranges):
            assert first <= cursor, f"unaccounted source geometry on edge {edge_id}"
            cursor = max(cursor, last)
        assert cursor == len(detailed.edges[edge_id].geometry) - 1, f"lost source edge {edge_id}"
    old_labels = _component_labels(original_nodes, original_edges)
    new_labels = _component_labels(routing.nodes, routing.edges)
    assert len(set(old_labels.values())) == len(set(new_labels.values())), "connected components changed"
    component_pairs = {(old_labels[n], new_labels[n]) for n in routing.nodes}
    assert len(component_pairs) == len(set(old_labels.values())), "free-space component mapping changed"
    mapping = routing.metadata["working_to_routing"]
    assert set(mapping) == set(original_nodes)
    assert all(target in routing.nodes and old_labels[source] == old_labels[target]
               for source, target in mapping.items())
    protected = [n for n in original_nodes if original_nodes[n].protected_reasons]
    for node in protected:
        assert node in routing.nodes, f"protected node {node} removed"
        assert routing.nodes[node].position == original_nodes[node].position
        assert routing.nodes[node].protected_reasons == original_nodes[node].protected_reasons
        assert mapping[node] == node
        # Pairwise connectivity after cutting EACH protected feature must match.
        # This also protects routes distinguished by the bottleneck they use.
        old_cut = _component_labels(original_nodes, original_edges, node)
        new_cut = _component_labels(routing.nodes, routing.edges, node)
        partitions = {(old_cut[n], new_cut[n]) for n in routing.nodes if n != node}
        assert len(partitions) == len({a for a, _ in partitions}) == len({b for _, b in partitions}), (
            f"connectivity around protected node {node} changed")
    original_cycles = _cycle_vectors(original_nodes, original_edges, detailed)
    routing_cycles = _cycle_vectors(routing.nodes, routing.edges, detailed)
    cycle_count = len(original_cycles)
    assert len(routing_cycles) == cycle_count, "independent route alternatives changed"
    if cycle_count:
        assert np.linalg.matrix_rank(routing_cycles, tol=1e-8) == cycle_count, "routing cycles are duplicated"
        assert np.linalg.matrix_rank(np.vstack([original_cycles, routing_cycles]), tol=1e-8) == cycle_count, (
            "routing cycle provenance differs from detailed route alternatives")
    # Deterministic pair sample without enumerating all O(N^2) pairs for huge maps.
    ids = sorted(routing.nodes)
    count = len(ids) * (len(ids) - 1) // 2
    if count <= config.validation_pairs:
        pairs = list(itertools.combinations(ids, 2))
    else:
        rng, pairs_set = np.random.default_rng(config.validation_seed), set()
        while len(pairs_set) < config.validation_pairs:
            a, b = map(int, rng.choice(ids, 2, replace=False))
            pairs_set.add(tuple(sorted((a, b))))
        pairs = sorted(pairs_set)
    roots = sorted({a for a, _ in pairs})
    old_distances = {root: _distances(original_nodes, original_edges, root, detailed) for root in roots}
    new_distances = {root: _distances(routing.nodes, routing.edges, root) for root in roots}
    ratios, excesses, warnings, connected = [], [], [], 0
    for a, b in pairs:
        before, after = old_distances[a].get(b), new_distances[a].get(b)
        assert (before is None) == (after is None), f"route connectivity changed for {a}, {b}"
        if before is None:
            continue
        assert after >= before - 1e-7, "routing metric has no valid detailed-path explanation"
        connected += 1
        ratio, excess = after / before, after - before
        ratios.append(ratio)
        excesses.append(excess)
        if excess > config.distortion_absolute_warning and ratio > 1 + config.distortion_relative_warning:
            warnings.append({"u": a, "v": b, "detailed_distance": before,
                             "routing_distance": after, "ratio": ratio, "extra_length": excess})
    return {"components": len(set(new_labels.values())), "independent_cycles": cycle_count,
            "protected_features": len(protected), "sampled_pairs": len(pairs),
            "connected_pairs": connected, "distance_ratio_median": float(np.median(ratios)) if ratios else None,
            "distance_ratio_p95": float(np.quantile(ratios, .95)) if ratios else None,
            "distance_ratio_max": max(ratios, default=None),
            "distance_extra_length_max": max(excesses, default=None),
            "distortion_warnings": sorted(warnings, key=lambda w: -w["ratio"])[:10],
            "distortion_warning_count": len(warnings)}
