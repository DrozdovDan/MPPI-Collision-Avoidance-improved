"""Collapse maximal degree-two chains without discarding their embedding."""

import numpy as np

from .model import CorridorEdge, CorridorGraph, CorridorNode, WidthSample


def make_edge(edge_id, u, v, points, space, bottlenecks=()):
    points = np.asarray(points, dtype=float)
    distances = np.r_[0., np.cumsum(np.linalg.norm(np.diff(points, axis=0), axis=1))]
    clearances = space.clearances(points)
    minimum = int(np.argmin(clearances))
    geometry = tuple(tuple(map(float, point)) for point in points)
    profile = tuple(WidthSample(float(distance), point, float(clearance),
                                float(2 * clearance))
                    for distance, point, clearance in zip(distances, geometry, clearances))
    return CorridorEdge(edge_id, u, v, geometry, float(distances[-1]),
                        float(clearances[minimum]), float(2 * clearances[minimum]),
                        geometry[minimum], profile, tuple(bottlenecks))


def extract_graph(skeleton, space):
    adjacency = {node: [] for node in skeleton.positions}
    for index, edge in enumerate(skeleton.edges):
        adjacency[edge.u].append(index)
        adjacency[edge.v].append(index)
    significant = {node for node, edges in adjacency.items() if len(edges) != 2}
    graph, node_ids = CorridorGraph(), {}

    def add_node(node, kind=None):
        if node not in node_ids:
            node_id = len(graph.nodes)
            node_ids[node] = node_id
            graph.nodes[node_id] = CorridorNode(
                node_id, skeleton.positions[node], kind or
                ("junction" if len(adjacency[node]) >= 3 else "terminal"))
        return node_ids[node]

    for node in sorted(significant):
        add_node(node)
    visited = set()

    def walk(start, first):
        current, index, points = start, first, []
        while True:
            visited.add(index)
            edge = skeleton.edges[index]
            geometry = edge.geometry if edge.u == current else edge.geometry[::-1]
            points.extend(geometry if not points else geometry[1:])
            current = edge.v if edge.u == current else edge.u
            if current in significant or current == start:
                break
            index = next(i for i in adjacency[current] if i != index)
        edge_id = len(graph.edges)
        graph.edges[edge_id] = make_edge(edge_id, add_node(start), add_node(current),
                                         points, space)

    for node in sorted(significant):
        for index in adjacency[node]:
            if index not in visited:
                walk(node, index)
    # A component consisting solely of degree-two vertices has no significant
    # vertex. Keep it as one self-loop at a deterministic synthetic anchor.
    for index, edge in enumerate(skeleton.edges):
        if index not in visited:
            add_node(edge.u, "cycle_anchor")
            walk(edge.u, index)
    return graph
