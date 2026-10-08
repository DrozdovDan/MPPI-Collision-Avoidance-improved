"""Robot-independent polygonal fixtures with explicit geometric reference walks.

The small hand-authored graphs isolate traffic extraction from Voronoi choices;
the rooms case and real-map demo also exercise the full ECM pipeline.
"""

from corridor_graph import (CorridorGraph, CorridorNode, FreeSpace, RoutingCorridorGraph,
                            RoutingEdge, RoutingNode, SourceSpan, build_corridor_graph,
                            simplify_corridor_graph)
from corridor_graph.extraction import make_edge
from corridor_graph.geometry import sample_polyline
from corridor_graph_demo import rooms_world


def rectangle(x0, y0, x1, y1):
    return [[x0, y0], [x1, y0], [x1, y1], [x0, y1]]


def reference_graph(world, positions, links, paths=None, bottlenecks=()):
    """Identity routing view: retain exact dense provenance without contraction."""
    space = FreeSpace(world)
    degree = {i: sum(i in pair for pair in links) for i in range(len(positions))}
    nodes = {i: CorridorNode(i, tuple(p), "bottleneck" if i in bottlenecks else
                            "junction" if degree[i] >= 3 else "terminal" if degree[i] == 1 else "transition")
             for i, p in enumerate(positions)}
    detailed = CorridorGraph(nodes)
    for i, (u, v) in enumerate(links):
        points = (paths or {}).get(i, [positions[u], positions[v]])
        detailed.edges[i] = make_edge(i, u, v, sample_polyline(points, .08), space)
    routing = RoutingCorridorGraph(
        {i: RoutingNode(i, n.position, n.kind, (i,), local_width=float(space.clearances([n.position])[0] * 2))
         for i, n in nodes.items()},
        {i: RoutingEdge(i, e.u, e.v, e.geometry, e.length, e.min_width, e.min_width_position,
                        (SourceSpan(i, 0, len(e.geometry) - 1),),
                        tuple(n for n in (e.u, e.v) if n in bottlenecks))
         for i, e in detailed.edges.items()})
    return world, detailed, routing


def traffic_fixtures():
    world = rooms_world()
    detailed = build_corridor_graph(world)
    fixtures = {"one_neck": (world, detailed, simplify_corridor_graph(detailed, world))}
    parallel = {"bounds": [0, 0, 14, 8], "obstacles":
                [rectangle(6, 0, 8, 1), rectangle(6, 3, 8, 5), rectangle(6, 7, 8, 8)]}
    fixtures["parallel_necks"] = reference_graph(
        parallel, [(3, 4), (11, 4), (1.5, 4), (12.5, 4)], [(0, 1), (0, 1), (2, 0), (1, 3)],
        {0: [(3, 4), (5, 2), (9, 2), (11, 4)], 1: [(3, 4), (5, 6), (9, 6), (11, 4)]})
    lower = [[4, 0], [14, 0], [14, 3], [12, 3], [12, 2.7], [11, 2.7], [11, 3.2],
             [9, 3.2], [9, 2.7], [8, 2.7], [8, 3.1], [6, 3.1], [6, 2.7], [5, 2.7], [5, 3], [4, 3]]
    upper = [[x, 8 - y] for x, y in reversed(lower)]
    several = {"bounds": [0, 0, 18, 8], "obstacles": [lower, upper]}
    fixtures["several_minima"] = reference_graph(
        several, [(2, 4), (4.5, 4), (7, 4), (10, 4), (13, 4), (16, 4)],
        [(0, 1), (1, 2), (2, 3), (3, 4), (4, 5)], bottlenecks=(1, 2, 3, 4))
    tee = {"bounds": [0, 0, 12, 12], "obstacles":
           [rectangle(0, 0, 12, 5), rectangle(0, 7, 5, 12), rectangle(7, 7, 12, 12)]}
    fixtures["narrow_tee"] = reference_graph(tee, [(6, 6), (2, 6), (10, 6), (6, 10)], [(0, 1), (0, 2), (0, 3)])
    wide = {"bounds": [0, 0, 20, 20], "obstacles":
            [rectangle(0, 0, 7, 7), rectangle(13, 0, 20, 7),
             rectangle(0, 13, 7, 20), rectangle(13, 13, 20, 20)]}
    fixtures["wide_crossing"] = reference_graph(wide, [(10, 10), (4, 10), (16, 10), (10, 4), (10, 16)],
                                               [(0, 1), (0, 2), (0, 3), (0, 4)])
    strip = {"bounds": [0, 0, 12, 2], "obstacles": []}
    fixtures["straight_strip"] = reference_graph(strip, [(2, 1), (6, 1), (10, 1)], [(0, 1), (1, 2)])
    nearly_closed = {"bounds": [0, 0, 12, 6], "obstacles": [rectangle(2, .00002, 10, 4)]}
    fixtures["near_boundary_gap"] = reference_graph(nearly_closed, [(3, .00001), (9, .00001)], [(0, 1)])
    corner = {"bounds": [0, 0, 10, 10], "obstacles": [rectangle(5, 5, 8, 8)]}
    fixtures["corner_degeneracy"] = reference_graph(corner, [(4.9, 4.9), (3, 3)], [(0, 1)])
    return fixtures
