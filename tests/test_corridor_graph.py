"""Continuous polygon-boundary skeleton, multigraph, and bottleneck regressions."""

from dataclasses import replace
import json
from pathlib import Path
import sys
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "benchmarks"))

from corridor_graph import (BottleneckConfig, CorridorEdge, FreeSpace, GeometryConfig,
                            WidthSample, build_corridor_graph, detect_bottlenecks,
                            validate_graph)
from corridor_graph.bottlenecks import split_bottlenecks
from corridor_graph.extraction import extract_graph
from corridor_graph.geometry import Skeleton, SkeletonEdge
from corridor_graph_demo import graph_statistics, rooms_world


class CorridorGeometryTests(unittest.TestCase):
    def test_rectangle_has_exact_side_bisectors(self):
        world = {"bounds": [0, 0, 10, 6], "obstacles": []}
        graph = build_corridor_graph(world)
        self.assertEqual((len(graph.nodes), len(graph.edges)), (6, 5))
        junctions = sorted(n.position for n in graph.nodes.values() if n.kind == "junction")
        np.testing.assert_allclose(junctions, [(3, 3), (7, 3)], atol=1e-8)
        central, = [e for e in graph.edges.values()
                    if all(graph.nodes[n].kind == "junction" for n in (e.u, e.v))]
        self.assertAlmostEqual(central.length, 4)
        self.assertAlmostEqual(central.min_width, 6)
        self.assertTrue(all(abs(s.width - 6) < 1e-8 for s in central.width_profile))
        self.assertLessEqual(max(np.linalg.norm(np.diff(e.geometry, axis=0), axis=1).max()
                                 for e in graph.edges.values()), GeometryConfig().sample_spacing + 1e-8)

    def test_rooms_passage_width_and_split(self):
        world = rooms_world()
        unsplit = build_corridor_graph(world, bottleneck_config=BottleneckConfig(enabled=False))
        split = build_corridor_graph(world)
        bottleneck, = [n for n in split.nodes.values() if n.kind == "bottleneck"]
        np.testing.assert_allclose(bottleneck.position, (7, 4), atol=.15)
        self.assertEqual(len(split.edges), len(unsplit.edges) + 1)
        self.assertEqual(len(split.adjacency()[bottleneck.id]), 2)
        self.assertAlmostEqual(sum(e.length for e in split.edges.values()),
                               sum(e.length for e in unsplit.edges.values()))
        passage = [e for e in split.edges.values() if bottleneck.id in (e.u, e.v)]
        self.assertEqual(len(passage), 2)
        for edge in passage:
            self.assertAlmostEqual(edge.min_width, 2)
        # All samples within the straight passage have the analytical width.
        for edge in passage:
            for sample in edge.width_profile:
                if 6 <= sample.position[0] <= 8:
                    self.assertAlmostEqual(sample.position[1], 4)
                    self.assertAlmostEqual(sample.width, 2)
        marked = build_corridor_graph(world, bottleneck_config=BottleneckConfig(split_edges=False))
        self.assertEqual(len(marked.edges), len(unsplit.edges))
        self.assertEqual(sum(len(e.bottleneck_positions) for e in marked.edges.values()), 1)
        self.assertEqual(len(split.connected_components()), 1)

    def test_polygon_hole_keeps_loop_topology(self):
        world = {"bounds": [0, 0, 10, 10],
                 "obstacles": [[[4, 4], [6, 4], [6, 6], [4, 6]]]}
        graph = build_corridor_graph(world)
        # One obstacle hole -> one independent graph cycle, including multiedges.
        self.assertEqual(len(graph.edges) - len(graph.nodes) + len(graph.connected_components()), 1)
        # The lower-left obstacle corner (4,4) versus the left wall produces
        # the parabola x = 2 + (y-4)^2/8. Vertex-only Voronoi cannot do this.
        curved = [p for e in graph.edges.values() for p in e.geometry
                  if 2 <= p[0] < 2.35 and 2.4 < p[1] < 3.9]
        self.assertGreater(len(curved), 5)
        for x, y in curved:
            self.assertAlmostEqual(x, 2 + (y - 4)**2 / 8, places=6)

    def test_disconnected_rooms_and_fully_blocked_world(self):
        graph = build_corridor_graph({"bounds": [0, 0, 10, 6],
            "obstacles": [[[4, -1], [6, -1], [6, 7], [4, 7]]]})
        self.assertEqual(len(graph.connected_components()), 2)
        empty = build_corridor_graph({"bounds": [0, 0, 2, 2],
            "obstacles": [[[-1, -1], [3, -1], [3, 3], [-1, 3]]]})
        self.assertFalse(empty.nodes)
        self.assertFalse(empty.edges)
        self.assertIsNone(graph_statistics(empty)["width_min"])

    def test_union_concavity_and_translation(self):
        world = {"bounds": [0, 0, 12, 10], "obstacles": [
            [[3, 2], [7, 2], [7, 4], [5, 4], [5, 7], [3, 7]],
            [[4, 3], [8, 3], [8, 5], [4, 5]],
            [[-1, 8], [2, 8], [2, 11], [-1, 11]]],}
        graph = build_corridor_graph(world)
        offset = np.array([10000., -20000.])
        shifted = {"bounds": (np.array(world["bounds"]) + np.tile(offset, 2)).tolist(),
                   "obstacles": [(np.array(p) + offset).tolist() for p in world["obstacles"]]}
        other = build_corridor_graph(shifted)
        self.assertEqual((len(graph.nodes), len(graph.edges)), (len(other.nodes), len(other.edges)))
        np.testing.assert_allclose(sorted(e.length for e in graph.edges.values()),
                                   sorted(e.length for e in other.edges.values()), atol=1e-7)

    def test_all_benchmark_maps(self):
        worlds = json.loads((ROOT / "benchmarks/bench_mr_tasks.json").read_text())["worlds"]
        expected_components = {"parking1": 1, "parking2": 1, "parking3": 2,
                               "warehouse": 2, "warehouse2": 2}
        for name, world in worlds.items():
            with self.subTest(world=name):
                graph = build_corridor_graph(world)  # full-chord and profile validation
                self.assertEqual(len(graph.connected_components()), expected_components[name])
                self.assertTrue(graph.nodes)
                for node_id, neighbors in graph.adjacency().items():
                    kind = graph.nodes[node_id].kind
                    if kind == "junction":
                        self.assertGreaterEqual(len(neighbors), 3)
                    elif kind == "terminal":
                        self.assertEqual(len(neighbors), 1)
                json.dumps(graph.to_dict(), allow_nan=False)

    def test_rejects_invalid_geometry_and_parameters(self):
        for bounds in ([0, 0, 0, 2], [0, 0, np.nan, 2]):
            with self.assertRaises(ValueError):
                FreeSpace({"bounds": bounds, "obstacles": []})
        with self.assertRaises(ValueError):
            GeometryConfig(sample_spacing=0)
        with self.assertRaises(ValueError):
            BottleneckConfig(max_width_ratio=1)
        with self.assertRaisesRegex(ValueError, "overflows"):
            build_corridor_graph({"bounds": [0, 0, 3000, 10], "obstacles": []})

    def test_validation_detects_obstacle_crossing_between_free_samples(self):
        world = {"bounds": [0, 0, 10, 10],
                 "obstacles": [[[4, 4], [6, 4], [6, 6], [4, 6]]]}
        graph = build_corridor_graph(world)
        edge = next(iter(graph.edges.values()))
        # Even if vertices are free, the chord itself may cross the hole.
        self.assertTrue(FreeSpace(world).contains_points([(3, 5), (7, 5)]).all())
        self.assertFalse(FreeSpace(world).line_is_free([(3, 5), (7, 5)]))
        graph.edges[edge.id] = replace(edge, min_width=edge.min_width + 1)
        with self.assertRaises(AssertionError):
            validate_graph(graph, FreeSpace(world))


class GraphExtractionTests(unittest.TestCase):
    def test_degree_two_chain_keeps_bends(self):
        space = FreeSpace({"bounds": [0, 0, 5, 5], "obstacles": []})
        skeleton = Skeleton({0: (1., 1.), 1: (2., 1.), 2: (2., 2.), 3: (3., 2.)}, [
            SkeletonEdge(0, 1, ((1., 1.), (2., 1.))),
            SkeletonEdge(1, 2, ((2., 1.), (2., 2.))),
            SkeletonEdge(2, 3, ((2., 2.), (3., 2.))),
        ])
        graph = extract_graph(skeleton, space)
        self.assertEqual((len(graph.nodes), len(graph.edges)), (2, 1))
        edge, = graph.edges.values()
        self.assertEqual(len(edge.geometry), 4)
        self.assertAlmostEqual(edge.length, 3)

    def test_pure_cycle_preserved_as_self_loop(self):
        points = [(1., 1.), (3., 1.), (3., 3.), (1., 3.)]
        skeleton = Skeleton(dict(enumerate(points)), [
            SkeletonEdge(i, (i + 1) % 4, (points[i], points[(i + 1) % 4])) for i in range(4)])
        space = FreeSpace({"bounds": [0, 0, 4, 4], "obstacles": []})
        graph = extract_graph(skeleton, space)
        node, = graph.nodes.values()
        edge, = graph.edges.values()
        self.assertEqual(node.kind, "cycle_anchor")
        self.assertEqual((edge.u, edge.v), (node.id, node.id))
        self.assertEqual(len(graph.adjacency()[node.id]), 2)
        self.assertAlmostEqual(edge.length, 8)

    def test_parallel_corridors_not_overwritten(self):
        skeleton = Skeleton({0: (1., 2.), 1: (4., 2.), 2: (.5, 2.), 3: (4.5, 2.)}, [
            SkeletonEdge(0, 1, ((1., 2.), (2., 3.), (4., 2.))),
            SkeletonEdge(0, 1, ((1., 2.), (2., 1.), (4., 2.))),
            SkeletonEdge(0, 2, ((1., 2.), (.5, 2.))),
            SkeletonEdge(1, 3, ((4., 2.), (4.5, 2.))),
        ])
        graph = extract_graph(skeleton, FreeSpace({"bounds": [0, 0, 5, 4], "obstacles": []}))
        self.assertEqual(len(graph.edges), 4)
        self.assertEqual(len([e for e in graph.edges.values() if (e.u, e.v) == (0, 1)]), 2)


class BottleneckTests(unittest.TestCase):
    @staticmethod
    def edge(distances, widths):
        points = tuple((float(x), 1.) for x in distances)
        profile = tuple(WidthSample(float(x), p, float(w / 2), float(w))
                        for x, p, w in zip(distances, points, widths))
        minimum = int(np.argmin(widths))
        return CorridorEdge(0, 0, 1, points, float(distances[-1]),
                            float(widths[minimum] / 2), float(widths[minimum]),
                            points[minimum], profile)

    def test_plateau_minimum_and_sampling_density(self):
        for count in (101, 401):
            x = np.linspace(0, 10, count)
            widths = np.interp(x, [0, 3, 4, 6, 7, 10], [6, 6, 2, 2, 6, 6])
            edge = self.edge(x, widths)
            indices = detect_bottlenecks(edge, BottleneckConfig())
            self.assertEqual(len(indices), 1)
            self.assertAlmostEqual(x[indices[0]], 5, delta=.2)

    def test_noise_and_one_sided_narrowing_ignored(self):
        x = np.linspace(0, 10, 201)
        for widths in (5 + .03 * np.sin(20 * x), 6 - .4 * x,
                       np.interp(x, [0, 4, 5, 6, 10], [6, 6, 2, 2.1, 2.1])):
            self.assertEqual(detect_bottlenecks(self.edge(x, widths), BottleneckConfig()), [])

    def test_separated_minima_and_configurable_threshold(self):
        x = np.linspace(0, 20, 401)
        widths = np.interp(x, [0, 3, 4, 5, 8, 11, 12, 13, 20], [6, 6, 2, 6, 6, 6, 3, 6, 6])
        edge = self.edge(x, widths)
        self.assertEqual(len(detect_bottlenecks(edge, BottleneckConfig(window_distance=3))), 2)
        strict = BottleneckConfig(window_distance=3, max_width_ratio=.4, smoothing_distance=0)
        indices = detect_bottlenecks(edge, strict)
        self.assertEqual(len(indices), 1)
        self.assertAlmostEqual(x[indices[0]], 4)

    def test_closed_edge_detects_minimum_at_anchor(self):
        angle = np.linspace(0, 2 * np.pi, 201)
        points = tuple((float(5 + 3 * np.cos(t)), float(5 + 3 * np.sin(t))) for t in angle)
        points = points[:-1] + (points[0],)
        distance = np.r_[0., np.cumsum(np.linalg.norm(np.diff(points, axis=0), axis=1))]
        widths = 2 + 4 * (1 - np.cos(angle)) / 2
        profile = tuple(WidthSample(float(d), p, float(w / 2), float(w))
                        for d, p, w in zip(distance, points, widths))
        edge = CorridorEdge(0, 0, 0, points, float(distance[-1]), 1., 2., points[0], profile)
        config = BottleneckConfig(smoothing_distance=0, max_width_ratio=.85,
                                  window_distance=7)
        self.assertEqual(detect_bottlenecks(edge, config), [0])
        from corridor_graph import CorridorGraph, CorridorNode
        graph = CorridorGraph({0: CorridorNode(0, points[0], "cycle_anchor")}, {0: edge})
        split = split_bottlenecks(graph, FreeSpace({"bounds": [0, 0, 10, 10], "obstacles": []}), config)
        self.assertEqual(len(split.edges), 1)
        self.assertEqual(split.nodes[0].kind, "bottleneck")
        self.assertGreater(split.edges[0].length, 0)


if __name__ == "__main__":
    unittest.main()
