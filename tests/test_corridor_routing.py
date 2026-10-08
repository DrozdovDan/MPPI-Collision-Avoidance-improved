"""Topology/provenance safety, corridor geometry and binary runtime regressions."""

from dataclasses import replace
import json
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "benchmarks"))

from corridor_graph import (BottleneckConfig, CorridorGraph, CorridorNode, FreeSpace,
                            RoutingCorridorGraph, SimplificationConfig, build_corridor_graph,
                            load_runtime, simplify_corridor_graph)
from corridor_graph.extraction import make_edge
from corridor_graph.geometry import sample_polyline
from corridor_graph.routing_validation import _cycle_vectors, validate_routing_graph
from corridor_graph.simplification import simplify_polyline
from corridor_graph_demo import rooms_world
from corridor_routing_demo import graph_from_json


def synthetic_graph(world, positions, links, kinds=None, paths=None, bottlenecks=None):
    space = FreeSpace(world)
    graph = CorridorGraph({i: CorridorNode(i, tuple(point), (kinds or {}).get(i, "junction"))
                           for i, point in enumerate(positions)})
    for i, (u, v) in enumerate(links):
        points = (paths or {}).get(i, [positions[u], positions[v]])
        graph.edges[i] = make_edge(i, u, v, sample_polyline(points, .1), space,
                                   (bottlenecks or {}).get(i, ()))
    return graph


OPEN_WORLD = {"bounds": [0, 0, 10, 10], "obstacles": []}


class RoutingRulesTests(unittest.TestCase):
    def test_chain_collapse_preserves_exact_metrics_and_reference(self):
        detailed = synthetic_graph(OPEN_WORLD, [(2, 4), (3, 4), (3, 6), (7, 6)],
                                   [(0, 1), (1, 2), (2, 3)])
        config = SimplificationConfig(prune_branches=False, junction_max_extent=0)
        routing = simplify_corridor_graph(detailed, OPEN_WORLD, config)
        self.assertEqual((len(routing.nodes), len(routing.edges)), (2, 1))
        edge, = routing.edges.values()
        self.assertEqual(set(edge.source_edge_ids), {0, 1, 2})
        self.assertAlmostEqual(edge.length, sum(e.length for e in detailed.edges.values()))
        self.assertAlmostEqual(edge.min_width, min(e.min_width for e in detailed.edges.values()))
        reference = edge.reference_geometry(detailed)
        self.assertIn((3., 4.), reference)
        self.assertIn((3., 6.), reference)
        self.assertGreater(len(reference), len(edge.geometry))

    def test_bottleneck_node_never_collapsed(self):
        detailed = synthetic_graph(OPEN_WORLD, [(2, 5), (4, 5), (6, 5), (8, 5)],
                                   [(0, 1), (1, 2), (2, 3)], kinds={1: "bottleneck"})
        routing = simplify_corridor_graph(detailed, OPEN_WORLD)
        self.assertIn(1, routing.nodes)
        self.assertEqual(routing.nodes[1].kind, "bottleneck")
        self.assertEqual(len(routing.edges), 2)
        self.assertEqual(len(routing.adjacency()[1]), 2)
        self.assertTrue(all(1 in e.bottleneck_node_ids for e in routing.edges.values()))

    def test_mark_only_and_untagged_width_minima_become_explicit(self):
        for detector in (BottleneckConfig(split_edges=False), BottleneckConfig(enabled=False)):
            detailed = build_corridor_graph(rooms_world(), bottleneck_config=detector)
            self.assertFalse(any(n.kind == "bottleneck" for n in detailed.nodes.values()))
            before = detailed.to_dict()
            routing = simplify_corridor_graph(detailed, rooms_world())
            protected, = [n for n in routing.nodes.values() if n.kind == "bottleneck"]
            self.assertTrue(protected.source_edge_samples)
            self.assertAlmostEqual(protected.local_width, 2)
            self.assertEqual(detailed.to_dict(), before)
            edges = [e for e in routing.edges.values() if protected.id in (e.u, e.v)]
            self.assertEqual(len(edges), 2)
            self.assertTrue(all(abs(e.min_width - 2) < 1e-8 for e in edges))

    def test_connected_tree_cluster_uses_real_junction_and_access_paths(self):
        points = [(4, 5), (4.2, 5), (3, 6), (3, 4), (6, 6), (6, 4)]
        detailed = synthetic_graph(OPEN_WORLD, points, [(0, 1), (0, 2), (0, 3), (1, 4), (1, 5)])
        routing = simplify_corridor_graph(detailed, OPEN_WORLD, SimplificationConfig(prune_branches=False))
        cluster, = routing.metadata["junction_clusters"]
        self.assertEqual(cluster["working_nodes"], [0, 1])
        anchor = routing.nodes[cluster["routing_node"]]
        self.assertIn(anchor.position, [tuple(points[0]), tuple(points[1])])
        self.assertEqual(anchor.source_node_ids, (0, 1))
        self.assertEqual(len(routing.edges), 4)
        # Cluster connector is deliberately reused in port-to-anchor paths;
        # it is not silently assigned zero length or lost from width aggregation.
        self.assertEqual(sum(0 in e.source_edge_ids for e in routing.edges.values()), 2)

    def test_cluster_extent_bounds_transitive_merging(self):
        points = [(4, 5), (4.7, 5), (5.4, 5), (4, 6), (4, 4), (4.7, 6), (5.4, 6), (5.4, 4)]
        detailed = synthetic_graph(OPEN_WORLD, points,
            [(0, 1), (1, 2), (0, 3), (0, 4), (1, 5), (2, 6), (2, 7)])
        routing = simplify_corridor_graph(detailed, OPEN_WORLD, SimplificationConfig(prune_branches=False))
        self.assertFalse(any(set(n.source_node_ids) >= {0, 1, 2} for n in routing.nodes.values()))
        self.assertTrue(routing.metadata["junction_clusters"])

    def test_triangle_cluster_keeps_cycle_and_parallel_alternatives(self):
        points = [(4, 4), (4.2, 4), (4.1, 4.2), (3, 4), (5, 4), (4.1, 5)]
        detailed = synthetic_graph(OPEN_WORLD, points,
                                   [(0, 1), (1, 2), (2, 0), (0, 3), (1, 4), (2, 5)])
        routing = simplify_corridor_graph(detailed, OPEN_WORLD, SimplificationConfig(prune_branches=False))
        self.assertEqual(routing.metadata["validation"]["independent_cycles"], 1)
        self.assertFalse(any(set(n.source_node_ids) >= {0, 1, 2} for n in routing.nodes.values()))
        self.assertGreaterEqual(len(routing.edges) - len(routing.nodes) + 1, 1)

    def test_nearby_junctions_across_wall_are_not_merged(self):
        world = {"bounds": [0, 0, 10, 10], "obstacles": [[[4.9, 2], [5.1, 2], [5.1, 8], [4.9, 8]]]}
        points = [(4.7, 5), (5.3, 5), (3, 4), (3, 6), (7, 4), (7, 6)]
        detailed = synthetic_graph(world, points, [(0, 1), (0, 2), (0, 3), (1, 4), (1, 5)],
                                   paths={0: [(4.7, 5), (4.7, 8.3), (5.3, 8.3), (5.3, 5)]})
        config = SimplificationConfig(prune_branches=False, junction_extent_clearance_ratio=10,
                                      junction_max_width_ratio=100)
        routing = simplify_corridor_graph(detailed, world, config)
        self.assertFalse(any(set(n.source_node_ids) >= {0, 1} for n in routing.nodes.values()))

    def test_boundary_tip_pruned_but_its_width_does_not_pollute_corridor(self):
        detailed = synthetic_graph(OPEN_WORLD, [(5, 5), (5, 9.9999), (2, 5), (8, 5)],
                                   [(0, 1), (0, 2), (0, 3)], kinds={1: "terminal"})
        routing = simplify_corridor_graph(detailed, OPEN_WORLD)
        self.assertEqual(len(routing.metadata["pruned_branches"]), 1)
        edge, = routing.edges.values()
        self.assertNotIn(0, edge.source_edge_ids)
        self.assertAlmostEqual(edge.min_width, 4)
        self.assertLess(detailed.edges[0].min_width, .001)
        self.assertIn(1, routing.metadata["working_to_routing"])

    def test_short_positive_width_passage_preserved(self):
        detailed = synthetic_graph(OPEN_WORLD, [(5, 5), (5, 5.4), (2, 5), (8, 5)],
                                   [(0, 1), (0, 2), (0, 3)], kinds={1: "terminal"})
        routing = simplify_corridor_graph(detailed, OPEN_WORLD)
        self.assertFalse(routing.metadata["pruned_branches"])
        self.assertIn(1, routing.nodes)

    def test_clearance_growing_room_not_pruned(self):
        detailed = synthetic_graph(OPEN_WORLD, [(3, 5), (9.9999, 5), (2, 4), (2, 6)],
                                   [(0, 1), (0, 2), (0, 3)], kinds={1: "terminal"})
        config = SimplificationConfig(branch_max_length=20, branch_length_clearance_ratio=4)
        routing = simplify_corridor_graph(detailed, OPEN_WORLD, config)
        self.assertIn(1, routing.nodes)
        self.assertFalse(routing.metadata["pruned_branches"])

    def test_symmetric_room_tips_are_pruned_in_disjoint_batches(self):
        detailed = build_corridor_graph(rooms_world())
        routing = simplify_corridor_graph(detailed, rooms_world())
        self.assertEqual(len(routing.metadata["pruned_branches"]), 8)
        terminals = [n for n in routing.nodes.values() if n.kind == "terminal"]
        self.assertEqual(len(terminals), 4)
        self.assertTrue(all(abs(n.local_width - 6) < 1e-8 for n in terminals))
        # Region access stays on each room's wide interior spine, rather than
        # at an arbitrary surviving obstacle/wall corner with near-zero width.
        self.assertTrue(all(2 < n.position[1] < 6 for n in terminals))

    def test_explicit_access_protection_overrides_pruning(self):
        detailed = synthetic_graph(OPEN_WORLD, [(5, 5), (5, 9.9999), (2, 5), (8, 5)],
                                   [(0, 1), (0, 2), (0, 3)], kinds={1: "terminal"})
        routing = simplify_corridor_graph(detailed, OPEN_WORLD, SimplificationConfig(protected_node_ids=(1,)))
        self.assertIn(1, routing.nodes)
        self.assertIn("access", routing.nodes[1].protected_reasons)
        self.assertFalse(routing.metadata["pruned_branches"])
        with self.assertRaisesRegex(ValueError, "unknown protected"):
            simplify_corridor_graph(detailed, OPEN_WORLD, SimplificationConfig(protected_node_ids=(99,)))

    def test_width_transition_is_protected(self):
        detailed = synthetic_graph(OPEN_WORLD, [(2, 5), (3, 5), (7, 5)], [(0, 1), (1, 2)])
        config = SimplificationConfig(prune_branches=False, width_transition_ratio=1.2)
        routing = simplify_corridor_graph(detailed, OPEN_WORLD, config)
        self.assertIn(1, routing.nodes)
        self.assertEqual(routing.nodes[1].kind, "transition")
        self.assertEqual(len(routing.edges), 2)

    def test_cycle_anchor_self_loop_preserved(self):
        points = [(3, 3), (7, 3), (7, 7), (3, 7)]
        detailed = synthetic_graph(OPEN_WORLD, points, [(0, 1), (1, 2), (2, 3), (3, 0)])
        routing = simplify_corridor_graph(detailed, OPEN_WORLD)
        node, = routing.nodes.values()
        edge, = routing.edges.values()
        self.assertEqual(node.kind, "cycle_anchor")
        self.assertEqual(edge.u, edge.v)
        self.assertEqual(len(routing.adjacency()[node.id]), 2)
        self.assertEqual(routing.metadata["validation"]["independent_cycles"], 1)

    def test_empty_and_disconnected_graphs(self):
        blocked = {"bounds": [0, 0, 2, 2], "obstacles": [[[-1, -1], [3, -1], [3, 3], [-1, 3]]]}
        empty = simplify_corridor_graph(build_corridor_graph(blocked), blocked)
        self.assertFalse(empty.nodes)
        self.assertFalse(empty.edges)
        wall = {"bounds": [0, 0, 10, 6], "obstacles": [[[4, -1], [6, -1], [6, 7], [4, 7]]]}
        detailed = build_corridor_graph(wall)
        routing = simplify_corridor_graph(detailed, wall)
        self.assertEqual(len(routing.connected_components()), 2)
        self.assertEqual(routing.metadata["validation"]["components"], 2)


class RoutingGeometryTests(unittest.TestCase):
    def test_shortcut_cannot_cross_obstacle_even_with_large_tolerance(self):
        world = {"bounds": [0, 0, 10, 10], "obstacles": [[[4, 4], [6, 4], [6, 6], [4, 6]]]}
        space = FreeSpace(world)
        points = tuple(map(tuple, sample_polyline([(3, 5), (3, 7), (7, 7), (7, 5)], .1)))
        reduced = simplify_polyline(points, space, 10)
        self.assertTrue(space.line_is_free(reduced))
        self.assertGreater(len(reduced), 2)

    def test_free_shortcut_still_cannot_leave_original_corridor(self):
        space = FreeSpace(OPEN_WORLD)
        points = tuple(map(tuple, sample_polyline([(2, 3), (2, 7), (8, 7), (8, 3)], .1)))
        self.assertTrue(space.line_is_free([points[0], points[-1]]))
        reduced = simplify_polyline(points, space, .05)
        self.assertGreater(len(reduced), 2)
        self.assertIn((2., 7.), reduced)
        self.assertIn((8., 7.), reduced)

    def test_zero_geometry_tolerance_and_minimum_pin(self):
        space = FreeSpace(OPEN_WORLD)
        points = tuple(map(tuple, sample_polyline([(2, 5), (8, 5)], .1)))
        self.assertEqual(simplify_polyline(points, space, 0), points)
        pinned = points[25]
        reduced = simplify_polyline(points, space, .1, [pinned])
        self.assertIn(pinned, reduced)
        self.assertEqual(len(reduced), 3)


class RoutingValidationTests(unittest.TestCase):
    def setUp(self):
        self.world = rooms_world()
        self.detailed = build_corridor_graph(self.world)
        self.routing = simplify_corridor_graph(self.detailed, self.world)

    def test_validation_detects_incorrect_width_and_provenance(self):
        edge = next(iter(self.routing.edges.values()))
        self.routing.edges[edge.id] = replace(edge, min_width=edge.min_width + 1)
        with self.assertRaises(AssertionError):
            validate_routing_graph(self.detailed, self.routing, FreeSpace(self.world), SimplificationConfig())
        self.routing.edges[edge.id] = replace(edge, source_spans=())
        with self.assertRaises(AssertionError):
            validate_routing_graph(self.detailed, self.routing, FreeSpace(self.world), SimplificationConfig())

    def test_cycle_basis_compares_alternatives_not_just_cycle_count(self):
        # Same number of graph cycles, but duplicating one reference loop gives
        # rank one instead of two. This models losing an obstacle-separated route.
        points = [(2, 5), (5, 5), (8, 5)]
        detailed = synthetic_graph(OPEN_WORLD, points, [(0, 1), (0, 1), (1, 2), (1, 2)], paths={
            0: [(2, 5), (3.5, 6), (5, 5)], 1: [(2, 5), (3.5, 4), (5, 5)],
            2: [(5, 5), (6.5, 6), (8, 5)], 3: [(5, 5), (6.5, 4), (8, 5)]})
        routing = simplify_corridor_graph(detailed, OPEN_WORLD, SimplificationConfig(prune_branches=False))
        vectors = _cycle_vectors(routing.nodes, routing.edges, detailed)
        self.assertEqual(np.linalg.matrix_rank(vectors), 2)
        self.assertEqual(len(vectors), 2)
        first, second = routing.edges.values()
        routing.edges[second.id] = replace(second, geometry=first.geometry, length=first.length,
                                          min_width=first.min_width, min_width_position=first.min_width_position,
                                          source_spans=first.source_spans)
        duplicated = _cycle_vectors(routing.nodes, routing.edges, detailed)
        self.assertEqual(len(duplicated), 2)
        self.assertEqual(np.linalg.matrix_rank(duplicated), 1)
        with self.assertRaises(AssertionError):
            validate_routing_graph(detailed, routing, FreeSpace(OPEN_WORLD), SimplificationConfig(prune_branches=False))

    def test_route_distortion_threshold_reports_explained_detours(self):
        detailed = synthetic_graph(OPEN_WORLD, [(4, 5), (4.8, 5), (3, 6), (3, 4), (6, 6), (6, 4)],
                                   [(0, 1), (0, 2), (0, 3), (1, 4), (1, 5)])
        config = SimplificationConfig(prune_branches=False, distortion_absolute_warning=0,
                                      distortion_relative_warning=0, validation_pairs=1000)
        routing = simplify_corridor_graph(detailed, OPEN_WORLD, config)
        self.assertGreater(routing.metadata["validation"]["distance_extra_length_max"], 1)
        self.assertGreater(routing.metadata["validation"]["distortion_warning_count"], 0)


class RuntimeAndBenchmarkTests(unittest.TestCase):
    def test_suite_topology_provenance_memory_and_detailed_immutability(self):
        worlds = json.loads((ROOT / "benchmarks/bench_mr_tasks.json").read_text())["worlds"]
        for name, world in worlds.items():
            with self.subTest(world=name), tempfile.TemporaryDirectory() as directory:
                detailed = build_corridor_graph(world)
                before = json.dumps(detailed.to_dict(), separators=(",", ":"))
                routing = simplify_corridor_graph(detailed, world)
                self.assertEqual(json.dumps(detailed.to_dict(), separators=(",", ":")), before)
                self.assertLess(len(routing.nodes), len(detailed.nodes))
                self.assertLess(len(routing.edges), len(detailed.edges))
                self.assertEqual(len(routing.connected_components()), len(detailed.connected_components()))
                self.assertEqual(routing.metadata["validation"]["independent_cycles"],
                                 len(detailed.edges) - len(detailed.nodes) + len(detailed.connected_components()))
                bottlenecks = [n for n in detailed.nodes.values() if n.kind == "bottleneck"]
                self.assertTrue(all(n.id in routing.nodes and routing.nodes[n.id].position == n.position
                                    for n in bottlenecks))
                self.assertEqual(routing.metadata["validation"]["sampled_pairs"], 200)
                self.assertEqual(routing.metadata["validation"]["distortion_warning_count"], 0)
                path = Path(directory) / "robot.npz"
                size = routing.save_runtime(path)
                runtime = load_runtime(path)
                self.assertEqual(runtime.adjacency(), routing.adjacency())
                self.assertLess(size["file_bytes"], len(before.encode()))
                self.assertEqual(size["array_bytes"], sum(a.nbytes for a in runtime.arrays.values()))
                self.assertFalse(any("profile" in key or "source" in key for key in runtime.arrays))
                for edge in routing.edges.values():
                    np.testing.assert_array_equal(runtime.edge_geometry(edge.id), edge.geometry)
                if name == "warehouse2":
                    # Regression for the dense repeated-shelf artifact: no tiny
                    # junction connector is left in the right-hand routing grid.
                    self.assertFalse(any(e.length < 1 and all(routing.nodes[n].position[0] > 85
                                         for n in (e.u, e.v)) for e in routing.edges.values()))
                detailed_path = Path(directory) / "detailed.json"
                detailed_path.write_text(before)
                self.assertEqual(graph_from_json(detailed_path).to_dict(), detailed.to_dict())

    def test_runtime_empty_graph_and_rejects_bad_offsets(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "empty.npz"
            RoutingCorridorGraph().save_runtime(path)
            empty = load_runtime(path)
            self.assertEqual(empty.adjacency(), {})
            detailed = build_corridor_graph(rooms_world())
            routing = simplify_corridor_graph(detailed, rooms_world())
            arrays = routing.runtime_arrays()
            arrays["geometry_offsets"][-1] += 1
            np.savez(path, **arrays)
            with self.assertRaisesRegex(ValueError, "offsets"):
                load_runtime(path)
            arrays = routing.runtime_arrays()
            arrays["edge_endpoints"][0, 0] = 999999
            np.savez(path, **arrays)
            with self.assertRaisesRegex(ValueError, "unknown node"):
                load_runtime(path)

    def test_runtime_preserves_submillimeter_corridor_positions(self):
        detailed = build_corridor_graph({"bounds": [0, 0, 1, 1], "obstacles": []})
        routing = simplify_corridor_graph(detailed, {"bounds": [0, 0, 1, 1], "obstacles": []},
                                          SimplificationConfig(prune_branches=False))
        self.assertEqual(routing.runtime_arrays()["geometry_points"].dtype, np.dtype("<f8"))
        self.assertTrue(any(abs(p[0] - .0001) < 1e-8 for e in routing.edges.values() for p in e.geometry))

    def test_invalid_parameters_rejected(self):
        for parameters in ({"junction_max_extent": -1}, {"geometry_tolerance": float("nan")},
                           {"width_transition_ratio": .9}, {"validation_pairs": -1},
                           {"protected_node_ids": [-1]}):
            with self.assertRaises(ValueError):
                SimplificationConfig(**parameters)


if __name__ == "__main__":
    unittest.main()
