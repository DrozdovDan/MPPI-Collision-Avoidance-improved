"""Traffic constraints, exact width witnesses, provenance and oriented usage."""

from dataclasses import replace
import json
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np
from shapely.geometry import LineString

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "benchmarks"))

from corridor_graph import (DirectedEdge, FreeSpace, TrafficConfig, WidthAuditor, audit_widths,
                            build_corridor_graph, simplify_corridor_graph,
                            extract_traffic_resources, path_resource_usage, load_traffic_resources,
                            validate_traffic_resources)
from corridor_graph.width_audit import continuous_minimum
from corridor_traffic_fixtures import reference_graph, traffic_fixtures, rectangle


class TrafficTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.fixtures = traffic_fixtures()
        cls.traffic = {name: extract_traffic_resources(d, r, w) for name, (w, d, r) in cls.fixtures.items()}

    def narrow(self, name):
        return [r for r in self.traffic[name].resources.values() if r.kind == "narrow_corridor"]

    def junctions(self, name):
        return [r for r in self.traffic[name].resources.values() if r.kind == "junction_region"]

    def test_rooms_one_neck_spans_two_edges_and_keeps_bottleneck(self):
        resource, = self.narrow("one_neck")
        self.assertEqual(len(resource.source_edge_ids), 2)
        self.assertAlmostEqual(resource.min_width, 2)
        self.assertEqual(len(resource.entrances), 2)
        self.assertEqual(resource.entrances, resource.exits)
        world, detailed, routing = self.fixtures["one_neck"]
        bottleneck, = [n for n in routing.nodes.values() if n.kind == "bottleneck"]
        self.assertIn(bottleneck.id, resource.source_node_ids)
        self.assertTrue(all(p.node_id != bottleneck.id for p in resource.entrances))
        only_narrow = extract_traffic_resources(detailed, routing, world, TrafficConfig(detect_junctions=False))
        self.assertTrue(any(not v for v in only_narrow.edge_intervals.values()))

    def test_parallel_routes_never_merge(self):
        resources = self.narrow("parallel_necks")
        self.assertEqual(len(resources), 2)
        self.assertTrue(any(r.source_edge_ids == (0,) for r in resources))
        self.assertTrue(any(r.source_edge_ids == (1,) for r in resources))
        self.assertLess(resources[0].length, 10)
        self.assertEqual(len(self.junctions("parallel_necks")), 2)

    def test_several_bottlenecks_one_physical_resource(self):
        resource, = self.narrow("several_minima")
        self.assertEqual(len(resource.source_edge_ids), 5)
        self.assertAlmostEqual(resource.min_width, 1.6)
        self.assertGreaterEqual(len(resource.source_node_ids), 4)
        self.assertGreater(resource.representative_width, resource.min_width)
        self.assertEqual(len(resource.entrances), 2)

    def test_narrow_t_stops_at_decision_and_has_independent_conflict(self):
        self.assertEqual(len(self.narrow("narrow_tee")), 3)
        conflict, = self.junctions("narrow_tee")
        self.assertEqual(len(conflict.entrances), 3)
        for resource in self.narrow("narrow_tee"):
            self.assertEqual(len(resource.source_edge_ids), 1)
            self.assertIn(0, [p.node_id for p in resource.entrances])
        self.assertEqual(len(conflict.possible_connections), 9)
        self.assertEqual(conflict.geometric_classification, "junction_candidate")

    def test_wide_crossing_conflict_without_narrow_resource(self):
        self.assertFalse(self.narrow("wide_crossing"))
        conflict, = self.junctions("wide_crossing")
        self.assertEqual(len(conflict.entrances), 4)
        self.assertGreater(conflict.min_width, 3)
        self.assertEqual(len(conflict.possible_connections), 16)
        self.assertFalse(hasattr(conflict, "direction_constraints"))
        self.assertFalse(any(k in self.traffic["wide_crossing"].to_dict() for k in ("capacity", "feasibility")))

    def test_forward_reverse_join_across_edges_and_repeated_traversal(self):
        _, _, routing = self.fixtures["straight_strip"]
        traffic = self.traffic["straight_strip"]
        forward = path_resource_usage(traffic, routing, [0, 1], start_node=0)
        reverse = path_resource_usage(traffic, routing, [DirectedEdge(1, -1), DirectedEdge(0, -1)])
        self.assertEqual(len(forward), 1)
        self.assertEqual(len(reverse), 1)
        self.assertEqual(forward[0].entry.position, reverse[0].exit.position)
        self.assertEqual(forward[0].exit.position, reverse[0].entry.position)
        self.assertEqual(forward[0].direction, "forward")
        self.assertEqual(reverse[0].direction, "reverse")
        self.assertAlmostEqual(forward[0].traversal_length, 8)
        repeat = path_resource_usage(traffic, routing,
                                    [DirectedEdge(0), DirectedEdge(1), DirectedEdge(1, -1), DirectedEdge(0, -1)])
        self.assertEqual(len(repeat), 2)
        self.assertEqual(repeat[0].resource_id, repeat[1].resource_id)
        self.assertEqual([p.direction for p in repeat], ["forward", "reverse"])

    def test_junction_usage_merges_arms_despite_overlapping_narrow_constraints(self):
        _, _, routing = self.fixtures["narrow_tee"]
        traffic = self.traffic["narrow_tee"]
        usage = path_resource_usage(traffic, routing, [DirectedEdge(0, -1), DirectedEdge(2)])
        conflicts = [p for p in usage if traffic.resources[p.resource_id].kind == "junction_region"]
        self.assertEqual(len(conflicts), 1)
        self.assertAlmostEqual(conflicts[0].traversal_length, 1.5)
        self.assertEqual(conflicts[0].entry.edge_id, 0)
        self.assertEqual(conflicts[0].exit.edge_id, 2)
        self.assertEqual(len(usage), 3)

    def test_disconnected_walk_and_ambiguous_parallel_orientation_rejected(self):
        _, _, routing = self.fixtures["parallel_necks"]
        traffic = self.traffic["parallel_necks"]
        with self.assertRaises(ValueError):
            path_resource_usage(traffic, routing, [0, 1])
        with self.assertRaises(ValueError):
            path_resource_usage(traffic, routing, [DirectedEdge(0), DirectedEdge(1)])
        with self.assertRaises(ValueError):
            traffic.directed_mapping(0, 0)
        self.assertEqual(path_resource_usage(traffic, routing, []), [])

    def test_reverse_mapping_orders_nested_constraints_by_actual_entry(self):
        from corridor_graph import ResourcePort, TrafficResourceGraph
        # Two geometrically overlapping constraints: [0, 4] encloses [1, 2].
        # Reversing forward list order would put the inner constraint first.
        template = self.traffic["narrow_tee"].edge_intervals[0][0]
        outer = replace(template, resource_id=0, start=0., end=4.,
                        start_port=ResourcePort(0, 0., (6., 6.)), end_port=ResourcePort(0, 4., (2., 6.)))
        inner = replace(template, resource_id=1, start=1., end=2.,
                        start_port=ResourcePort(0, 1., (5., 6.)), end_port=ResourcePort(0, 2., (4., 6.)))
        graph = TrafficResourceGraph(edge_intervals={0: (outer, inner)})
        self.assertEqual([r for r, *_ in graph.directed_mapping(0)], [0, 1])
        self.assertEqual([r for r, *_ in graph.directed_mapping(0, -1)], [0, 1])

    def test_disconnected_regions_remain_separate(self):
        world = {"bounds": [0, 0, 12, 2], "obstacles": [rectangle(5, 0, 7, 2)]}
        w, d, r = reference_graph(world, [(1.5, 1), (3.5, 1), (8.5, 1), (10.5, 1)], [(0, 1), (2, 3)])
        traffic = extract_traffic_resources(d, r, w)
        self.assertEqual(len(traffic.resources), 2)
        self.assertEqual({r.source_edge_ids for r in traffic.resources.values()}, {(0,), (1,)})

    def test_geometry_metrics_ports_and_ecm_provenance_for_all_fixtures(self):
        for name, (world, detailed, routing) in self.fixtures.items():
            with self.subTest(name=name):
                traffic = self.traffic[name]
                validate_traffic_resources(traffic, detailed, routing, FreeSpace(world))
                self.assertEqual(set(traffic.edge_intervals), set(routing.edges))
                for resource in traffic.resources.values():
                    self.assertAlmostEqual(resource.min_width,
                                           min(continuous_minimum(g, FreeSpace(world))[0] for g in resource.geometry))
                    self.assertTrue(set(resource.detailed_edge_ids).issubset(detailed.edges))
                    self.assertGreater(resource.min_width, 0)
                    self.assertGreaterEqual(resource.representative_width + 1e-8, resource.min_width)
                    self.assertTrue(all(FreeSpace(world).line_is_free(g) for g in resource.geometry))

    def test_controls_independent_and_inputs_immutable(self):
        world, detailed, routing = self.fixtures["narrow_tee"]
        before = (detailed.to_dict(), routing.to_dict())
        only_narrow = extract_traffic_resources(detailed, routing, world, TrafficConfig(detect_junctions=False))
        only_conflict = extract_traffic_resources(detailed, routing, world, TrafficConfig(detect_narrow=False))
        self.assertEqual(len(only_narrow.resources), 3)
        self.assertEqual(len(only_conflict.resources), 1)
        self.assertEqual(before, (detailed.to_dict(), routing.to_dict()))
        self.assertEqual(len(extract_traffic_resources(detailed, routing, world, TrafficConfig(narrow_width=1.5)).resources), 1)

    def test_audit_checks_segments_and_outer_wall_and_detects_bad_scalar(self):
        world, detailed, routing = self.fixtures["straight_strip"]
        report = audit_widths(detailed, routing, FreeSpace(world))
        self.assertEqual(report["units"], "map units")
        self.assertEqual(report["max_profile_error"], 0)
        witness = WidthAuditor(FreeSpace(world)).witness((6, 1))
        self.assertEqual(witness.classification, "opposed_boundary_gap")
        self.assertEqual({c.kind for c in witness.contacts}, {"outer"})
        self.assertTrue(all(c.position not in c.segment for c in witness.contacts))
        old = routing.edges[0]
        routing.edges[0] = replace(old, min_width=old.min_width + .1)
        try:
            with self.assertRaises(AssertionError):
                audit_widths(detailed, routing, FreeSpace(world))
        finally:
            routing.edges[0] = old

    def test_same_corner_contacts_are_not_an_opposing_gap(self):
        world = {"bounds": [0, 0, 10, 10], "obstacles": [rectangle(5, 5, 8, 8)]}
        witness = WidthAuditor(FreeSpace(world)).witness((4.9, 4.9))
        self.assertEqual(len(witness.contacts), 2)
        self.assertEqual(witness.classification, "local_clearance_only")
        self.assertIsNone(witness.opposite_pair)

    def test_invalid_geometric_configuration(self):
        for params in ({"narrow_width": -1}, {"narrow_width": float("nan")},
                       {"junction_clearance_fraction": 1}, {"junction_min_angle_degrees": 0},
                       {"width_hysteresis": -.1}):
            with self.assertRaises(ValueError):
                TrafficConfig(**params)

    def test_corrupted_interval_and_resource_minimum_are_rejected(self):
        world, detailed, routing = self.fixtures["straight_strip"]
        traffic = extract_traffic_resources(detailed, routing, world)
        resource = traffic.resources[0]
        traffic.resources[0] = replace(resource, min_width=resource.min_width + .1)
        with self.assertRaises(AssertionError):
            validate_traffic_resources(traffic, detailed, routing, FreeSpace(world))
        traffic.resources[0] = resource
        traffic.edge_intervals[0] = (replace(traffic.edge_intervals[0][0], end=99),)
        with self.assertRaises(AssertionError):
            validate_traffic_resources(traffic, detailed, routing, FreeSpace(world))

    def test_subsample_narrow_dip_is_retained(self):
        # One chord with wide endpoints but a tiny physical gap in its middle.
        from corridor_graph.extraction import make_edge
        from corridor_graph import RoutingEdge, SourceSpan
        world = {"bounds": [0, 0, 12, 8], "obstacles": [rectangle(5, 0, 7, 3), rectangle(5, 5, 7, 8)]}
        w, d, r = reference_graph(world, [(3, 4), (9, 4)], [(0, 1)])
        coarse = make_edge(0, 0, 1, [(3, 4), (9, 4)], FreeSpace(w))
        d.edges[0] = coarse
        r.edges[0] = RoutingEdge(0, 0, 1, coarse.geometry, coarse.length, coarse.min_width,
                                coarse.min_width_position, (SourceSpan(0, 0, 1),))
        traffic = extract_traffic_resources(d, r, w)
        resource, = traffic.resources.values()
        self.assertEqual(resource.kind, "narrow_corridor")
        self.assertAlmostEqual(resource.min_width, 2)
        self.assertLess(resource.length, coarse.length)

    def test_empty_graph_and_closed_cycle(self):
        from corridor_graph import CorridorGraph, RoutingCorridorGraph
        world = {"bounds": [0, 0, 12, 2], "obstacles": []}
        empty = extract_traffic_resources(CorridorGraph(), RoutingCorridorGraph(), world)
        self.assertFalse(empty.resources)
        self.assertEqual(empty.edge_intervals, {})
        w, d, r = reference_graph(world, [(2, 1)], [(0, 0)],
                                  {0: [(2, 1), (3, 1.1), (3, .9), (2, 1)]})
        traffic = extract_traffic_resources(d, r, w)
        resource, = traffic.resources.values()
        self.assertTrue(resource.closed)
        usage = path_resource_usage(traffic, r, [DirectedEdge(0), DirectedEdge(0, -1)])
        self.assertEqual(len(usage), 2)
        self.assertAlmostEqual(sum(p.traversal_length for p in usage), 2 * r.edges[0].length)
        laps = path_resource_usage(traffic, r, [DirectedEdge(0), DirectedEdge(0)])
        self.assertEqual(len(laps), 2)

    def test_small_junction_loop_is_counted_once(self):
        world = {"bounds": [0, 0, 10, 10], "obstacles": []}
        w, d, r = reference_graph(world, [(5, 5), (2, 5), (8, 5)], [(0, 0), (0, 1), (0, 2)],
                                  {0: [(5, 5), (5.2, 5.2), (4.8, 5.2), (5, 5)]})
        traffic = extract_traffic_resources(d, r, w, TrafficConfig(narrow_width=.1))
        conflict, = traffic.resources.values()
        loop_intervals = traffic.edge_intervals[0]
        self.assertEqual(len(loop_intervals), 2)
        self.assertAlmostEqual(sum(i.end - i.start for i in loop_intervals), r.edges[0].length)
        usage = path_resource_usage(traffic, r, [DirectedEdge(0)])
        self.assertEqual(len(usage), 1)
        self.assertAlmostEqual(usage[0].traversal_length, r.edges[0].length)

    def test_real_maps_audit_and_resource_overlay_preserve_provenance(self):
        worlds = json.loads((ROOT / "benchmarks/bench_mr_tasks.json").read_text())["worlds"]
        for name, world in worlds.items():
            with self.subTest(world=name):
                detailed = build_corridor_graph(world)
                routing = simplify_corridor_graph(detailed, world)
                before = (detailed.to_dict(), routing.to_dict())
                space = FreeSpace(world)
                audit = audit_widths(detailed, routing, space)
                traffic = extract_traffic_resources(detailed, routing, space)
                self.assertEqual(before, (detailed.to_dict(), routing.to_dict()))
                self.assertEqual(audit["max_profile_error"], 0)
                self.assertLess(audit["max_sampled_overestimate"], .02)
                self.assertGreater(traffic.geometry_size()["compact_json_bytes"], 0)
                for edge_id, intervals in traffic.edge_intervals.items():
                    for interval in intervals:
                        resource = traffic.resources[interval.resource_id]
                        self.assertIn(edge_id, resource.source_edge_ids)
                        self.assertTrue(space.line_is_free(interval.geometry))
                if name == "warehouse2":
                    for expected in (.151, .232, .327):
                        edge = min(routing.edges.values(), key=lambda e: abs(e.min_width - expected))
                        witness = WidthAuditor(space).witness(edge.min_width_position)
                        self.assertAlmostEqual(witness.local_width, edge.min_width)
                        self.assertEqual(witness.classification, "opposed_boundary_gap")
                    tiny = min(routing.edges.values(), key=lambda e: abs(e.min_width - .151))
                    self.assertIn("outer", {c.kind for c in WidthAuditor(space).witness(tiny.min_width_position).contacts})

    def test_genuine_tiny_boundary_gap_not_classified_as_numerical_degeneracy(self):
        world, detailed, routing = self.fixtures["near_boundary_gap"]
        resource, = self.narrow("near_boundary_gap")
        self.assertAlmostEqual(resource.min_clearance, .00001)
        self.assertAlmostEqual(resource.min_width, .00002)
        self.assertEqual(resource.width_witness.classification, "opposed_boundary_gap")
        self.assertEqual({c.kind for c in resource.width_witness.contacts}, {"outer", "obstacle"})
        self.assertEqual(resource.geometric_classification, "constrained_passage")
        self.assertFalse(resource.boundary_terminations)

    def test_boundary_trim_adapts_to_preserve_a_genuine_alternative(self):
        world, _, _ = self.fixtures["near_boundary_gap"]
        detailed = build_corridor_graph(world)
        self.assertLess(detailed.metadata["effective_boundary_trim"], .00001)
        self.assertEqual(len(detailed.edges) - len(detailed.nodes) + len(detailed.connected_components()), 1)
        routing = simplify_corridor_graph(detailed, world)
        traffic = extract_traffic_resources(detailed, routing, world)
        self.assertEqual(len(routing.edges) - len(routing.nodes) + len(routing.connected_components()), 1)
        from shapely.geometry import Point
        self.assertTrue(any(LineString(g).distance(Point(6, .00001)) < 1e-9
                            for resource in traffic.resources.values() for g in resource.geometry))

    def test_angled_contacts_do_not_become_opposed_due_to_absolute_tolerance(self):
        world = {"bounds": [0, 0, 2, 2], "obstacles": [[[1, 0], [2, 1], [2, 0]]]}
        witness = WidthAuditor(FreeSpace(world)).witness((.9999585786437627, .0001))
        self.assertGreaterEqual(len(witness.contacts), 2)
        self.assertEqual(witness.classification, "local_clearance_only")

    def test_subresolution_positive_gap_fails_explicitly_and_is_preserved_at_finer_scale(self):
        from corridor_graph import GeometryConfig
        world = {"bounds": [0, 0, 12, 6], "obstacles": [rectangle(2, .0000004, 10, 4)]}
        with self.assertRaisesRegex((ValueError, AssertionError), "quantization|coordinate_scale|quantized"):
            build_corridor_graph(world)
        detailed = build_corridor_graph(world, GeometryConfig(coordinate_scale=10_000_000))
        self.assertEqual(len(detailed.edges) - len(detailed.nodes) + len(detailed.connected_components()), 1)

    def test_width_envelope_prevents_threshold_noise_fragmentation_without_clamping(self):
        world = {"bounds": [0, 0, 12, 3.00008], "obstacles":
                 [rectangle(2, 0, 4, .00008), rectangle(8, 0, 10, .00008)]}
        w, d, r = reference_graph(world, [(1.6, 1.50004), (10.4, 1.50004)], [(0, 1)])
        core_only = extract_traffic_resources(d, r, w, TrafficConfig(width_hysteresis=0))
        envelope = extract_traffic_resources(d, r, w)
        self.assertEqual(len(core_only.resources), 2)
        self.assertEqual(len(envelope.resources), 1)
        resource, = envelope.resources.values()
        self.assertAlmostEqual(resource.min_width, 2.99992)
        self.assertGreater(resource.representative_width, 3)
        self.assertEqual(envelope.metadata["width_class_envelope"], 3.0001)

    def test_exact_capsules_find_separate_necks_inside_one_coarse_chord(self):
        from corridor_graph.extraction import make_edge
        from corridor_graph import RoutingEdge, SourceSpan
        world = {"bounds": [0, 0, 14, 8], "obstacles":
                 [rectangle(x, 0, x + 1, 3) for x in (3, 9)] + [rectangle(x, 5, x + 1, 8) for x in (3, 9)]}
        w, d, r = reference_graph(world, [(1.6, 4), (12.4, 4)], [(0, 1)])
        coarse = make_edge(0, 0, 1, [(1.6, 4), (12.4, 4)], FreeSpace(w))
        d.edges[0] = coarse
        r.edges[0] = RoutingEdge(0, 0, 1, coarse.geometry, coarse.length, coarse.min_width,
                                coarse.min_width_position, (SourceSpan(0, 0, 1),))
        traffic = extract_traffic_resources(d, r, w)
        self.assertEqual(len(traffic.resources), 2)
        self.assertTrue(all(abs(x.min_width - 2) < 1e-10 for x in traffic.resources.values()))
        dense_w, dense_d, dense_r = reference_graph(world, [(1.6, 4), (12.4, 4)], [(0, 1)])
        dense = extract_traffic_resources(dense_d, dense_r, dense_w)
        np.testing.assert_allclose([(i.start, i.end) for i in traffic.edge_intervals[0]],
                                   [(i.start, i.end) for i in dense.edge_intervals[0]], atol=1e-10)

    def test_trimmed_taper_is_annotated_and_preserved(self):
        world = {"bounds": [0, 0, 10, 6], "obstacles": [[[0, 0], [5, 2], [5, 6], [0, 6]]]}
        # Exact bisector between y=0 and y=.4*x, trimmed where clearance=.0001.
        slope = .4 / (1 + np.sqrt(1 + .4**2))
        tip = (.0001 / slope, .0001)
        w, d, r = reference_graph(world, [tip, (2, 2 * slope)], [(0, 1)])
        traffic = extract_traffic_resources(d, r, w)
        resource, = traffic.resources.values()
        self.assertEqual(resource.geometric_classification, "boundary_taper")
        self.assertEqual(len(resource.boundary_terminations), 1)
        pair, = resource.boundary_terminations[0]["feature_pairs"]
        self.assertEqual(pair["feature_separation"], 0)
        self.assertAlmostEqual(resource.min_width, .0002)
        self.assertEqual(set(traffic.edge_intervals), {0})

    def test_overlap_relationships_and_junction_connections_are_explicit(self):
        traffic = self.traffic["narrow_tee"]
        junction, = self.junctions("narrow_tee")
        relations = [r for r in traffic.relationships if junction.id in r["resource_ids"]]
        self.assertEqual(len(relations), 3)
        self.assertTrue(all(r["relation"] == "overlap" for r in relations))
        self.assertTrue(all(r["shared_routing_intervals"] for r in relations))
        self.assertEqual(set(junction.possible_connections), {(a, b) for a in range(3) for b in range(3)})

    def test_junction_ports_clip_to_stored_polygon_for_oblique_arms(self):
        from shapely.geometry import Polygon, Point
        world = {"bounds": [0, 0, 10, 10], "obstacles": []}
        w, d, r = reference_graph(world, [(5, 5), (8, 6.2), (2, 6.2), (5, 2)], [(0, 1), (0, 2), (0, 3)])
        traffic = extract_traffic_resources(d, r, w, TrafficConfig(detect_narrow=False))
        region, = traffic.resources.values()
        polygon = Polygon(region.region)
        for port in region.entrances:
            self.assertEqual(port.boundary_kind, "junction_boundary")
            self.assertLess(Point(port.position).distance(polygon.boundary), 1e-10)
        self.assertTrue(all(polygon.buffer(1e-10).covers(LineString(g)) for g in region.geometry))

    def test_schema_roundtrip_contains_geometry_and_supports_route_usage_without_ecm(self):
        original = self.traffic["narrow_tee"]
        world, detailed, routing = self.fixtures["narrow_tee"]
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "offline.json"
            path.write_text(json.dumps(original.to_dict(), allow_nan=False))
            loaded = load_traffic_resources(path)
            validate_traffic_resources(loaded, detailed, routing, FreeSpace(world))
            self.assertEqual(loaded.routing_graph().to_dict(), routing.to_dict())
            self.assertEqual(path_resource_usage(loaded, loaded.routing_graph(), [DirectedEdge(0, -1), DirectedEdge(2)]),
                             path_resource_usage(original, routing, [DirectedEdge(0, -1), DirectedEdge(2)]))
            self.assertEqual(loaded.map_geometry, original.map_geometry)
            self.assertEqual(len(loaded.boundary_features), len(original.boundary_features))
            for edge in loaded.routing_geometry["edges"]:
                self.assertEqual(len(edge["reference_geometry"]), len(edge["reference_clearances"]))
                self.assertGreater(edge["continuous_min_clearance"], 0)
            value = json.loads(path.read_text())
            value["routing_geometry"]["edges"][0]["reference_geometry"][0][0] += .1
            path.write_text(json.dumps(value))
            with self.assertRaisesRegex(ValueError, "geometry cache hash"):
                load_traffic_resources(path)
            path.write_text(json.dumps(original.to_dict()))
            value = json.loads(path.read_text())
            value["map_geometry"]["bounds"][0] -= 1
            path.write_text(json.dumps(value))
            with self.assertRaisesRegex(ValueError, "map geometry hash"):
                load_traffic_resources(path)

    def test_loaded_custom_width_class_uses_saved_geometric_configuration(self):
        world, detailed, routing = self.fixtures["wide_crossing"]
        traffic = extract_traffic_resources(detailed, routing, world, TrafficConfig(narrow_width=7.))
        self.assertTrue(any(r.kind == "narrow_corridor" and r.min_width > 3 for r in traffic.resources.values()))
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "offline.json"
            path.write_text(json.dumps(traffic.to_dict()))
            loaded = load_traffic_resources(path)
            validate_traffic_resources(loaded, detailed, routing, FreeSpace(world))


if __name__ == "__main__":
    unittest.main()
