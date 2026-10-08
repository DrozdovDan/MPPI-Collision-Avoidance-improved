"""Physical corridor graph construction for existing Bench-MR polygon worlds."""

from dataclasses import asdict

import numpy as np
from shapely.geometry import Point

from .bottlenecks import BottleneckConfig, detect_bottlenecks, split_bottlenecks
from .extraction import extract_graph
from .geometry import FreeSpace, GeometryConfig, build_skeleton, _independent_holes
from .model import CorridorEdge, CorridorGraph, CorridorNode, WidthSample
from .routing_model import RoutingCorridorGraph, RoutingEdge, RoutingNode, SourceSpan
from .simplification import SimplificationConfig, simplify_corridor_graph
from .runtime import load_runtime
from .traffic_model import (TrafficResource, TrafficResourceGraph, ResourcePort,
                            ResourceInterval, DirectedEdge, ResourceTraversal)
from .traffic_extraction import TrafficConfig, extract_traffic_resources, validate_traffic_resources
from .traffic_usage import path_resource_usage
from .width_audit import audit_widths, WidthAuditor
from .traffic_serialization import load_traffic_resources

DetailedCorridorGraph = CorridorGraph


def validate_graph(graph, space):
    """Check full polylines, physical profiles, and one graph per free component.

    Full Shapely line checks cover every chord, not just sampled points.
    Raise AssertionError with diagnostic context rather than silently accepting
    missing/disconnected geometry. An empty, fully blocked world is supported.
    """
    for node in graph.nodes.values():
        assert space.contains_points([node.position]).all(), f"node {node.id} outside free space"
    for edge in graph.edges.values():
        assert edge.u in graph.nodes and edge.v in graph.nodes, f"edge {edge.id} missing endpoint"
        assert edge.geometry[0] == graph.nodes[edge.u].position
        assert edge.geometry[-1] == graph.nodes[edge.v].position
        assert space.contains_points(edge.geometry).all(), f"edge {edge.id} sample outside free space"
        assert space.line_is_free(edge.geometry), f"edge {edge.id} crosses obstacle or wall"
        profile = edge.width_profile
        assert len(profile) == len(edge.geometry)
        assert tuple(s.position for s in profile) == edge.geometry
        distances = np.r_[0., np.cumsum(np.linalg.norm(np.diff(edge.geometry, axis=0), axis=1))]
        np.testing.assert_allclose([s.distance for s in profile], distances, atol=1e-9)
        np.testing.assert_allclose([s.clearance for s in profile], space.clearances(edge.geometry), atol=1e-9)
        np.testing.assert_allclose([s.width for s in profile], [2 * s.clearance for s in profile], atol=1e-9)
        assert np.isclose(edge.length, distances[-1]) and edge.length > 0
        minimum = min(profile, key=lambda s: s.width)
        assert np.isclose(edge.min_width, minimum.width)
        assert np.isclose(edge.min_clearance, minimum.clearance)
        assert edge.min_width_position == minimum.position
    components = graph.connected_components()
    assignment = []
    for component in components:
        containing = {i for node in component for i, polygon in enumerate(space.components)
                      if polygon.contains(Point(graph.nodes[node].position))}
        assert len(containing) == 1, "graph component spans disconnected free-space components"
        assignment.append(containing.pop())
    assert sorted(assignment) == list(range(len(space.components))), (
        f"{len(components)} graph components for {len(space.components)} free-space components; "
        "check coordinate scale, curve spacing, and boundary_trim")
    cycles = len(graph.edges) - len(graph.nodes) + len(components)
    expected = sum(_independent_holes(p) for p in space.components)
    assert cycles == expected, (
        f"graph has {cycles} alternatives for {expected} independent free-space holes; "
        "quantization or clipping changed passage topology; increase coordinate_scale")


def build_corridor_graph(world, geometry_config=None, bottleneck_config=None, validate=True):
    geometry_config = geometry_config or GeometryConfig()
    bottleneck_config = bottleneck_config or BottleneckConfig()
    space = FreeSpace(world)
    skeleton = build_skeleton(space, geometry_config)
    graph = extract_graph(skeleton, space)
    graph = split_bottlenecks(graph, space, bottleneck_config)
    graph.metadata.update(
        geometry_config=asdict(geometry_config), bottleneck_config=asdict(bottleneck_config),
        bounds=list(space.bounds), free_space_components=len(space.components),
        skeleton_vertices=len(skeleton.positions), skeleton_edges=len(skeleton.edges),
        clearance_includes_outer_boundary=True,
        effective_boundary_trim=skeleton.effective_boundary_trim,
        effective_merge_tolerance=skeleton.effective_merge_tolerance,
    )
    if validate:
        validate_graph(graph, space)
    return graph


__all__ = ["CorridorGraph", "CorridorNode", "CorridorEdge", "WidthSample", "FreeSpace",
           "GeometryConfig", "BottleneckConfig", "build_corridor_graph", "validate_graph",
           "detect_bottlenecks", "DetailedCorridorGraph", "RoutingCorridorGraph",
           "RoutingEdge", "RoutingNode", "SourceSpan", "SimplificationConfig",
           "simplify_corridor_graph", "load_runtime", "TrafficResource", "TrafficResourceGraph",
           "ResourcePort", "ResourceInterval", "DirectedEdge", "ResourceTraversal", "TrafficConfig",
           "extract_traffic_resources", "validate_traffic_resources", "path_resource_usage",
           "audit_widths", "WidthAuditor", "load_traffic_resources"]
