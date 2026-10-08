"""Reload the offline geometric output without rebuilding ECM or routing."""

from dataclasses import fields
import json
from pathlib import Path

from .routing_model import RoutingCorridorGraph, RoutingEdge, RoutingNode, SourceSpan
from .traffic_geometry import geometry_hash
from .traffic_model import (DetailedInterval, ResourceInterval, ResourcePort, TrafficResource,
                            TrafficResourceGraph)
from .width_audit import BoundaryContact, WidthWitness


def _tuples(value):
    if isinstance(value, list):
        return tuple(_tuples(v) for v in value)
    if isinstance(value, dict):
        return {k: _tuples(v) for k, v in value.items()}
    return value


def _witness(raw):
    raw = _tuples(raw)
    raw["contacts"] = tuple(BoundaryContact(**c) for c in raw["contacts"])
    return WidthWitness(**raw)


def routing_from_cache(cache):
    nodes = {n["id"]: RoutingNode(**_tuples(n)) for n in cache["nodes"]}
    edge_fields = {f.name for f in fields(RoutingEdge)}
    edges = {}
    for raw in cache["edges"]:
        edge = {k: _tuples(v) for k, v in raw.items() if k in edge_fields}
        edge["source_spans"] = tuple(SourceSpan(**s) for s in edge["source_spans"])
        edges[edge["id"]] = RoutingEdge(**edge)
    return RoutingCorridorGraph(nodes, edges, cache.get("metadata", {}))


def load_traffic_resources(path):
    """Load schema v2 geometry and verify its map/routing identity.

    JSON supplies all polygon and centerline data needed by future queries.
    It contains no robot profile, traffic policy or feasibility answers.
    """
    raw = json.loads(Path(path).read_text())
    if raw["metadata"].get("schema_version") != 2:
        raise ValueError("expected robot-independent geometric traffic schema version 2")
    resources = {}
    for record in raw["resources"] + raw["junction_regions"]:
        r = _tuples(record)
        for name in ("entrances", "exits"):
            r[name] = tuple(ResourcePort(**p) for p in r[name])
        r["local_minima"] = tuple(_witness(w) for w in record["local_minima"])
        r["width_witness"] = _witness(record["width_witness"])
        resource = TrafficResource(**r)
        if resource.id in resources:
            raise ValueError("duplicate geometric resource ID")
        resources[resource.id] = resource
    intervals = {}
    for edge_id, records in raw["edge_intervals"].items():
        parts = []
        for record in records:
            i = _tuples(record)
            i["start_port"], i["end_port"] = ResourcePort(**i["start_port"]), ResourcePort(**i["end_port"])
            i["source_intervals"] = tuple(DetailedInterval(**s) for s in i["source_intervals"])
            parts.append(ResourceInterval(**i))
        intervals[int(edge_id)] = tuple(parts)
    traffic = TrafficResourceGraph(resources, intervals, raw["metadata"], raw["map_geometry"],
                                   tuple(raw["boundary_features"]), raw["routing_geometry"], tuple(raw["relationships"]))
    if geometry_hash(traffic.map_geometry) != traffic.metadata["map_sha256"]:
        raise ValueError("map geometry hash mismatch")
    cached = {"map_geometry": traffic.map_geometry, "boundary_features": traffic.boundary_features,
              "routing_geometry": traffic.routing_geometry}
    if geometry_hash(cached) != traffic.metadata["geometry_cache_sha256"]:
        raise ValueError("geometry cache hash mismatch")
    if geometry_hash(traffic.routing_graph().to_dict()) != traffic.metadata["routing_sha256"]:
        raise ValueError("routing geometry hash mismatch")
    if set(intervals) != set(traffic.routing_graph().edges):
        raise ValueError("missing routing edge mapping")
    for edge_id, pieces in intervals.items():
        for piece in pieces:
            if piece.edge_id != edge_id or piece.resource_id not in resources:
                raise ValueError("invalid edge-to-resource reference")
    return traffic
