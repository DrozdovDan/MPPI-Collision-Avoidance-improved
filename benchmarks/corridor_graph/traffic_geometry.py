"""Offline geometry cache, precise ECM coverage and explicit region relationships."""

from dataclasses import asdict
import hashlib
import json
import math

from shapely.geometry import LineString, Polygon, mapping
from shapely.ops import unary_union
from shapely.strtree import STRtree

from .traffic_model import DetailedInterval
from .width_audit import continuous_minimum


def geometry_hash(value):
    return hashlib.sha256(json.dumps(value, separators=(",", ":"), sort_keys=True, allow_nan=False).encode()).hexdigest()


def detailed_intervals(edge, detailed, start, end):
    result, offset = [], 0.
    for span in edge.source_spans:
        profile = detailed.edges[span.edge_id].width_profile
        first, last = profile[span.first].distance, profile[span.last].distance
        length = abs(last - first)
        left, right = max(start, offset), min(end, offset + length)
        if right > left:
            sign = 1 if last > first else -1
            result.append(DetailedInterval(span.edge_id, float(first + sign * (left - offset)),
                                           float(first + sign * (right - offset))))
        offset += length
    return tuple(result)


def cache_routing_geometry(detailed, routing, profiles, auditor):
    result = routing.to_dict()
    for record in result["edges"]:
        profile = profiles[record["id"]]
        minimum, position = continuous_minimum(profile[1], auditor.space)
        record.update(reference_geometry=profile[1].tolist(), reference_distances=profile[0].tolist(),
                      reference_clearances=auditor.space.clearances(profile[1]).tolist(),
                      continuous_min_clearance=minimum / 2, continuous_min_clearance_position=position,
                      minimum_boundary_witness=asdict(auditor.witness(position)))
    return result


def resource_relationships(traffic):
    """Centerline/disk intersection and shared routing coverage, never robot conflicts."""
    ids = list(traffic.resources)
    geometries = [Polygon(r.region) if r.region else unary_union([LineString(p) for p in r.geometry])
                  for r in traffic.resources.values()]
    tree, intervals, result = STRtree(geometries), {}, []
    for edge_id, pieces in traffic.edge_intervals.items():
        for i, a in enumerate(pieces):
            for b in pieces[:i]:
                if a.resource_id == b.resource_id:
                    continue
                start, end = max(a.start, b.start), min(a.end, b.end)
                if end >= start:
                    key = tuple(sorted((a.resource_id, b.resource_id)))
                    intervals.setdefault(key, []).append((edge_id, start, end))
    for i, geometry in enumerate(geometries):
        for j in tree.query(geometry, predicate="intersects"):
            if j <= i:
                continue
            key = (ids[i], ids[j])
            intersection = geometry.intersection(geometries[j])
            result.append({"resource_ids": key, "relation": "touch" if geometry.touches(geometries[j]) else "overlap",
                           "shared_routing_intervals": intervals.get(tuple(sorted(key)), []),
                           "intersection_geometry": mapping(intersection)})
    return tuple(sorted(result, key=lambda r: r["resource_ids"]))
