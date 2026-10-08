"""Exact exposed-segment witnesses for local widths, in unscaled map units."""

from dataclasses import asdict, dataclass
import math

import numpy as np
from shapely.geometry import LineString, Point
from shapely.ops import nearest_points

from .routing_model import metrics_from_spans


@dataclass(frozen=True)
class BoundaryContact:
    segment_id: int
    kind: str
    segment: tuple
    position: tuple
    distance: float


@dataclass(frozen=True)
class WidthWitness:
    position: tuple
    local_width: float
    contacts: tuple[BoundaryContact, ...]
    opposite_pair: tuple[int, int] | None
    classification: str

    @property
    def clearance(self):
        return self.local_width / 2


class WidthAuditor:
    """Use exposed union boundaries, including clipped obstacles and outer wall.

    Tied segments at a corner are retained, but two copies of the same contact
    cannot establish an opposing cross-section. An opposing pair is evidence of
    a local physical gap, not a global separating cut or a vehicle turning test.
    """

    def __init__(self, space, tolerance=1e-5):
        if not math.isfinite(tolerance) or tolerance <= 0:
            raise ValueError("audit tolerance must be positive and finite")
        self.space, self.tolerance = space, tolerance
        self.segments = []
        self.features = []
        for component_id, component in enumerate(space.components):
            for ring_id, ring in enumerate((component.exterior, *component.interiors)):
                for a, b in zip(ring.coords[:-1], ring.coords[1:]):
                    segment = LineString([a, b])
                    if segment.length:
                        kind = "outer" if space.workspace.boundary.covers(segment) else "obstacle"
                        self.segments.append((segment, kind))
                        self.features.append({"id": len(self.features), "kind": kind,
                                              "component_id": component_id, "ring_id": ring_id,
                                              "segment": tuple(segment.coords)})
        self.starts = np.asarray([s.coords[0] for s, _ in self.segments]).reshape(-1, 2)
        self.deltas = np.asarray([np.subtract(s.coords[1], s.coords[0]) for s, _ in self.segments]).reshape(-1, 2)
        self.squared_lengths = np.sum(self.deltas**2, axis=1)
        self.precision = 32 * np.finfo(float).eps * max(1., *map(abs, space.bounds))

    def witness(self, position):
        point = Point(position)
        clearance = float(point.distance(self.space.boundary))
        contacts = []
        projections = np.clip(np.sum((np.asarray(position) - self.starts) * self.deltas, axis=1)
                              / self.squared_lengths, 0., 1.)
        nearest = self.starts + projections[:, None] * self.deltas
        distances = np.linalg.norm(nearest - position, axis=1)
        # An absolute 1e-5 tie tolerance overwhelms a 1e-4 or smaller gap.
        # Use relative local precision for contacts, retaining genuine tiny gaps.
        tie = max(self.precision, min(self.tolerance, clearance * 1e-4))
        for i in np.flatnonzero(distances <= clearance + tie):
            segment, kind = self.segments[i]
            contacts.append(BoundaryContact(int(i), kind, tuple(segment.coords),
                                            tuple(map(float, nearest[i])), float(distances[i])))
        pair, best = None, 0.
        for i, a in enumerate(contacts):
            for j, b in enumerate(contacts[:i]):
                gap = math.dist(a.position, b.position)
                if gap > best:
                    best, pair = gap, (j, i)
        opposite = False
        if pair is not None and clearance > self.precision:
            a, b = (np.asarray(contacts[i].position) - position for i in pair)
            cosine = float(np.dot(a, b) / (np.linalg.norm(a) * np.linalg.norm(b)))
            opposite = best > self.precision and cosine <= -.995
        if opposite:
            a, b = (contacts[i].position for i in pair)
            # Endpoints belong to boundaries; the open gap must be free.
            chord = LineString([a, b])
            inner = LineString([chord.interpolate(.00001, normalized=True),
                                chord.interpolate(.99999, normalized=True)])
            opposite = self.space.geometry.contains(inner)
        category = ("opposed_boundary_gap" if opposite else "local_clearance_only")
        if clearance <= self.precision:
            category = "precision_limited_clearance"
        return WidthWitness(tuple(map(float, position)), 2 * clearance, tuple(contacts),
                            pair if opposite else None, category)

    def boundary_limit(self, position, trim):
        """Explain a numerical trim endpoint using physical feature separation.

        A zero feature separation is a true polygon contact/taper. A positive
        separation is retained as a real near-gap, never rounded to zero.
        This is diagnostic evidence, not a reason to delete a graph edge.
        """
        witness = self.witness(position)
        contacts = witness.contacts
        pairs = [(a, b) for i, a in enumerate(contacts) for b in contacts[:i]
                 if math.dist(a.position, b.position) > self.precision]
        records = []
        for a, b in pairs:
            sa, sb = self.segments[a.segment_id][0], self.segments[b.segment_id][0]
            pa, pb = nearest_points(sa, sb)
            records.append({"feature_ids": [a.segment_id, b.segment_id],
                            "feature_separation": float(sa.distance(sb)),
                            "nearest_feature_points": [tuple(pa.coords[0]), tuple(pb.coords[0])],
                            "relation": "touching_boundaries" if sa.intersects(sb) else "positive_boundary_gap"})
        return {"position": tuple(position), "clearance": witness.clearance,
                "configured_boundary_trim": trim, "classification": "construction_trim_endpoint",
                "feature_pairs": records, "witness": asdict(witness)}


def audit_widths(detailed, routing, space, small_width=.5, tolerance=1e-5):
    """Check stored profiles/scalars and report continuous-chord discrepancies.

    No value is clamped or dropped. Small widths are diagnostics, not errors.
    Failure of a stored-width invariant raises instead of masking bad inputs.
    """
    if not math.isfinite(small_width) or small_width <= 0:
        raise ValueError("small_width must be finite and positive")
    auditor = WidthAuditor(space, tolerance)
    errors, sample_count = [], 0
    for edge in detailed.edges.values():
        actual = 2 * space.clearances(edge.geometry)
        stored = np.asarray([sample.width for sample in edge.width_profile])
        assert len(actual) == len(stored), f"detailed edge {edge.id} profile mismatch"
        errors.extend(abs(actual - stored))
        assert np.allclose(stored, [2 * s.clearance for s in edge.width_profile], atol=tolerance, rtol=0)
        assert abs(edge.min_width - min(stored)) <= tolerance
        assert abs(edge.min_clearance * 2 - edge.min_width) <= tolerance
        assert edge.min_width_position == min(edge.width_profile, key=lambda s: s.width).position
        sample_count += len(stored)
    assert max(errors, default=0.) <= tolerance, "stored widths disagree with exact boundary distances"
    for node in routing.nodes.values():
        assert abs(node.local_width - 2 * space.clearances([node.position])[0]) <= tolerance
    records, chord_errors, terminal_limits = [], [], []
    trim = detailed.metadata.get("effective_boundary_trim") or detailed.metadata.get("geometry_config", {}).get("boundary_trim", .0001)
    for node in routing.nodes.values():
        if node.kind == "terminal" and abs(node.local_width / 2 - trim) <= max(auditor.precision, trim * 1e-4):
            terminal_limits.append({"routing_node_id": node.id, **auditor.boundary_limit(node.position, trim)})
    for edge in routing.edges.values():
        _, minimum, position = metrics_from_spans(edge.source_spans, detailed)
        assert abs(edge.min_width - minimum) <= tolerance
        assert math.dist(edge.min_width_position, position) <= tolerance
        line = LineString(edge.reference_geometry(detailed))
        continuous = 2 * line.distance(space.boundary)
        chord_errors.append(max(0., edge.min_width - continuous))
        witness = auditor.witness(edge.min_width_position)
        flags = []
        if edge.min_width < small_width:
            flags.append("small_width")
        if witness.classification != "opposed_boundary_gap":
            flags.append(witness.classification)
        if edge.min_width - continuous > tolerance:
            flags.append("sampled_min_above_continuous_min")
        if any(routing.nodes[n].kind == "terminal" and
               math.dist(routing.nodes[n].position, edge.min_width_position) <= tolerance and
               routing.nodes[n].local_width <= 2 * trim + tolerance for n in (edge.u, edge.v)):
            flags.append("trimmed_boundary_terminal")
        if flags:
            records.append({"routing_edge_id": edge.id, "flags": flags,
                            "stored_min_width": edge.min_width, "continuous_min_width": continuous,
                            "witness": asdict(witness)})
    return {"units": "map units", "clearance_includes_outer_boundary": True,
            "sample_count": sample_count, "max_profile_error": float(max(errors, default=0.)),
            "max_sampled_overestimate": max(chord_errors, default=0.),
            "small_width_threshold": small_width, "tolerance": tolerance,
            "small_width_edge_count": sum("small_width" in r["flags"] for r in records),
            "local_clearance_only_edge_count": sum("local_clearance_only" in r["flags"] for r in records),
            "trimmed_boundary_terminal_edge_count": sum("trimmed_boundary_terminal" in r["flags"] for r in records),
            "sampled_overestimate_edge_count": sum("sampled_min_above_continuous_min" in r["flags"] for r in records),
            "terminal_limits": terminal_limits,
            "geometry_policy": "exact exposed segments; graph and widths unchanged; trim endpoints annotated",
            "diagnostics": records}


def continuous_minimum(points, space):
    """Exact minimum over every reference chord; includes between-sample dips."""
    line = LineString(points)
    point, _ = nearest_points(line, space.boundary)
    return 2 * line.distance(space.boundary), tuple(point.coords[0])
