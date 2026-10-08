"""Routing graph with reversible provenance; dense geometry stays in the ECM."""

from dataclasses import asdict, dataclass, field

from .model import CorridorGraph


@dataclass(frozen=True)
class SourceSpan:
    """Inclusive, oriented sample range in a detailed edge (indices may decrease)."""

    edge_id: int
    first: int
    last: int

    def reversed(self):
        return SourceSpan(self.edge_id, self.last, self.first)

    def indices(self):
        direction = 1 if self.last > self.first else -1
        return range(self.first, self.last + direction, direction)


@dataclass(frozen=True)
class RoutingNode:
    id: int
    position: tuple[float, float]
    kind: str
    source_node_ids: tuple[int, ...]
    source_edge_samples: tuple[tuple[int, int], ...] = ()
    protected_reasons: tuple[str, ...] = ()
    local_width: float = 0.0


@dataclass(frozen=True)
class RoutingEdge:
    id: int
    u: int
    v: int
    geometry: tuple[tuple[float, float], ...]
    length: float  # length of the represented detailed walk, not its display shortcut
    min_width: float
    min_width_position: tuple[float, float]
    source_spans: tuple[SourceSpan, ...]
    bottleneck_node_ids: tuple[int, ...] = ()

    @property
    def source_edge_ids(self):
        return tuple(span.edge_id for span in self.source_spans)

    @property
    def simplified_geometry(self):
        return self.geometry

    def reference_geometry(self, detailed):
        """Recover the complete oriented reference polyline without dense copies."""
        return geometry_from_spans(self.source_spans, detailed)


def geometry_from_spans(spans, detailed):
    result = []
    for span in spans:
        points = [detailed.edges[span.edge_id].geometry[i] for i in span.indices()]
        if result:
            if result[-1] != points[0]:
                raise ValueError("source spans do not form a continuous detailed walk")
            result.extend(points[1:])
        else:
            result.extend(points)
    return tuple(result)


def metrics_from_spans(spans, detailed):
    length, minimum, location = 0., float("inf"), None
    for span in spans:
        profile = detailed.edges[span.edge_id].width_profile
        length += abs(profile[span.last].distance - profile[span.first].distance)
        for index in span.indices():
            if profile[index].width < minimum:
                minimum, location = profile[index].width, profile[index].position
    return float(length), float(minimum), location


@dataclass
class RoutingCorridorGraph(CorridorGraph):
    nodes: dict[int, RoutingNode] = field(default_factory=dict)
    edges: dict[int, RoutingEdge] = field(default_factory=dict)

    def to_dict(self):
        result = {"nodes": [asdict(n) for n in self.nodes.values()],
                  "edges": [], "metadata": self.metadata}
        for edge in self.edges.values():
            value = asdict(edge)
            value["source_edge_ids"] = list(edge.source_edge_ids)
            result["edges"].append(value)
        return result

    def runtime_arrays(self):
        from .runtime import runtime_arrays
        return runtime_arrays(self)

    def save_runtime(self, path):
        from .runtime import save_runtime
        return save_runtime(self, path)
