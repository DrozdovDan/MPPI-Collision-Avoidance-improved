"""Traffic regions and their directed incidence on an unchanged routing graph."""

from dataclasses import asdict, dataclass, field
import json
import sys


@dataclass(frozen=True)
class ResourcePort:
    edge_id: int
    distance: float  # arclength from routing u along its detailed reference walk
    position: tuple[float, float]
    node_id: int | None = None
    arm: str | None = None  # junction attachment at routing u/v; distinguishes loop arms
    boundary_kind: str = "routing_edge_cut"


@dataclass(frozen=True)
class DetailedInterval:
    edge_id: int
    first_distance: float
    last_distance: float  # oriented distances on the original detailed edge


@dataclass(frozen=True)
class ResourceInterval:
    resource_id: int
    edge_id: int
    start: float
    end: float
    direction: int  # +1 along canonical narrow chain; 0 for a junction arm
    geometry: tuple
    start_port: ResourcePort
    end_port: ResourcePort
    source_intervals: tuple[DetailedInterval, ...] = ()


@dataclass(frozen=True)
class TrafficResource:
    id: int
    kind: str
    source_node_ids: tuple[int, ...]  # routing nodes
    source_edge_ids: tuple[int, ...]  # routing edges
    detailed_node_ids: tuple[int, ...]
    detailed_edge_ids: tuple[int, ...]
    geometry: tuple[tuple[tuple[float, float], ...], ...]  # one chain or multiple junction arms
    region: tuple[tuple[float, float], ...]  # optional conflict polygon, strictly inside free space
    length: float  # chain length, or sum of junction arm lengths; NOT a longitudinal capacity
    min_width: float  # exact continuous clearance diameter along reference geometry
    min_width_position: tuple[float, float]
    representative_width: float  # arclength-weighted median of local width estimates
    entrances: tuple[ResourcePort, ...]
    exits: tuple[ResourcePort, ...]  # geometric ports; no traffic direction policy
    min_clearance: float
    representative_clearance: float
    geometric_classification: str
    local_minima: tuple = ()
    boundary_terminations: tuple = ()
    possible_connections: tuple[tuple[int, int], ...] = ()  # junction port indices, not movement-pair conflicts
    junction_node_id: int | None = None
    closed: bool = False
    width_witness: object = None


@dataclass(frozen=True)
class DirectedEdge:
    edge_id: int
    direction: int = 1  # +1 u->v, -1 v->u (also explicit for self-loops)


@dataclass(frozen=True)
class ResourceTraversal:
    resource_id: int
    entry: ResourcePort
    exit: ResourcePort
    direction: str  # forward/reverse, or port-to-port for junctions
    traversal_length: float


@dataclass
class TrafficResourceGraph:
    resources: dict[int, TrafficResource] = field(default_factory=dict)
    edge_intervals: dict[int, tuple[ResourceInterval, ...]] = field(default_factory=dict)
    metadata: dict = field(default_factory=dict)
    map_geometry: dict = field(default_factory=dict)
    boundary_features: tuple = ()
    routing_geometry: dict = field(default_factory=dict)
    relationships: tuple = ()

    def directed_mapping(self, edge_id, direction=1):
        """Explicit mappings in traversal order; unconstrained edges return ()."""
        if direction not in (-1, 1):
            raise ValueError("direction must be +1 or -1")
        intervals = self.edge_intervals[edge_id]
        if direction == 1:
            return tuple((i.resource_id, i.start_port, i.end_port, i.direction) for i in intervals)
        ordered = sorted(intervals, key=lambda i: (-i.end, -i.start, i.resource_id))
        return tuple((i.resource_id, i.end_port, i.start_port, -i.direction) for i in ordered)

    def to_dict(self):
        return {"resources": [asdict(r) for r in self.resources.values() if r.kind == "narrow_corridor"],
                "junction_regions": [asdict(r) for r in self.resources.values() if r.kind == "junction_region"],
                "edge_intervals": {str(k): [asdict(i) for i in v] for k, v in self.edge_intervals.items()},
                "relationships": self.relationships, "map_geometry": self.map_geometry,
                "boundary_features": self.boundary_features, "routing_geometry": self.routing_geometry,
                "metadata": self.metadata}

    def geometry_size(self):
        """Actual compact JSON bytes and approximate offline Python object size.

        Includes polygons, routing reference profiles, feature witnesses,
        provenance and relationships; this is not a binary runtime format.
        """
        seen = set()
        def deep_size(value):
            if id(value) in seen:
                return 0
            seen.add(id(value))
            size = sys.getsizeof(value)
            if isinstance(value, dict):
                size += sum(deep_size(k) + deep_size(v) for k, v in value.items())
            elif isinstance(value, (tuple, list)):
                size += sum(deep_size(v) for v in value)
            elif hasattr(value, "__dict__"):
                size += deep_size(vars(value))
            return size
        return {"python_object_bytes_estimate": deep_size(self),
                "compact_json_bytes": len(json.dumps(self.to_dict(), separators=(",", ":"), allow_nan=False).encode())}

    def routing_graph(self):
        from .traffic_serialization import routing_from_cache
        return routing_from_cache(self.routing_geometry)
