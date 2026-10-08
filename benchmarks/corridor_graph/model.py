"""Undirected embedded multigraph; lengths are suitable for future graph search."""

from dataclasses import asdict, dataclass, field


@dataclass(frozen=True)
class CorridorNode:
    id: int
    position: tuple[float, float]
    kind: str = "terminal"  # junction, bottleneck, or cycle_anchor


@dataclass(frozen=True)
class WidthSample:
    distance: float  # arclength from u, in map units
    position: tuple[float, float]
    clearance: float
    width: float


@dataclass(frozen=True)
class CorridorEdge:
    id: int
    u: int
    v: int
    geometry: tuple[tuple[float, float], ...]
    length: float
    min_clearance: float
    min_width: float
    min_width_position: tuple[float, float]
    width_profile: tuple[WidthSample, ...]
    bottleneck_positions: tuple[tuple[float, float], ...] = ()


@dataclass
class CorridorGraph:
    nodes: dict[int, CorridorNode] = field(default_factory=dict)
    edges: dict[int, CorridorEdge] = field(default_factory=dict)
    metadata: dict = field(default_factory=dict)

    def adjacency(self):
        """node -> [(neighbor, edge_id, length)], preserving parallel edges/loops."""
        result = {node: [] for node in self.nodes}
        for edge in self.edges.values():
            result[edge.u].append((edge.v, edge.id, edge.length))
            result[edge.v].append((edge.u, edge.id, edge.length))
        return result

    def connected_components(self):
        adjacency = self.adjacency()
        unseen, components = set(self.nodes), []
        while unseen:
            stack, component = [min(unseen)], set()
            while stack:
                node = stack.pop()
                if node in component:
                    continue
                component.add(node)
                stack.extend(neighbor for neighbor, _, _ in adjacency[node]
                             if neighbor not in component)
            unseen -= component
            components.append(component)
        return components

    def to_dict(self):
        """JSON-compatible output; this serializes the graph, not a new map format."""
        return {"nodes": [asdict(n) for n in self.nodes.values()],
                "edges": [asdict(e) for e in self.edges.values()],
                "metadata": self.metadata}
