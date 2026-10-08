"""Versioned numeric-array runtime graph: no dense profiles or debug provenance."""

from dataclasses import dataclass
from pathlib import Path
from types import SimpleNamespace

import numpy as np


KIND_FLAGS = {"terminal": 0, "junction": 1, "bottleneck": 2,
              "transition": 3, "cycle_anchor": 4, "region": 5}


def runtime_arrays(graph):
    """Flat float64/int32 arrays; float64 protects very thin physical corridors."""
    nodes = sorted(graph.nodes.values(), key=lambda node: node.id)
    edges = sorted(graph.edges.values(), key=lambda edge: edge.id)
    geometry, offsets, bottlenecks, bottleneck_offsets = [], [0], [], [0]
    for edge in edges:
        geometry.extend(edge.geometry)
        offsets.append(len(geometry))
        bottlenecks.extend(edge.bottleneck_node_ids)
        bottleneck_offsets.append(len(bottlenecks))
    return {
        "version": np.array([1], dtype="<u2"),
        "node_ids": np.array([n.id for n in nodes], dtype="<i4"),
        "node_positions": np.array([n.position for n in nodes], dtype="<f8").reshape(-1, 2),
        "node_flags": np.array([KIND_FLAGS[n.kind] for n in nodes], dtype="u1"),
        "node_widths": np.array([n.local_width for n in nodes], dtype="<f8"),
        "edge_ids": np.array([e.id for e in edges], dtype="<i4"),
        "edge_endpoints": np.array([(e.u, e.v) for e in edges], dtype="<i4").reshape(-1, 2),
        "edge_lengths": np.array([e.length for e in edges], dtype="<f8"),
        "edge_min_widths": np.array([e.min_width for e in edges], dtype="<f8"),
        "edge_min_width_positions": np.array([e.min_width_position for e in edges], dtype="<f8").reshape(-1, 2),
        "geometry_offsets": np.array(offsets, dtype="<i4"),
        "geometry_points": np.array(geometry, dtype="<f8").reshape(-1, 2),
        "bottleneck_offsets": np.array(bottleneck_offsets, dtype="<i4"),
        "bottleneck_node_ids": np.array(bottlenecks, dtype="<i4"),
    }


def save_runtime(graph, path):
    """Uncompressed NPZ: standard binary array container, no pickle dependencies."""
    arrays = runtime_arrays(graph)
    with Path(path).open("wb") as stream:
        np.savez(stream, **arrays)
    return {"array_bytes": sum(value.nbytes for value in arrays.values()),
            "file_bytes": Path(path).stat().st_size}


@dataclass
class RuntimeRoutingGraph:
    arrays: dict[str, np.ndarray]

    def adjacency(self):
        result = {int(node): [] for node in self.arrays["node_ids"]}
        for edge, (u, v), length in zip(self.arrays["edge_ids"], self.arrays["edge_endpoints"],
                                      self.arrays["edge_lengths"]):
            result[int(u)].append((int(v), int(edge), float(length)))
            result[int(v)].append((int(u), int(edge), float(length)))
        return result

    def edge_geometry(self, edge_id):
        matches = np.flatnonzero(self.arrays["edge_ids"] == edge_id)
        if not len(matches):
            raise KeyError(edge_id)
        index = int(matches[0])
        start, end = self.arrays["geometry_offsets"][index:index + 2]
        return self.arrays["geometry_points"][start:end]


def load_runtime(path):
    with np.load(path, allow_pickle=False) as archive:
        arrays = {name: archive[name] for name in archive.files}
    expected_types = runtime_arrays(SimpleNamespace(nodes={}, edges={}))
    required = set(expected_types)
    if set(arrays) != required or not np.array_equal(arrays["version"], [1]):
        raise ValueError("unsupported or incomplete runtime graph")
    for name, value in arrays.items():
        if value.dtype != expected_types[name].dtype or not np.isfinite(value).all():
            raise ValueError(f"invalid numeric array: {name}")
    n, m = len(arrays["node_ids"]), len(arrays["edge_ids"])
    shapes = {"version": (1,), "node_ids": (n,), "node_positions": (n, 2), "node_flags": (n,), "node_widths": (n,),
              "edge_ids": (m,), "edge_endpoints": (m, 2), "edge_lengths": (m,),
              "edge_min_widths": (m,), "edge_min_width_positions": (m, 2),
              "geometry_offsets": (m + 1,), "bottleneck_offsets": (m + 1,)}
    if any(arrays[name].shape != shape for name, shape in shapes.items()):
        raise ValueError("invalid runtime array shapes")
    if (arrays["geometry_points"].ndim != 2 or arrays["geometry_points"].shape[1] != 2
            or arrays["bottleneck_node_ids"].ndim != 1):
        raise ValueError("invalid runtime flat geometry/bottleneck shapes")
    for name in ("node_ids", "edge_ids"):
        if len(set(map(int, arrays[name]))) != len(arrays[name]) or np.any(arrays[name] < 0):
            raise ValueError("invalid runtime identifiers")
    ids = set(map(int, arrays["node_ids"]))
    if not set(map(int, arrays["edge_endpoints"].flat)) <= ids:
        raise ValueError("runtime edge references unknown node")
    if not set(map(int, arrays["bottleneck_node_ids"])) <= ids:
        raise ValueError("runtime bottleneck references unknown node")
    if (np.any(arrays["edge_lengths"] <= 0) or np.any(arrays["edge_min_widths"] <= 0)
            or np.any(arrays["node_widths"] <= 0)
            or np.any(arrays["node_flags"] > max(KIND_FLAGS.values()))):
        raise ValueError("invalid runtime lengths, widths or node flags")
    for offsets_name, data_name, minimum in (("geometry_offsets", "geometry_points", 2),
                                            ("bottleneck_offsets", "bottleneck_node_ids", 0)):
        offsets = arrays[offsets_name]
        if offsets[0] != 0 or offsets[-1] != len(arrays[data_name]) or np.any(np.diff(offsets) < minimum):
            raise ValueError("invalid runtime offsets")
    positions = dict(zip(map(int, arrays["node_ids"]), arrays["node_positions"]))
    flags = dict(zip(map(int, arrays["node_ids"]), arrays["node_flags"]))
    for index, (u, v) in enumerate(arrays["edge_endpoints"]):
        first, last = arrays["geometry_offsets"][index:index + 2]
        if (not np.array_equal(arrays["geometry_points"][first], positions[int(u)])
                or not np.array_equal(arrays["geometry_points"][last - 1], positions[int(v)])):
            raise ValueError("runtime geometry does not meet its nodes")
        a, b = arrays["bottleneck_offsets"][index:index + 2]
        if any(node not in (u, v) or flags[int(node)] != KIND_FLAGS["bottleneck"]
               for node in arrays["bottleneck_node_ids"][a:b]):
            raise ValueError("invalid edge bottleneck metadata")
    return RuntimeRoutingGraph(arrays)
