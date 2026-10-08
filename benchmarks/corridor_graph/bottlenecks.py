"""Replaceable physical-distance width-minimum detector and graph splitting."""

from dataclasses import dataclass, replace
import math

import numpy as np
from scipy.ndimage import gaussian_filter1d
from scipy.signal import find_peaks

from .extraction import make_edge
from .model import CorridorGraph, CorridorNode


@dataclass(frozen=True)
class BottleneckConfig:
    enabled: bool = True
    split_edges: bool = True
    min_edge_length: float = 3.0
    window_distance: float = 5.0
    min_side_length: float = 0.5
    max_width_ratio: float = 0.75
    min_width_drop: float = 0.25
    min_separation: float = 1.0
    smoothing_distance: float = 0.15
    shoulder_quantile: float = 0.75

    def __post_init__(self):
        for name in ("min_edge_length", "window_distance", "min_side_length",
                     "min_width_drop", "min_separation", "smoothing_distance"):
            value = getattr(self, name)
            if not math.isfinite(value) or value < 0:
                raise ValueError(f"{name} must be finite and nonnegative")
        if self.window_distance <= 0 or self.min_side_length <= 0:
            raise ValueError("window_distance and min_side_length must be positive")
        if self.min_side_length > self.window_distance:
            raise ValueError("min_side_length must not exceed window_distance")
        if not 0 < self.max_width_ratio < 1:
            raise ValueError("max_width_ratio must be between zero and one")
        if not 0 <= self.shoulder_quantile <= 1:
            raise ValueError("shoulder_quantile must be between zero and one")


def detect_bottlenecks(edge, config):
    """Return original profile indices, requiring substantial shoulders BOTH sides.

    Resample by arclength before smoothing, so curved-edge sample density does
    not affect the criterion. Plateau minima are represented by one midpoint.
    Smoothing is only for detection; stored physical widths are never smoothed.
    Endpoints are excluded. Closed edges are rotated to put the seam at a width
    maximum, allowing minima at the original cycle anchor to be detected.
    """
    if not config.enabled or edge.length < config.min_edge_length:
        return []
    distances = np.array([sample.distance for sample in edge.width_profile])
    widths = np.array([sample.width for sample in edge.width_profile])
    if len(widths) < 3:
        return []
    if edge.u == edge.v and edge.geometry[0] == edge.geometry[-1]:
        last = len(widths) - 1
        seam = int(np.argmax(widths[:-1]))
        order = list(range(seam, last)) + list(range(seam + 1))
        points = tuple(edge.geometry[i] for i in order)
        rotated_distances = np.r_[0., np.cumsum(np.linalg.norm(np.diff(points, axis=0), axis=1))]
        profile = tuple(replace(edge.width_profile[i], distance=float(distance))
                        for i, distance in zip(order, rotated_distances))
        rotated = replace(edge, u=0, v=1, geometry=points, width_profile=profile)
        return sorted({order[i] for i in detect_bottlenecks(rotated, config)})
    step = float(np.median(np.diff(distances)))
    grid = np.linspace(0, edge.length, max(3, math.ceil(edge.length / step) + 1))
    step = grid[1] - grid[0]
    smooth = np.interp(grid, distances, widths)
    if config.smoothing_distance:
        smooth = gaussian_filter1d(smooth, config.smoothing_distance / step, mode="nearest")
    candidates, properties = find_peaks(
        -smooth, prominence=config.min_width_drop, plateau_size=(1, None))
    significant = []
    for candidate, left_edge, right_edge in zip(
            candidates, properties["left_edges"], properties["right_edges"]):
        if (grid[left_edge] < config.min_side_length
                or edge.length - grid[right_edge] < config.min_side_length):
            continue
        left = smooth[(grid >= grid[left_edge] - config.window_distance)
                      & (grid < grid[left_edge])]
        right = smooth[(grid > grid[right_edge])
                       & (grid <= grid[right_edge] + config.window_distance)]
        if not len(left) or not len(right):
            continue
        shoulders = [np.quantile(side, config.shoulder_quantile) for side in (left, right)]
        if any(smooth[candidate] > shoulder * config.max_width_ratio
               or shoulder - smooth[candidate] < config.min_width_drop
               for shoulder in shoulders):
            continue
        index = int(np.argmin(np.abs(distances - grid[candidate])))
        if 0 < index < len(widths) - 1:
            significant.append(index)
    # Keep the deepest minima first; do not repeatedly split a noisy valley.
    selected = []
    for index in sorted(set(significant), key=lambda i: (widths[i], i)):
        if all(abs(distances[index] - distances[other]) >= config.min_separation
               for other in selected):
            selected.append(index)
    return sorted(selected)


def split_bottlenecks(graph, space, config):
    result = CorridorGraph(dict(graph.nodes), metadata=dict(graph.metadata))
    for edge in graph.edges.values():
        indices = detect_bottlenecks(edge, config)
        positions = tuple(edge.geometry[i] for i in indices)
        if config.split_edges and edge.u == edge.v and 0 in indices:
            # The anchor already exists; never create a zero-length split edge.
            result.nodes[edge.u] = replace(result.nodes[edge.u], kind="bottleneck")
            indices.remove(0)
        if not indices or not config.split_edges:
            edge_id = len(result.edges)
            result.edges[edge_id] = replace(edge, id=edge_id, bottleneck_positions=positions)
            continue
        nodes = [edge.u]
        for position in (edge.geometry[i] for i in indices):
            node_id = max(result.nodes, default=-1) + 1
            result.nodes[node_id] = CorridorNode(node_id, position, "bottleneck")
            nodes.append(node_id)
        nodes.append(edge.v)
        cuts = [0, *indices, len(edge.geometry) - 1]
        for u, v, first, last in zip(nodes[:-1], nodes[1:], cuts[:-1], cuts[1:]):
            edge_id = len(result.edges)
            local_positions = tuple(position for position in positions
                                    if position in (edge.geometry[first], edge.geometry[last]))
            result.edges[edge_id] = make_edge(
                edge_id, u, v, edge.geometry[first:last + 1], space, local_positions)
    return result
