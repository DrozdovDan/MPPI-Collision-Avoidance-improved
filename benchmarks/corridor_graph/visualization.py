"""Matplotlib overlay using the same polygon drawing conventions as Bench-MR."""

import numpy as np
from shapely.geometry import Point
from shapely.ops import nearest_points

from .geometry import FreeSpace


def plot_corridor_graph(world, graph, ax=None, color_width=True, label_count=10,
                        clearance_samples=(), title=None):
    """Draw physical widths, junctions and bottlenecks without internal primitives.

    clearance_samples contains (edge_id, profile_index) pairs. Width labels prefer
    corridors without terminal tips, whose near-zero widths are boundary effects.
    Returns (figure, axes); caller owns saving/display and backend selection.
    """
    import matplotlib.pyplot as plt
    from matplotlib.collections import LineCollection
    from matplotlib.colors import Normalize
    from matplotlib.patches import Polygon as Patch, Rectangle

    if ax is None:
        fig, ax = plt.subplots(figsize=(12, 8), constrained_layout=True)
    else:
        fig = ax.figure
    for polygon in world["obstacles"]:
        ax.add_patch(Patch(polygon, facecolor="0.78", edgecolor="0.35", linewidth=.5))
    x0, y0, x1, y1 = world["bounds"]
    ax.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0, fill=False,
                           edgecolor="0.15", linewidth=1.5))
    segments, widths = [], []
    for edge in graph.edges.values():
        points = np.asarray(edge.geometry)
        segments.extend(zip(points[:-1], points[1:]))
        profile = np.array([s.width for s in edge.width_profile])
        widths.extend((profile[:-1] + profile[1:]) / 2)
    if segments:
        if color_width:
            collection = LineCollection(segments, cmap="viridis", linewidth=2, capstyle="round",
                                        norm=Normalize(0, max(widths)))
            collection.set_array(np.asarray(widths))
            fig.colorbar(collection, ax=ax, label="Physical corridor width (map units)", shrink=.8)
        else:
            collection = LineCollection(segments, colors="#246ba1", linewidth=1.7, capstyle="round")
        ax.add_collection(collection)
    styles = [("terminal", "o", "#246ba1", 8, "Terminal"),
              ("junction", "o", "#69d5ee", 36, "Junction"),
              ("cycle_anchor", "s", "#69d5ee", 25, "Cycle anchor")]
    for kind, marker, color, size, label in styles:
        positions = np.asarray([n.position for n in graph.nodes.values() if n.kind == kind])
        if len(positions):
            ax.scatter(*positions.T, marker=marker, c=color, s=size, edgecolors="0.1",
                       linewidths=.5, label=label, zorder=4)
    bottlenecks = {n.position for n in graph.nodes.values() if n.kind == "bottleneck"}
    bottlenecks.update(position for e in graph.edges.values() for position in e.bottleneck_positions)
    if bottlenecks:
        positions = np.asarray(sorted(bottlenecks))
        ax.scatter(*positions.T, marker="D", c="#ed4b35", s=40, edgecolors="white",
                   linewidths=.6, label="Bottleneck", zorder=5)
    internal = [e for e in graph.edges.values()
                if all(graph.nodes[node].kind != "terminal" for node in (e.u, e.v))]
    labeled = sorted(internal or list(graph.edges.values()), key=lambda e: (
        not bool(e.bottleneck_positions), e.min_width, -e.length))
    label_positions = []
    for edge in labeled:
        if len(label_positions) >= label_count:
            break
        # Label near the midpoint to avoid stacking labels at a shared bottleneck.
        sample = min(edge.width_profile, key=lambda s: abs(s.distance - edge.length / 2))
        # Space labels in map coordinates so neighboring split edges do not
        # produce overlapping text at repeated shelf bottlenecks.
        relative = np.asarray(sample.position) / [x1 - x0, y1 - y0]
        if any(np.linalg.norm(relative - other) < .09 for other in label_positions):
            continue
        label_positions.append(relative)
        ax.annotate(f"e{edge.id}: {edge.min_width:.3g}", sample.position,
                    xytext=(3, 5), textcoords="offset points", fontsize=7, color="0.1",
                    bbox=dict(facecolor="white", alpha=.8, edgecolor="none", pad=1), zorder=6)
    if clearance_samples:
        space = FreeSpace(world)
        for edge_id, index in clearance_samples:
            sample = graph.edges[edge_id].width_profile[index]
            nearest = nearest_points(Point(sample.position), space.boundary)[1]
            ax.plot([sample.position[0], nearest.x], [sample.position[1], nearest.y],
                    "--", color="#d534aa", linewidth=1, zorder=3)
        ax.plot([], [], "--", color="#d534aa", label="Nearest boundary / clearance")
    pad = .02 * max(x1 - x0, y1 - y0)
    ax.set(xlim=(x0 - pad, x1 + pad), ylim=(y0 - pad, y1 + pad),
           xlabel="x (map units)", ylabel="y (map units)")
    ax.set_aspect("equal")
    ax.set_title(title or f"{len(graph.nodes)} nodes, {len(graph.edges)} corridors; labels = edge minimum width")
    if ax.get_legend_handles_labels()[0]:
        ax.legend(loc="upper center", bbox_to_anchor=(.5, -.10), ncol=4,
                  fontsize=8, framealpha=.95)
    return fig, ax


def plot_routing_graph(world, detailed, routing, ax=None, show_detailed=True, label_count=8,
                       title=None):
    """Prominent routing overlay; dense ECM is optional faint geometric context."""
    import matplotlib.pyplot as plt
    from matplotlib.collections import LineCollection
    from matplotlib.patches import Polygon as Patch, Rectangle

    if ax is None:
        fig, ax = plt.subplots(figsize=(12, 8), constrained_layout=True)
    else:
        fig = ax.figure
    for polygon in world["obstacles"]:
        ax.add_patch(Patch(polygon, facecolor="0.84", edgecolor="0.50", linewidth=.5))
    x0, y0, x1, y1 = world["bounds"]
    ax.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0, fill=False,
                           edgecolor="0.2", linewidth=1.2))
    if show_detailed and detailed.edges:
        ax.add_collection(LineCollection([e.geometry for e in detailed.edges.values()],
                                        colors="#939baa", alpha=.35, linewidth=.7, zorder=1))
    if routing.edges:
        ax.add_collection(LineCollection([e.geometry for e in routing.edges.values()],
                                        colors="#1655a5", linewidth=2.0, capstyle="round", zorder=2))
        ax.plot([], [], color="#1655a5", linewidth=2, label="Routing corridor")
    for kind, marker, color, size, label in [
            ("junction", "o", "#51cde8", 32, "Routing junction"),
            ("bottleneck", "D", "#ef553b", 26, "Preserved bottleneck"),
            ("transition", "s", "#cf8be9", 25, "Width transition"),
            ("terminal", "o", "#1655a5", 9, "Region access"),
            ("region", "s", "#1655a5", 25, "Local region"),
            ("cycle_anchor", "s", "#51cde8", 25, "Cycle anchor")]:
        positions = np.asarray([n.position for n in routing.nodes.values() if n.kind == kind])
        if len(positions):
            ax.scatter(*positions.T, s=size, marker=marker, c=color, edgecolors="white",
                       linewidths=.5, label=label, zorder=4)
    anchors = []
    ordered = sorted(routing.edges.values(), key=lambda e: (not bool(e.bottleneck_node_ids), e.min_width, -e.length))
    for edge in ordered:
        if len(anchors) >= label_count:
            break
        points = np.asarray(edge.geometry)
        distance = np.r_[0., np.cumsum(np.linalg.norm(np.diff(points, axis=0), axis=1))]
        point = points[int(np.argmin(np.abs(distance - distance[-1] / 2)))]
        normalized = point / [x1 - x0, y1 - y0]
        if any(np.linalg.norm(normalized - other) < .12 for other in anchors):
            continue
        anchors.append(normalized)
        ax.annotate(f"e{edge.id}: w={edge.min_width:.3g}", point, xytext=(3, 4),
                    textcoords="offset points", fontsize=7,
                    bbox=dict(facecolor="white", alpha=.85, edgecolor="none", pad=1), zorder=5)
    pad = .02 * max(x1 - x0, y1 - y0)
    ax.set(xlim=(x0 - pad, x1 + pad), ylim=(y0 - pad, y1 + pad),
           xlabel="x (map units)", ylabel="y (map units)")
    ax.set_aspect("equal")
    ax.set_title(title or f"Routing: {len(routing.nodes)} nodes / {len(routing.edges)} edges")
    if ax.get_legend_handles_labels()[0]:
        ax.legend(loc="upper center", bbox_to_anchor=(.5, -.10), ncol=3, fontsize=8)
    return fig, ax


def plot_routing_comparison(world, detailed, routing, show_detailed=True, title=None):
    """Map / detailed ECM / routing graph on exactly the same coordinates."""
    import matplotlib.pyplot as plt
    from .model import CorridorGraph

    fig, axes = plt.subplots(1, 3, figsize=(23, 8), constrained_layout=True)
    plot_corridor_graph(world, CorridorGraph(), ax=axes[0], color_width=False,
                        label_count=0, title="Original map")
    plot_corridor_graph(world, detailed, ax=axes[1], color_width=False, label_count=0,
                        title=f"Detailed ECM: {len(detailed.nodes)} nodes / {len(detailed.edges)} edges")
    plot_routing_graph(world, detailed, routing, ax=axes[2], show_detailed=show_detailed,
                       label_count=0)
    if title:
        fig.suptitle(title, fontsize=15)
    return fig, axes
