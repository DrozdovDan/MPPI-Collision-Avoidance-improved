"""Sparse resource overlays and exact nearest-boundary debug panels."""

import math

import numpy as np
from shapely.geometry import LineString, Polygon as RegionPolygon


def _map(ax, world):
    from matplotlib.patches import Polygon, Rectangle
    for vertices in world["obstacles"]:
        ax.add_patch(Polygon(vertices, facecolor=".85", edgecolor=".5", linewidth=.45))
    x0, y0, x1, y1 = world["bounds"]
    ax.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0, fill=False, edgecolor=".2", linewidth=1))
    ax.set(xlim=(x0, x1), ylim=(y0, y1), xlabel="x (map units)", ylabel="y (map units)")
    ax.set_aspect("equal")


def _bottlenecks(resource, routing):
    return [routing.nodes[n].position for n in resource.source_node_ids if routing.nodes[n].kind == "bottleneck"]


def plot_traffic_resources(world, routing, traffic, title=None, label_count=18, ax=None):
    import matplotlib.pyplot as plt
    from matplotlib.collections import LineCollection
    from matplotlib.patches import Polygon
    if ax is None:
        fig, ax = plt.subplots(figsize=(16, 10), constrained_layout=True)
    else:
        fig = ax.figure
    _map(ax, world)
    ax.add_collection(LineCollection([e.geometry for e in routing.edges.values()], colors="#3971a5",
                                    linewidth=.9, alpha=.75, zorder=2))
    colors = plt.get_cmap("tab20")
    ports, minima, trim_points = [], [], []
    for resource in traffic.resources.values():
        if resource.kind == "junction_region":
            ax.add_patch(Polygon(resource.region, facecolor="#ac60d9", edgecolor="#7939a6",
                                 linewidth=.45, alpha=.25, zorder=3))
        else:
            color = colors((resource.id * 7) % 20)
            taper = bool(resource.boundary_terminations)
            ax.add_collection(LineCollection(resource.geometry, colors=[color], linewidth=2 if taper else 3.2,
                                            linestyles="dashed" if taper else "solid", zorder=4))
            minima.extend(_bottlenecks(resource, routing))
            trim_points.extend(t["position"] for t in resource.boundary_terminations)
        ports.extend(p.position for p in resource.entrances)
    if ports:
        ax.scatter(*np.asarray(sorted(set(ports))).T, s=7, c="white", edgecolors="#176546", linewidths=.55, zorder=6)
    if minima:
        ax.scatter(*np.asarray(minima).T, s=14, c="#222222", marker="D", edgecolors="white", linewidths=.3, zorder=7)
    if trim_points:
        ax.scatter(*np.asarray(sorted(set(trim_points))).T, s=25, c="#d12fa0", marker="^", zorder=8)
    scale = np.asarray(world["bounds"][2:]) - world["bounds"][:2]
    anchors = []
    narrow_order = sorted((r for r in traffic.resources.values() if r.kind == "narrow_corridor"),
                          key=lambda r: (-len(_bottlenecks(r, routing)), r.min_width, -r.length))
    junction_order = sorted((r for r in traffic.resources.values() if r.kind == "junction_region"),
                            key=lambda r: -r.min_width)
    # Interleave so both resource kinds receive sparse labels.
    ordered = []
    for i in range(max(len(narrow_order), len(junction_order))):
        if i < len(narrow_order):
            ordered.append(narrow_order[i])
        if i % 3 == 0 and i // 3 < len(junction_order):
            ordered.append(junction_order[i // 3])
    for resource in ordered:
        if len(anchors) >= label_count:
            break
        if resource.kind == "junction_region":
            position = tuple(RegionPolygon(resource.region).centroid.coords[0])
        else:
            position = tuple(LineString(resource.geometry[0]).interpolate(.5, normalized=True).coords[0])
        normalized = np.asarray(position) / scale
        if any(np.linalg.norm(normalized - a) < .07 for a in anchors):
            continue
        anchors.append(normalized)
        bottlenecks = len(_bottlenecks(resource, routing))
        prefix = "J" if resource.kind == "junction_region" else "N"
        label = f"R{resource.id} ({prefix})  w={resource.min_width:.3g}"
        if bottlenecks > 1:
            label += f"  ({bottlenecks} minima)"
        ax.annotate(label, position, xytext=(3, 6), textcoords="offset points", fontsize=7,
                    bbox=dict(facecolor="white", alpha=.88, edgecolor="none", pad=1.2), zorder=8)
    ax.plot([], [], color="#3971a5", lw=1, label="Routing graph")
    ax.plot([], [], color="#d98523", lw=3, label="Narrow resources (colors distinguish regions)")
    ax.plot([], [], color="#ac60d9", lw=8, alpha=.35, label="Junction candidates (geometry only)")
    ax.plot([], [], "o", ms=3, markerfacecolor="white", markeredgecolor="#176546", label="Resource boundary ports")
    if trim_points:
        ax.plot([], [], "^", ms=4, color="#d12fa0", label="Numerical trim endpoints; dashed tapers")
    ax.plot([], [], "D", ms=3, color="#222222", label="Geometric bottleneck markers")
    ax.legend(loc="upper center", bbox_to_anchor=(.5, -.08), ncol=3, fontsize=8)
    ax.set_title(title or "Traffic resources; widths in map units")
    return fig, ax


def plot_resource_debug(world, routing, traffic, resource_ids, witnesses=None):
    import matplotlib.pyplot as plt
    count = len(resource_ids)
    columns = min(3, count)
    fig, axes = plt.subplots(math.ceil(count / columns), columns, figsize=(5.3 * columns, 4.8 * math.ceil(count / columns)),
                             squeeze=False, constrained_layout=True)
    colors = ["#d62f2f", "#187bc0", "#a549ad", "#34863d"]
    for ax, resource_id in zip(axes.flat, resource_ids):
        _map(ax, world)
        resource = traffic.resources[resource_id]
        for geometry in resource.geometry:
            ax.plot(*np.asarray(geometry).T, color="#e7a025", linewidth=3, zorder=3)
        witness = (witnesses or {}).get(resource_id, resource.width_witness)
        center = np.asarray(witness.position)
        for i, contact in enumerate(witness.contacts):
            color = colors[i % len(colors)]
            ax.plot(*np.asarray(contact.segment).T, color=color, linewidth=2.2, zorder=4)
            ax.plot(*np.asarray([witness.position, contact.position]).T, "--", color=color, linewidth=1.1, zorder=5)
            ax.scatter(*contact.position, color=color, s=20, zorder=6)
        ax.scatter(*center, color="black", s=15, zorder=6)
        radius = max(.6, resource.min_width * 1.25)
        ax.set(xlim=(center[0] - radius, center[0] + radius), ylim=(center[1] - radius, center[1] + radius))
        qualifier = "stored sample" if witnesses else "continuous chain"
        ax.set_title(f"R{resource.id}: {qualifier} width {witness.local_width:.6g} map units\n{witness.classification}", fontsize=10)
        ax.text(.02, .02, "Dashed: exact clearance to boundary segments\n" +
                ", ".join(f"{c.kind}: {c.distance:.6g}" for c in witness.contacts),
                transform=ax.transAxes, fontsize=7, bbox=dict(facecolor="white", alpha=.85, edgecolor="none"))
        if resource.boundary_terminations:
            facts = [p["relation"] + f" ({p['feature_separation']:.3g})" for t in resource.boundary_terminations
                     for p in t["feature_pairs"]]
            ax.text(.02, .96, "Construction trim endpoint\n" + "; ".join(dict.fromkeys(facts)),
                    transform=ax.transAxes, va="top", fontsize=8,
                    bbox=dict(facecolor="white", alpha=.9, edgecolor="#d12fa0"))
            # A second scale makes 0.0002 clearances visible without losing the
            # surrounding boundary contact in the main panel.
            inset = ax.inset_axes([.60, .61, .37, .30])
            _map(inset, world)
            inset.plot(*np.asarray(resource.geometry[0]).T, color="#e7a025", linewidth=1.5)
            for i, contact in enumerate(witness.contacts):
                color = colors[i % len(colors)]
                inset.plot(*np.asarray(contact.segment).T, color=color, linewidth=1.2)
                inset.plot(*np.asarray([witness.position, contact.position]).T, "--", color=color, linewidth=.8)
            radius = max(witness.clearance * 4, np.finfo(float).eps * max(abs(center)) * 100)
            inset.set(xlim=(center[0] - radius, center[0] + radius), ylim=(center[1] - radius, center[1] + radius),
                      xlabel="", ylabel="", xticks=[], yticks=[])
            inset.set_title(f"span {2 * radius:.3g} map units", fontsize=7)
    for ax in list(axes.flat)[count:]:
        ax.set_visible(False)
    return fig, axes
