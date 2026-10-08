"""Build/validate physical corridor graphs from the bundled Bench-MR worlds."""

import argparse
import json
import os
from pathlib import Path
import time

import numpy as np

from corridor_graph import BottleneckConfig, GeometryConfig, build_corridor_graph


def rooms_world():
    """Two 6x8 rooms joined by a 2-unit-wide, 2-unit-long passage."""
    return {"bounds": [0, 0, 14, 8], "obstacles": [
        [[6, 0], [8, 0], [8, 3], [6, 3]],
        [[6, 5], [8, 5], [8, 8], [6, 8]],
    ]}


def graph_statistics(graph, narrowest_count=5):
    """Mean width is arclength-weighted over the physical width profiles."""
    samples = [sample.width for edge in graph.edges.values() for sample in edge.width_profile]
    length = sum(edge.length for edge in graph.edges.values())
    integral = sum(sum((b.distance - a.distance) * (a.width + b.width) / 2
                       for a, b in zip(edge.width_profile[:-1], edge.width_profile[1:]))
                   for edge in graph.edges.values())
    ordered = sorted(graph.edges.values(), key=lambda edge: (edge.min_width, edge.id))

    def describe(edge):
        return {"id": edge.id, "u": edge.u, "v": edge.v, "length": edge.length,
                "min_width": edge.min_width, "min_width_position": edge.min_width_position}

    internal = [edge for edge in ordered
                if all(graph.nodes[node].kind != "terminal" for node in (edge.u, edge.v))]
    return {"nodes": len(graph.nodes), "edges": len(graph.edges),
            "graph_components": len(graph.connected_components()),
            "free_space_components": graph.metadata["free_space_components"],
            "bottlenecks": len({position for edge in graph.edges.values()
                                for position in edge.bottleneck_positions}),
            "total_length": length,
            "width_min": min(samples) if samples else None,
            "width_mean": integral / length if length else None,
            "width_max": max(samples) if samples else None,
            "narrowest_corridors": [describe(edge) for edge in ordered[:narrowest_count]],
            "narrowest_internal_corridors": [describe(edge) for edge in internal[:narrowest_count]]}


def main():
    root = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--suite", type=Path, default=root / "bench_mr_tasks.json")
    parser.add_argument("--world", nargs="+", default=["parking1", "parking3", "warehouse2"],
                        help="world names, 'all', and/or 'rooms' (synthetic)")
    parser.add_argument("--output", type=Path, default=root / "results/corridors")
    parser.add_argument("--spacing", type=float, default=GeometryConfig().sample_spacing)
    parser.add_argument("--scale", type=int, default=GeometryConfig().coordinate_scale)
    parser.add_argument("--boundary-trim", type=float, default=GeometryConfig().boundary_trim)
    parser.add_argument("--no-bottlenecks", action="store_true")
    parser.add_argument("--no-split", action="store_true", help="mark bottlenecks while keeping original edges")
    parser.add_argument("--bottleneck-window", type=float, default=BottleneckConfig().window_distance)
    parser.add_argument("--bottleneck-ratio", type=float, default=BottleneckConfig().max_width_ratio)
    parser.add_argument("--bottleneck-drop", type=float, default=BottleneckConfig().min_width_drop)
    parser.add_argument("--labels", type=int, default=10)
    parser.add_argument("--clearance-lines", type=int, default=0)
    parser.add_argument("--plain-edges", action="store_true")
    parser.add_argument("--show", action="store_true")
    args = parser.parse_args()
    worlds = json.loads(args.suite.read_text())["worlds"]
    names = list(worlds) if "all" in args.world else list(dict.fromkeys(args.world))
    if "all" in args.world and "rooms" in args.world:
        names.append("rooms")
    worlds["rooms"] = rooms_world()
    unknown = set(names) - worlds.keys()
    if unknown:
        parser.error(f"unknown worlds: {sorted(unknown)}; choose {list(worlds)}")
    geometry = GeometryConfig(coordinate_scale=args.scale, sample_spacing=args.spacing,
                              boundary_trim=args.boundary_trim)
    bottlenecks = BottleneckConfig(enabled=not args.no_bottlenecks,
                                  split_edges=not args.no_split,
                                  window_distance=args.bottleneck_window,
                                  max_width_ratio=args.bottleneck_ratio,
                                  min_width_drop=args.bottleneck_drop)
    os.environ.setdefault("MPLCONFIGDIR", "/tmp/mppi-matplotlib")
    import matplotlib
    if not args.show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from corridor_graph.visualization import plot_corridor_graph

    args.output.mkdir(parents=True, exist_ok=True)
    summaries = {}
    for name in names:
        start = time.monotonic()
        graph = build_corridor_graph(worlds[name], geometry, bottlenecks)
        stats = graph_statistics(graph)
        stats["build_seconds"] = time.monotonic() - start
        summaries[name] = stats
        print(f"{name}: {stats['nodes']} nodes, {stats['edges']} edges, "
              f"{stats['graph_components']} components, {stats['bottlenecks']} bottlenecks")
        if graph.edges:
            print("  width min / mean / max: " + " / ".join(
                f"{stats[key]:.4f}" for key in ("width_min", "width_mean", "width_max")))
        for key, label in [("narrowest_corridors", "Narrowest corridors"),
                           ("narrowest_internal_corridors", "Narrowest internal corridors")]:
            print(f"  {label} (edge id, min width, minimum location):")
            for edge in stats[key]:
                print(f"    e{edge['id']}: {edge['min_width']:.4f} at {edge['min_width_position']}")
        print("  Terminal branches approach wall corners; their small minima are physical tip widths.")
        selected = stats["narrowest_internal_corridors"][:max(0, args.clearance_lines)]
        clearance_samples = [(edge["id"], int(np.argmin(
            [s.width for s in graph.edges[edge["id"]].width_profile]))) for edge in selected]
        fig, _ = plot_corridor_graph(worlds[name], graph, color_width=not args.plain_edges,
                                    label_count=max(0, args.labels), clearance_samples=clearance_samples,
                                    title=f"{name}: {len(graph.nodes)} nodes, {len(graph.edges)} corridors")
        fig.savefig(args.output / f"{name}.png", dpi=180)
        (args.output / f"{name}.graph.json").write_text(json.dumps(graph.to_dict(), indent=2) + "\n")
        if args.show:
            plt.show()
        plt.close(fig)
    (args.output / "statistics.json").write_text(json.dumps(summaries, indent=2) + "\n")
    print(f"Saved overlays, graphs and statistics to {args.output}")


if __name__ == "__main__":
    main()
