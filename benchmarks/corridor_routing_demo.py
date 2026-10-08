"""Benchmark detailed ECM -> routing graph, provenance and onboard serialization."""

import argparse
from dataclasses import fields
import json
import os
from pathlib import Path
import time

from corridor_graph import SimplificationConfig, build_corridor_graph, simplify_corridor_graph, load_runtime
from corridor_graph_demo import rooms_world


def graph_from_json(path):
    """Read the existing detailed graph output, preserving its IDs and samples."""
    from corridor_graph import CorridorEdge, CorridorGraph, CorridorNode, WidthSample
    value = json.loads(Path(path).read_text())
    nodes = {n["id"]: CorridorNode(n["id"], tuple(n["position"]), n["kind"]) for n in value["nodes"]}
    edges = {}
    for raw in value["edges"]:
        samples = tuple(WidthSample(s["distance"], tuple(s["position"]), s["clearance"], s["width"])
                        for s in raw["width_profile"])
        edge = CorridorEdge(raw["id"], raw["u"], raw["v"], tuple(map(tuple, raw["geometry"])),
                            raw["length"], raw["min_clearance"], raw["min_width"],
                            tuple(raw["min_width_position"]), samples,
                            tuple(map(tuple, raw.get("bottleneck_positions", ()))))
        edges[edge.id] = edge
    return CorridorGraph(nodes, edges, value.get("metadata", {}))


def benchmark_graph(detailed, routing, runtime_sizes):
    compact = json.dumps(detailed.to_dict(), separators=(",", ":"), allow_nan=False).encode()
    return {"detailed_nodes": len(detailed.nodes), "detailed_edges": len(detailed.edges),
            "routing_nodes": len(routing.nodes), "routing_edges": len(routing.edges),
            "detailed_geometry_samples": sum(len(e.geometry) for e in detailed.edges.values()),
            "routing_geometry_samples": sum(len(e.geometry) for e in routing.edges.values()),
            "detailed_serialized_bytes": len(compact),
            "runtime_array_bytes": runtime_sizes["array_bytes"],
            "runtime_serialized_bytes": runtime_sizes["file_bytes"],
            "junction_clusters": len(routing.metadata["junction_clusters"]),
            "pruned_branches": len(routing.metadata["pruned_branches"]),
            "validation": routing.metadata["validation"]}


def main():
    root = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--suite", type=Path, default=root / "bench_mr_tasks.json")
    parser.add_argument("--world", nargs="+", default=["warehouse2", "parking1", "parking3"])
    parser.add_argument("--output", type=Path, default=root / "results/routing_corridors")
    parser.add_argument("--detailed-dir", type=Path, help="load existing <world>.graph.json ECM files")
    parser.add_argument("--hide-detailed", action="store_true")
    parser.add_argument("--show", action="store_true")
    parser.add_argument("--no-prune", action="store_true")
    parser.add_argument("--protected-node-ids", type=int, nargs="*", default=[],
                        help="detailed nodes requiring explicit access; prevent their removal/merging")
    # Expose every numeric simplification/validation parameter, with the exact
    # API field names so a benchmark command describes its configuration.
    defaults = SimplificationConfig()
    for field in fields(defaults):
        if field.name in ("prune_branches", "protected_node_ids"):
            continue
        value = getattr(defaults, field.name)
        parser.add_argument("--" + field.name.replace("_", "-"), type=type(value), default=value)
    args = parser.parse_args()
    config = SimplificationConfig(prune_branches=not args.no_prune, **{
        field.name: getattr(args, field.name) for field in fields(defaults) if field.name != "prune_branches"})
    worlds = json.loads(args.suite.read_text())["worlds"]
    names = list(worlds) if "all" in args.world else list(dict.fromkeys(args.world))
    if "all" in args.world and "rooms" in args.world:
        names.append("rooms")
    worlds["rooms"] = rooms_world()
    if set(names) - worlds.keys():
        parser.error(f"unknown worlds: {set(names) - worlds.keys()}")
    os.environ.setdefault("MPLCONFIGDIR", "/tmp/mppi-matplotlib")
    import matplotlib
    if not args.show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from corridor_graph.visualization import plot_routing_comparison, plot_routing_graph

    args.output.mkdir(parents=True, exist_ok=True)
    summaries = {}
    for name in names:
        start = time.monotonic()
        detailed = (graph_from_json(args.detailed_dir / f"{name}.graph.json")
                    if args.detailed_dir else build_corridor_graph(worlds[name]))
        routing = simplify_corridor_graph(detailed, worlds[name], config)
        size = routing.save_runtime(args.output / f"{name}.runtime.npz")
        runtime = load_runtime(args.output / f"{name}.runtime.npz")
        assert runtime.adjacency() == routing.adjacency()
        stats = benchmark_graph(detailed, routing, size)
        stats["seconds"] = time.monotonic() - start
        summaries[name] = stats
        print(f"{name}: nodes {stats['detailed_nodes']} -> {stats['routing_nodes']}; "
              f"edges {stats['detailed_edges']} -> {stats['routing_edges']}; "
              f"geometry samples {stats['detailed_geometry_samples']} -> {stats['routing_geometry_samples']}")
        print(f"  serialized: detailed JSON {stats['detailed_serialized_bytes']:,} B -> "
              f"runtime NPZ {stats['runtime_serialized_bytes']:,} B "
              f"({stats['runtime_array_bytes']:,} B numeric payload)")
        validation = stats["validation"]
        print(f"  preserved: {validation['components']} components, {validation['independent_cycles']} cycles, "
              f"{validation['protected_features']} protected features; "
              f"{validation['sampled_pairs']} route pairs checked")
        if validation["distance_ratio_p95"] is not None:
            print(f"  distance ratio p95/max: {validation['distance_ratio_p95']:.4f} / "
                  f"{validation['distance_ratio_max']:.4f}; "
                  f"{validation['distortion_warning_count']} distortion warnings")
        for warning in validation["distortion_warnings"]:
            print(f"  distance warning: {warning}")
        (args.output / f"{name}.detailed.graph.json").write_text(
            json.dumps(detailed.to_dict(), separators=(",", ":"), allow_nan=False) + "\n")
        (args.output / f"{name}.routing.graph.json").write_text(
            json.dumps(routing.to_dict(), indent=2, allow_nan=False) + "\n")
        fig, _ = plot_routing_comparison(worlds[name], detailed, routing,
                                         show_detailed=not args.hide_detailed, title=name)
        fig.savefig(args.output / f"{name}.comparison.png", dpi=180)
        plt.close(fig)
        fig, _ = plot_routing_graph(worlds[name], detailed, routing,
                                    show_detailed=not args.hide_detailed, title=f"{name}: routing corridor graph")
        fig.savefig(args.output / f"{name}.routing.png", dpi=180)
        if args.show:
            plt.show()
        plt.close(fig)
    (args.output / "statistics.json").write_text(json.dumps(summaries, indent=2, allow_nan=False) + "\n")
    print(f"Saved comparison figures, debug graphs and onboard arrays to {args.output}")


if __name__ == "__main__":
    main()
