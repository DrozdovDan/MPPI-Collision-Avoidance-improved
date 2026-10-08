"""Audit widths and benchmark the independent traffic-resource overlay."""

import argparse
import json
import os
from pathlib import Path
import time

import numpy as np

from corridor_graph import (FreeSpace, RoutingCorridorGraph, RoutingEdge, RoutingNode, SourceSpan,
                            TrafficConfig, audit_widths, build_corridor_graph,
                            extract_traffic_resources, simplify_corridor_graph, WidthAuditor)
from corridor_routing_demo import graph_from_json


def routing_from_json(path):
    raw = json.loads(Path(path).read_text())
    nodes = {}
    for n in raw["nodes"]:
        for key in ("position", "source_node_ids", "protected_reasons"):
            n[key] = tuple(n[key])
        n["source_edge_samples"] = tuple(map(tuple, n["source_edge_samples"]))
        nodes[n["id"]] = RoutingNode(**n)
    edges = {}
    for e in raw["edges"]:
        e.pop("source_edge_ids", None)
        e["geometry"] = tuple(map(tuple, e["geometry"]))
        e["min_width_position"] = tuple(e["min_width_position"])
        e["source_spans"] = tuple(SourceSpan(**s) for s in e["source_spans"])
        e["bottleneck_node_ids"] = tuple(e["bottleneck_node_ids"])
        edges[e["id"]] = RoutingEdge(**e)
    return RoutingCorridorGraph(nodes, edges, raw.get("metadata", {}))


def distribution(values):
    if not values:
        return {k: None for k in ("min", "p25", "median", "p75", "max")}
    return dict(zip(("min", "p25", "median", "p75", "max"), map(float, np.quantile(values, [0, .25, .5, .75, 1]))))


def benchmark_traffic(routing, traffic, audit, seconds, baseline=None):
    kinds = {kind: [r for r in traffic.resources.values() if r.kind == kind]
             for kind in ("narrow_corridor", "junction_region")}
    covered = 0.
    for intervals in traffic.edge_intervals.values():
        end = 0.
        for interval in intervals:
            covered += max(0., interval.end - max(end, interval.start))
            end = max(end, interval.end)
    return {"routing_nodes": len(routing.nodes), "routing_edges": len(routing.edges),
            "geometric_bottlenecks": sum(n.kind == "bottleneck" for n in routing.nodes.values()),
            "narrow_resources_before_refinement": baseline.get("narrow_resources") if baseline else None,
            "narrow_resources": len(kinds["narrow_corridor"]), "junction_regions": len(kinds["junction_region"]),
            "boundary_taper_resources": sum(r.geometric_classification == "boundary_taper" for r in traffic.resources.values()),
            "trimmed_boundary_endpoint_resources": sum(bool(r.boundary_terminations) for r in traffic.resources.values()),
            "region_relationships": len(traffic.relationships),
            "resources_with_multiple_bottlenecks": sum(sum(routing.nodes[n].kind == "bottleneck" for n in r.source_node_ids) > 1
                                                       for r in kinds["narrow_corridor"]),
            "unconstrained_edges": sum(not v for v in traffic.edge_intervals.values()),
            "resource_covered_reference_length": covered,
            "unconstrained_reference_length": sum(e.length for e in routing.edges.values()) - covered,
            "preprocessing_seconds": seconds, "geometry_size": traffic.geometry_size(),
            "distributions": {kind: {"width": distribution([r.min_width for r in resources]),
                                       "length": distribution([r.length for r in resources])}
                              for kind, resources in kinds.items()},
            "width_audit": {k: v for k, v in audit.items() if k not in ("diagnostics", "terminal_limits")},
            "schema_version": 2}


def main():
    root = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--suite", type=Path, default=root / "bench_mr_tasks.json")
    parser.add_argument("--world", nargs="+", default=["warehouse2", "warehouse", "parking1"])
    parser.add_argument("--routing-dir", type=Path, help="reuse <world>.detailed.graph.json and <world>.routing.graph.json")
    parser.add_argument("--output", type=Path, default=root / "results/traffic_resources")
    parser.add_argument("--synthetic", action="store_true", help="also run geometric synthetic fixtures")
    parser.add_argument("--narrow-width", type=float, default=3.)
    parser.add_argument("--width-hysteresis", type=float, default=.0001)
    parser.add_argument("--baseline-statistics", type=Path, help="previous statistics.json for before/after counts")
    parser.add_argument("--junction-max-radius", type=float, default=1.5)
    parser.add_argument("--junction-clearance-fraction", type=float, default=.75)
    parser.add_argument("--no-narrow", action="store_true")
    parser.add_argument("--no-junctions", action="store_true")
    parser.add_argument("--show", action="store_true")
    args = parser.parse_args()
    config = TrafficConfig(narrow_width=args.narrow_width, detect_narrow=not args.no_narrow,
                           detect_junctions=not args.no_junctions, junction_max_radius=args.junction_max_radius,
                           junction_clearance_fraction=args.junction_clearance_fraction,
                           width_hysteresis=args.width_hysteresis)
    worlds = json.loads(args.suite.read_text())["worlds"]
    names = list(worlds) if "all" in args.world else list(dict.fromkeys(args.world))
    if set(names) - worlds.keys():
        parser.error(f"unknown worlds: {set(names) - worlds.keys()}")
    os.environ.setdefault("MPLCONFIGDIR", "/tmp/mppi-matplotlib")
    import matplotlib
    if not args.show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from corridor_graph.traffic_visualization import plot_traffic_resources, plot_resource_debug
    args.output.mkdir(parents=True, exist_ok=True)
    baseline_path = args.baseline_statistics or args.output / "baseline_statistics.json"
    if not baseline_path.exists() and (args.output / "statistics.json").exists():
        previous = json.loads((args.output / "statistics.json").read_text())
        baseline_path.write_text(json.dumps({n: v for n, v in previous.items() if n in worlds}, indent=2) + "\n")
    baseline = json.loads(baseline_path.read_text()) if baseline_path.exists() else {}
    jobs = {}
    for name in names:
        if args.routing_dir:
            detailed = graph_from_json(args.routing_dir / f"{name}.detailed.graph.json")
            routing = routing_from_json(args.routing_dir / f"{name}.routing.graph.json")
        else:
            detailed = build_corridor_graph(worlds[name])
            routing = simplify_corridor_graph(detailed, worlds[name])
        jobs[name] = (worlds[name], detailed, routing)
    if args.synthetic:
        from corridor_traffic_fixtures import traffic_fixtures
        jobs.update(traffic_fixtures())
    summaries = {}
    for name, (world, detailed, routing) in jobs.items():
        audit_start = time.perf_counter()
        audit = audit_widths(detailed, routing, FreeSpace(world))
        audit_seconds = time.perf_counter() - audit_start
        start = time.perf_counter()
        traffic = extract_traffic_resources(detailed, routing, world, config)
        seconds = time.perf_counter() - start
        stats = benchmark_traffic(routing, traffic, audit, seconds, baseline.get(name))
        stats["resource_extraction_seconds"] = seconds
        stats["width_audit_seconds"] = audit_seconds
        stats["preprocessing_seconds"] = seconds + audit_seconds
        stats["timing_scope"] = "width audit plus resource geometry/cache/validation; graph construction, statistics, I/O and plotting excluded"
        summaries[name] = stats
        (args.output / f"{name}.resources.json").write_text(json.dumps(traffic.to_dict(), indent=2, allow_nan=False) + "\n")
        stats["serialized_geometry_bytes"] = (args.output / f"{name}.resources.json").stat().st_size
        (args.output / f"{name}.width_audit.json").write_text(json.dumps(audit, indent=2, allow_nan=False) + "\n")
        fig, ax = plot_traffic_resources(world, routing, traffic, title=f"{name}: narrow passages and geometric junction candidates (map units)")
        fig.savefig(args.output / f"{name}.resources.png", dpi=180, bbox_inches="tight")
        if name == "warehouse2":
            # Same overlay and resource IDs, enlarged repeated-obstacle area.
            x0, y0, x1, y1 = world["bounds"]
            ax.set(xlim=(x0 + .67 * (x1 - x0), x1), ylim=(y0, y1))
            fig.set_size_inches(9, 14)
            ax.legend(loc="upper center", bbox_to_anchor=(.5, -.045), ncol=2, fontsize=7)
            fig.savefig(args.output / f"{name}.right_detail.png", dpi=180, bbox_inches="tight")
        if args.show:
            plt.show()
        plt.close(fig)
        narrow = [r for r in traffic.resources.values() if r.kind == "narrow_corridor"]
        if narrow:
            selected = []
            # Select the historically questioned widths by value, not map IDs;
            # the same debug mechanism also works on the synthetic fixtures.
            if name == "warehouse2":
                selected.extend(min(narrow, key=lambda r: abs(r.min_width - width)).id for width in (.151, .232, .327))
            multi = sorted(narrow, key=lambda r: (-sum(routing.nodes[n].kind == "bottleneck" for n in r.source_node_ids), r.min_width))
            selected.extend(r.id for r in multi[:3])
            selected = list(dict.fromkeys(selected))[:6]
            fig, _ = plot_resource_debug(world, routing, traffic, selected)
            fig.savefig(args.output / f"{name}.width_debug.png", dpi=180, bbox_inches="tight")
            plt.close(fig)
            if name == "warehouse2":
                # Retain the original sampled witnesses as well as the stricter
                # continuous resource minima; neither metric overwrites the other.
                witnesses = {}
                auditor = WidthAuditor(FreeSpace(world))
                source_records = []
                for width in (.151, .232, .327):
                    edge = min(routing.edges.values(), key=lambda e: abs(e.min_width - width))
                    intervals = [i for i in traffic.edge_intervals[edge.id]
                                 if traffic.resources[i.resource_id].kind == "narrow_corridor"]
                    if intervals:
                        interval = min(intervals, key=lambda i: min(np.linalg.norm(np.asarray(p) - edge.min_width_position)
                                                                   for p in i.geometry))
                        witness = auditor.witness(edge.min_width_position)
                        witnesses[interval.resource_id] = witness
                        source_records.append({"routing_edge_id": edge.id, "resource_id": interval.resource_id,
                                               "stored_min_width": edge.min_width,
                                               "continuous_resource_min_width": traffic.resources[interval.resource_id].min_width,
                                               "position": edge.min_width_position})
                if witnesses:
                    fig, _ = plot_resource_debug(world, routing, traffic, list(witnesses), witnesses)
                    fig.savefig(args.output / "warehouse2.selected_widths.png", dpi=180, bbox_inches="tight")
                    plt.close(fig)
                    (args.output / "warehouse2.selected_widths.json").write_text(json.dumps(source_records, indent=2) + "\n")
                suspicious = [r for r in narrow if r.boundary_terminations]
                # Show both physical closure and positive near-gap evidence.
                suspicious.sort(key=lambda r: (-any(p["relation"] == "positive_boundary_gap"
                                                      for t in r.boundary_terminations for p in t["feature_pairs"]), r.id))
                if suspicious:
                    fig, _ = plot_resource_debug(world, routing, traffic, [r.id for r in suspicious[:6]])
                    fig.savefig(args.output / "warehouse2.boundary_diagnostics.png", dpi=180, bbox_inches="tight")
                    plt.close(fig)
        print(f"{name}: routing {len(routing.nodes)}/{len(routing.edges)}, "
              f"bottlenecks {stats['geometric_bottlenecks']}, narrow {stats['narrow_resources']}, "
              f"junction regions {stats['junction_regions']}, {stats['preprocessing_seconds']:.3f} s, "
              f"{stats['serialized_geometry_bytes']:,} B serialized geometry; "
              f"small widths {audit['small_width_edge_count']}")
    (args.output / "statistics.json").write_text(json.dumps(summaries, indent=2, allow_nan=False) + "\n")
    print(f"Saved resource overlays, boundary debug panels, graphs, audits and statistics to {args.output}")


if __name__ == "__main__":
    main()
