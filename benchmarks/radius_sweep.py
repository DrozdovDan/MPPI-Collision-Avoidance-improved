"""Run native radius experiments; Python only analyses JSON and plots geometry.

No ECM, graph extraction, lane placement or collision checks run in Python.
"""
import argparse
import csv
import hashlib
import json
import os
from pathlib import Path
import statistics
import subprocess

import numpy as np

from complete_routes_demo import ROOT, arrows, plot_decompositions
from corridor_graph.traffic_visualization import _map

RADII = [.10, .20, .35, .50, .75, 1.00, 1.45, 2.95]
CORRIDOR_RADII = [.10, .20, .35, .50, .75, 1.00, 1.40, 1.425, 1.45, 2.80, 2.90, 2.95, 3.00]
STAGES = ["attachment", "graph_filtering", "topology_search", "lane_decomposition",
          "transitions", "collision_validation", "reconstruction", "other"]


def scenarios(world):
    if world == "warehouse2":
        return [("long", [7.5, -10], [116, -70]),
                ("central", [45, -40], [75, -40]),
                ("major_obstacle", [55, -30], [55, -50]),
                ("dense", [20, -70], [105, -70]),
                ("narrow_passage", [96.4, -46], [96.4, -52])]
    return [("testing_warehouse", [7.5, -10], [76.5, -10]),
            ("dense", [8.3, -28], [42.3, -28]),
            ("short", [7.5, -10], [7.7, -10])]


def traversals(*pairs):
    return [dict(edge_id=edge, direction=direction) for edge, direction in pairs]


def corridors():
    return [dict(name="wide_horizontal", traversals=[dict(edge_id=11, direction=1,
                 first_distance=10, last_distance=25)]),
            dict(name="wide_vertical", traversals=traversals((97, -1), (98, 1))),
            dict(name="narrow_passage", traversals=[dict(edge_id=154, direction=1, first_distance=4.5),
                 dict(edge_id=140, direction=-1, last_distance=1.4)]),
            dict(name="narrow_bent_reference", traversals=traversals((140, -1), (141, 1))),
            dict(name="variable_width", traversals=traversals((107, 1), (125, 1), (154, 1),
                 (140, -1), (141, 1), (157, -1))),
            dict(name="junction_west_south", traversals=traversals((85, 1), (103, 1), (97, -1), (98, 1))),
            dict(name="junction_north_east", traversals=traversals((101, 1), (103, 1), (107, 1)))]


def csv_file(path, rows):
    if not rows:
        return
    columns = list(dict.fromkeys(k for row in rows for k in row))
    with path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=columns)
        writer.writeheader()
        writer.writerows({k: json.dumps(v) if isinstance(v, (dict, list)) else v for k, v in row.items()}
                        for row in rows)


def query_summary(world, scenario, q, repeats):
    metrics = [r["metrics"] for r in q["routes"]]
    length = sum(m["length"] for m in metrics)
    split = sum(m["split_length"] for m in metrics)
    row = dict(world=world, scenario=scenario, start=q["start"], goal=q["goal"], radius=q["radius"],
               safety_margin=q["safety_margin"], effective_radius=q["radius"] + q["safety_margin"],
               feasible_physical_edges=q["feasible_physical_edges"],
               physical_route_reachable=q["physical_route_reachable"],
               generated_routes=len(metrics),
               shortest_route_length=min((m["length"] for m in metrics), default=None),
               mean_route_length=statistics.fmean(m["length"] for m in metrics) if metrics else None,
               total_split_length=split, total_shared_length=sum(m["shared_length"] for m in metrics),
               total_transition_length=sum(m["transition_length"] for m in metrics),
               split_fraction=split / length if length else None,
               split_intervals=sum(m["split_interval_count"] for m in metrics),
               shared_intervals=sum(m["shared_interval_count"] for m in metrics),
               mode_transitions=sum(m["mode_transition_count"] for m in metrics),
               rejected_split_candidates=q["rejected_split_candidates"],
               rejected_lane_routes=q["rejected_lane_routes"],
               candidate_path_validation_failures=q["profile"]["path_validation_failures"],
               split_footprint_rejection_spans=sum(reason in {"lane_footprint_clearance", "local_transition_footprint_clearance"}
                                                  for reason in q["split_failure_reasons"]),
               returned_route_validation_failures=0, self_intersections=sum(m["self_intersection_count"] for m in metrics),
               planning_ms=q["planning_seconds"] * 1000,
               cached_planning_ms=statistics.median(z["planning_seconds"] * 1000 for z in repeats),
               validation_passed=q["validation_passed"], failure_reasons=q["lane_failures"],
               physical_edge_sequences=[r["edge_ids"] for r in q["routes"]],
               min_opposing_separation=min((m["min_opposing_separation"] for m in metrics if m["split_length"]), default=None),
               min_selected_clearance=min((m["min_clearance"] for m in metrics), default=None))
    row.update({stage + "_ms": q["profile"]["stage_seconds"][stage] * 1000 for stage in STAGES})
    row.update({"profile_" + k: v for k, v in q["profile"].items() if k != "stage_seconds"})
    return row


def draw_lane(ax, d, radius, disks=True):
    from matplotlib.patches import Circle
    center = np.asarray(d["center"])
    forward, reverse = np.asarray(d["forward_lane"]), np.asarray(d["reverse_lane"])
    station = np.asarray(d["reference_distances"])
    ax.plot(*center.T, c=".6", lw=.8, zorder=3)
    for interval in d["intervals"]:
        mask = (station >= interval["first_distance"] - 1e-8) & (station <= interval["last_distance"] + 1e-8)
        state = interval["state"]
        if state == "SPLIT":
            for path, color, back in [(forward, "#167bc1", False), (reverse, "#f18813", True)]:
                p = path[mask]
                ax.plot(*p.T, c=color, lw=1.8, zorder=5)
                arrows(ax, p, color, back)
                if disks and len(p) > 1:
                    ax.add_patch(Circle(p[len(p)//2], radius, color=color, alpha=.13, zorder=4))
            for s in [interval["first_distance"], interval["last_distance"]]:
                i = int(np.argmin(np.abs(station - s)))
                ax.plot([forward[i, 0], reverse[i, 0]], [forward[i, 1], reverse[i, 1]], c=".45", ls=":", lw=.7)
        elif state == "SHARED":
            ax.plot(*center[mask].T, "k--", lw=1.8, zorder=6)
        elif state == "TRANSITION":
            for path in [forward, reverse]:
                ax.plot(*path[mask].T, c="#a7338a", ls=":", lw=1.7, zorder=6)
        else:
            ax.plot(*center[mask].T, c="crimson", ls="--", lw=2, zorder=6)
    ax.set_aspect("equal", adjustable="box")


def lane_legend(fig):
    from matplotlib.lines import Line2D
    items = [(".6", "-", "Original center"), ("#167bc1", "-", "Forward keep-right"),
             ("#f18813", "-", "Reverse keep-right"), ("black", "--", "SHARED"),
             ("#a7338a", ":", "Transition (coordination required)"), ("crimson", "--", "Blocked"),
             (".45", ":", "SPLIT boundary")]
    fig.legend(handles=[Line2D([], [], color=c, ls=s, label=l) for c, s, l in items],
               loc="lower center", ncol=4, fontsize=9)


def independent_zooms(geometry, sweeps, output):
    import matplotlib.pyplot as plt
    panels = [("wide_horizontal", "Wide horizontal", (18, 38, -11, -3)),
              ("wide_vertical", "Wide vertical", (78, 89, -58, -42)),
              ("narrow_bent_reference", "Narrow passage", (94.5, 98.5, -53, -45)),
              ("variable_width", "Wide → narrow → wide", (83, 100, -59, -38)),
              ("junction_west_south", "Multiple incident corridors; crossing turns need coordination", (76, 94, -45, -32))]
    fig, axes = plt.subplots(2, 3, figsize=(17, 13), layout="constrained")
    for ax, (name, title, bounds) in zip(axes.flat, panels):
        _map(ax, geometry)
        targets = [s for s in sweeps if s["radius"] == .2 and (s["name"] == name or
                   (name.startswith("junction") and s["name"] == "junction_north_east"))]
        for s in targets:
            draw_lane(ax, s["decomposition"], s["effective_radius"])
        ax.set(xlim=bounds[:2], ylim=bounds[2:], title=title, xlabel="x", ylabel="y")
    axes.flat[-1].axis("off")
    fig.suptitle("Independent physical-corridor certificates · radius 0.20 + margin 0.05\n"
                 "Each opposing pair is certified separately; crossings and transitions require coordination")
    lane_legend(fig)
    fig.get_layout_engine().set(rect=(0, .07, 1, .88))
    fig.savefig(output / "warehouse2.independent_lane_debug.png", dpi=160)
    plt.close(fig)


def lane_comparison(geometry, rows, name, output, filename):
    import matplotlib.pyplot as plt
    selected = [s for s in rows if s["name"] == name and s["radius"] in RADII]
    height = 8 if name in {"wide_horizontal", "wide_W6", "variable_W6_neck1.2"} else 12
    fig, axes = plt.subplots(2, 4, figsize=(23, height), layout="constrained")
    for ax, s in zip(axes.flat, selected):
        _map(ax, geometry)
        draw_lane(ax, s["decomposition"], s["effective_radius"])
        p = np.asarray(s["decomposition"]["center"])
        lo, hi = p.min(axis=0), p.max(axis=0)
        pad = 4 if name in {"wide_horizontal", "wide_W6", "variable_W6_neck1.2"} else 2
        total = s["decomposition"]["reference_distances"][-1]
        ax.set(xlim=(lo[0]-pad, hi[0]+pad), ylim=(lo[1]-pad, hi[1]+pad), xlabel="x", ylabel="y",
               title=f"r={s['radius']:.3f}, R={s['effective_radius']:.3f}\n"
                     f"SPLIT {s['split_reference_length']/total:.0%} · "
                     f"{'traversable' if s['center_traversable'] else 'IMPASSABLE'}")
    fig.suptitle(f"{name} · fixed physical reference, unchanged graph · paired lanes at each robot radius")
    lane_legend(fig)
    fig.get_layout_engine().set(rect=(0, .07, 1, .88))
    fig.savefig(output / filename, dpi=150)
    plt.close(fig)


def route_comparison(world, geometry, queries, output):
    import matplotlib.pyplot as plt
    from matplotlib.lines import Line2D
    scenario = scenarios(world)[0][0]
    fig, axes = plt.subplots(2, 4, figsize=(24, 13), layout="constrained")
    colors = plt.get_cmap("tab10")
    for ax, radius in zip(axes.flat, RADII):
        q = queries[f"{scenario}_r{radius:g}"]
        _map(ax, geometry)
        for r in q["routes"]:
            p = np.asarray(r["path"])
            ax.plot(*p.T, color=colors(r["id"]), lw=1.7, label=f"Route {r['id']}")
        ax.scatter(*q["start"], color="green", s=20, zorder=8)
        ax.scatter(*q["goal"], color="red", marker="*", s=55, zorder=8)
        ax.set_title(f"radius={radius:g}, R={radius+.05:g} · {len(q['routes'])} routes")
        bounds = geometry["bounds"]
        ax.set(xlim=(bounds[0], bounds[2]), ylim=(bounds[1], bounds[3]))
        ax.set_aspect("equal", adjustable="box")
        if not q["routes"]:
            reasons = list(dict.fromkeys(q["lane_failures"]))
            status = "NO COMPLETE ROUTE" if q["physical_route_reachable"] else "NO FEASIBLE ATTACHED ROUTE"
            ax.text(.5, .5, status + "\n" + "\n".join(reasons[:3]), transform=ax.transAxes,
                    ha="center", color="crimson", fontsize=9, bbox=dict(facecolor="white", alpha=.9, edgecolor="none"))
    fig.suptitle(f"{world}.{scenario} · identical start/goal, fixed graph and plot limits; consistent route colors")
    fig.legend(handles=[Line2D([], [], color=colors(i), label=f"Route {i}") for i in range(5)],
               loc="lower center", ncol=5)
    fig.get_layout_engine().set(rect=(0, .04, 1, .90))
    fig.savefig(output / f"{world}.route_radius_comparison.png", dpi=150)
    plt.close(fig)


def quantitative_plots(rows, output):
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(2, 3, figsize=(17, 10), layout="constrained")
    keys = [("split_fraction", "SPLIT fraction"), ("feasible_physical_edges", "Feasible complete physical edges"),
            ("generated_routes", "Generated complete routes"), ("planning_ms", "First-query planning (ms)"),
            ("shortest_route_length", "Shortest final route length"), ("cached_planning_ms", "Cached planning median (ms)")]
    for ax, (key, title) in zip(axes.flat, keys):
        groups = sorted(set((r["world"], r["scenario"]) for r in rows if r["scenario"] != "short"))
        for world, scenario in groups:
            data = [r for r in rows if r["world"] == world and r["scenario"] == scenario]
            line, = ax.plot([r["radius"] for r in data], [np.nan if r[key] is None else r[key] for r in data],
                            "o-", ms=4, label=f"{world}.{scenario}")
            invalid = [r for r in data if not r["generated_routes"]]
            ax.scatter([r["radius"] for r in invalid], [.02]*len(invalid), transform=ax.get_xaxis_transform(),
                       color=line.get_color(), marker="x", s=45)
        ax.set(xlabel="robot radius (fixed margin 0.05)", title=title)
        ax.grid(alpha=.25)
    handles, labels = axes.flat[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="lower center", ncol=3, fontsize=8)
    fig.suptitle("Radius sensitivity · × at panel bottom = no returned complete route; missing lengths/fractions remain gaps")
    fig.get_layout_engine().set(rect=(0, .12, 1, .82))
    fig.savefig(output / "radius_metrics.png", dpi=160)
    plt.close(fig)


def new_split_length(small, large):
    """Analyse native station intervals, without recomputing any geometry."""
    added = 0
    for b in large["intervals"]:
        if b["state"] != "SPLIT":
            continue
        covered = sum(max(0, min(a["last_distance"], b["last_distance"]) -
                          max(a["first_distance"], b["first_distance"]))
                      for a in small["intervals"] if a["state"] == "SPLIT")
        added += max(0, b["last_distance"] - b["first_distance"] - covered)
    return added


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=ROOT / "benchmarks/results/radius_sweep")
    parser.add_argument("--binary", type=Path, default=ROOT / "build/routes/corridor_plan")
    parser.add_argument("--repeats", type=int, default=2)
    parser.add_argument("--analyse-only", action="store_true", help="reuse verified native JSON to regenerate tables and plots")
    args = parser.parse_args()
    if args.repeats < 1:
        parser.error("--repeats must be at least 1")
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    os.environ.setdefault("MPLCONFIGDIR", "/tmp/mppi-matplotlib")
    import matplotlib
    matplotlib.use("Agg")
    rows, aggregate, sweep_rows, monotonicity = [], [], [], []
    native, bundle_hashes = {}, {}
    for world in ["warehouse2", "warehouse"]:
        map_path = ROOT / f"benchmarks/results/complete_routes/input/{world}.resources.json"
        bundle_hashes[world] = hashlib.sha256(map_path.read_bytes()).hexdigest()
        traffic = json.loads(map_path.read_text())
        queries = []
        # Every size first sees every scenario before the next size. Native lanes
        # remain radius-keyed; repeat measurements reuse only matching certificates.
        for radius in RADII:
            for scenario, start, goal in scenarios(world):
                q = dict(name=f"{scenario}_r{radius:g}", start=start, goal=goal, radius=radius, safety_margin=.05, k=5)
                queries += [q, dict(q, name=q["name"]+"_baseline", k=1)]
                queries += [dict(q, name=q["name"]+f"_repeat{i}") for i in range(args.repeats)]
        document = dict(queries=queries)
        if world == "warehouse2":
            document["corridors"] = [dict(c, radii=CORRIDOR_RADII, safety_margin=.05, taper_endpoints=False)
                                     for c in corridors()]
        inputs, results = output/f"{world}.queries.json", output/f"{world}.routes.json"
        if not args.analyse_only:
            inputs.write_text(json.dumps(document, indent=2)+"\n")
            subprocess.run([str(args.binary.resolve()), "--map", str(map_path), "--queries", str(inputs),
                            "--output", str(results), "--verify", "--debug-lanes"], check=True)
        data = json.loads(results.read_text())
        native[world] = data
        if hashlib.sha256(map_path.read_bytes()).hexdigest() != bundle_hashes[world]:
            raise RuntimeError("offline bundle changed during experiment")
        by_name = {q["name"]:q for q in data["queries"]}
        for radius in RADII:
            previous_radius = RADII[RADII.index(radius)-1] if radius != RADII[0] else None
            for scenario, _, _ in scenarios(world):
                q = by_name[f"{scenario}_r{radius:g}"]
                repeats = [by_name[q["name"]+f"_repeat{i}"] for i in range(args.repeats)]
                row = query_summary(world, scenario, q, repeats)
                previous = by_name[f"{scenario}_r{previous_radius:g}"] if previous_radius is not None else q
                gained = set(q["feasible_edge_ids"]) - set(previous["feasible_edge_ids"])
                if gained:
                    raise RuntimeError(f"nonmonotonic physical edge feasibility: {world} {radius}: {gained}")
                row["physical_routes_changed_from_previous_radius"] = [r["edge_ids"] for r in q["routes"]] != [r["edge_ids"] for r in previous["routes"]]
                rows.append(row)
            subset = [r for r in rows if r["world"] == world and r["radius"] == radius]
            route_count = sum(r["generated_routes"] for r in subset)
            total_length = sum((r["mean_route_length"] or 0) * r["generated_routes"] for r in subset)
            total_split = sum(r["total_split_length"] for r in subset)
            aggregate.append(dict(world=world, radius=radius, effective_radius=radius+.05,
                feasible_physical_edges=subset[0]["feasible_physical_edges"],
                reachable_start_goal_pairs=sum(r["physical_route_reachable"] for r in subset),
                complete_route_pairs=sum(r["generated_routes"]>0 for r in subset),
                generated_routes=route_count,
                shortest_route_length=min((r["shortest_route_length"] for r in subset if r["shortest_route_length"] is not None), default=None),
                mean_route_length=total_length/route_count if route_count else None,
                total_split_length=total_split,
                total_shared_length=sum(r["total_shared_length"] for r in subset),
                total_transition_length=sum(r["total_transition_length"] for r in subset),
                split_fraction=total_split/total_length if total_length else None,
                mode_transitions=sum(r["mode_transitions"] for r in subset),
                rejected_split_candidates=sum(r["rejected_split_candidates"] for r in subset),
                candidate_path_validation_failures=sum(r["candidate_path_validation_failures"] for r in subset),
                split_footprint_rejection_spans=sum(r["split_footprint_rejection_spans"] for r in subset),
                returned_route_validation_failures=0,
                sum_first_query_ms=sum(r["planning_ms"] for r in subset)))
        route_comparison(world, traffic["map_geometry"], by_name, output)
        if world == "warehouse2":
            sweeps = data["corridor_sweeps"]
            independent_zooms(traffic["map_geometry"], sweeps, output)
            for name in ["wide_horizontal", "variable_width", "narrow_bent_reference"]:
                lane_comparison(traffic["map_geometry"], sweeps, name, output, f"warehouse2.{name}.radius_comparison.png")
            for s in sweeps:
                sweep_rows.append({k:v for k,v in s.items() if k != "decomposition"} | dict(
                    rejections=s["decomposition"]["rejections"], intervals=s["decomposition"]["intervals"]))
            for name in sorted(set(s["name"] for s in sweeps)):
                selected = [s for s in sweeps if s["name"] == name]
                for small, large in zip(selected, selected[1:]):
                    added = new_split_length(small["decomposition"], large["decomposition"])
                    if added > 1e-6:
                        monotonicity.append(dict(corridor=name, smaller_radius=small["radius"],
                            larger_radius=large["radius"], newly_split_reference_length=added,
                            smaller_rejections=small["decomposition"]["rejections"],
                            larger_rejections=large["decomposition"]["rejections"]))
    synthetic_path = output / "synthetic_radius.routes.json"
    if not args.analyse_only:
        subprocess.run([str(ROOT/"build/routes/corridor_radius_test"), str(synthetic_path)], check=True)
    synth = json.loads(synthetic_path.read_text())["cases"]
    for name in ["wide_W6", "variable_W6_neck1.2"]:
        selected = [dict(s, name=name) for s in synth if s["name"] == name]
        # Complete scalar totals come from C++; no Python geometry is used.
        lane_comparison(selected[0]["map_geometry"], selected, name, output, f"synthetic.{name}.radius_comparison.png")
    quantitative_plots(rows, output)
    csv_file(output/"query_metrics.csv", rows)
    csv_file(output/"radius_summary.csv", aggregate)
    csv_file(output/"corridor_metrics.csv", sweep_rows)
    report = dict(radii=RADII, safety_margin=.05,
                  timing_scope="exclusive native per-query stages; map loading, independent corridor sweeps, JSON, plotting and verification audit excluded",
                  map_load_ms={w:d["map_load_seconds"]*1000 for w,d in native.items()},
                  offline_bundle_sha256=bundle_hashes,
                  queries=rows, radius_summary=aggregate, corridor_sweeps=sweep_rows,
                  fixed_corridor_constructor_nonmonotonicity=monotonicity)
    (output/"statistics.json").write_text(json.dumps(report, indent=2)+"\n")
    print(f"Radius sweep complete: {len(rows)} configurations; {len(monotonicity)} fixed-corridor construction increases to investigate. {output}")


if __name__ == "__main__":
    main()
