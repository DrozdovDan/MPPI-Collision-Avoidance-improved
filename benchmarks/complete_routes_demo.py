"""Visualize native lane-aware routes and print the C++ benchmark measurements.

Topology, lane construction, swept collision/separation validation and diversity
checks run in C++. Python only prepares demo inputs, reads JSON and draws figures.
"""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "benchmarks"))
from corridor_graph.traffic_visualization import _map


def poses(world, suite):
    if world == "warehouse2":
        return [("long", (7.5, -10), (116, -70)),
                ("central", (45, -40), (75, -40)),
                ("major_obstacle", (55, -30), (55, -50)),
                ("dense", (20, -70), (105, -70)),
                ("nearby", (7.5, -10), (8, -10)),
                ("narrow", (60, -40), (59, -40))]
    tasks = [t for t in suite["tasks"] if t["world"] == world]
    selected = [t for t in tasks if t["id"].startswith("testing_")]
    selected += [t for t in tasks if not t["id"].startswith("testing_")][:1]
    result = [(t["id"], t["start"][:2], t["goal"][:2]) for t in selected]
    if world == "warehouse":
        result.append(("dense", (8.3, -28), (42.3, -28)))
    if selected:
        start = selected[0]["start"][:2]
        result.append(("nearby", start, (start[0] + .2, start[1])))
    return result


def relative(filename):
    return str(filename.relative_to(ROOT)) if filename.is_relative_to(ROOT) else str(filename)


def arrows(ax, points, color, reverse=False):
    p = np.asarray(points)
    if len(p) < 2:
        return
    for i in np.linspace(0, len(p) - 2, min(4, len(p) - 1), dtype=int):
        a, b = p[i], p[i + 1]
        if reverse:
            a, b = b, a
        if np.linalg.norm(b - a) < 1e-8:
            continue
        angle = np.degrees(np.arctan2(b[1] - a[1], b[0] - a[0]))
        # Local direction markers cannot draw fictitious chords across a bend.
        midpoint = (a + b) / 2
        ax.plot(*midpoint, marker=(3, 0, angle - 90), ms=6, color=color, zorder=8)


def zoom(ax, points, bounds):
    p = np.asarray(points)
    lo, hi = p.min(axis=0), p.max(axis=0)
    margin = max(1.5, (hi - lo).max() * .12)
    ax.set(xlim=(max(bounds[0], lo[0] - margin), min(bounds[2], hi[0] + margin)),
           ylim=(max(bounds[1], lo[1] - margin), min(bounds[3], hi[1] + margin)))


def plot(query, traffic, folder, name):
    import matplotlib.pyplot as plt
    from matplotlib.collections import LineCollection
    routes = query["routes"]
    if not routes:
        return []
    current_route_files = {f"{name}.route{r['id']}.png" for r in routes}
    for previous in folder.glob(f"{name}.route*.png"):
        if previous.name not in current_route_files:
            previous.unlink()
    paths = []
    colors = plt.get_cmap("tab10")
    for selected in [routes] + [[r] for r in routes]:
        fig, ax = plt.subplots(figsize=(13, 9), constrained_layout=True)
        _map(ax, traffic["map_geometry"])
        if "routing_geometry" in traffic:
            ax.add_collection(LineCollection([e["geometry"] for e in traffic["routing_geometry"]["edges"]], colors=".5", linewidths=.5, alpha=.3))
        for r in selected:
            color = colors(r["id"])
            p = np.asarray(r["path"])
            ax.plot(*p.T, c=color, lw=1.8, alpha=.6, zorder=4,
                    label=f"Route {r['id']}: {r['metrics']['length']:.2f} (physical {r['physical_metrics']['length']:.2f})")
            for s in r["segments"]:
                points = np.asarray(s["geometry"])
                if len(points) < 2:
                    continue
                if selected is not routes:
                    ref = np.asarray(s["reference_geometry"])
                    if len(ref) > 1:
                        ax.plot(*ref.T, c=".6", lw=.8, alpha=.6)
                state = s["state"]
                style = "-" if state == "SPLIT" else ":" if state == "TRANSITION" else "--"
                ax.plot(*points.T, c=color if selected is routes or state == "SPLIT" else "#ae2c83" if state == "TRANSITION" else "#b76b11",
                        lw=2.4 if state == "SPLIT" else 2, ls=style, zorder=6)
            arrows(ax, r["path"], color)
            if selected is not routes:
                labels = []
                for u in r["resource_traversals"]:
                    p0 = np.asarray(u["entry"])
                    if all(np.linalg.norm(p0 - q) > 3 for q in labels):
                        ax.annotate(f"R{u['resource_id']}", p0, xytext=(3, 5), textcoords="offset points", fontsize=7,
                                    bbox=dict(facecolor="white", edgecolor="none", alpha=.7), zorder=9)
                        labels.append(p0)
                        if len(labels) >= 6:
                            break
            m = r["metrics"]
            if selected is not routes:
                ax.set_title(f"{name} · route {r['id']} · R={query['radius'] + query['safety_margin']:.2f}\n"
                             f"length {m['length']:.2f} / physical {m['physical_length']:.2f} · ratio {m['length_inflation_ratio']:.3f} · intersections {m['self_intersection_count']}\n"
                             f"SPLIT {m['split_length']:.2f} · SHARED {m['shared_length']:.2f} · transition length {m['transition_length']:.2f} · mode changes {m['mode_transition_count']}")
        ax.scatter(*query["start"], marker="o", s=65, c="#16a34a", edgecolors="white", zorder=10, label="Start")
        ax.scatter(*query["goal"], marker="*", s=130, c="#dc2626", edgecolors="white", zorder=10, label="Goal")
        ax.plot([], [], color=".25", lw=2.4, label="SPLIT: mandatory right-hand lane")
        ax.plot([], [], color="#b76b11", ls="--", lw=2, label="SHARED / local connectors")
        ax.plot([], [], color="#ae2c83", ls=":", lw=2, label="Validated mode transition")
        if selected is routes:
            ax.set_title(f"{name} · R={query['radius'] + query['safety_margin']:.2f} · {query['planning_seconds'] * 1000:.2f} ms\n"
                         "Physical topology selected in C++; mandatory keep-right placement")
        else:
            zoom(ax, selected[0]["path"], traffic["map_geometry"]["bounds"])
        ax.legend(fontsize=8, loc="upper center", bbox_to_anchor=(.5, -.08), ncol=3)
        suffix = "overview" if selected is routes else f"route{selected[0]['id']}"
        filename = folder / f"{name}.{suffix}.png"
        fig.savefig(filename, dpi=140)
        plt.close(fig)
        paths.append(relative(filename))
    return paths


def plot_decompositions(world, decompositions, folder, name):
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle
    count = len(decompositions)
    if not count:
        return None
    columns = min(2, count)
    fig, axes = plt.subplots((count + columns - 1) // columns, columns,
                             figsize=(12 if count == 1 else 14, 4.5 if count == 1 else 5 * ((count + 1) // 2)),
                             squeeze=False, constrained_layout=True)
    for ax, d in zip(axes.flat, decompositions):
        _map(ax, world)
        center, forward, reverse = map(np.asarray, [d["center"], d["forward_lane"], d["reverse_lane"]])
        distances = np.asarray(d["reference_distances"])
        ax.plot(*center.T, c=".6", lw=1, label="Physical reference")
        circle_count = 0
        for interval in d["intervals"]:
            mask = (distances >= interval["first_distance"] - 1e-8) & (distances <= interval["last_distance"] + 1e-8)
            state = interval["state"]
            if state == "SPLIT":
                ax.plot(*forward[mask].T, c="#187bc0", lw=2.2)
                ax.plot(*reverse[mask].T, c="#ed8b16", lw=2.2)
                arrows(ax, forward[mask], "#187bc0")
                arrows(ax, reverse[mask], "#ed8b16", reverse=True)
                if circle_count < 3:
                    indices = np.flatnonzero(mask)
                    i = indices[len(indices) // 2]
                    for p, color in [(forward[i], "#187bc0"), (reverse[i], "#ed8b16")]:
                        ax.add_patch(Circle(p, d["effective_radius"], facecolor=color, alpha=.2, edgecolor=color, lw=.8))
                    circle_count += 1
            elif state == "TRANSITION":
                ax.plot(*forward[mask].T, color="#ae2c83", ls=":", lw=1.8)
                ax.plot(*reverse[mask].T, color="#ae2c83", ls=":", lw=1.8)
            elif state == "SHARED":
                ax.plot(*center[mask].T, color="black", ls="--", lw=2)
        ax.plot([], [], color="#187bc0", lw=2, label="u → v: canonical right lane")
        ax.plot([], [], color="#ed8b16", lw=2, label="v → u: reverse-travel right lane")
        ax.plot([], [], color="black", ls="--", label="Shared center")
        ax.plot([], [], color="#ae2c83", ls=":", label="Shared ↔ split transitions")
        zoom(ax, np.concatenate([center, forward, reverse]), world["bounds"])
        label = f"Physical edge {d['physical_edge_id']}" if d['physical_edge_id'] >= 0 else "Continuous corridor section"
        ax.set_title(f"{label} · R={d['effective_radius']:.2f}")
    for ax in list(axes.flat)[count:]:
        ax.set_visible(False)
    handles, labels = axes.flat[0].get_legend_handles_labels()
    fig.legend(handles, labels, fontsize=7, loc="lower center", ncol=2)
    fig.get_layout_engine().set(rect=(0, .11, 1, .89))
    filename = folder / f"{name}.lanes.png"
    fig.savefig(filename, dpi=150)
    plt.close(fig)
    return relative(filename)


def plot_lane_zooms(query, world, folder, name):
    """Draw saved C++ certificates; no feasibility/validation is performed here."""
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle
    views = [("Wide horizontal corridor", 0, (18, 38, -12, -3)),
             ("Wide vertical corridor · southbound right is west", 0, (78, 88, -57, -44)),
             ("Narrow passage · SHARED center", 3, (94.5, 98.5, -54, -44)),
             ("Wide junction · continuous right-hand connection", 0, (75, 88, -45, -35))]
    fig, axes = plt.subplots(2, 2, figsize=(14, 11), constrained_layout=True)
    for ax, (title, route_index, bounds) in zip(axes.flat, views):
        _map(ax, world)
        route = query["routes"][min(route_index, len(query["routes"]) - 1)]
        # Segment station metadata selects the actual certificate used by this route.
        d = query["lane_decompositions"][route["id"]]
        if "Narrow passage" in title:
            for candidate in query["lane_decompositions"]:
                p = np.asarray(candidate["center"])
                stations = np.asarray(candidate["reference_distances"])
                for interval in candidate["intervals"]:
                    if interval["state"] == "SHARED" and np.any(
                        (stations >= interval["first_distance"]) & (stations <= interval["last_distance"]) &
                        (p[:, 0] > bounds[0]) & (p[:, 0] < bounds[1]) & (p[:, 1] > bounds[2]) & (p[:, 1] < bounds[3])):
                        d = candidate
                        break
        center, forward, reverse = map(np.asarray, [d["center"], d["forward_lane"], d["reverse_lane"]])
        stations = np.asarray(d["reference_distances"])
        ax.plot(*center.T, color=".6", lw=.9, zorder=3)
        circles = 0
        for interval in d["intervals"]:
            ids = np.flatnonzero((stations >= interval["first_distance"] - 1e-8) &
                                 (stations <= interval["last_distance"] + 1e-8))
            state = interval["state"]
            if state == "SHARED":
                ax.plot(*center[ids].T, color="black", ls="--", lw=2, zorder=5)
                continue
            colors = ("#187bc0", "#ed8b16") if state == "SPLIT" else ("#ae2c83", "#ae2c83")
            for points, color, backwards in [(forward, colors[0], False), (reverse, colors[1], True)]:
                ax.plot(*points[ids].T, color=color, lw=2, ls="-" if state == "SPLIT" else ":", zorder=5)
                visible = [i for i in ids[:-1] if bounds[0] <= points[i][0] <= bounds[1] and bounds[2] <= points[i][1] <= bounds[3]]
                if visible:
                    for i in np.asarray(visible)[np.linspace(0, len(visible) - 1, min(3, len(visible)), dtype=int)]:
                        arrows(ax, points[i:i + 2], color, reverse=backwards)
            if state == "SPLIT":
                for i in (ids[0], ids[-1]):
                    ax.plot([forward[i, 0], reverse[i, 0]], [forward[i, 1], reverse[i, 1]], color=".4", ls=":", lw=.8, zorder=4)
                visible = [i for i in ids if bounds[0] < center[i][0] < bounds[1] and bounds[2] < center[i][1] < bounds[3]]
                if visible and circles < 2:
                    i = visible[len(visible) // 2]
                    for p, color in [(forward[i], "#187bc0"), (reverse[i], "#ed8b16")]:
                        ax.add_patch(Circle(p, d["effective_radius"], facecolor=color, edgecolor=color, alpha=.23, zorder=6))
                    circles += 1
        ax.set(xlim=bounds[:2], ylim=bounds[2:], title=title)
        ax.set_aspect("equal", adjustable="box")
    ax = axes.flat[0]
    ax.plot([], [], color=".6", lw=.9, label="Original corridor center")
    ax.plot([], [], color="#187bc0", lw=2, label="Forward right-hand lane")
    ax.plot([], [], color="#ed8b16", lw=2, label="Reverse right-hand lane")
    ax.plot([], [], color="black", ls="--", label="SHARED (no opposing passing)")
    ax.plot([], [], color="#ae2c83", ls=":", label="Finite transition")
    ax.plot([], [], color=".4", ls=":", label="SPLIT boundaries")
    fig.legend(*ax.get_legend_handles_labels(), loc="lower center", ncol=3, fontsize=9)
    fig.get_layout_engine().set(rect=(0, .07, 1, .93))
    fig.suptitle(f"{name} · C++ opposing-lane certificates · R={query['radius'] + query['safety_margin']:.2f}")
    filename = folder / f"{name}.lane_zoom.png"
    fig.savefig(filename, dpi=160)
    plt.close(fig)
    return relative(filename)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--binary", type=Path, default=ROOT / "build/routes/corridor_plan")
    parser.add_argument("--traffic-dir", type=Path, default=ROOT / "benchmarks/results/complete_routes/input")
    parser.add_argument("--output", type=Path, default=ROOT / "benchmarks/results/lane_routes")
    parser.add_argument("--world", nargs="+", default=["warehouse2", "warehouse", "parking1", "parking3"])
    parser.add_argument("--repeats", type=int, default=3)
    parser.add_argument("--no-plots", action="store_true")
    parser.add_argument("--synthetic", action="store_true")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    os.environ.setdefault("MPLCONFIGDIR", "/tmp/mppi-matplotlib")
    import matplotlib
    matplotlib.use("Agg")
    suite = json.loads((ROOT / "benchmarks/bench_mr_tasks.json").read_text())
    report = {"timing_scope": "native topology, cached lane decomposition, validated connections, reconstruction, resource remapping and final collision checks; loading, JSON and additional native verification excluded", "worlds": {}}
    for world in args.world:
        filename = args.traffic_dir / f"{world}.resources.json"
        traffic = json.loads(filename.read_text())
        queries, original = [], []
        for label, start, goal in poses(world, suite):
            for radius in ([.2, .8, 1.2] if world == "warehouse2" else [.2, .5]):
                q = dict(name=f"{label}_r{radius}", start=list(start), goal=list(goal), radius=radius, safety_margin=.05, k=5)
                original.append(q)
                queries += [q, dict(q, name=q["name"] + "_baseline", k=1)]
                queries += [dict(q, name=q["name"] + f"_repeat{i}") for i in range(args.repeats)]
        inputs, outputs = args.output / f"{world}.queries.json", args.output / f"{world}.routes.json"
        inputs.write_text(json.dumps({"queries": queries}, indent=2) + "\n")
        subprocess.run([str(args.binary.resolve()), "--map", str(filename), "--queries", str(inputs), "--output", str(outputs), "--verify", "--debug-lanes"], check=True)
        results = json.loads(outputs.read_text())
        by_name = {q["name"]: q for q in results["queries"]}
        entries = []
        for source in original:
            q = by_name[source["name"]]
            repeats = [by_name[source["name"] + f"_repeat{i}"] for i in range(args.repeats)]
            images = []
            if not args.no_plots and source["radius"] == .2 and ("long" in source["name"] or "central" in source["name"] or source is original[0]):
                images = plot(q, traffic, args.output, f"{world}.{source['name']}")
                debug = plot_decompositions(traffic["map_geometry"], q["lane_decompositions"][:4], args.output, f"{world}.{source['name']}")
                if debug:
                    images.append(debug)
                if world == "warehouse2" and source["name"] == "long_r0.2":
                    images.append(plot_lane_zooms(q, traffic["map_geometry"], args.output, f"{world}.{source['name']}"))
            entries.append(dict(name=source["name"], start=source["start"], goal=source["goal"], radius=source["radius"], count=len(q["routes"]),
                                lengths=[r["metrics"]["length"] for r in q["routes"]], physical_lengths=[r["physical_metrics"]["length"] for r in q["routes"]],
                                narrow_exposure=[r["metrics"]["narrow_exposure"] for r in q["routes"]], shared_exposure=[r["metrics"]["shared_exposure"] for r in q["routes"]],
                                split_length=[r["metrics"]["split_length"] for r in q["routes"]], transition_length=[r["metrics"]["transition_length"] for r in q["routes"]],
                                length_inflation_ratio=[r["metrics"]["length_inflation_ratio"] for r in q["routes"]], self_intersection_count=[r["metrics"]["self_intersection_count"] for r in q["routes"]],
                                max_lateral_deviation=[r["metrics"]["max_lateral_deviation"] for r in q["routes"]], mode_transition_count=[r["metrics"]["mode_transition_count"] for r in q["routes"]],
                                min_clearance=[r["metrics"]["min_clearance"] for r in q["routes"]], bottleneck_penalty=[r["metrics"]["bottleneck_penalty"] for r in q["routes"]],
                                lane_reports=[dict(metrics=r["metrics"], rejections=r["lane_rejections"], diagnostics=r["lane_diagnostics"]) for r in q["routes"]],
                                lane_placement_ms=q["lane_placement_seconds"] * 1000, route_search_ms=q["route_search_seconds"] * 1000,
                                lane_failures=q["lane_failures"], rejected_lane_routes=q["rejected_lane_routes"],
                                rejected_split_candidates=q["rejected_split_candidates"], split_failure_reasons=q["split_failure_reasons"],
                                overlap=q["resource_overlap"], planning_ms=q["planning_seconds"] * 1000, validation_passed=q["validation_passed"],
                                repeated_ms=[r["planning_seconds"] * 1000 for r in repeats], images=images))
        report["worlds"][world] = dict(map_load_ms=results["map_load_seconds"] * 1000, queries=entries)
    if args.synthetic:
        filename = args.output / "synthetic.routes.json"
        subprocess.run([str(ROOT / "build/routes/corridor_lanes_test"), str(filename)], check=True)
        cases = json.loads(filename.read_text())["cases"]
        if not args.no_plots:
            for case in cases:
                plot(case, {"map_geometry": case["map_geometry"]}, args.output, case["name"])
                plot_decompositions(case["map_geometry"], [case["decomposition"]], args.output, case["name"])
    (args.output / "statistics.json").write_text(json.dumps(report, indent=2) + "\n")
    print(f"Native validation passed. Measurements and figures: {args.output}")


if __name__ == "__main__":
    main()
