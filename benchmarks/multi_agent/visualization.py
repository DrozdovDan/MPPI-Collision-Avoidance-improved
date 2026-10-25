"""Actual footprints, assignment overviews, conflict zooms, and recorded runs."""

from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import shutil

import numpy as np

from .geometry import footprints, load_suite
from corridor_graph.traffic_visualization import _map


def _plotting():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    return plt


def conflict_region(task):
    if task["resources"]:
        uses = Counter(i for r in task["robots"] for i, _ in r["geometry_evidence"]["resource_uses"])
        selected = max(task["resources"], key=lambda f: (uses[f["id"]], -f["width"]))
        minimum = 3. if task["robots"][0]["model"] == "car_like" else 5.
        return np.array(selected["center"]), max(3 * max(r["radius"] for r in task["robots"]), selected["radius"]*2, minimum)
    shared = Counter()
    for robot in task["robots"]:
        shared.update(set(tuple(np.round(p, 5)) for p in robot["geometry_evidence"]["reference_path"]))
    point = max(shared, key=lambda p: (shared[p], p))
    return np.asarray(point), max(1., 5 * max(r["radius"] for r in task["robots"]))


def draw_body(ax, robot, pose, color, *, goal=False):
    from matplotlib.patches import Circle, Polygon
    if robot["footprint"] == "rectangle":
        body = footprints([pose], robot)[0]
        ax.add_patch(Polygon(np.asarray(body.exterior.coords), facecolor="none" if goal else color,
                             edgecolor=color if goal else "black", linewidth=.65, alpha=.75, zorder=5))
    else:
        ax.add_patch(Circle(pose[:2], robot["radius"], facecolor="none" if goal else color,
                            edgecolor=color if goal else "black", linewidth=.6, alpha=.8, zorder=5))
    if not goal:
        length = robot["length"]*.6
        ax.arrow(pose[0], pose[1], length*math.cos(pose[2]), length*math.sin(pose[2]),
                 width=0., head_width=robot["width"]*.32, color="black", linewidth=.6, zorder=6,
                 length_includes_head=True)


def overview(task, world, filename, *, zoom=False):
    plt = _plotting()
    from matplotlib.patches import Circle
    fig, ax = plt.subplots(figsize=(10, 7))
    fig.subplots_adjust(bottom=.12, top=.90, left=.10, right=.98)
    _map(ax, world)
    if zoom:
        center, radius = conflict_region(task)
        x0, y0, x1, y1 = world["bounds"]
        ax.set(xlim=(max(x0, center[0]-radius), min(x1, center[0]+radius)),
               ylim=(max(y0, center[1]-radius), min(y1, center[1]+radius)))
    fig.canvas.draw()
    label_boxes = []
    def label(text, position, color, size, radius):
        x, y = ax.transData.transform(position)
        if not ax.bbox.contains(x, y): return
        scale = abs(ax.transData.transform([position[0]+radius, position[1]])[0]-x)
        shift = scale * 72 / fig.dpi + 4
        offsets = [(shift,shift), (shift,-shift-6), (-shift-12,shift), (-shift-12,-shift-6),
                   (shift+10,2), (-shift-20,2), (-4,shift+8), (-4,-shift-12)]
        scored = []
        for dx, dy in offsets:
            px, py = x+dx*fig.dpi/72, y+dy*fig.dpi/72
            box = (px,py,px+len(text)*size*fig.dpi/120,py+size*fig.dpi/72)
            overlap = sum(max(0,min(box[2],b[2])-max(box[0],b[0])) * max(0,min(box[3],b[3])-max(box[1],b[1])) for b in label_boxes)
            outside = not (ax.bbox.contains(box[0],box[1]) and ax.bbox.contains(box[2],box[3]))
            scored.append((overlap+10000*outside, dx*dx+dy*dy, dx,dy,box))
        _,_,dx,dy,box = min(scored)
        label_boxes.append(box)
        ax.annotate(text,position,xytext=(dx,dy),textcoords="offset points",fontsize=size,color=color,
                    bbox=dict(fc="white",ec="none",alpha=.65,pad=.2),zorder=8)
    colors = plt.get_cmap("tab20")
    for index, resource in enumerate(task["resources"]):
        ax.add_patch(Circle(resource["center"], resource["radius"], facecolor="#e9ad24", edgecolor="#aa7100",
                            alpha=.14, linewidth=1.1, zorder=2))
        if index < 4:
            ax.annotate(f"R{resource['id']}", resource["center"], fontsize=6, color="#815600")
    for i, robot in enumerate(task["robots"]):
        color = colors(i % 20)
        start, goal = robot["start"], robot["goal"]["pose"]
        draw_body(ax, robot, start, color)
        draw_body(ax, robot, goal, color, goal=True)
        ax.plot(*goal[:2], marker="*", ms=5, color=color, zorder=7)
        label(f"{i:02d}",start[:2],color,6.,robot["radius"])
        label(f"g{i:02d}",goal[:2],color,4.5,robot["radius"])
        if task["robot_count"] <= 12:
            ax.annotate("", goal[:2], xytext=start[:2], arrowprops=dict(arrowstyle="->", color=color, alpha=.18, lw=.6))
    model = "car-like rectangles" if task["robots"][0]["model"] == "car_like" else "differential-drive disks"
    ax.set_title(f"{task['id']} · {task['category']} · N={task['robot_count']}\n"
                 f"{model} · intended {task['intended_difficulty']}" + (" · conflict zoom" if zoom else ""), fontsize=10)
    fig.text(.5, .02, "Filled footprint + arrow: start / heading     Outline + star: goal     Yellow: geometric resource     Units: native map units", ha="center", fontsize=7)
    fig.savefig(filename, dpi=120)
    plt.close(fig)


def contact_sheet(images, filename, *, columns, tile=(420, 310)):
    from PIL import Image, ImageDraw
    rows = math.ceil(len(images)/columns)
    sheet = Image.new("RGB", (columns*tile[0], rows*tile[1]), "white")
    draw = ImageDraw.Draw(sheet)
    for i, (label, path) in enumerate(images):
        with Image.open(path) as opened:
            thumb = opened.convert("RGB")
            thumb.thumbnail((tile[0], tile[1]-22))
            x = (i % columns)*tile[0] + (tile[0]-thumb.width)//2
            y = (i // columns)*tile[1] + 22
            sheet.paste(thumb, (x, y))
        draw.text(((i % columns)*tile[0]+8, (i // columns)*tile[1]+4), label, fill="black")
    sheet.save(filename)


def visualize_suite(folder):
    folder = Path(folder)
    _, worlds, tasks = load_suite(folder)
    output = folder / "visualizations"; output.mkdir(exist_ok=True)
    images, by_map = [], {}
    for task in tasks:
        filename = output / f"{task['id']}.png"
        overview(task, worlds[task["source_map"]], filename)
        entry = (f"{task['id']} | N={task['robot_count']} | {task['intended_difficulty']}", filename)
        images.append(entry); by_map.setdefault(task["source_map"], []).append(entry)
        if task["intended_difficulty"] == "deadlock_prone":
            overview(task, worlds[task["source_map"]], output / f"{task['id']}.zoom.png", zoom=True)
    for name, entries in by_map.items():
        contact_sheet(entries, output / f"{name}.contact.png", columns=5)
    contact_sheet(images, output / "all_50.contact.png", columns=10, tile=(350, 265))
    write_report(folder)
    return output


def animate_run(task, world, states, filename, *, result=None, frame_count=48):
    plt = _plotting()
    from matplotlib.animation import FuncAnimation, PillowWriter
    frames = np.unique(np.linspace(0, len(states)-1, min(frame_count, len(states))).astype(int))
    fig, axes = plt.subplots(1, 2, figsize=(12, 5), constrained_layout=True)
    colors = plt.get_cmap("tab20")
    center, radius = conflict_region(task)
    def update(frame):
        step = frames[frame]
        for ax in axes:
            ax.clear(); _map(ax, world)
        axes[1].set(xlim=(center[0]-radius, center[0]+radius), ylim=(center[1]-radius, center[1]+radius), title="Intended conflict region")
        for i, robot in enumerate(task["robots"]):
            color = colors(i % 20)
            for ax in axes:
                ax.plot(states[:step+1, i, 0], states[:step+1, i, 1], color=color, lw=1., alpha=.7)
                draw_body(ax, robot, states[step, i], color)
                ax.plot(*robot["goal"]["pose"][:2], "*", ms=5, color=color)
        status = result["status"] if result else "recorded motion"
        axes[0].set_title(f"{task['id']} · {step*task['dt']:.1f} s\n{status} at end of observation", fontsize=10)
    movie = FuncAnimation(fig, update, frames=len(frames), interval=125)
    Path(filename).parent.mkdir(parents=True, exist_ok=True)
    movie.save(filename, writer=PillowWriter(fps=8)); plt.close(fig)


def write_report(folder):
    folder = Path(folder)
    _, worlds, tasks = load_suite(folder)
    report = json.loads((folder / "generation_report.json").read_text())
    lines = ["# Saved multi-agent benchmark scenarios", "",
             f"{len(tasks)} tasks, {sum(t['robot_count'] for t in tasks)} robot assignments. "
             f"Generation: {report.get('generation_seconds', 0):.2f} s (includes feasibility checks; cached graph reuse recorded separately).", "",
             "All maps retain their original coordinates and bounds. Transform: identity, scale 1, no crop. "
             "Distances are native map units; a meters-per-unit calibration is not provided by the imported data.", "",
             "Difficulty is a design intention. No scenario is called an observed deadlock based on its label or timeout.", "",
             "| Scenario | Source map | Robots (target) | Model | Size classes | Category | Intended difficulty | Transform | Validation | Alternative-route witnesses |",
             "| --- | --- | ---: | --- | --- | --- | --- | --- | --- | ---: |"]
    for t in tasks:
        alternatives = sum(r["geometry_evidence"]["alternative_length"] is not None for r in t["robots"])
        sizes="/".join(c for c in ("small","medium","large") if any(r["size_class"]==c for r in t["robots"]))
        lines.append(f"| [{t['id']}](tasks/{t['id']}.json) | {t['source_map']} | {t['robot_count']} ({t['requested_robot_count']}) | "
                     f"{t['robots'][0]['model']} | {sizes} | {t['category']} | {t['intended_difficulty']} | identity | "
                     f"{'PASS' if t['validation']['passed'] else 'FAIL'} | {alternatives} |")
    lines += ["", "## Size classes", "", "Car dimensions retain length/width = 2.2 and wheelbase/length = 0.65. "
              "Radii derive from usable interior corridor widths, excluding tiny terminal tapers. These are map-scale dimensions, not claimed SI vehicle measurements.", "",
              "| Map | Class | Footprint | Length × width | Planning radius | Wheelbase |", "| --- | --- | --- | --- | ---: | ---: |"]
    for name, info in report["maps"].items():
        for p in info["profiles"]:
            lines.append(f"| {name} | {p['size_class']} | {p['footprint']} | {p['length']:.3f} × {p['width']:.3f} | {p['radius']:.3f} | {p['wheelbase'] if p['wheelbase'] is not None else '—'} |")
    lines += ["", "## Geometry limitations", ""]
    for t in tasks:
        for limitation in t["limitations"]:
            lines.append(f"- **{t['id']}**: {limitation}")
    lines += ["", "The category-08 parking adaptations are explicit: the car-sized configuration spaces have zero holes, "
              "so the suite does not invent alternative topological routes or modify obstacles to create them. "
              "Warehouse category-08 tasks store independently checked geometry-graph alternatives. "
              "These checks never call the proposed K-route planner.", "",
              "## Actual pilot observations", ""]
    pilot_index = folder / "pilots/index.json"
    if pilot_index.exists():
        pilots = json.loads(pilot_index.read_text())["runs"]
        lines += ["Short pilots are separate from task selection; they are not full-budget benchmark rankings.", "",
                  "| Scenario | Planner | Completion | Operational deadlocks | Collisions | Timeout | Control limits valid | Observation (s) |",
                  "| --- | --- | ---: | ---: | ---: | --- | --- | ---: |"]
        for run in pilots:
            limits = run.get("physical_validation",{}).get("passed","not simulated")
            contacts = run.get("collisions") if run.get("collisions") is not None else "not observed"
            lines.append(f"| {run['scenario_id']} | {run['planner']} | {run['completion_fraction']:.2f} | {run.get('deadlock_count',0)} | {contacts} | {run.get('timeout',False)} | {limits} | {run.get('simulated_seconds',0):.1f} |")
        observed = [r["scenario_id"] for r in pilots if r.get("deadlock_count",0) and r.get("control_limits_valid",False)]
        invalid = [r["scenario_id"] for r in pilots if r.get("deadlock_count",0) and not r.get("control_limits_valid",False)]
        lines += ["", "Operational deadlock detected in valid-control runs: " + (", ".join(sorted(set(observed))) if observed else "none of the recorded pilots") + "."]
        if invalid: lines += ["", "Additional detections in control-limit-violating traces (excluded from valid-control outcomes): " + ", ".join(sorted(set(invalid))) + "."]
        lines += ["", "Runs marked with invalid control limits retain their original traces and are flagged separately. "
                  "The current differential-drive controller can reset yaw rate immediately in its emergency fallback, "
                  "exceeding the requested angular-acceleration cap. The benchmark detects this and does not modify the controller or smooth the evidence."]
    else:
        lines.append("No pilot observations have been recorded. Geometry-based labels are not measured outcomes.")
    (folder / "SCENARIOS.md").write_text("\n".join(lines)+"\n")


def catalog_pilots(folder):
    """Publish fixed recorded category examples without changing scenario data."""
    from . import CATEGORIES
    from .generation import save_json
    from .runner import report_saved_runs
    folder=Path(folder)
    runs=report_saved_runs(folder,folder/"pilots",audit=True)
    _,worlds,tasks=load_suite(folder)
    tasks_by_id={t["id"]:t for t in tasks}
    save_json(folder/"pilots/index.json",dict(runs=runs,
        selection_scope="after generation; pilot outcomes never select or reject scenarios",
        generation_manifest_sha256=hashlib.sha256((folder/"manifest.json").read_bytes()).hexdigest(),
        observation_steps=300, profile="pilot", planner_selection_seed=42, controller_seed=42,
        rrt_configuration=dict(iterations=15000,parking_step=.5,warehouse_step=1.5,smoothing_iterations=800),
        k_configuration=dict(k=5,max_candidates=24,max_detour_ratio=1.8,diversity_threshold=.2)))
    target=folder/"animations";target.mkdir(exist_ok=True)
    entries=[]
    for number,(category,_,_) in enumerate(CATEGORIES,1):
        # A parking example illustrates actual heterogeneous car dimensions;
        # the other fixed examples use warehouse differential-drive robots.
        scenario="parking1_09" if number==9 else f"warehouse2_{number:02d}"
        run=next(r for r in runs if r["scenario_id"]==scenario and r["planner"]=="rrt")
        source=folder/"pilots"/run["run_id"]/"animation.gif"
        if not source.exists():
            trace=folder/"pilots"/run["run_id"]/"trajectory.npz"
            if not trace.exists():raise ValueError(f"No recorded trajectory for category {category}")
            with np.load(trace,allow_pickle=False) as archive:states=archive["states"]
            animate_run(tasks_by_id[scenario],worlds[tasks_by_id[scenario]["source_map"]],states,source,result=run)
        filename=f"{number:02d}_{category}.gif"
        shutil.copyfile(source,target/filename)
        entries.append(dict(category=category,scenario=scenario,planner="rrt",run_id=run["run_id"],file=filename,
                            observation_seconds=run["simulated_seconds"],
                            control_limits_valid=run["physical_validation"]["passed"]))
    save_json(target/"index.json",dict(examples=entries,trace_source="recorded existing-simulator states; no animation-only motion"))
    lines=["# Recorded category examples","","These are 30-second pilot observations, not full-budget success claims. "
           "Control-limit violations in the existing differential-drive fallback are reported explicitly.","",
           "| Category | Scenario | Animation | Control limits valid |","| --- | --- | --- | --- |"]
    for e in entries:lines.append(f"| {e['category']} | {e['scenario']} | [GIF]({e['file']}) | {e['control_limits_valid']} |")
    (target/"README.md").write_text("\n".join(lines)+"\n")
    write_report(folder)
    return entries
