"""Fixed 32-robot warehouse designs with independent endpoint replicates.

This module only composes the existing geometry-first generator. It never runs
MPPI or either evaluated route planner to select assignments.
"""

from collections import Counter, defaultdict
from concurrent.futures import ProcessPoolExecutor, as_completed
import copy
import gzip
import hashlib
import importlib.metadata
import json
from pathlib import Path
import platform
import time

import numpy as np

from .analysis import AnalysisGraph
from .generation import DEFAULT_CONFIG, Generator, save_json
from .geometry import ROOT, WorldGeometry, fingerprint, load_suite, profiles_for, validate_task


SUITE_TYPE = "warehouse_uniform_v1"
WAREHOUSE_MAPS = ("warehouse", "warehouse2")
DIAMETERS_METERS = (.6, .7, .8, .9, 1.)
# Existing generator category IDs: keep the traffic semantics and feasibility
# checks, while replacing the old varying counts and heterogeneous profiles.
PATTERNS = ((3, "distributed"), (5, "opposing_bottleneck"),
            (6, "intersection"), (7, "symmetric_swap"),
            (8, "competing_bottlenecks"))
ROBOT_COUNT = 32
VARIANT_COUNT = 5
SCENARIOS_PER_MAP = len(DIAMETERS_METERS) * len(PATTERNS)
WAREHOUSE_CONFIG = DEFAULT_CONFIG | dict(seed=20261026, candidate_count=1,
    max_pair_attempts=160, max_task_attempts=64,
    selection="first geometry-valid candidate; no controller or evaluated planner used",
    meters_per_map_unit=1., robot_count=ROBOT_COUNT, variants_per_scenario=VARIANT_COUNT,
    physical_scale_source="explicit benchmark calibration, not a measured property of Bench-MR")
WAREHOUSE_CONFIG.update(certificate_shortcut_iterations=1, certificate_shortcut_seed=1731)
WAREHOUSE_CONFIG.update(feature_staging_diagonal_fraction=.22, feature_staging_minimum_radius=12.)
WAREHOUSE_CONFIG.update(minimum_staging_capacity_per_side=16, staging_packing_seed=1731)
WAREHOUSE_CONFIG.update(feature_connectivity_probes=32, minimum_crossing_probes=8)
WAREHOUSE_CONFIG.update(focused_core_robot_count=8)


def uniform_profile(name, traffic, diameter_meters, meters_per_map_unit=1.):
    if not .6 <= diameter_meters <= 1.:
        raise ValueError("Warehouse agent diameter must be between 0.6 and 1.0 meters")
    if meters_per_map_unit != 1.:
        raise ValueError("This suite uses the explicit 1 map unit = 1 meter calibration")
    profile = copy.deepcopy(profiles_for(name, traffic)[1])
    diameter = diameter_meters / meters_per_map_unit
    profile.update(size_class=f"diameter_{round(diameter_meters*100):03d}cm",
                   radius=diameter/2, length=diameter, width=diameter,
                   diameter_meters=diameter_meters)
    # Size is specified physically here rather than inferred from map necks.
    profile.pop("size_reference_width", None)
    return profile


class UniformWarehouseGenerator(Generator):
    def __init__(self, name, world, traffic, diameter_meters, config):
        if name not in WAREHOUSE_MAPS:
            raise ValueError("Only the two warehouse maps belong to this suite")
        self.name, self.world, self.traffic, self.config = name, world, traffic, config
        self.geometry = WorldGeometry(world)
        profile = uniform_profile(name, traffic, diameter_meters, config["meters_per_map_unit"])
        graph = AnalysisGraph(traffic, self.geometry, profile, config["warehouse_graph_spacing"])
        # The existing generator indexes three profiles for its old categories.
        # All three aliases refer to the same physical size in this suite.
        self.profiles, self.graphs = [profile] * 3, [graph] * 3
        self.diagonal = np.linalg.norm(np.subtract(world["bounds"][2:], world["bounds"][:2]))
        self.rejections = Counter()
        self.witness_cache = {}
        self.staging_pools = {}
        self.staging_capacities = {}
        self.feature_path_checks = {}
        self.feature_order = self._rank_features()
        self.alternative_features = self._alternative_features()
        self.width_accessibility = []

    def _staging_pool(self, graph, feature):
        key = feature["id"]
        if key not in self.staging_pools:
            delta = graph.points[graph.pool] - feature["center"]
            side = delta @ feature["tangent"]
            distance = np.linalg.norm(delta, axis=1)
            reach = max(self.config["feature_staging_minimum_radius"],
                        self.diagonal*self.config["feature_staging_diagonal_fraction"],
                        feature["width"]*6)
            separation = max(feature["width"]*.5, graph.robot["radius"]*2)
            self.staging_pools[key] = (graph.pool[(side < -separation) & (distance < reach)],
                                       graph.pool[(side > separation) & (distance < reach)])
        return self.staging_pools[key]

    def _rank_features(self):
        ranked = super()._rank_features()
        graph = self.graphs[0]
        required = self.config["minimum_staging_capacity_per_side"]
        separation = 2*graph.robot["radius"] + .08
        viable = []
        for feature in ranked:
            capacities = []
            for side_index, pool in enumerate(self._staging_pool(graph, feature)):
                rng = np.random.default_rng([self.config["staging_packing_seed"], feature["id"], side_index])
                picked = []
                for node in rng.permutation(pool):
                    point = graph.points[node]
                    if not picked or np.min(np.linalg.norm(np.asarray(picked)-point, axis=1)) > separation:
                        picked.append(point)
                    if len(picked) >= required:
                        break
                capacities.append(len(picked))
            self.staging_capacities[feature["id"]] = capacities
            if min(capacities) < required:
                continue
            # Local aperture width alone does not guarantee that both ends of
            # this resource remain connected for this radius. Verify actual
            # geometry-reference traversals in the radius-filtered graph.
            first, second = self._staging_pool(graph, feature)
            rng = np.random.default_rng([self.config["staging_packing_seed"], feature["id"]])
            hits = 0
            for _ in range(self.config["feature_connectivity_probes"]):
                path = graph.shortest(int(rng.choice(first)), int(rng.choice(second)))
                hits += bool(graph.features_for_path(path, [feature]))
            self.feature_path_checks[feature["id"]] = hits
            if hits >= self.config["minimum_crossing_probes"]:
                viable.append(feature)
        if not viable:
            raise RuntimeError("No corridor has verified staging capacity for the fixed fleet")
        return viable

    def _pair(self, graph, rng, task_index, robot_index, feature, junction):
        if robot_index >= self.config.get('focused_core_robot_count', ROBOT_COUNT):
            feature = None
            junction = None
        if feature is None or junction is not None:
            return super()._pair(graph, rng, task_index, robot_index, feature, junction)
        # The old small-fleet generator stages immediately beside a neck. At
        # realistic diameters those strips cannot hold a 32-agent assignment.
        # Use real connected rooms farther along the two approaches instead.
        first, second = self._staging_pool(graph, feature)
        if not len(first) or not len(second):
            return super()._pair(graph, rng, task_index, robot_index, feature, junction)
        pair = (int(rng.choice(first)), int(rng.choice(second)))
        return pair if robot_index % 2 == 0 else pair[::-1]

    def _witness(self, class_id, start_id, goal_id, path):
        # Reuse the existing visibility-shortcut utility for the individual
        # certificate only. Avoid stopping at every densely sampled graph node.
        # This never supplies a route to the evaluated RRT/K-route controllers.
        planner = self.geometry.planner(self.profiles[class_id])
        planner.smoothing_iterations = self.config["certificate_shortcut_iterations"]
        planner.rng = np.random.default_rng([self.config["certificate_shortcut_seed"], start_id, goal_id])
        return super()._witness(class_id, start_id, goal_id, path)


def scenario_definitions():
    result = []
    for name in WAREHOUSE_MAPS:
        for size_index, diameter in enumerate(DIAMETERS_METERS):
            for pattern_index, (category_number, pattern) in enumerate(PATTERNS):
                number = size_index * len(PATTERNS) + pattern_index + 1
                result.append(dict(id=f"{name}_s{number:02d}", source_map=name,
                    scenario_number=number, diameter_meters=diameter, pattern=pattern,
                    generator_category=category_number, focus_index=size_index,
                    robot_count=ROBOT_COUNT, variants=VARIANT_COUNT))
    return result


def assignment_fingerprint(task):
    return fingerprint([dict(start=r["start"], goal=r["goal"]) for r in task["robots"]])


def construct_variant(generator, design, variant_index, map_index, config, previous_variants=()):
    """Change endpoint seeds, keeping size, traffic pattern and focus fixed."""
    base_seed = (config["seed"] + map_index * 10_000_000
                 + design["scenario_number"] * 100_000 + variant_index * 1000)
    audit = []
    for attempt in range(config["max_task_attempts"]):
        seed = base_seed + attempt
        candidate = generator.candidate(design["generator_category"], ROBOT_COUNT,
                                        seed, design["focus_index"])
        audit.append(dict(attempt=attempt, seed=seed, valid=candidate is not None))
        if candidate is None:
            continue
        task, arrays = candidate
        if any(any(np.array_equal(robot["start"][:2], prior["robots"][i]["start"][:2])
                   or np.array_equal(robot["goal"]["pose"][:2], prior["robots"][i]["goal"]["pose"][:2])
                   for i, robot in enumerate(task["robots"])) for prior in previous_variants):
            audit[-1].update(valid=False, reason="repeated per-agent endpoint across variants")
            continue
        task.update(id=f"{design['id']}_v{variant_index:02d}", suite_type=SUITE_TYPE,
                    scenario_id=design["id"], scenario_number=design["scenario_number"],
                    variant_index=variant_index, variant_seed=base_seed,
                    traffic_pattern=design["pattern"], requested_robot_count=ROBOT_COUNT,
                    physical_scale=dict(meters_per_map_unit=config["meters_per_map_unit"],
                        source=config["physical_scale_source"],
                        diameter_meters=design["diameter_meters"]),
                    homogeneous_agents=True)
        # The old category had a different target. No count has been reduced in
        # this suite: failure to pack all 32 is an error, never a smaller task.
        task["limitations"] = [text for text in task["limitations"] if not text.startswith("Requested ")]
        task["generation"].update(attempt=attempt, variant_seed=base_seed,
            fixed_focus_index=design["focus_index"], selection="first geometry-valid candidate")
        task["assignment_sha256"] = assignment_fingerprint(task)
        return task, arrays, audit
    raise RuntimeError(f"Cannot construct all 32 agents for {design['id']} variant {variant_index}; "
                       "no reduced-count task was saved")


def _traffic_snapshot(folder, name):
    candidates = [folder / f"geometry_cache/{name}.resources.json.gz",
                  ROOT / f"benchmarks/scenarios/multi_agent/geometry_cache/{name}.resources.json.gz"]
    snapshot = next((path for path in candidates if path.exists()), None)
    if snapshot is None:
        raise FileNotFoundError(f"Missing reproducible geometry snapshot for {name}")
    content = gzip.decompress(snapshot.read_bytes())
    target = folder / f"geometry_cache/{name}.resources.json.gz"
    target.parent.mkdir(exist_ok=True)
    target.write_bytes(gzip.compress(content, mtime=0))
    return json.loads(content)


def _generate_size_group(folder, name, world, traffic, diameter, config, map_index, designs):
    """Independent physical-size group; worker writes only its unique task files."""
    folder = Path(folder)
    generator = UniformWarehouseGenerator(name, world, traffic, diameter, config)
    graph = generator.graphs[0]
    size_info = dict(profile=generator.profiles[0], graph_nodes=len(graph.pool),
        resource_count=len(generator.feature_order), junction_count=len(graph.junction_features),
        configuration_space_holes=generator.configuration_space_holes,
        alternative_resource_ids=[f["id"] for f in generator.alternative_features])
    size_info["staging_capacity_checks"] = generator.staging_capacities
    size_info["feature_connectivity_checks"] = generator.feature_path_checks
    entries, audits = [], []
    for design in designs:
        assignments = set()
        previous_variants = []
        for variant in range(1, VARIANT_COUNT+1):
            tick = time.perf_counter()
            task, arrays, audit = construct_variant(generator, design, variant, map_index, config,
                                                    previous_variants)
            if task["assignment_sha256"] in assignments:
                raise ValueError("Duplicate start-goal variant")
            assignments.add(task["assignment_sha256"])
            previous_variants.append(task)
            task["witness_file"] = f"witnesses/{task['id']}.npz"
            np.savez_compressed(folder / task["witness_file"], **arrays)
            task["validation"] = validate_task(task, world, arrays)
            task["generation_seconds"] = time.perf_counter()-tick
            save_json(folder / f"tasks/{task['id']}.json", task)
            entries.append(dict(id=task["id"], source_map=name,
                scenario_id=design["id"], variant_index=variant,
                file=f"tasks/{task['id']}.json", robot_count=ROBOT_COUNT,
                diameter_meters=diameter, traffic_pattern=design["pattern"],
                category=task["category"], intended_difficulty=task["intended_difficulty"],
                scenario_sha256=fingerprint({k:v for k,v in task.items() if k != "generation_seconds"})))
            audits.append(dict(task=task["id"], attempts=audit))
            print(f"{task['id']}: 32 agents, diameter {diameter:.1f} m, "
                  f"{design['pattern']}, variant {variant}/5, "
                  f"{task['generation_seconds']:.2f}s", flush=True)
    size_info["rejection_counts"] = dict(generator.rejections)
    return name, diameter, size_info, entries, audits


def generate_warehouse_suite(folder, *, seed=20261026, workers=4):
    if workers < 1:
        raise ValueError("workers must be positive")
    folder = Path(folder)
    folder.mkdir(parents=True, exist_ok=True)
    for subdir in ("tasks", "witnesses", "geometry_cache"):
        (folder / subdir).mkdir(exist_ok=True)
    config = WAREHOUSE_CONFIG | dict(seed=seed)
    source = json.loads((ROOT / "benchmarks/bench_mr_tasks.json").read_text())
    worlds = {name: source["worlds"][name] for name in WAREHOUSE_MAPS}
    save_json(folder / "maps.json", dict(units="native_map_units", worlds=worlds,
        source=source["source"], source_revision=source["revision"],
        source_scaling=source["scaling"], svg_y_flipped=True, meters_per_map_unit=1.,
        physical_scale_source=config["physical_scale_source"]))
    definitions = scenario_definitions()
    save_json(folder / "scenario_definitions.json", definitions)
    manifest = dict(schema_version=1, suite_type=SUITE_TYPE, scenario_count=0,
        task_count=0, scenario_design_count=len(definitions), scenarios_per_map=SCENARIOS_PER_MAP,
        variants_per_scenario=VARIANT_COUNT, robot_count_per_task=ROBOT_COUNT,
        expected_task_count=len(definitions)*VARIANT_COUNT, scenarios=[])
    report = dict(config=config, config_sha256=fingerprint(config), source_sha256=fingerprint(source),
        maps={}, selection_audit=[], no_pilot_or_k_route_selection=True, execution_workers=workers,
        maximum_agent_count_claim="fixed 32 per task; no claim of a mathematical packing maximum",
        implementation_sha256={path.name: hashlib.sha256(path.read_bytes()).hexdigest()
            for path in [Path(__file__), Path(__file__).with_name("generation.py"),
                         Path(__file__).with_name("analysis.py"), Path(__file__).with_name("geometry.py")]},
        versions=dict(python=platform.python_version(),
            **{name: importlib.metadata.version(name) for name in ("numpy", "scipy", "shapely")}))
    start_time = time.perf_counter()
    jobs = []
    for map_index, name in enumerate(WAREHOUSE_MAPS):
        traffic = _traffic_snapshot(folder, name)
        map_info = dict(graph_sha256=fingerprint(traffic),
            bundled_geometry=f"geometry_cache/{name}.resources.json.gz", sizes={})
        report["maps"][name] = map_info
        for diameter in DIAMETERS_METERS:
            designs = [d for d in definitions if d["source_map"] == name and d["diameter_meters"] == diameter]
            jobs.append((str(folder), name, worlds[name], traffic, diameter, config, map_index, designs))
    save_json(folder / "manifest.json", manifest)
    with ProcessPoolExecutor(max_workers=workers) as pool:
        futures = [pool.submit(_generate_size_group, *job) for job in jobs]
        for future in as_completed(futures):
            name, diameter, size_info, entries, audits = future.result()
            report["maps"][name]["sizes"][str(diameter)] = size_info
            manifest["scenarios"].extend(entries)
            manifest["scenarios"].sort(key=lambda entry: (WAREHOUSE_MAPS.index(entry["source_map"]), entry["id"]))
            manifest["scenario_count"] = manifest["task_count"] = len(manifest["scenarios"])
            report["selection_audit"].extend(audits)
            report["selection_audit"].sort(key=lambda item: item["task"])
            save_json(folder / "manifest.json", manifest)
            save_json(folder / "generation_report.json", report)
    report.update(generation_seconds=time.perf_counter()-start_time,
                  total_robots=len(manifest["scenarios"])*ROBOT_COUNT)
    save_json(folder / "generation_report.json", report)
    return manifest, report


def validate_warehouse_suite(folder):
    folder = Path(folder)
    manifest, worlds, tasks = load_suite(folder)
    expected = len(WAREHOUSE_MAPS)*SCENARIOS_PER_MAP*VARIANT_COUNT
    if manifest.get("suite_type") != SUITE_TYPE or len(tasks) != expected:
        raise ValueError(f"Expected exactly {expected} uniform warehouse tasks")
    if len({t["id"] for t in tasks}) != expected or set(worlds) != set(WAREHOUSE_MAPS):
        raise ValueError("Duplicate task IDs or incorrect source maps")
    maps_metadata = json.loads((folder / "maps.json").read_text())
    if maps_metadata.get("meters_per_map_unit") != 1.:
        raise ValueError("Missing explicit physical calibration")
    definitions = scenario_definitions()
    if json.loads((folder / "scenario_definitions.json").read_text()) != definitions:
        raise ValueError("Scenario definitions differ from the declared design")
    groups = defaultdict(list)
    results = []
    for task, item in zip(tasks, manifest["scenarios"]):
        if fingerprint({k:v for k,v in task.items() if k != "generation_seconds"}) != item["scenario_sha256"]:
            raise ValueError("Scenario fingerprint mismatch")
        if task["robot_count"] != ROBOT_COUNT or task["requested_robot_count"] != ROBOT_COUNT:
            raise ValueError("Every task must contain exactly 32 agents")
        if task["physical_scale"]["meters_per_map_unit"] != 1. or not task["homogeneous_agents"]:
            raise ValueError("Incorrect calibration or heterogeneous task")
        profiles = [{k:v for k,v in r.items() if k not in ("id", "start", "goal", "geometry_evidence")}
                    for r in task["robots"]]
        if len({fingerprint(p) for p in profiles}) != 1:
            raise ValueError("Agents within a task have different physical profiles")
        diameter = task["physical_scale"]["diameter_meters"]
        if diameter not in DIAMETERS_METERS or any(not np.isclose(2*r["radius"], diameter)
                                                  or r["diameter_meters"] != diameter for r in task["robots"]):
            raise ValueError("Incorrect physical diameter")
        if assignment_fingerprint(task) != task["assignment_sha256"]:
            raise ValueError("Assignment fingerprint mismatch")
        with np.load(folder / task["witness_file"], allow_pickle=False) as witness:
            results.append(dict(id=task["id"], **validate_task(task, worlds[task["source_map"]], witness)))
        groups[task["scenario_id"]].append(task)
    if set(groups) != {d["id"] for d in definitions}:
        raise ValueError("Missing scenario designs")
    for design in definitions:
        group = groups[design["id"]]
        if len(group) != VARIANT_COUNT or {t["variant_index"] for t in group} != set(range(1,6)):
            raise ValueError("Every scenario needs five variants")
        if len({t["assignment_sha256"] for t in group}) != VARIANT_COUNT:
            raise ValueError("Variants repeat a start-goal assignment")
        for task in group:
            if (task["source_map"], task["traffic_pattern"], task["physical_scale"]["diameter_meters"]) != (
                    design["source_map"], design["pattern"], design["diameter_meters"]):
                raise ValueError("Variant changed its scenario design")
        for robot_index in range(ROBOT_COUNT):
            starts = {tuple(t["robots"][robot_index]["start"][:2]) for t in group}
            goals = {tuple(t["robots"][robot_index]["goal"]["pose"][:2]) for t in group}
            if len(starts) != VARIANT_COUNT or len(goals) != VARIANT_COUNT:
                raise ValueError("Each agent needs five different starts and five different goals")
    report = dict(passed=True, scenario_design_count=len(groups), task_count=len(tasks),
        tasks_per_map=dict(Counter(t["source_map"] for t in tasks)),
        robots_checked=sum(r["checked_robots"] for r in results),
        all_agents_per_task_identical=True, minimum_diameter_meters=min(DIAMETERS_METERS),
        maximum_diameter_meters=max(DIAMETERS_METERS), results=results)
    save_json(folder / "validation_report.json", report)
    return report


def visualize_warehouse_suite(folder):
    from .visualization import contact_sheet, overview
    folder = Path(folder)
    _, worlds, tasks = load_suite(folder)
    output = folder / "visualizations"; output.mkdir(exist_ok=True)
    groups = defaultdict(list)
    first_variants = defaultdict(list)
    for task in tasks:
        filename = output / f"{task['id']}.png"
        overview(task, worlds[task["source_map"]], filename)
        label = f"{task['id']} | {task['physical_scale']['diameter_meters']:.1f} m | N=32"
        groups[task["scenario_id"]].append((label, filename))
        if task["variant_index"] == 1:
            first_variants[task["source_map"]].append((label, filename))
        if task["variant_index"] == 5:
            print(f"Rendered {task['scenario_id']}: all five variants", flush=True)
    for name, images in groups.items():
        contact_sheet(images, output / f"{name}.variants.png", columns=5, tile=(560, 420))
    for name, images in first_variants.items():
        contact_sheet(images, output / f"{name}.25_scenarios.png", columns=5, tile=(480, 360))
    write_warehouse_report(folder)
    return output


def write_warehouse_report(folder):
    folder = Path(folder)
    _, worlds, tasks = load_suite(folder)
    report = json.loads((folder / "generation_report.json").read_text())
    lines = ["# Uniform-agent warehouse suite", "",
        "**250 tasks: 2 maps × 25 scenario designs × 5 start–goal variants.** "
        "Every task contains exactly 32 identical differential-drive disk agents "
        "(8,000 individually validated assignments).", "",
        "Each map has all combinations of five diameters (0.6, 0.7, 0.8, 0.9, 1.0 m) "
        "and five patterns: distributed traffic, opposing bottleneck traffic, intersection "
        "crossings, symmetric swaps, and competing bottlenecks. A design keeps its diameter, "
        "pattern and geometric focus across all five variants. Every robot receives five "
        "different starts and five different goals.", "",
        "The imported maps have no measured SI calibration. This suite explicitly adopts "
        "**1 native map unit = 1 meter** as a benchmark convention; it does not claim the "
        "source measured that scale. Geometry and bounds retain the identity transform. "
        "The 32-agent count is fixed for every task, not a claim of a packing maximum.", "",
        "All start and goal footprints are disjoint and clear of walls. Every assignment "
        "has a replayed bounded-speed, bounded-acceleration unicycle witness with swept-disk "
        "clearance. These are individual feasibility certificates, not joint schedules or "
        "MPPI trajectories. No controller or evaluated K-route/RRT outcome selected a task. "
        "The simulation uses its original unicycle equations.", "",
        f"Generation and individual feasibility checks: {report['generation_seconds']:.2f} s.", "",
        "| Map | Width × height under calibration | Designs | Variants/design | Tasks | Agents/task |",
        "| --- | --- | ---: | ---: | ---: | ---: |"]
    for name in WAREHOUSE_MAPS:
        x0,y0,x1,y1 = worlds[name]["bounds"]
        lines.append(f"| {name} | {x1-x0:.2f} × {y1-y0:.2f} m | 25 | 5 | 125 | 32 |")
    lines += ["", "## All 50 scenario designs", "",
        "The preview for each design shows all five variants; the overview sheets show "
        "variant 1 of all 25 designs on each map.", "",
        "- [warehouse: 25 designs](visualizations/warehouse.25_scenarios.png)",
        "- [warehouse2: 25 designs](visualizations/warehouse2.25_scenarios.png)", "",
        "| Design | Diameter | Pattern | Tasks | Five-variant preview |",
        "| --- | ---: | --- | ---: | --- |"]
    for design in scenario_definitions():
        name = design["id"]
        lines.append(f"| {name} | {design['diameter_meters']:.1f} m | {design['pattern']} | 5 | "
                     f"[preview](visualizations/{name}.variants.png) |")
    lines += ["", "## All 250 tasks", "",
        "| Task | Design | Variant | Agents | Diameter | Validation | Assignment |",
        "| --- | --- | ---: | ---: | ---: | --- | --- |"]
    for task in tasks:
        lines.append(f"| [{task['id']}](visualizations/{task['id']}.png) | {task['scenario_id']} | "
            f"{task['variant_index']} | 32 | {task['physical_scale']['diameter_meters']:.1f} m | "
            f"passed | [JSON](tasks/{task['id']}.json) |")
    if (folder/'joint_validation_report.json').exists():
        joint=json.loads((folder/'joint_validation_report.json').read_text())
        if joint['passed'] and joint['tasks']==len(tasks):
            lines += ["", "All 250 tasks additionally have independently replayed constructive "
                "joint schedules: [joint validation](joint_validation_report.json). These "
                "certificates check bounded controls and every stationary robot, and never "
                "supply evaluated anchors or controller fallbacks. Six endpoint variants "
                "were repaired using geometry and joint certification before the final "
                "comparison; [repair history](revisions/joint_feasibility_repairs.json) "
                "preserves their originals and replacement hashes.", ""]
    lines += ["", "No rollout success or deadlock rate is claimed by generation. The paired "
        "`evaluate-warehouse` command compares routing methods with a shared MPPI/ORCA "
        "profile and joint braking-safety layer. Native fallback acceleration bounds "
        "are corrected, and the measured execution audit remains active.", ""]
    (folder / "SCENARIOS.md").write_text("\n".join(lines))
