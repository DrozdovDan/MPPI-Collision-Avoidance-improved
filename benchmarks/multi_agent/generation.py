"""Deterministic geometry-first task generation, with bounded rejection."""

from collections import Counter
import copy
import gzip
import hashlib
import importlib.metadata
import json
import math
from pathlib import Path
import shutil
import platform
import time

import numpy as np
from shapely.geometry import LineString, Point

from . import CATEGORIES, MAP_NAMES, SCHEMA_VERSION
from .analysis import AnalysisGraph
from .geometry import (ROOT, WorldGeometry, car_witness, diff_drive_witness,
                       fingerprint, profiles_for, validate_task)
from corridor_graph.geometry import polygon_parts
from k_routes import prepare_corridor_map


DEFAULT_CONFIG = dict(seed=20261016, dt=.1, candidate_count=3,
                      max_pair_attempts=320, max_task_attempts=8,
                      car_witness_max_steps=1100, parking_graph_spacing=.25,
                      warehouse_graph_spacing=.7,
                      selection="maximum geometric score among valid candidates; ordinary uses first",
                      pilot_selection=False, k_route_selection=False)


def save_json(path, value):
    Path(path).write_text(json.dumps(value, indent=2, allow_nan=False) + "\n")


def geometric_score(robots, features):
    """A documented design score, not a prediction of observed deadlock."""
    uses = Counter(); directions = {}; crossings = 0; total_length = 0.
    for robot in robots:
        total_length += robot["geometry_evidence"]["shortest_length"]
        for feature_id, direction in robot["geometry_evidence"]["resource_uses"]:
            uses[feature_id] += 1
            directions.setdefault(feature_id, Counter())[direction] += 1
        crossings += len(robot["geometry_evidence"]["junction_uses"])
    n = len(robots)
    concentration = max(uses.values(), default=0) / n
    opposing = min(1., sum(min(v[1], v[-1]) for v in directions.values()) / max(1., n/2))
    by_id = {f["id"]: f for f in features}
    load = min(1., max((count * max(r["radius"] for r in robots) / max(.01, by_id[k]["width"]) / 4
                       for k, count in uses.items() if k in by_id), default=0.))
    density = min(1., sum(math.pi * r["radius"]**2 for r in robots) / max(1., total_length) * 3)
    junction = min(1., crossings / n)
    alternative = sum(r["geometry_evidence"].get("alternative_length") is not None for r in robots) / n
    components = dict(density=density, bottleneck_load=load, opposing_flow=opposing,
                      concentration=concentration, junction_usage=junction, alternative_fraction=alternative)
    weights = dict(density=.15, bottleneck_load=.20, opposing_flow=.25,
                   concentration=.20, junction_usage=.10, alternative_fraction=.10)
    return dict(value=sum(weights[k] * v for k, v in components.items()), components=components,
                weights=weights, interpretation="geometric design score; not a measured outcome")


class Generator:
    def __init__(self, name, world, traffic, config):
        self.name, self.world, self.traffic, self.config = name, world, traffic, config
        self.geometry = WorldGeometry(world)
        self.profiles = profiles_for(name, traffic)
        spacing = config["parking_graph_spacing" if name.startswith("parking") else "warehouse_graph_spacing"]
        self.graphs = [AnalysisGraph(traffic, self.geometry, p, spacing) for p in self.profiles]
        self.diagonal = np.linalg.norm(np.subtract(world["bounds"][2:], world["bounds"][:2]))
        self.rejections = Counter()
        self.witness_cache = {}
        self.feature_order = self._rank_features()
        self.alternative_features = self._alternative_features()
        self.width_accessibility = [f for f in self.feature_order
                                    if f["width"] < (self.profiles[2]["width"] if name.startswith("parking")
                                                     else 2 * self.profiles[2]["radius"])]

    def _rank_features(self):
        graph = self.graphs[0]
        # Favor real narrow resources with usable staging points on both sides.
        ranked = []
        for feature in graph.resource_features:
            delta = graph.points[graph.pool] - feature["center"]
            side = delta @ feature["tangent"]
            if np.count_nonzero(side < -feature["width"]*.5) >= 12 and np.count_nonzero(side > feature["width"]*.5) >= 12:
                ranked.append(feature)
        return sorted(ranked, key=lambda f: (f["width"], f["id"]))

    def _alternative_features(self):
        graph = self.graphs[0]
        # Topological loops are absent from the car-sized parking configuration
        # spaces. Record the limitation instead of inventing another route.
        r = self.profiles[0]["radius"] + self.profiles[0]["safety_margin"]
        center_free = self.geometry.workspace.buffer(-r).difference(self.geometry.obstacles.buffer(r))
        self.configuration_space_holes = sum(len(p.interiors) for p in polygon_parts(center_free))
        if not self.configuration_space_holes:
            return []
        result = []
        for feature in self.feature_order:
            rng = np.random.default_rng(5000 + feature["id"])
            for start, goal in graph.feature_pairs(feature, rng, 12):
                path = graph.shortest(start, goal)
                if graph.features_for_path(path, [feature]) and graph.alternative(path, feature) is not None:
                    result.append(feature)
                    break
            if len(result) >= 6:
                break
        return result

    def _compatible(self, position, radius, occupied, pad=.08):
        return all(np.linalg.norm(position - p) > radius + other + pad for p, other in occupied)

    def _pair(self, graph, rng, task_index, robot_index, feature, junction):
        if junction is not None:
            delta = graph.points[graph.pool] - junction["center"]
            angles = np.arctan2(delta[:, 1], delta[:, 0])
            radius = max(5., self.diagonal*.20)
            band = np.linalg.norm(delta, axis=1) < radius
            first_angle = (robot_index % 4) * math.pi / 2
            first = graph.pool[band & (np.cos(angles-first_angle) > .6)]
            second = graph.pool[band & (np.cos(angles-first_angle-math.pi) > .6)]
            if len(first) and len(second):
                return int(rng.choice(first)), int(rng.choice(second))
        if feature is not None:
            pairs = graph.feature_pairs(feature, rng, 1)
            pair = next(pairs, None)
            if pair is not None:
                return pair if robot_index % 2 == 0 else pair[::-1]
        return tuple(map(int, rng.choice(graph.pool, 2, replace=False)))

    def _witness(self, class_id, start_id, goal_id, path):
        key = (class_id, start_id, goal_id)
        if key in self.witness_cache:
            return self.witness_cache[key]
        robot = self.profiles[class_id]
        if robot["model"] == "car_like":
            witness = car_witness(path, robot, self.geometry, dt=self.config["dt"],
                                  max_steps=self.config["car_witness_max_steps"])
        else:
            # Reuse the existing shortcut utility before constructing the
            # bounded unicycle certificate. This is not the evaluated route.
            simplified = self.geometry.planner(robot).smooth(path)
            witness = diff_drive_witness(simplified, robot, self.config["dt"])
            if self.geometry.witness_clearance(witness[0], robot) < robot["safety_margin"]:
                witness = None
        self.witness_cache[key] = witness
        return witness

    def candidate(self, number, count, seed, candidate_index):
        category, requested, difficulty = CATEGORIES[number-1]
        rng = np.random.default_rng(seed)
        selected_features = []
        if number in (4, 5, 8, 9, 10) and self.feature_order:
            available = self.alternative_features if number == 8 and self.alternative_features else self.feature_order
            if number == 9 and self.width_accessibility:
                available = self.width_accessibility
            # Different candidate seeds explore different physical resources.
            offset = candidate_index % min(6, len(available))
            selected_features = [available[offset]]
            if number in (8, 10) and len(available) > 1:
                selected_features.append(available[(offset+1) % len(available)])
        junction = None
        if number == 6:
            junctions = self.graphs[0].junction_features
            if junctions:
                # Prefer spacious real junctions, not terminal corner artifacts.
                ordered = sorted(junctions, key=lambda f: (-f["width"], f["id"]))
                junction = ordered[candidate_index % min(5, len(ordered))]
        robots, arrays = [], {}
        occupied_starts, occupied_goals = [], []
        attempts = 0
        for index in range(count):
            if number in (1, 2, 3):
                class_id = number-1
            elif number in (9, 10):
                class_id = index % 3
            else:
                class_id = 0
            graph, profile = self.graphs[class_id], self.profiles[class_id]
            feature = selected_features[(index//2) % len(selected_features)] if selected_features else None
            found = False
            for attempt in range(self.config["max_pair_attempts"]):
                attempts += 1
                # Keep an opposing/conflicting core, but let additional dense
                # traffic use other map regions if a narrow staging strip is
                # full. A one-dimensional queue's capacity is not map capacity.
                expanded_traffic = index >= 8 and attempt >= self.config["max_pair_attempts"] // 2
                assignment_feature = None if expanded_traffic else feature
                if number == 7 and index % 2:
                    previous = robots[-1]
                    start_id, goal_id = previous["geometry_evidence"]["end_node"], previous["geometry_evidence"]["start_node"]
                else:
                    start_id, goal_id = self._pair(graph, rng, number, index, assignment_feature, junction)
                start, goal = graph.points[[start_id, goal_id]]
                if not self._compatible(start, profile["radius"], occupied_starts) or not self._compatible(goal, profile["radius"], occupied_goals):
                    self.rejections["endpoint_packing"] += 1
                    continue
                path = graph.shortest(start_id, goal_id)
                if path is None or LineString(path).length < max(4 * profile["radius"], self.diagonal*.06):
                    self.rejections["short_or_disconnected"] += 1
                    continue
                resource_uses = graph.features_for_path(path, self.graphs[0].resource_features)
                if assignment_feature is not None and (class_id == 0 or number not in (9, 10)) and assignment_feature["id"] not in {f[0] for f in resource_uses}:
                    self.rejections["does_not_cross_selected_resource"] += 1
                    continue
                if junction is not None and not graph.features_for_path(path, [junction]):
                    self.rejections["does_not_cross_selected_junction"] += 1
                    continue
                witness = self._witness(class_id, start_id, goal_id, path)
                if witness is None:
                    self.rejections["bounded_feasibility_not_established"] += 1
                    # A failed first direction makes this swap pair unusable.
                    if number == 7 and index % 2:
                        return None
                    continue
                states, controls = witness
                if not self.geometry.planner(profile).point_is_free(states[0, :2]):
                    continue
                # The goal is a fixed region at the geometric endpoint. The
                # car witness must reach it without changing that assignment.
                goal_radius = .30 if profile["model"] == "car_like" else .35
                robot = copy.deepcopy(profile)
                robot.update(id=f"r{index:02d}", start=states[0, :3].tolist(),
                             goal=dict(pose=[float(goal[0]), float(goal[1]), float(states[-1, 2])],
                                       radius=goal_radius, heading_required=False))
                alternative = graph.alternative(path, feature) if number == 8 and feature in self.alternative_features else None
                robot["geometry_evidence"] = dict(start_node=start_id, end_node=goal_id,
                    focused_assignment=assignment_feature is not None or junction is not None,
                    shortest_length=float(LineString(path).length),
                    resource_uses=resource_uses,
                    junction_uses=graph.features_for_path(path, self.graphs[0].junction_features),
                    alternative_length=float(LineString(alternative).length) if alternative is not None else None,
                    reference_path=path.tolist(),
                    alternative_path=alternative.tolist() if alternative is not None else None)
                arrays[robot["id"]+"_states"] = states
                arrays[robot["id"]+"_controls"] = controls
                robots.append(robot)
                occupied_starts.append((start, profile["radius"]))
                occupied_goals.append((goal, profile["radius"]))
                found = True
                break
            if not found:
                return None
        features = {f["id"]: f for f in selected_features}
        if junction is not None:
            features[junction["id"]] = junction
        all_features = {f["id"]: f for f in self.graphs[0].resource_features + self.graphs[0].junction_features}
        actual_ids = {i for r in robots for i, _ in r["geometry_evidence"]["resource_uses"]}
        for feature_id in actual_ids:
            features[feature_id] = all_features[feature_id]
        limitations = []
        if number == 8 and not self.alternative_features:
            limitations.append("No alternative topological corridor route exists in the smallest planning-circle configuration space (zero holes); task 08 uses multiple competing resources without claiming route alternatives.")
        if count != requested:
            limitations.append(f"Requested {requested}; bounded packing/feasibility search established {count}, not a proof of global packing optimality.")
        task = dict(schema_version=SCHEMA_VERSION, id=f"{self.name}_{number:02d}",
                    source_map=self.name, map_sha256=fingerprint(self.world), units="native_map_units",
                    transformation=dict(kind="identity", scale=1., crop=None),
                    generation_seed=seed, category=category, intended_difficulty=difficulty,
                    difficulty_is_measured=False, requested_robot_count=requested, robot_count=count,
                    dt=self.config["dt"], robots=robots,
                    resources=list(features.values()), relevant_resource_ids=sorted(features),
                    geometric_difficulty=geometric_score(robots, list(all_features.values())),
                    generation=dict(config=self.config, candidate_index=candidate_index,
                                    pair_attempts=attempts, controller_or_k_used_for_selection=False),
                    limitations=limitations, pilot_results=None)
        if number == 9 and self.name.startswith("parking"):
            task["size_accessibility"] = [dict(resource_id=f["id"], local_neck_width=f["width"],
                vehicle_minimum_widths={p["size_class"]: p["width"] for p in self.profiles},
                planning_envelope_diameters={p["size_class"]: 2*(p["radius"]+p["safety_margin"]) for p in self.profiles},
                small_witness_robot_ids=[r["id"] for r in robots if r["size_class"] == "small"
                                         and f["id"] in {k for k,_ in r["geometry_evidence"]["resource_uses"]}],
                classification="local aperture width; not a claim that all alternative large-car routes are blocked")
                for f in selected_features]
        try:
            task["validation"] = validate_task(task, self.world, arrays, replay=False)
        except (ValueError, AssertionError):
            self.rejections["final_geometry_validation"] += 1
            return None
        return task, arrays


def generate_suite(folder, *, source=ROOT / "benchmarks/bench_mr_tasks.json", config=None, maps=MAP_NAMES):
    folder = Path(folder); folder.mkdir(parents=True, exist_ok=True)
    (folder / "tasks").mkdir(exist_ok=True); (folder / "witnesses").mkdir(exist_ok=True)
    config = DEFAULT_CONFIG | (config or {})
    source_data = json.loads(Path(source).read_text())
    save_json(folder / "maps.json", dict(units="native_map_units", source=source_data["source"],
              source_revision=source_data["revision"], svg_y_flipped=True, source_scaling=source_data["scaling"],
              meters_per_map_unit=None, worlds=source_data["worlds"]))
    start_time = time.perf_counter()
    report = dict(config=config, config_sha256=fingerprint(config), source_revision=source_data["revision"],
                  source_sha256=fingerprint(source_data), maps={}, selection_audit=[],
                  no_pilot_or_k_route_selection=True)
    manifest = dict(schema_version=SCHEMA_VERSION, scenario_count=0, scenarios=[])
    for map_index, name in enumerate(MAP_NAMES):
        if name not in maps:
            continue
        tick = time.perf_counter()
        world = source_data["worlds"][name]
        # Existing caches are an acceleration only. Fresh checkouts build each
        # robot-independent graph exactly once via the existing adapter.
        cache = ROOT / f"benchmarks/results/multi_agent_cache/{name}.resources.json"
        snapshots = [folder / f"geometry_cache/{name}.resources.json.gz",
                     ROOT / f"benchmarks/scenarios/multi_agent/geometry_cache/{name}.resources.json.gz"]
        snapshot = next((p for p in snapshots if p.exists()), None)
        if snapshot is not None:
            content = gzip.decompress(snapshot.read_bytes())
            cache.parent.mkdir(parents=True, exist_ok=True)
            if not cache.exists() or cache.read_bytes() != content:
                cache.write_bytes(content)
        original = ROOT / f"benchmarks/results/traffic_resources/{name}.resources.json"
        if not cache.exists() and original.exists():
            cache.parent.mkdir(parents=True, exist_ok=True); shutil.copyfile(original, cache)
        try:
            traffic = prepare_corridor_map(world, cache)
            rebuilt_cache = False
        except ValueError:
            # Only our derived cache is regenerated; never alter source maps or
            # an older experiment's saved graph when its fingerprint is stale.
            traffic = prepare_corridor_map(world, cache, rebuild=True)
            rebuilt_cache = True
        generator = Generator(name, world, traffic, config)
        map_info = dict(profiles=generator.profiles, graph_sha256=fingerprint(traffic),
                        rebuilt_stale_cache=rebuilt_cache,
                        graph_cache=str(cache.relative_to(ROOT)),
                        graph_nodes=[len(g.pool) for g in generator.graphs],
                        smallest_configuration_space_holes=generator.configuration_space_holes,
                        alternative_resource_ids=[f["id"] for f in generator.alternative_features])
        report["maps"][name] = map_info
        for number, (category, target, difficulty) in enumerate(CATEGORIES, 1):
            task_start = time.perf_counter()
            candidates = []
            desired_candidates = 1 if difficulty == "ordinary" else config["candidate_count"]
            count = target
            while not candidates and count >= 2:
                for attempt in range(config["max_task_attempts"]):
                    seed = config["seed"] + map_index * 100_000 + number * 1000 + attempt
                    candidate = generator.candidate(number, count, seed, attempt)
                    report["selection_audit"].append(dict(map=name, task=number, candidate_index=attempt,
                        seed=seed, count=count, valid=candidate is not None,
                        geometric_score=candidate[0]["geometric_difficulty"]["value"] if candidate else None))
                    if candidate is not None:
                        candidates.append(candidate)
                        if len(candidates) >= desired_candidates:
                            break
                if not candidates:
                    count -= 2
            if not candidates:
                raise RuntimeError(f"Could not construct valid task {name}_{number:02d}; inspect bounded search configuration")
            task, arrays = max(candidates, key=lambda c: c[0]["geometric_difficulty"]["value"])
            witness_file = f"witnesses/{task['id']}.npz"
            np.savez_compressed(folder / witness_file, **arrays)
            task["witness_file"] = witness_file
            task["validation"] = validate_task(task, world, arrays, replay=True)
            task["generation_seconds"] = time.perf_counter() - task_start
            save_json(folder / f"tasks/{task['id']}.json", task)
            manifest["scenarios"].append(dict(id=task["id"], source_map=name,
                file=f"tasks/{task['id']}.json", robot_count=task["robot_count"], category=category,
                intended_difficulty=difficulty, scenario_sha256=fingerprint({k:v for k,v in task.items() if k != "generation_seconds"})))
            manifest["scenario_count"] = len(manifest["scenarios"])
            save_json(folder / "manifest.json", manifest)
            print(f"{task['id']}: {task['robot_count']}/{target}, {difficulty}, score={task['geometric_difficulty']['value']:.3f}, {task['generation_seconds']:.2f}s", flush=True)
        map_info.update(seconds=time.perf_counter()-tick, rejection_counts=dict(generator.rejections))
        save_json(folder / "generation_report.json", report)
    report["generation_seconds"] = time.perf_counter() - start_time
    report["total_robots"] = sum(s["robot_count"] for s in manifest["scenarios"])
    save_json(folder / "generation_report.json", report)
    write_generation_provenance(folder)
    return manifest, report


def write_generation_provenance(folder):
    """Save exact reusable geometry inputs and all fixed analysis constants."""
    folder = Path(folder)
    report = json.loads((folder / "generation_report.json").read_text())
    target = folder / "geometry_cache"; target.mkdir(exist_ok=True)
    for name, info in report["maps"].items():
        source = ROOT / info["graph_cache"]
        content = source.read_bytes()
        if fingerprint(json.loads(content)) != info["graph_sha256"]:
            raise ValueError("Derived cache differs from the actual generation input")
        (target / f"{name}.resources.json.gz").write_bytes(gzip.compress(content, mtime=0))
        info["bundled_geometry"] = f"geometry_cache/{name}.resources.json.gz"
    report["analysis_parameters"] = dict(endpoint_sampling_seed=1731, endpoint_uniform_draws=7000,
        maximum_attached_endpoints=dict(parking=1200, warehouse=2200), attachment_neighbors=1,
        candidate_selection="ordinary first valid; others maximum score among up to 3 valid candidates in 8 attempts",
        dense_traffic_expansion="After eight focused robots, allow unfocused traffic after half the pair-attempt budget; retain per-robot focused_assignment flags",
        physical_parking_size_factors=[.40, .55, 1.30], warehouse_size_factors=[.18, .32, .55],
        reference_width_filter=[.45, 2.], car_preview_factors=[1.6, 2.5, 1.0],
        car_witness_cruise_cap=.45, car_witness_goal_capture=.18,
        score_comparison="within each map/category only; footprint-load index uses the fixed native map scale")
    report["implementation_sha256"] = {p.name: hashlib.sha256(p.read_bytes()).hexdigest()
        for p in sorted(Path(__file__).parent.glob("*.py")) if p.name in ("generation.py", "geometry.py", "analysis.py")}
    report["reused_source_sha256"] = {name: hashlib.sha256((ROOT/name).read_bytes()).hexdigest()
        for name in ("example/dyn_models.py", "benchmarks/rrt_connect.py", "benchmarks/k_routes.py", "benchmarks/corridor_graph/geometry.py")}
    report["versions"] = dict(python=platform.python_version(),
        **{name: importlib.metadata.version(name) for name in ("numpy", "scipy", "shapely", "pyvoronoi", "Pillow", "matplotlib")})
    save_json(folder / "generation_report.json", report)


def validate_suite(folder):
    from .geometry import load_suite
    manifest, worlds, tasks = load_suite(folder)
    if len(tasks) != 50 or Counter(t["source_map"] for t in tasks) != Counter({m: 10 for m in MAP_NAMES}):
        raise ValueError("Suite must have exactly ten tasks on each of five maps")
    if len({t["id"] for t in tasks}) != 50:
        raise ValueError("Duplicate scenario IDs")
    results = []
    for task, item in zip(tasks, manifest["scenarios"]):
        if fingerprint({k:v for k,v in task.items() if k != "generation_seconds"}) != item["scenario_sha256"]:
            raise ValueError("Scenario fingerprint mismatch")
        with np.load(Path(folder) / task["witness_file"], allow_pickle=False) as witness:
            results.append(dict(id=task["id"], **validate_task(task, worlds[task["source_map"]], witness)))
    report = dict(passed=True, scenario_count=len(tasks), robots_checked=sum(r["checked_robots"] for r in results), results=results)
    save_json(Path(folder) / "validation_report.json", report)
    return report
