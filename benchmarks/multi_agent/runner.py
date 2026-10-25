"""Global-routing adapters around the existing launch_mppi simulation."""

from concurrent.futures import ProcessPoolExecutor
import csv
import gzip
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time

import numpy as np

from .geometry import ROOT, agent_parameters, fingerprint, load_suite
from .generation import save_json
from .metrics import DeadlockConfig, evaluate_run
from bench_mr import boundary_obstacles, parameters
from k_routes import plan_k_routes, prepare_corridor_map
from rrt_connect import RRTConnectPlanner


def controller_profile(task, *, pilot=False):
    """One documented profile per motion model, shared by both routing methods."""
    car = task["robots"][0]["model"] == "car_like"
    profile = parameters()
    profile.update(anchor_path_weight=3000. if car else 2000.,
                   anchor_heading_weight=1000. if car else 0., control_smoothness_weight=20.,
                   car_like_dyn=car)
    if car:
        profile.update(batch_size=256, time_steps=61,
            control_cov=np.diag([.25, .0225]), control_noise_correlation=.92,
            anchor_acquisition_scale=.02, anchor_guided_fraction=.25,
            jerk_cost_weight=1000., steering_rate_cost_weight=2000.,
            jerk_change_cost_weight=4000., steering_rate_change_cost_weight=10000.,
            dist_to_goal_weight=300., terminal_weight=2000.,
            anchor_forward_weight=80000., reverse_cost_weight=1000.,
            ma_dist_weight=1200., ma_dist_threashhold=4.,
            terminal_lookahead_distance=max(1., 3 * np.median([r["length"] for r in task["robots"]])))
    if pilot:
        profile.update(batch_size=96, time_steps=31 if car else 21)
    profile["dt"] = task["dt"]
    profiles = []
    for robot in task["robots"]:
        current = dict(profile, rad_eps=robot["safety_margin"])
        if car:
            current["dist_between_axles"] = robot["wheelbase"]
        profiles.append(current)
    return profiles


def serializable_profiles(profiles):
    return [{k: v.tolist() if isinstance(v, np.ndarray) else v for k,v in p.items()} for p in profiles]


def global_routes(task, world, planner, selection_seed, folder, k=5, *, suite_folder=None):
    tick = time.perf_counter()
    if planner == "rrt":
        paths, timings = [], []
        for i, robot in enumerate(task["robots"]):
            start = time.perf_counter()
            c = robot["radius"] + robot["safety_margin"]
            bounds = np.asarray(world["bounds"]) + np.array([1, 1, -1, -1]) * c
            query = RRTConnectPlanner(world["obstacles"], bounds, robot["radius"], robot["safety_margin"],
                step_size=.5 if task["source_map"].startswith("parking") else 1.5, max_iterations=15_000,
                smoothing_iterations=800, seed=int(selection_seed+i))
            try:
                paths.append(query.plan(robot["start"][:2], robot["goal"]["pose"][:2]).path)
            except RuntimeError as error:
                raise RuntimeError(f"RRT failed for {robot['id']}: {error}") from error
            timings.append(time.perf_counter()-start)
        choices = dict(planner="rrt", seed=selection_seed, independent_queries=True,
                       extra_traffic_waypoints=False, per_robot_seconds=timings)
    elif planner == "k_routes":
        cache = ROOT / f"benchmarks/results/multi_agent_cache/{task['source_map']}-{task['map_sha256'][:12]}.resources.json"
        snapshot = Path(suite_folder) / f"geometry_cache/{task['source_map']}.resources.json.gz" if suite_folder is not None else None
        if snapshot is not None and snapshot.exists():
            content = gzip.decompress(snapshot.read_bytes())
            cache.parent.mkdir(parents=True, exist_ok=True)
            if not cache.exists() or cache.read_bytes() != content:
                temporary = cache.with_suffix(f".{os.getpid()}.tmp")
                temporary.write_bytes(content)
                temporary.replace(cache)
        prepare_corridor_map(world, cache)
        queries = [dict(name=r["id"], start=r["start"][:2], goal=r["goal"]["pose"][:2],
                        radius=r["radius"], safety_margin=r["safety_margin"], k=k) for r in task["robots"]]
        report = plan_k_routes(ROOT / "build/routes/corridor_plan", cache, queries, folder,
                               max_candidates=24, max_detour_ratio=1.8, diversity_threshold=.2)
        rng = np.random.default_rng(selection_seed)
        paths, choices, timings = [], dict(planner="k_routes", seed=selection_seed, robots=[]), []
        for robot, result in zip(task["robots"], report["queries"]):
            selected = int(rng.integers(len(result["routes"])))
            paths.append(result["routes"][selected]["path"])
            timings.append(float(result["planning_seconds"]))
            choices["robots"].append(dict(id=robot["id"], available=len(result["routes"]),
                selected_index=selected, selection_probability=1 / len(result["routes"])))
        choices["native_map_load_seconds"] = report["map_load_seconds"]
        choices["map_bundle_sha256"] = fingerprint(json.loads(cache.read_text()))
    else:
        raise ValueError(f"Unsupported planner {planner}")
    planning_seconds = time.perf_counter()-tick
    save_json(folder / "route_choices.json", choices)
    save_json(folder / "paths.json", paths)
    return paths, planning_seconds, timings


def _provenance():
    binary = ROOT / "build/release/libcontroller_library.so"
    return dict(git_revision=subprocess.check_output(["git", "-C", str(ROOT), "rev-parse", "HEAD"], text=True).strip(),
                controller_sha256=hashlib.sha256(binary.read_bytes()).hexdigest() if binary.exists() else None,
                numpy_version=np.__version__,
                simulator_sha256=hashlib.sha256((ROOT / "example/utils.py").read_bytes()).hexdigest())


def run_one(folder, scenario_id, planner, output, *, selection_seed=42, controller_seed=42,
            max_steps=None, wall_seconds=120., pilot=False, k=5, deadlock_config=None, animate=False):
    folder, output = Path(folder), Path(output)
    manifest, worlds, tasks = load_suite(folder)
    task = next((t for t in tasks if t["id"] == scenario_id), None)
    if task is None: raise ValueError(f"Unknown scenario {scenario_id}")
    world = worlds[task["source_map"]]
    if fingerprint(world) != task["map_sha256"]: raise ValueError("Map fingerprint mismatch")
    run_id = f"{scenario_id}__{planner}__p{selection_seed}__c{controller_seed}__{'pilot' if pilot else 'default'}"
    destination = output / run_id; destination.mkdir(parents=True, exist_ok=True)
    profiles = controller_profile(task, pilot=pilot)
    profile_data = serializable_profiles(profiles)
    default_seconds = 2 * max(r["geometry_evidence"]["shortest_length"] / r["motion_limits"]["v_max"] for r in task["robots"]) + 60
    step_budget = max_steps if max_steps is not None else int(math.ceil(default_seconds/task["dt"]))
    if step_budget < 1 or wall_seconds <= 0: raise ValueError("Budgets must be positive")
    result = dict(run_id=run_id, scenario_id=scenario_id, source_map=task["source_map"],
                  category=task["category"], intended_difficulty=task["intended_difficulty"],
                  robot_count=task["robot_count"], planner=planner,
                  scenario_sha256=fingerprint({k:v for k,v in task.items() if k != "generation_seconds"}),
                  generation_seed=task["generation_seed"], planner_selection_seed=selection_seed,
                  controller_seed=controller_seed, profile_name="pilot" if pilot else "default",
                  controller_profiles=profile_data, controller_profile_sha256=fingerprint(profile_data),
                  dt=task["dt"], max_steps=step_budget, wall_time_limit_seconds=wall_seconds,
                  provenance=_provenance())
    planning_started = time.perf_counter()
    try:
        paths, planning_seconds, robot_planning_seconds = global_routes(task, world, planner, selection_seed, destination, k, suite_folder=folder)
    except (RuntimeError, ValueError) as error:
        result.update(status="global_planning_failure", error=str(error), completion_fraction=0.,
                      all_robots_success=False, makespan_seconds=None, timeout=False,
                      simulation_executed=False, sum_travel_times=None,
                      total_global_planning_seconds=time.perf_counter()-planning_started,
                      deadlock_count=0, collisions=None, simulation_runtime_seconds=0.)
        save_json(destination / "run.json", result)
        export_csv([result], destination / "run.csv")
        print(f"{run_id}: global_planning_failure: {error}", flush=True)
        return result
    sys.path.insert(0, str(ROOT / "example"))
    import dyn_models
    import utils
    car = task["robots"][0]["model"] == "car_like"
    n = task["robot_count"]
    start_states = np.c_[[r["start"] for r in task["robots"]], np.zeros((n, 3 if car else 2))]
    goal_states = np.c_[[r["goal"]["pose"] for r in task["robots"]], np.zeros((n, 3 if car else 2))]
    models = [dyn_models.CarLikeDyn(r["wheelbase"]) if car else dyn_models.DiffDriveDyn() for r in task["robots"]]
    parameters = [agent_parameters(r) for r in task["robots"]]
    controls, inference_times = [], []
    started = time.perf_counter()
    deadline_hit = False
    def observe(step, states, commands, seconds):
        nonlocal deadline_hit
        controls.append(commands); inference_times.append(seconds)
        deadline_hit = time.perf_counter()-started >= wall_seconds
        return not deadline_hit
    radii = {r["goal"]["radius"] for r in task["robots"]}
    if len(radii) != 1: raise ValueError("Simulator currently requires one goal tolerance per run")
    # Use the existing utility's exact parameter names.
    from params_names import EXP_TIME_STEP, XY_GOAL_TOLERANCE, MAX_EXP_STEPS
    exp_params = {EXP_TIME_STEP: task["dt"], XY_GOAL_TOLERANCE: next(iter(radii)), MAX_EXP_STEPS: step_budget}
    success, envelope_collisions, steps, duration, log = utils.launch_mppi(
        start_states, goal_states, n, parameters, profiles, exp_params, models,
        seed=controller_seed, anchor_paths=paths,
        obstacles=world["obstacles"] + boundary_obstacles(world["bounds"]),
        park_at_goal=True, on_step=observe)
    states = np.asarray(log, dtype=float); controls = np.asarray(controls)
    np.savez_compressed(destination / "trajectory.npz", states=states, controls=controls,
                        controller_seconds=np.asarray(inference_times), dt=task["dt"])
    measured = evaluate_run(task, world, states, controls, paths,
                            timeout=not success and (deadline_hit or steps == step_budget), deadlock_config=deadlock_config)
    result.update(measured, simulation_executed=True, total_global_planning_seconds=planning_seconds,
                  per_robot_global_planning_seconds=robot_planning_seconds,
                  planning_timing_scope="RRT: construction+search+shortcut per robot; K: native query per robot, total additionally includes map load, JSON and verification",
                  simulation_runtime_seconds=duration,
                  local_controller_seconds=np.sum(inference_times, axis=0).tolist(),
                  conservative_envelope_collision_steps=float(envelope_collisions),
                  wall_timeout=bool(deadline_hit and not success),
                  status="control_limit_violation" if not measured["physical_validation"]["passed"] else
                         "collision" if measured["collisions"] else "success" if measured["all_robots_success"] else "timeout" if measured["timeout"] else "unfinished")
    save_json(destination / "run.json", result)
    export_csv([result], destination / "run.csv")
    with (destination / "robots.csv").open("w") as stream:
        writer = csv.writer(stream)
        writer.writerow(["robot", "arrival_seconds", "censored", "path_length", "stopped_seconds", "global_planning_seconds"])
        for i, robot in enumerate(task["robots"]):
            writer.writerow([robot["id"], measured["individual_travel_times"][i],
                measured["individual_travel_times"][i] is None, measured["per_robot_path_lengths"][i],
                measured["event_diagnostics"]["per_robot"][i]["total_stopped_seconds"], robot_planning_seconds[i]])
    if animate:
        from .visualization import animate_run
        animate_run(task, world, states, destination / "animation.gif", result=result)
    print(f"{run_id}: {result['status']}, completion={result['completion_fraction']:.2f}, deadlocks={result['deadlock_count']}, collisions={result['collisions']}", flush=True)
    return result


CSV_FIELDS = ["run_id", "scenario_id", "source_map", "category", "robot_count", "planner",
              "planner_selection_seed", "controller_seed", "profile_name", "controller_profile_sha256",
              "status", "completion_fraction", "all_robots_success", "makespan_seconds", "sum_travel_times",
              "restricted_total_time", "total_stopped_seconds", "no_progress_event_count", "deadlock_count",
              "collisions", "minimum_inter_robot_clearance", "minimum_swept_envelope_clearance",
              "timeout", "wall_timeout", "total_global_planning_seconds", "simulation_runtime_seconds"]
CSV_FIELDS.append("control_limits_valid")


def export_csv(results, filename):
    with Path(filename).open("w") as stream:
        writer = csv.DictWriter(stream, CSV_FIELDS, extrasaction="ignore")
        writer.writeheader(); writer.writerows(results)


def summarize(results, output):
    output = Path(output); output.mkdir(parents=True, exist_ok=True)
    groups = {}
    for run in results:
        key = (run["planner"], run["profile_name"])
        groups.setdefault(key, []).append(run)
    aggregate = []
    for (planner, profile), runs in sorted(groups.items()):
        completed = [r["makespan_seconds"] for r in runs if r.get("all_robots_success")]
        valid = [r for r in runs if r.get("control_limits_valid", False)]
        aggregate.append(dict(planner=planner, profile=profile, runs=len(runs),
            mean_completion_fraction=float(np.mean([r["completion_fraction"] for r in runs])),
            all_robots_success_rate=float(np.mean([r["all_robots_success"] for r in runs])),
            successful_makespan_mean=float(np.mean(completed)) if completed else None,
            successful_makespan_sample_count=len(completed),
            valid_control_run_count=len(valid),
            mean_completion_fraction_valid_controls=float(np.mean([r["completion_fraction"] for r in valid])) if valid else None,
            runs_with_operational_deadlock=sum(r.get("deadlock_count", 0)>0 for r in runs),
            valid_control_runs_with_operational_deadlock=sum(r.get("deadlock_count",0)>0 for r in valid),
            timeouts=sum(r.get("timeout", False) for r in runs),
            runs_with_collisions=sum((r.get("collisions") or 0)>0 for r in runs),
            global_planning_failures=sum(r["status"] == "global_planning_failure" for r in runs),
            control_limit_violation_runs=sum(r["status"] == "control_limit_violation" for r in runs)))
    save_json(output / "results.json", dict(runs=results, aggregate=aggregate))
    export_csv(results, output / "runs.csv")
    save_json(output / "aggregate.json", aggregate)
    return aggregate


def run_batch(folder, tasks, planners, output, selection_seeds, controller_seeds, *, workers=1, **options):
    jobs = [dict(folder=str(folder), scenario_id=t["id"], planner=planner, output=str(output),
                 selection_seed=selection, controller_seed=control, **options)
            for t in tasks for planner in planners for selection in selection_seeds for control in controller_seeds]
    if workers == 1:
        results = [_job(job) for job in jobs]
    else:
        with ProcessPoolExecutor(max_workers=workers) as pool:
            results = list(pool.map(_job, jobs))
    summarize(results, output)
    return results


def report_saved_runs(folder, output, *, audit=False):
    """Aggregate saved runs, optionally recomputing metrics from untouched traces."""
    _, worlds, tasks = load_suite(folder)
    by_id = {t["id"]: t for t in tasks}
    results = []
    for filename in sorted(Path(output).glob("*/run.json")):
        result = json.loads(filename.read_text())
        if audit and result.get("simulation_executed", result["status"] != "global_planning_failure"):
            task = by_id[result["scenario_id"]]
            if result["scenario_sha256"] != fingerprint({k:v for k,v in task.items() if k != "generation_seconds"}):
                raise ValueError(f"Cannot audit {result['run_id']}: scenario changed")
            with np.load(filename.parent / "trajectory.npz", allow_pickle=False) as archive:
                states, controls = archive["states"], archive["controls"]
            paths = json.loads((filename.parent / "paths.json").read_text())
            config = DeadlockConfig(**result.get("event_diagnostics", {}).get("config", {}))
            measured = evaluate_run(task, worlds[task["source_map"]], states, controls, paths,
                                    timeout=result.get("timeout",False), deadlock_config=config)
            result.update(measured, status="control_limit_violation" if not measured["physical_validation"]["passed"] else
                          "collision" if measured["collisions"] else "success" if measured["all_robots_success"] else
                          "timeout" if measured["timeout"] else "unfinished")
            result["audit"] = dict(trace_modified=False,
                metrics_source_sha256=hashlib.sha256((Path(__file__).parent / "metrics.py").read_bytes()).hexdigest())
            save_json(filename,result)
            export_csv([result],filename.parent / "run.csv")
        results.append(result)
    summarize(results,output)
    return results


def _job(job):
    # Separate processes own native random generators and avoid shared-state
    # coupling across concurrent runs. No threads share an MPPI sampler.
    return run_one(**job)
