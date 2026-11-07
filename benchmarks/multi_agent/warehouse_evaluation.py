"""Run the frozen warehouse suite with paired anchors and measured termination.

No arrival time is imputed for a failed run. Persistent stalls and livelocks
remain distinct from evidence of mutual obstruction.
"""

from collections import Counter, defaultdict, deque
from concurrent.futures import ProcessPoolExecutor, as_completed
from dataclasses import asdict, dataclass
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import time

import numpy as np
import shapely
from shapely.ops import substring
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import connected_components

from .generation import save_json as _write_json
from .geometry import ROOT, agent_parameters, fingerprint
from .metrics import completion_data, intervals, validate_execution
from .runner import controller_profile, export_csv, global_routes, serializable_profiles, _provenance
from .warehouse_preflight import check_paths,check_joint_certificate,anchor_identity


def save_json(path,value):
    """Publish completed results atomically while other workers aggregate them."""
    path=Path(path)
    temporary=path.with_suffix(f".{os.getpid()}.tmp")
    _write_json(temporary,value)
    temporary.replace(path)


@dataclass(frozen=True)
class EvaluationConfig:
    speed_mps: float = 1.2
    batch_size: int = 128
    horizon_states: int = 31
    anchor_yield_after_seconds: float = 15.
    selection_seed: int = 42
    controller_seed: int = 42
    k: int = 5
    deadlock_window_seconds: float = 30.
    stall_window_seconds: float = 45.
    progress_threshold_meters: float = .15
    stationary_speed_mps: float = .03
    obstruction_clearance_meters: float = .04
    persistence_fraction: float = .85
    minimum_duration_seconds: float = 600.
    reference_time_multiplier: float = 6.
    reference_time_allowance_seconds: float = 180.
    wall_limit_seconds: float = 7200.


def load_task(folder, task_id):
    """Load one task, avoiding all 250 large JSONs in every worker."""
    folder = Path(folder)
    manifest = json.loads((folder / "manifest.json").read_text())
    item = next((x for x in manifest["scenarios"] if x["id"] == task_id), None)
    if item is None:
        raise ValueError(f"Unknown task {task_id}")
    task = json.loads((folder / item["file"]).read_text())
    if fingerprint({k:v for k,v in task.items() if k != "generation_seconds"}) != item["scenario_sha256"]:
        raise ValueError("Frozen task fingerprint mismatch")
    worlds = json.loads((folder / "maps.json").read_text())["worlds"]
    if fingerprint(worlds[task["source_map"]]) != task["map_sha256"]:
        raise ValueError("Frozen map fingerprint mismatch")
    return task, worlds[task["source_map"]], item["scenario_sha256"]


class TerminalMonitor:
    """Observe actual states; never modifies a pose, command, or anchor."""

    def __init__(self, task, paths, config):
        self.task, self.cfg = task, config
        self.dt = task["dt"]
        self.lines = np.array(
            [shapely.LineString(path) for path in paths], dtype=object)
        self.goals = np.array([r["goal"]["pose"][:2] for r in task["robots"]])
        self.goal_radii = np.array([r["goal"]["radius"] for r in task["robots"]])
        self.radii = np.array([r["radius"] for r in task["robots"]])
        self.maximum_progress = np.zeros(task["robot_count"])
        self.best_goal_distance = np.linalg.norm(np.array([r["start"][:2] for r in task["robots"]])-self.goals, axis=1)
        self.history = deque(maxlen=int(math.ceil(config.stall_window_seconds/self.dt))+2)
        self.obstruction_events = []
        self.last_evidence = None
        self.last_check_step = 0

    def observe(self, step, states, controls):
        xy = states[:, :2]
        distance = np.linalg.norm(xy-self.goals, axis=1)
        finished = (distance <= self.goal_radii+1e-8) & (np.max(abs(controls), axis=1) <= 1e-8)
        self.maximum_progress = np.maximum(self.maximum_progress,
            shapely.line_locate_point(self.lines, shapely.points(xy)))
        self.best_goal_distance = np.minimum(self.best_goal_distance, distance)
        self.history.append(dict(step=step, xy=xy.copy(), speed=np.linalg.norm(states[:,3:5],axis=1),
            progress=self.maximum_progress.copy(), distance=self.best_goal_distance.copy()))
        if finished.all():
            return "success", dict(finished_robot_count=int(finished.sum()), observed_seconds=step*self.dt)
        # Run the graph test once per simulated second, independently of CPU speed.
        if step-self.last_check_step < max(1,round(1/self.dt)):
            return None, None
        self.last_check_step = step
        window = int(math.ceil(self.cfg.deadlock_window_seconds/self.dt))
        history = list(self.history)
        if len(history) <= window:
            return None, None
        recent = history[-window-1:]
        stagnant = ((self.maximum_progress-recent[0]["progress"] <= self.cfg.progress_threshold_meters) &
                    (recent[0]["distance"]-self.best_goal_distance <= self.cfg.progress_threshold_meters))
        stationary = np.mean([h["speed"] for h in recent[1:]],axis=0) <= self.cfg.stationary_speed_mps
        eligible = stagnant & stationary & ~finished
        positions = np.array([h["xy"] for h in recent[1:]])
        # Use the actual swept route within the planning horizon. A distance
        # cutoff misses stopped queues whose avoidance policies leave a gap;
        # nearby robots beside/behind a route are not necessarily blockers.
        projected = shapely.line_locate_point(self.lines,shapely.points(xy))
        lookahead=self.cfg.speed_mps*(self.cfg.horizon_states-1)*self.dt
        active=np.flatnonzero(eligible)
        obstruction=np.zeros((len(xy),len(xy)),dtype=bool)
        if len(active):
            sections=[]
            for i in active:
                section=substring(self.lines[i],projected[i],projected[i]+lookahead)
                coordinates=list(section.coords)
                if np.linalg.norm(xy[i]-coordinates[0])>1e-10:coordinates.insert(0,xy[i])
                sections.append(shapely.LineString(coordinates) if len(coordinates)>1 else shapely.Point(coordinates[0]))
            distances=shapely.distance(np.array(sections,dtype=object)[None,:,None],
                                      shapely.points(positions)[:,None,:])
            radii=self.radii[active,None]+self.radii[None,:]+self.cfg.obstruction_clearance_meters
            persistent=np.mean(distances<radii[None],axis=0)>=self.cfg.persistence_fraction
            obstruction[active]=persistent & (eligible|finished)[None,:]
        np.fill_diagonal(obstruction,False)
        _, labels = connected_components(csr_matrix(obstruction), directed=True, connection="strong")
        groups = [np.flatnonzero(labels==label).tolist() for label in set(labels)
                  if np.count_nonzero(labels==label)>=2]
        cycle_members = {i for group in groups for i in group}
        # A chain into an observed cycle is also unable to advance.
        parked_blocked = {int(i) for i in np.flatnonzero(eligible)
                          if np.any(obstruction[i] & finished)}
        parked_blockers = {int(j) for i in parked_blocked
                           for j in np.flatnonzero(obstruction[i] & finished)}
        blocked = set(cycle_members) | parked_blocked
        changed = True
        while changed:
            extra = {int(i) for i in np.flatnonzero(eligible) if any(obstruction[i,j] for j in blocked)}
            changed = not extra.issubset(blocked)
            blocked.update(extra)
        evidence = dict(observed_seconds=step*self.dt, evidence_window_seconds=self.cfg.deadlock_window_seconds,
            finished_robot_count=int(finished.sum()), stationary_robot_ids=np.flatnonzero(eligible).tolist(),
            mutual_obstruction_groups=groups, blocked_robot_ids=sorted(blocked),
            parked_goal_blocker_robot_ids=sorted(parked_blockers),
            obstruction_lookahead_meters=lookahead,
            obstruction_edges=np.argwhere(obstruction).tolist(),
            obstruction_definition='persistent swept-route obstruction within the MPPI planning horizon',
            maximum_route_improvement_meters=(self.maximum_progress-recent[0]["progress"]).tolist(),
            best_goal_improvement_meters=(recent[0]["distance"]-self.best_goal_distance).tolist())
        self.last_evidence = evidence
        if groups:
            self.obstruction_events.append(evidence)
        unfinished = {int(i) for i in np.flatnonzero(~finished)}
        if unfinished.issubset(blocked):
            evidence['deadlock_type'] = 'mutual_obstruction_and_parked_goals' if groups and parked_blocked else (
                'parked_goal_obstruction' if parked_blocked else 'mutual_obstruction')
            return "deadlock", evidence
        long_window = int(math.ceil(self.cfg.stall_window_seconds/self.dt))
        if len(history)>long_window:
            old = history[-long_window-1]
            stagnant_long = ((self.maximum_progress-old["progress"] <= self.cfg.progress_threshold_meters) &
                             (old["distance"]-self.best_goal_distance <= self.cfg.progress_threshold_meters))
            if np.all(stagnant_long | finished):
                evidence["evidence_window_seconds"] = self.cfg.stall_window_seconds
                evidence["unfinished_robot_ids"] = sorted(unfinished)
                evidence["unclassified_as_mutual_deadlock_robot_ids"] = sorted(unfinished-blocked)
                return "stalled" if np.all(stationary | finished) else "livelock", evidence
        return None, None


def disk_collisions(task, world, states):
    """Exact relative-segment disk contacts and segment/polygon distances."""
    xy = np.asarray(states)[:,:,:2]
    robots, dt = task["robots"], task["dt"]
    tree = shapely.STRtree([shapely.Polygon(p) for p in world["obstacles"]])
    bounds = np.array(world["bounds"])
    events, minimum = [], float("inf")
    for i, robot in enumerate(robots):
        segments = shapely.linestrings(np.stack([xy[:-1,i],xy[1:,i]],axis=1))
        hit = np.zeros(len(segments),dtype=bool)
        if len(tree.geometries):
            contacts = tree.query(segments,predicate="dwithin",distance=robot["radius"]-1e-9)
            if contacts.size: hit[np.unique(contacts[0])] = True
        r=robot["radius"]
        outside = ((xy[:,i,0]<bounds[0]+r-1e-9)|(xy[:,i,0]>bounds[2]-r+1e-9)|
                   (xy[:,i,1]<bounds[1]+r-1e-9)|(xy[:,i,1]>bounds[3]-r+1e-9))
        hit |= outside[:-1] | outside[1:]
        events.extend(dict(e,kind="static",robots=[robot["id"]]) for e in intervals(hit,dt))
        for j in range(i+1,len(robots)):
            relative=xy[:,i]-xy[:,j];delta=np.diff(relative,axis=0)
            squared=np.sum(delta*delta,axis=1)
            fraction=np.divide(-np.sum(relative[:-1]*delta,axis=1),squared,
                               out=np.zeros(len(delta)),where=squared>0)
            gap=np.linalg.norm(relative[:-1]+np.clip(fraction,0,1)[:,None]*delta,axis=1)-r-robots[j]["radius"]
            minimum=min(minimum,float(gap.min()))
            events.extend(dict(e,kind="robot_robot",robots=[robot["id"],robots[j]["id"]])
                          for e in intervals(gap < -1e-9,dt))
    return dict(collisions=len(events),collision_events=events,
        minimum_inter_robot_clearance=minimum if math.isfinite(minimum) else None)


def run_evaluation(folder, task_id, planner, output, config=None, *,
                   anchor_provider=None, execution_sources=(), protocol_inputs=None):
    """Run the shared controller, optionally with a versioned anchor strategy.

    Extension sources and frozen input hashes enter the evaluation identity;
    the integration, controller profile and stopping logic remain shared.
    """
    cfg=EvaluationConfig(**(config or {}))
    folder,output=Path(folder),Path(output)
    task,world,task_hash=load_task(folder,task_id)
    joint_certificate=check_joint_certificate(folder,task)
    if task["robots"][0]["model"]!="diff_drive" or task["physical_scale"]["meters_per_map_unit"]!=1.:
        raise ValueError("This evaluation requires calibrated differential-drive warehouse tasks")
    # The immutable task stores this exact 1.2 m/s bound already. Changing it
    # requires a new scenario/configuration rather than silently editing JSON.
    if any(r["motion_limits"]["v_max"] != cfg.speed_mps for r in task["robots"]):
        raise ValueError("Configured speed differs from the frozen physical task")
    run_id=f"{task_id}__{planner}__p{cfg.selection_seed}__c{cfg.controller_seed}"
    destination=output/run_id;destination.mkdir(parents=True,exist_ok=True)
    config_data=asdict(cfg)
    sources=(
        Path(__file__),ROOT/"src/controller.cpp",ROOT/"example/utils.py",ROOT/"example/dyn_models.py",
        ROOT/"benchmarks/multi_agent/runner.py",ROOT/"benchmarks/multi_agent/metrics.py",
        ROOT/"example/mppi_safety.py",ROOT/"benchmarks/rrt_connect.py",
        ROOT/"benchmarks/multi_agent/warehouse_preflight.py",
        ROOT/"benchmarks/multi_agent/geometry.py",
        ROOT/"build/release/libcontroller_library.so",ROOT/"build/release/mppi_ca_cpp.so",
        ROOT/"build/routes/corridor_plan")+tuple(Path(p).resolve() for p in execution_sources)
    code_hashes={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sources}
    identity_data=dict(task=task_hash,config=config_data,code=code_hashes,planner=planner)
    if protocol_inputs is not None:identity_data['protocol_inputs']=protocol_inputs
    identity=fingerprint(identity_data)
    if (destination/"run.json").exists():
        saved=json.loads((destination/"run.json").read_text())
        if saved.get("evaluation_identity")==identity:return saved
        raise ValueError(f"Existing run has another evaluation identity: {destination}")
    profiles=controller_profile(task,pilot=True)
    for profile in profiles:
        profile.update(batch_size=cfg.batch_size,time_steps=cfg.horizon_states,
                       anchor_yield_after=cfg.anchor_yield_after_seconds)
    profile_data=serializable_profiles(profiles)
    duration=max(cfg.minimum_duration_seconds,cfg.reference_time_multiplier*max(
        r["geometry_evidence"]["shortest_length"]/cfg.speed_mps for r in task["robots"])+cfg.reference_time_allowance_seconds)
    result=dict(run_id=run_id,task_id=task_id,scenario_id=task["scenario_id"],variant_index=task["variant_index"],
        source_map=task["source_map"],traffic_pattern=task["traffic_pattern"],
        joint_feasibility_certificate=joint_certificate,
        diameter_meters=task["physical_scale"]["diameter_meters"],robot_count=task["robot_count"],
        planner=planner,scenario_sha256=task_hash,evaluation_identity=identity,
        config=config_data,code_sha256=code_hashes,controller_profiles=profile_data,
        controller_profile_sha256=fingerprint(profile_data),provenance=_provenance(),
        maximum_simulated_seconds=duration,success=False,all_robots_success=False,
        makespan_seconds=None,completion_fraction=0.,simulation_executed=False)
    if protocol_inputs is not None:result['protocol_inputs']=protocol_inputs
    save_json(destination/"running.json",result)
    start=time.perf_counter()
    try:
        provider=global_routes if anchor_provider is None else anchor_provider
        paths,planning_seconds,robot_planning=provider(task,world,planner,cfg.selection_seed,
            destination,cfg.k,suite_folder=folder)
        check_paths(task, world, paths)
        anchor_validation=dict(passed=True,identity=anchor_identity(task,planner,cfg.selection_seed,cfg.k),
            task_id=task_id,planner=planner,total_global_planning_seconds=planning_seconds,
            per_robot_global_planning_seconds=robot_planning,
            paths_sha256=hashlib.sha256((destination/'paths.json').read_bytes()).hexdigest(),
            choices_sha256=hashlib.sha256((destination/'route_choices.json').read_bytes()).hexdigest())
        save_json(destination/'anchors_validation.json',anchor_validation)
        result['anchor_validation']=anchor_validation
    except (RuntimeError,ValueError) as error:
        result.update(status="global_planning_failure",error=str(error),collisions=None,
            total_global_planning_seconds=time.perf_counter()-start,simulation_runtime_seconds=0.,
            control_limits_valid=None,termination_evidence=None)
        save_json(destination/"run.json",result)
        (destination/"running.json").unlink(missing_ok=True)
        return result
    sys.path.insert(0,str(ROOT/"example"))
    import dyn_models
    import utils
    from mppi_safety import JointBrakingSafetyFilter
    from bench_mr import boundary_obstacles
    from params_names import EXP_TIME_STEP,XY_GOAL_TOLERANCE,MAX_EXP_STEPS
    n=task["robot_count"]
    states0=np.c_[[r["start"] for r in task["robots"]],np.zeros((n,2))]
    goals=np.c_[[r["goal"]["pose"] for r in task["robots"]],np.zeros((n,2))]
    monitor=TerminalMonitor(task,paths,cfg)
    commands=[];timings=[];terminal=None;evidence=None
    previous_xy=states0[:,:2].copy()
    collision_tree=shapely.STRtree([shapely.Polygon(p) for p in world["obstacles"]])
    bounds=np.array(world["bounds"])
    pair_i,pair_j=np.triu_indices(n,1)
    started=time.perf_counter()
    def observe(step,states,controls,seconds):
        nonlocal terminal,evidence,previous_xy
        commands.append(controls.copy());timings.append(seconds)
        xy=states[:,:2]
        relative0=previous_xy[pair_i]-previous_xy[pair_j]
        relative1=xy[pair_i]-xy[pair_j]
        delta=relative1-relative0;squared=np.sum(delta*delta,axis=1)
        fraction=np.divide(-np.sum(relative0*delta,axis=1),squared,
                           out=np.zeros(len(delta)),where=squared>0)
        gap=np.linalg.norm(relative0+np.clip(fraction,0,1)[:,None]*delta,axis=1)-monitor.radii[pair_i]-monitor.radii[pair_j]
        static=False
        if len(collision_tree.geometries):
            segments=shapely.linestrings(np.stack([previous_xy,xy],axis=1))
            static=bool(collision_tree.query(segments,predicate="dwithin",distance=monitor.radii-1e-9).size)
        outside=bool(np.any((xy[:,0]<bounds[0]+monitor.radii-1e-9)|(xy[:,0]>bounds[2]-monitor.radii+1e-9)|
            (xy[:,1]<bounds[1]+monitor.radii-1e-9)|(xy[:,1]>bounds[3]-monitor.radii+1e-9)))
        previous_xy=xy.copy()
        if np.any(gap < -1e-9) or static or outside:
            terminal="collision";evidence=dict(observed_seconds=step*task["dt"],
                pair_contact=bool(np.any(gap < -1e-9)),static_contact=static,boundary_contact=outside)
            return False
        terminal,evidence=monitor.observe(step,states,controls)
        if step%100==0:
            save_json(destination/"progress.json",dict(simulated_seconds=step*task["dt"],
                wall_seconds=time.perf_counter()-started,
                finished_robots=int(np.sum((np.linalg.norm(states[:,:2]-monitor.goals,axis=1)<=monitor.goal_radii)&
                    (np.max(abs(controls),axis=1)<1e-8)))))
        if time.perf_counter()-started>cfg.wall_limit_seconds:
            terminal="wall_limit";evidence=dict(observed_seconds=step*task["dt"])
        return terminal is None
    safety = JointBrakingSafetyFilter([agent_parameters(r) for r in task["robots"]], profiles,
        world["obstacles"]+boundary_obstacles(world["bounds"]), task["dt"])
    success,envelope,steps,wall,log=utils.launch_mppi(states0,goals,n,
        [agent_parameters(r) for r in task["robots"]],profiles,
        {EXP_TIME_STEP:task["dt"],XY_GOAL_TOLERANCE:task["robots"][0]["goal"]["radius"],
         MAX_EXP_STEPS:math.ceil(duration/task["dt"])},[dyn_models.DiffDriveDyn() for _ in task["robots"]],
        seed=cfg.controller_seed,anchor_paths=paths,
        obstacles=world["obstacles"]+boundary_obstacles(world["bounds"]),park_at_goal=True,on_step=observe,
        safety_filter=safety)
    states=np.asarray(log,dtype=float);controls=np.asarray(commands)
    np.savez_compressed(destination/"trajectory.npz",states=states,controls=controls,
        controller_seconds=np.asarray(timings),dt=task["dt"])
    physical=validate_execution(task,states,controls)
    _,stable,arrivals=completion_data(task,states,controls)
    collision=disk_collisions(task,world,states)
    finished=all(a is not None for a in arrivals)
    valid_success=finished and physical["passed"] and collision["collisions"]==0
    status=("control_limit_violation" if not physical["passed"] else "collision" if collision["collisions"] else
            "success" if valid_success else terminal or "simulation_limit")
    result.update(status=status,terminal_outcome=terminal or ("success" if success else "simulation_limit"),
        success=valid_success,all_robots_success=valid_success,
        makespan_seconds=max(arrivals) if valid_success else None,
        completion_fraction=float(np.mean([a is not None for a in arrivals])),individual_arrival_seconds=arrivals,
        simulated_seconds=steps*task["dt"],simulation_runtime_seconds=wall,
        simulation_executed=True,physical_validation=physical,control_limits_valid=physical["passed"],
        termination_evidence=evidence,mutual_obstruction_observations=monitor.obstruction_events,
        total_global_planning_seconds=planning_seconds,per_robot_global_planning_seconds=robot_planning,
        restricted_completion_time_seconds=steps*task["dt"],**collision)
    result["joint_safety_filter"] = safety.statistics()
    delta=np.diff(np.concatenate([np.zeros((1,n,2)),controls]),axis=0)/task["dt"]
    result.update(peak_speed_mps=float(controls[:,:,0].max()),mean_speed_mps=float(controls[:,:,0].mean()),
        peak_yaw_rate_radps=float(abs(controls[:,:,1]).max()),
        peak_linear_acceleration_mps2=float(delta[:,:,0].max()),
        peak_linear_braking_mps2=float(-delta[:,:,0].min()),
        peak_yaw_acceleration_radps2=float(abs(delta[:,:,1]).max()))
    save_json(destination/"run.json",result)
    (destination/"running.json").unlink(missing_ok=True)
    return result


def aggregate_rows(runs,keys):
    groups=defaultdict(list)
    for run in runs:groups[tuple(run[k] for k in keys)].append(run)
    rows=[]
    for group,items in sorted(groups.items()):
        successful=[r["makespan_seconds"] for r in items if r["success"]]
        completion=[r["completion_fraction"] for r in items]
        rows.append(dict(zip(keys,group),runs=len(items),successes=len(successful),
            fleet_success_rate=len(successful)/len(items),mean_agent_completion_fraction=float(np.mean(completion)),
            successful_makespan_mean_seconds=float(np.mean(successful)) if successful else None,
            successful_makespan_std_seconds=float(np.std(successful,ddof=1)) if len(successful)>1 else None,
            successful_makespan_sample_count=len(successful),
            outcomes=dict(Counter(r["status"] for r in items))))
    return rows


def write_csv(rows,path):
    if not rows:return
    fields=list(dict.fromkeys(k for row in rows for k in row))
    with Path(path).open("w") as stream:
        writer=csv.DictWriter(stream,fields);writer.writeheader()
        writer.writerows({k:json.dumps(v,sort_keys=True) if isinstance(v,(dict,list)) else v for k,v in row.items()} for row in rows)


def summarize_evaluation(output):
    output=Path(output)
    runs=[json.loads(p.read_text()) for p in sorted(output.glob("*/run.json"))]
    views=dict(by_scenario=["source_map","scenario_id","diameter_meters","traffic_pattern","planner"],
        by_map_size=["source_map","diameter_meters","planner"],
        by_map_pattern=["source_map","traffic_pattern","planner"],
        by_map=["source_map","planner"],overall=["planner"])
    tables={name:aggregate_rows(runs,keys) for name,keys in views.items()}
    for name,rows in tables.items():save_json(output/f"{name}.json",rows);write_csv(rows,output/f"{name}.csv")
    brief=[{k:r.get(k) for k in ("run_id","task_id","scenario_id","variant_index","source_map","traffic_pattern",
        "diameter_meters","planner","status","success","completion_fraction","makespan_seconds",
        "simulated_seconds","collisions","control_limits_valid","total_global_planning_seconds","simulation_runtime_seconds")} for r in runs]
    write_csv(brief,output/"runs.csv")
    save_json(output/"summary.json",dict(run_count=len(runs),expected_run_count=500,
        makespan_definition="Simulated time until all 32 agents settle at their assigned goals, on collision-free physically valid successes only; failures are null.",
        success_definition="All 32 agents finish, measured equations and limits pass, zero collisions.",
        tables=tables))
    return runs,tables


def run_suite(folder,output,*,workers=12,config=None,task_ids=None):
    folder,output=Path(folder),Path(output);output.mkdir(parents=True,exist_ok=True)
    cfg=EvaluationConfig(**(config or {}))
    manifest=json.loads((folder/"manifest.json").read_text())
    entries=[x for x in manifest["scenarios"] if task_ids is None or x["id"] in task_ids]
    jobs=[(str(folder),entry["id"],planner,str(output),asdict(cfg))
          for entry in entries for planner in ("rrt","k_routes")]
    save_json(output/"experiment.json",dict(config=asdict(cfg),workers=workers,expected_runs=len(jobs),
        frozen_manifest_sha256=hashlib.sha256((folder/"manifest.json").read_bytes()).hexdigest(),
        operating_speed_source="OMRON LD-250 manual I642, maximum translational speed 1.2 m/s",
        speed_reference_url="https://assets.omron.eu/downloads/manual/en/v3/i642_ld-250_user_s_guide_users_manual_en.pdf",
        deadlock_definition="30 seconds stationary/no-progress with persistent obstruction of the forward route within the 3-second MPPI horizon; all unfinished agents blocked by cycles or robots parked at their goals. Observed controller deadlock under fixed routes and parking policy, not proof of task infeasibility.",
        stalled_definition="45 seconds with no route or best-goal improvement for every unfinished robot, without sufficient mutual-deadlock evidence",
        physics="Original DiffDriveDyn integration and motion bounds; MPPI/ORCA plus synchronous joint braking certificate, no pose changes",
        joint_feasibility_required=True))
    completed=0
    with ProcessPoolExecutor(max_workers=workers) as pool:
        futures={pool.submit(run_evaluation,*job):job for job in jobs}
        for future in as_completed(futures):
            result=future.result();completed+=1
            print(f"[{completed}/{len(jobs)}] {result['run_id']}: {result['status']}, "
                  f"agent completion={result['completion_fraction']:.3f}, makespan={result['makespan_seconds']}",flush=True)
            if completed%10==0:summarize_evaluation(output)
    return summarize_evaluation(output)
