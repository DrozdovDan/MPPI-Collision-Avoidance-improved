"""Tables and exported figures from actual paired warehouse executions."""

from collections import Counter
from concurrent.futures import ProcessPoolExecutor
import hashlib
import json
import math
import os
from pathlib import Path
import re
import shutil

import numpy as np
import shapely
from shapely.ops import substring

from .geometry import ROOT, fingerprint
from .metrics import completion_data,validate_execution
from .warehouse_preflight import check_paths,check_joint_certificate,anchor_identity
from .runner import controller_profile,serializable_profiles
from .warehouse_evaluation import summarize_evaluation,save_json,write_csv,load_task,disk_collisions,TerminalMonitor,EvaluationConfig


def audit_deadlock_geometry(task,world,paths,states,evidence,config):
    """Corroborate recorded blocking edges using footprint-clear route joins.

    A geometric nearest point can lie behind shelving. In that case retain the
    recorded label only if the same edges persist on the closest visible join.
    This checks saved motion; it does not change controls, anchors or outcomes.
    """
    obstacles=shapely.union_all([shapely.Polygon(p) for p in world['obstacles']])
    radii=np.array([r['radius'] for r in task['robots']])
    xy=states[-1,:,:2]
    window=round(config.deadlock_window_seconds/task['dt'])
    recent=states[-window:,:,:2]
    if len(recent)!=window:raise ValueError('Insufficient deadlock observation history')
    corrections=[]
    for i in evidence['blocked_robot_ids']:
        path=np.asarray(paths[i]);line=shapely.LineString(path)
        station=line.project(shapely.Point(xy[i]));point=np.asarray(line.interpolate(station).coords)[0]
        join=shapely.LineString([xy[i],point]);margin=task['robots'][i]['safety_margin']
        original_clearance=join.distance(obstacles)-radii[i]
        if original_clearance>=margin-1e-6:continue
        delta=np.diff(path,axis=0);squared=np.sum(delta*delta,axis=1)
        t=np.clip(np.divide(np.sum((xy[i]-path[:-1])*delta,axis=1),squared,
            out=np.zeros(len(delta)),where=squared>0),0.,1.)
        points=path[:-1]+t[:,None]*delta
        joins=shapely.linestrings(np.stack([np.tile(xy[i],(len(points),1)),points],axis=1))
        clearances=shapely.distance(joins,obstacles)-radii[i]
        visible=np.flatnonzero((squared>1e-20)&(clearances>=margin-1e-9))
        if not len(visible):raise ValueError(f'No visible route join for blocked robot {i}')
        lengths=np.r_[0.,np.cumsum(np.sqrt(squared))]
        stations=lengths[:-1]+t*np.sqrt(squared)
        distances=np.sum((points-xy[i])**2,axis=1)
        j=visible[np.lexsort((stations[visible],distances[visible]))[0]]
        station=float(stations[j])
        section=substring(line,station,station+config.speed_mps*(config.horizon_states-1)*task['dt'])
        coordinates=list(section.coords)
        if np.linalg.norm(xy[i]-coordinates[0])>1e-10:coordinates.insert(0,xy[i])
        section=shapely.LineString(coordinates) if len(coordinates)>1 else shapely.Point(coordinates[0])
        ratios={}
        for source,target in evidence['obstruction_edges']:
            if source!=i:continue
            distances=shapely.distance(section,shapely.points(recent[:,target]))
            ratio=float(np.mean(distances<radii[i]+radii[target]+config.obstruction_clearance_meters))
            if ratio<config.persistence_fraction:
                raise ValueError(f'Blocked edge {i}->{target} is not corroborated on the visible route')
            ratios[str(target)]=ratio
        if not ratios:raise ValueError(f'Blocked robot {i} has no corroborating outgoing edge')
        corrections.append(dict(robot_index=i,original_join_clearance_meters=float(original_clearance),
            visible_join_clearance_meters=float(clearances[j]),visible_anchor_station_meters=station,
            visible_projection=points[j].tolist(),preserved_obstruction_edges=ratios))
    return dict(passed=True,checked_blocked_robots=len(evidence['blocked_robot_ids']),
        visible_projection_corroborations=corrections)


def _audit_run(arguments):
    filename,suite=map(Path,arguments)
    run=json.loads(filename.read_text());directory=filename.parent
    task,world,task_hash=load_task(suite,run['task_id'])
    if task_hash!=run['scenario_sha256']:raise ValueError('Run used another task')
    expected=fingerprint(dict(task=task_hash,config=run['config'],code=run['code_sha256'],planner=run['planner']))
    if expected!=run['evaluation_identity']:raise ValueError('Invalid evaluation identity')
    profiles=controller_profile(task,pilot=True)
    for profile in profiles:
        profile.update(batch_size=run['config']['batch_size'],time_steps=run['config']['horizon_states'],
                       anchor_yield_after=run['config']['anchor_yield_after_seconds'])
    profile_hash=fingerprint(serializable_profiles(profiles))
    if profile_hash!=run['controller_profile_sha256'] or fingerprint(run['controller_profiles'])!=profile_hash:
        raise ValueError('Recorded controller settings differ from the shared profile')
    if check_joint_certificate(suite,task)!=run['joint_feasibility_certificate']:
        raise ValueError('Joint certificate changed')
    paths=json.loads((directory/'paths.json').read_text());check_paths(task,world,paths)
    anchors=run['anchor_validation']
    if anchors['identity']!=anchor_identity(task,run['planner'],run['config']['selection_seed'],run['config']['k']):
        raise ValueError('Anchor construction protocol changed')
    for name,key in [('paths.json','paths_sha256'),('route_choices.json','choices_sha256')]:
        if hashlib.sha256((directory/name).read_bytes()).hexdigest()!=anchors[key]:
            raise ValueError('Saved anchors changed')
    if run['planner']=='k_routes':
        choices=json.loads((directory/'route_choices.json').read_text())
        rng=np.random.default_rng(run['config']['selection_seed'])
        if len(choices['robots'])!=32:raise ValueError('Incomplete uniform choices')
        for choice,robot in zip(choices['robots'],task['robots']):
            count=choice['available']
            if (choice['id']!=robot['id'] or not 1<=count<=run['config']['k'] or
                choice['selected_index']!=int(rng.integers(count)) or
                not np.isclose(choice['selection_probability'],1/count)):
                raise ValueError('Uniform sampling does not replay')
    trace=directory/'trajectory.npz'
    with np.load(trace,allow_pickle=False) as data:
        states,controls=data['states'],data['controls']
        if len(states)!=len(controls)+1 or not np.isfinite(states).all() or not np.isfinite(controls).all():
            raise ValueError('Invalid saved trace')
        np.testing.assert_allclose(states[0,:,:3],[r['start'] for r in task['robots']],atol=1e-10)
        if not validate_execution(task,states,controls)['passed']:raise ValueError('Physics audit failed')
        collisions=disk_collisions(task,world,states)
        if collisions['collisions']!=run['collisions']:raise ValueError('Collision count differs')
        _,_,arrivals=completion_data(task,states,controls)
        complete=all(a is not None for a in arrivals)
        success=complete and collisions['collisions']==0
        if success!=run['success']:raise ValueError('Recorded success differs from trace')
        np.testing.assert_allclose(np.mean([a is not None for a in arrivals]),run['completion_fraction'],atol=1e-12)
        if success:np.testing.assert_allclose(max(arrivals),run['makespan_seconds'],atol=1e-10)
        elif run['makespan_seconds'] is not None:raise ValueError('Failure has a makespan')
        monitor=TerminalMonitor(task,paths,EvaluationConfig(**run['config']))
        observed=None
        for step,control in enumerate(controls,1):
            observed,_=monitor.observe(step,states[step],control)
            if observed:break
        if observed!=run['status'] or step!=len(controls):
            raise ValueError('Terminal evidence does not replay from saved states')
        geometry=(audit_deadlock_geometry(task,world,paths,states,run['termination_evidence'],
            EvaluationConfig(**run['config'])) if run['status']=='deadlock' else None)
    return dict(run_id=run['run_id'],trajectory_sha256=hashlib.sha256(trace.read_bytes()).hexdigest(),
        run_sha256=hashlib.sha256(filename.read_bytes()).hexdigest(),passed=True,
        checked_steps=len(controls),checked_robots=task['robot_count'],collisions=collisions['collisions'],
        deadlock_geometry=geometry)


def audit_results(output,*,suite=ROOT/'benchmarks/scenarios/warehouse_uniform',workers=8):
    """Recompute physical outcomes and sampling from the saved raw executions."""
    output,suite=Path(output),Path(suite)
    experiment=json.loads((output/'experiment.json').read_text())
    files=sorted(output.glob('*/run.json'))
    manifest=json.loads((suite/'manifest.json').read_text())
    expected={(t['id'],planner) for t in manifest['scenarios'] for planner in ('rrt','k_routes')}
    runs=[json.loads(p.read_text()) for p in files]
    actual={(r['task_id'],r['planner']) for r in runs}
    if len(files)!=500 or actual!=expected or experiment['expected_runs']!=500:
        raise ValueError('Audit requires all 500 paired runs exactly once')
    if hashlib.sha256((suite/'manifest.json').read_bytes()).hexdigest()!=experiment['frozen_manifest_sha256']:
        raise ValueError('Frozen manifest changed')
    common=runs[0]['code_sha256']
    for filename,value in common.items():
        if hashlib.sha256((ROOT/filename).read_bytes()).hexdigest()!=value:
            raise ValueError(f'Evaluated source/binary changed: {filename}')
    for run in runs:
        if run['code_sha256']!=common or run['config']!=experiment['config']:
            raise ValueError('Mixed experimental protocols')
        if run['status'] not in ('success','deadlock'):
            raise ValueError(f"Unresolved terminal outcome: {run['run_id']}: {run['status']}")
    with ProcessPoolExecutor(max_workers=workers) as pool:
        records=list(pool.map(_audit_run,[(str(p),str(suite)) for p in files]))
    report=dict(passed=True,runs=len(records),checked_agent_runs=sum(r['checked_robots'] for r in records),
        checked_steps=sum(r['checked_steps'] for r in records),collisions=sum(r['collisions'] for r in records),
        source_and_binary_hashes=common,config=experiment['config'],traces=records,
        auditor_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        checked_blocked_robots=sum(r['deadlock_geometry']['checked_blocked_robots'] for r in records if r['deadlock_geometry']),
        visible_projection_corroborated_runs=[r['run_id'] for r in records if r['deadlock_geometry'] and
            r['deadlock_geometry']['visible_projection_corroborations']])
    save_json(output/'audit.json',report)
    save_json(output/'preflight.json',dict(expected_runs=500,passed_runs=500,passed=True,
        runs=[r['anchor_validation'] for r in runs]))
    return report


def render_examples(output,*,suite=ROOT/"benchmarks/scenarios/warehouse_uniform"):
    from .visualization import animate_run
    output=Path(output)
    runs=[json.loads(p.read_text()) for p in sorted(output.glob("*/run.json"))]
    selected=[]
    for name in ("warehouse","warehouse2"):
        for method in ("rrt","k_routes"):
            candidates=[r for r in runs if r["source_map"]==name and r["planner"]==method and r["simulation_executed"]]
            if candidates:
                successes=[r for r in candidates if r["success"]]
                selected.append((successes or candidates)[0])
        for pattern in ("distributed","opposing_bottleneck","intersection","symmetric_swap","competing_bottlenecks"):
            candidates=[r for r in runs if r['source_map']==name and r['traffic_pattern']==pattern and r['simulation_executed']]
            if candidates:
                choice=next((r for r in candidates if r['success']),candidates[0])
                if choice['run_id'] not in {r['run_id'] for r in selected}:selected.append(choice)
    for status in ("deadlock","collision"):
        cases=[r for r in runs if r["status"]==status and r["run_id"] not in {r["run_id"] for r in selected}]
        if cases:selected.append(cases[0])
    target=output/"animations";target.mkdir(exist_ok=True)
    index=[]
    for run in selected:
        filename=target/f"{run['run_id']}__{run['status']}.gif"
        task,world,_=load_task(suite,run["task_id"])
        with np.load(output/run["run_id"]/"trajectory.npz",allow_pickle=False) as d:states=d["states"]
        evidence=run.get("termination_evidence") or {}
        groups=evidence.get("mutual_obstruction_groups",[])
        focus=None
        if groups:
            focus=(states[-1,groups[0],:2].mean(axis=0),4.)
        elif not run["success"]:
            pending=[i for i,a in enumerate(run.get("individual_arrival_seconds",[])) if a is None]
            if pending:focus=(states[-1,pending[0],:2],4.)
        if not filename.exists():
            animate_run(task,world,states,filename,result=run,frame_count=64,focus=focus)
        index.append(dict(run_id=run["run_id"],source_map=run["source_map"],planner=run["planner"],
            traffic_pattern=run['traffic_pattern'],
            status=run["status"],makespan_seconds=run["makespan_seconds"],
            observed_simulated_seconds=run["simulated_seconds"],file=filename.name,
            selection="first saved success per map/method and map/traffic-pattern when available, otherwise first simulated run; additional first deadlock and collision examples"))
        print("Rendered measured rollout:",filename,flush=True)
    save_json(target/"index.json",index)
    lines=["# Recorded warehouse evaluation motion","",
        "These GIFs use actual saved states. The displayed timestamps are simulated "
        "seconds; playback compresses the observation into at most eight seconds. "
        "Agent footprints and headings retain the measured poses. The right panel "
        "zooms into the terminal blockage when available.","",
        "| Map | Method | Traffic pattern | Outcome | Observed simulated seconds | GIF |",
        "| --- | --- | --- | --- | ---: | --- |"]
    for row in index:lines.append(f"| {row['source_map']} | {row['planner']} | {row['traffic_pattern']} | "
        f"{row['status']} | {row['observed_simulated_seconds']:.1f} | [animation]({row['file']}) |")
    (target/"README.md").write_text("\n".join(lines)+"\n")
    return index


def format_time(row):
    mean=row["successful_makespan_mean_seconds"]
    count=row["successful_makespan_sample_count"]
    if mean is None:return "— (0 successes)"
    std=row["successful_makespan_std_seconds"]
    return f"{mean:.1f}"+(f" ± {std:.1f}" if std is not None else "")+f" (n={count})"


def plot_results(output,tables):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    output=Path(output)
    rows=tables["by_map_size"]
    lookup={(r["source_map"],r["diameter_meters"],r["planner"]):r for r in rows}
    sizes=[.6,.7,.8,.9,1.]
    fig,axes=plt.subplots(1,2,figsize=(12,4.5),layout="constrained",sharey=True)
    for ax,name in zip(axes,("warehouse","warehouse2")):
        for offset,method,color,label in ((-.18,"rrt","#3278a5","RRT-Connect"),(.18,"k_routes","#df8433","K routes, uniform")):
            values=[lookup.get((name,d,method),{}).get("fleet_success_rate",np.nan)*100 for d in sizes]
            ax.bar(np.arange(5)+offset,values,.34,label=label,color=color)
            for i,value in enumerate(values):
                if np.isfinite(value):ax.text(i+offset,value+1.2,f"{value:.0f}%",ha="center",fontsize=8)
        ax.set(title=name,xlabel="Agent diameter (m)",xticks=range(5),xticklabels=[f"{d:.1f}" for d in sizes],ylim=(0,108))
        ax.grid(axis="y",alpha=.2);ax.set_axisbelow(True)
    axes[0].set_ylabel("All 32 agents finish successfully (%)")
    axes[1].legend(loc="upper right",fontsize=9)
    fig.suptitle("Warehouse fleet success · 25 tasks per size and map · 1.2 m/s limit")
    fig.savefig(output/"success_by_size.png",dpi=170);fig.savefig(output/"success_by_size.svg");plt.close(fig)
    patterns=("distributed","opposing_bottleneck","intersection","symmetric_swap","competing_bottlenecks")
    data={(r["source_map"],r["diameter_meters"],r["traffic_pattern"],r["planner"]):r for r in tables["by_scenario"]}
    fig,axes=plt.subplots(2,2,figsize=(13,9),layout="constrained")
    image=None
    for row,name in enumerate(("warehouse","warehouse2")):
        for col,method in enumerate(("rrt","k_routes")):
            matrix=np.array([[data.get((name,d,p,method),{}).get("fleet_success_rate",np.nan)*100 for p in patterns] for d in sizes])
            ax=axes[row,col];image=ax.imshow(matrix,cmap="YlGnBu",vmin=0,vmax=100)
            for i in range(5):
                for j in range(5):
                    if np.isfinite(matrix[i,j]):
                        cohort=data[(name,sizes[i],patterns[j],method)]
                        ax.text(j,i,f"{matrix[i,j]:.0f}%\n({cohort['successes']}/{cohort['runs']})",ha="center",va="center",
                                color="white" if matrix[i,j]>55 else "black",fontsize=9)
            ax.set(title=f"{name} · {'RRT-Connect' if method=='rrt' else 'K routes, uniform'}",
                xticks=range(5),xticklabels=["Distributed","Opposing","Intersection","Swap","Competing"],
                yticks=range(5),yticklabels=[f"{d:.1f} m" for d in sizes])
            ax.tick_params(axis="x",labelrotation=18)
    fig.colorbar(image,ax=axes,shrink=.7,label="All-agent success across five start–goal variants (%)")
    fig.suptitle("Each cell is one scenario design · successes/executed variants shown")
    fig.savefig(output/"success_scenario_matrix.png",dpi=170);fig.savefig(output/"success_scenario_matrix.svg");plt.close(fig)
    # Failure-only cohorts stay empty; an absent makespan is never plotted as 0.
    fig,axes=plt.subplots(1,2,figsize=(12,4.5),layout="constrained")
    for ax,name in zip(axes,("warehouse","warehouse2")):
        for method,color,label in (("rrt","#3278a5","RRT-Connect"),("k_routes","#df8433","K routes, uniform")):
            values=[lookup.get((name,d,method),{}).get("successful_makespan_mean_seconds") for d in sizes]
            y=[np.nan if v is None else v for v in values]
            ax.plot(sizes,y,"o-",label=label,color=color)
            for d,v in zip(sizes,values):
                if v is not None:
                    n=lookup[(name,d,method)]["successful_makespan_sample_count"]
                    ax.annotate(f"n={n}",(d,v),xytext=(0,7),textcoords="offset points",ha="center",fontsize=8)
        ax.set(title=name,xlabel="Agent diameter (m)",ylabel="Makespan on successful runs only (s)",xticks=sizes)
        ax.grid(alpha=.2);ax.legend(fontsize=8)
        if not any(r.get("successful_makespan_sample_count",0) for r in rows if r["source_map"]==name):
            ax.text(.5,.5,"No all-agent successes\nMakespan is undefined",transform=ax.transAxes,ha="center",va="center")
    fig.savefig(output/"successful_makespan_by_size.png",dpi=170);fig.savefig(output/"successful_makespan_by_size.svg");plt.close(fig)


def paired_table(runs,keys):
    from collections import defaultdict
    pairs=defaultdict(dict)
    for run in runs:pairs[run["task_id"]][run["planner"]]=run
    groups=defaultdict(list)
    for pair in pairs.values():
        if set(pair)!={"rrt","k_routes"}:continue
        groups[tuple(pair["rrt"][k] for k in keys)].append(pair)
    rows=[]
    for key,items in sorted(groups.items()):
        row=dict(zip(keys,key),paired_tasks=len(items))
        for method in ("rrt","k_routes"):
            method_runs=[pair[method] for pair in items]
            success=[r for r in method_runs if r["success"]]
            row.update({f"{method}_success_rate":len(success)/len(items),
                f"{method}_successful_makespan_mean_seconds":float(np.mean([r["makespan_seconds"] for r in success])) if success else None,
                f"{method}_successful_makespan_sample_count":len(success),
                f"{method}_mean_agent_completion":float(np.mean([r["completion_fraction"] for r in method_runs]))})
        paired_success=[pair for pair in items if pair["rrt"]["success"] and pair["k_routes"]["success"]]
        row.update(success_rate_difference_k_minus_rrt=row["k_routes_success_rate"]-row["rrt_success_rate"],
            both_methods_successful_tasks=len(paired_success),
            paired_makespan_difference_k_minus_rrt_seconds=float(np.mean([
                pair["k_routes"]["makespan_seconds"]-pair["rrt"]["makespan_seconds"]
                for pair in paired_success])) if paired_success else None)
        rows.append(row)
    return rows


def write_report(output,*,destination=None):
    output=Path(output).resolve()
    runs,tables=summarize_evaluation(output)
    group_keys=dict(by_scenario=("source_map","scenario_id","diameter_meters","traffic_pattern","planner"),
        by_map_size=("source_map","diameter_meters","planner"),
        by_map_pattern=("source_map","traffic_pattern","planner"),by_map=("source_map","planner"),overall=("planner",))
    for name,rows in tables.items():
        for row in rows:
            cohort=[r for r in runs if all(r[k]==row[k] for k in group_keys[name])]
            planned=[r for r in cohort if r["simulation_executed"]]
            row.update(global_planning_successes=len(planned),global_planning_success_rate=len(planned)/len(cohort),
                fleet_success_rate_given_global_plan=row["successes"]/len(planned) if planned else None,
                mean_agent_completion_given_global_plan=float(np.mean([r["completion_fraction"] for r in planned])) if planned else None,
                runs_with_observed_mutual_obstruction=sum(bool(r.get("mutual_obstruction_observations")) for r in cohort))
        save_json(output/f"{name}.json",rows);write_csv(rows,output/f"{name}.csv")
    summary=json.loads((output/"summary.json").read_text());summary["tables"]=tables
    save_json(output/"summary.json",summary)
    for name,keys in (("paired_scenarios",("source_map","scenario_id","diameter_meters","traffic_pattern")),
        ("paired_sizes",("source_map","diameter_meters")),("paired_patterns",("source_map","traffic_pattern"))):
        paired=paired_table(runs,keys);save_json(output/f"{name}.json",paired);write_csv(paired,output/f"{name}.csv")
    plot_results(output,tables)
    path=Path(destination) if destination else output/"REPORT.md"
    path=path.resolve()
    def link(filename):return os.path.relpath(output/filename,path.parent)
    experiment=json.loads((output/"experiment.json").read_text())
    statuses=Counter(r["status"] for r in runs)
    availability={}
    for run in runs:
        if run['planner']!='k_routes' or not run['simulation_executed']:continue
        counts=availability.setdefault(run['source_map'],Counter())
        choices=json.loads((output/run['run_id']/'route_choices.json').read_text())
        counts.update(c['available'] for c in choices['robots'])
    availability_rows=[]
    requested=experiment['config']['k']
    for name,counts in sorted(availability.items()):
        total=sum(counts.values())
        availability_rows.append(dict(source_map=name,requested_k=requested,agent_queries=total,
            mean_returned_k=sum(k*n for k,n in counts.items())/total,
            **{f'returned_{k}':counts[k] for k in range(1,requested+1)}))
    save_json(output/'route_availability.json',availability_rows)
    write_csv(availability_rows,output/'route_availability.csv')
    lines=["# Warehouse MPPI/ORCA routing comparison", "",
        f"**{len(runs)}/{experiment['expected_runs']} planner–task runs.** "
        "Each method uses the same 250 frozen assignments: 25 scenario designs × five "
        "start–goal variants × two maps, with 32 identical agents per task.", "",
        "Forward speed is bounded at **1.2 m/s**, matching the maximum translational "
        "speed in [OMRON's LD-250 manual](https://assets.omron.eu/downloads/manual/en/v3/i642_ld-250_user_s_guide_users_manual_en.pdf). "
        "The benchmark convention remains 1 map unit = 1 m. Existing physical limits "
        "are acceleration 0.8 m/s², braking 1.2 m/s², yaw rate ±1.8 rad/s and yaw acceleration "
        "2 rad/s². The native fallback's angular-rate jump was corrected; both methods "
        "use that same binary, original unicycle integration and physical parking. "
        "Differential-drive anchor projection also requires a clear swept-footprint "
        "connection, preventing false progress onto a later aisle across shelving. "
        "Guided proposals include a bounded turn in place toward that visible join before "
        "moving; off-anchor heading cost points toward the join and blends back to the "
        "path tangent as the robot reacquires it.", "",
        f"The common controller yields after {experiment['config']['anchor_yield_after_seconds']:.0f} seconds "
        "without 0.15 m of route or best-goal improvement, when a stationary robot "
        "has blocked route reacquisition or the forward planning horizon for at least "
        "three seconds. MPPI/ORCA is still evaluated; the controller chooses its "
        "bounded braking sequence while waiting and resumes after two seconds clear. "
        "This suppresses repeated low-speed avoidance oscillations without changing "
        "poses, anchors, motion bounds or the deadlock detector.", "",
        f"The common MPPI profile uses {experiment['config']['batch_size']} rollouts and "
        f"{experiment['config']['horizon_states']} horizon states at dt=0.1 s "
        f"({(experiment['config']['horizon_states']-1)*.1:.1f} seconds of control horizon), with ORCA enabled. "
        "Both methods use the same synchronous joint safety filter: each proposed fleet "
        "command must have a continuously collision-free, bounded-braking continuation "
        "to rest. Unsafe proposals fall back to the previous certified stop, with safe "
        "progress retained by unaffected robots. This is MPPI/ORCA **with a shared safety "
        "layer**, not a safety guarantee supplied by decentralized ORCA alone. The "
        "certificate assumes exact synchronous states and the documented unicycle dynamics. "
        "Its clearance reserve is 0.04 m from static obstacles and 0.08 m between "
        "physical robot bodies (two per-agent margins). Task feasibility uses its "
        "declared 0.04 m inter-body margin; the controller's larger pair reserve is "
        "a common conservative policy for both routing methods.", "",
        "RRT uses independent RRT-Connect queries with configuration-space triangulation "
        "biasing its samples; uniform exploration is retained, and the mesh never supplies "
        "a fallback output path. Each query has a 100,000-iteration ceiling, a 1.5 m extension step "
        "and 800 shortcut attempts. K requests up to five radius-aware keep-right routes "
        "with 24 physical candidates, a 1.8 detour ratio and 0.2 diversity threshold, and "
        "selects each agent's route uniformly from the returned set. Planner-selection "
        "seed and controller seed are both 42, held fixed across the paired tasks.", "",
        "Success requires all 32 agents stopped inside their assigned 0.35 m goal "
        "regions, zero measured footprint collisions, and valid measured dynamics and "
        "control bounds. Agent completion is the fraction settled when the run terminates. "
        "Every evaluated assignment also has a separately replayed constructive joint "
        "completion schedule, including bounded controls and every stationary robot. "
        "These schedules are validation evidence and are never used as anchor paths or "
        "controller fallbacks. Six original endpoint variants lacked such certificates "
        "and were replaced using geometry and joint feasibility, before final measurements. "
        "Their original files and repair history are preserved under the scenario suite's "
        "`revisions/` directory; no MPPI outcome selects a replacement.", "",
        "A deadlock needs 30 seconds of stationary/no-progress evidence and persistent "
        "directed obstruction accounting for every unfinished agent: cycles, queues into "
        "cycles, or queues into robots permanently parked at their goals. Its evidence "
        "records the blocking IDs, directed edges and the deadlock subtype. A blocking "
        "disk must persistently intersect the agent's forward route within the common "
        "3-second MPPI horizon (3.6 m at maximum speed), including route reacquisition. "
        "This describes observed controller deadlock under the fixed routes and parking policy. "
        "An isolated stall or livelock needs 45 seconds without route or best-goal "
        "improvement for all unfinished agents, and is reported separately. Planning "
        "failure, collision and observation limits are also explicit outcomes. They "
        "remain in the task-success denominator. Finite observation does not prove "
        "mathematical deadlock or impossibility of eventual recovery.", "",
        "**Makespan is simulated time until the entire fleet finishes.** The reported "
        "mean ± sample standard deviation uses successful runs only; `(n=...)` is "
        "its sample count. Failed makespans are null. No failure is averaged as zero. "
        "Therefore a cohort with no full-fleet successes has no measured makespan.", "",
        "## Overall", "",
        "| Routing | Runs | Global plans received | Fleet successes | Fleet success | Mean agent completion | Successful makespan (s) |",
        "| --- | ---: | ---: | ---: | ---: | ---: | --- |"]
    for r in tables["overall"]:
        lines.append(f"| {r['planner']} | {r['runs']} | {r['global_planning_successes']} | {r['successes']} | {r['fleet_success_rate']:.1%} | "
            f"{r['mean_agent_completion_fraction']:.1%} | {format_time(r)} |")
    lines += ["", "The fleet-success denominator above includes global planning failures. "
        "The following view isolates controller behavior after all 32 anchors were available.", "",
        "| Routing | Simulated tasks | Fleet success given global plans | Mean agent completion given global plans |",
        "| --- | ---: | ---: | ---: |"]
    for r in tables["overall"]:
        conditional=r["fleet_success_rate_given_global_plan"]
        agent=r["mean_agent_completion_given_global_plan"]
        lines.append(f"| {r['planner']} | {r['global_planning_successes']} | "
            f"{conditional:.1%} | {agent:.1%} |" if conditional is not None else f"| {r['planner']} | 0 | — | — |")
    lines += ["", "**Fleet success and agent completion are different measurements.** "
        "Fleet success requires every one of the 32 robots to finish. Agent completion "
        "counts each finished robot, including those in a run where others deadlocked.", "",
        "| Routing | Finished agents / assigned agents | Agent completion | All-32 fleet success |",
        "| --- | ---: | ---: | ---: |"]
    for method in ('rrt','k_routes'):
        selected=[r for r in runs if r['planner']==method]
        assigned=sum(r['robot_count'] for r in selected)
        finished=round(sum(r['robot_count']*r['completion_fraction'] for r in selected))
        successes=sum(r['success'] for r in selected)
        lines.append(f"| {method} | {finished}/{assigned} | {finished/assigned:.1%} | "
            f"{successes}/{len(selected)} ({successes/len(selected):.1%}) |")
    lines += ["", "## Requested K and actual returned route sets", "",
        f"K={requested} is the requested upper bound. Each agent samples uniformly from "
        "its returned set. A shortfall can follow from the bounded search and diversity "
        "filters; it is not a certificate that no further feasible routes exist.", "",
        "| Map | Agent queries | "+" | ".join(f'{k} returned' for k in range(1,requested+1))+" | Mean returned K |",
        "| --- | ---: | "+" | ".join('---:' for _ in range(requested))+" | ---: |"]
    for row in availability_rows:
        lines.append(f"| {row['source_map']} | {row['agent_queries']} | "+
            " | ".join(str(row[f'returned_{k}']) for k in range(1,requested+1))+
            f" | {row['mean_returned_k']:.3f} |")
    lines += ["",f"[Actual route-count data]({link('route_availability.csv')})", ""]
    if (output/'REVIEW.md').exists():
        lines += [f"[Review of the low success rates and controlled diagnostic runs]({link('REVIEW.md')})", ""]
    lines += ["", "## Outcomes", "", "| Outcome | RRT-Connect | K routes, uniform |", "| --- | ---: | ---: |"]
    for status in sorted(statuses):
        counts=[sum(r["status"]==status and r["planner"]==m for r in runs) for m in ("rrt","k_routes")]
        lines.append(f"| {status} | {counts[0]} | {counts[1]} |")
    lines += ["", f"![Success by size]({link('success_by_size.png')})", "",
        f"![Scenario matrix]({link('success_scenario_matrix.png')})", "",
        f"![Conditional makespan]({link('successful_makespan_by_size.png')})", ""]
    for title,key,label_keys in (("Average across traffic patterns for each size and map","by_map_size",("source_map","diameter_meters")),
        ("Average across all sizes and traffic patterns for each map","by_map",("source_map",)),
        ("Average across sizes for each traffic pattern and map","by_map_pattern",("source_map","traffic_pattern")),
        ("Five start–goal variants per scenario and map","by_scenario",("source_map","scenario_id","diameter_meters","traffic_pattern"))):
        lines += [f"## {title}", "",
            "| Cohort | Method | Runs | Fleet success | Mean agent completion | Successful makespan (s) |",
            "| --- | --- | ---: | ---: | ---: | --- |"]
        for r in tables[key]:
            label=" / ".join(str(r[k]) for k in label_keys)
            lines.append(f"| {label} | {r['planner']} | {r['runs']} | {r['fleet_success_rate']:.1%} "
                f"({r['successes']}/{r['runs']}) | {r['mean_agent_completion_fraction']:.1%} | {format_time(r)} |")
        lines += [""]
    lines += ["## Machine-readable data and reproduction", "",
        f"- [All executed runs]({link('runs.csv')})", f"- [Five-variant scenario averages]({link('by_scenario.csv')})",
        f"- [Map and size averages]({link('by_map_size.csv')})",f"- [Map and traffic-pattern averages]({link('by_map_pattern.csv')})",
        f"- [Map averages]({link('by_map.csv')})",
        f"- [Paired scenario comparisons]({link('paired_scenarios.csv')})",
        f"- [Paired size comparisons]({link('paired_sizes.csv')})",
        f"- [Paired traffic-pattern comparisons]({link('paired_patterns.csv')})",
        f"- [Experiment configuration]({link('experiment.json')})",f"- [JSON summary]({link('summary.json')})", "",
        "Each run directory retains the selected anchors, uniform selection probabilities, "
        "native planning diagnostics, all executed states/controls, measured physics audit "
        "and termination evidence. The final task revision was fixed before its paired "
        "measurements; all 250 joint-certified tasks remain in the denominator. Earlier "
        "diagnostic runs use a separate output directory and are not pooled into these results.", "",
        "```bash", ".venv/bin/python benchmarks/multi_agent_bench.py evaluate-warehouse --workers 12", "```", "",
        "A run with the same task/configuration/code fingerprint is reused. A conflicting "
        "saved configuration raises an error rather than mixing experiments. "
        "Simulation budgets are common to both routing methods and derived from the "
        "saved geometry-reference length, not the chosen planner's path.", ""]
    lines += ["Paired makespan differences in the comparison CSVs use the same task only "
        "when both methods succeeded. Their sample counts are provided separately "
        "from the individual methods' successful-run averages.",""]
    if (output/'audit.json').exists():
        lines += [f"[Independent trace, physics, sampling and terminal-evidence audit]({link('audit.json')}). "
            f"[All 500 continuously checked anchor sets]({link('preflight.json')}).",""]
        audit=json.loads((output/'audit.json').read_text())
        if 'checked_blocked_robots' in audit:
            lines += [f"The independent geometry audit checked {audit['checked_blocked_robots']} blocked robots. "
                "If a nearest geometric projection crosses shelving, the audit requires the "
                "same recorded blocking edges to persist on the closest swept-footprint-clear "
                "join over the full observation window. Uncorroborated labels reject publication. "
                f"{len(audit['visible_projection_corroborated_runs'])} run required this extra "
                "visible-join check; its outcome and recorded motion were unchanged.",""]
    if (output/"animations/README.md").exists():
        lines += [f"[Actual rollout animations]({link('animations/README.md')})",""]
    path.parent.mkdir(parents=True,exist_ok=True);path.write_text("\n".join(lines))
    return path


def publish_results(output,*,target=ROOT/"benchmarks/scenarios/warehouse_uniform/evaluation"):
    """Keep compact result artifacts beside the frozen tasks; raw traces stay local."""
    output,target=Path(output).resolve(),Path(target).resolve()
    experiment=json.loads((output/"experiment.json").read_text())
    count=len(list(output.glob("*/run.json")))
    if count!=experiment["expected_runs"]:
        raise ValueError(f"Evaluation is incomplete: {count}/{experiment['expected_runs']} runs")
    audit_results(output)
    render_examples(output)
    write_report(output)
    target.mkdir(parents=True,exist_ok=True)
    for path in output.iterdir():
        if path.is_file() and path.suffix in (".csv",".json",".png",".svg",".md"):
            shutil.copyfile(path,target/path.name)
    if (output/"animations").exists():
        shutil.copytree(output/"animations",target/"animations",dirs_exist_ok=True)
    report=(target/"REPORT.md").read_text()
    report += "\nRaw per-run states, controls and native routing diagnostics are retained locally "
    report += f"under `{os.path.relpath(output,ROOT)}`. `audit.json` records their trace hashes.\n"
    (target/"REPORT.md").write_text(report)
    primary=ROOT/"benchmarks/WAREHOUSE_EVALUATION_RESULTS.md"
    def relocate(match):
        value=match.group(1)
        if value.startswith(("http:","https:","#")):return match.group(0)
        return "]("+os.path.relpath(target/value,primary.parent)+")"
    primary.write_text(re.sub(r"\]\(([^)]+)\)",relocate,report))
    return primary
