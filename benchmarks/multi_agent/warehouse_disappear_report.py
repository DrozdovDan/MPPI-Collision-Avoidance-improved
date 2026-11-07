"""Independently audit presence, native dynamics and paired target policies."""
from collections import Counter
from concurrent.futures import ProcessPoolExecutor, as_completed
from dataclasses import asdict
import json
from pathlib import Path
import numpy as np

from .geometry import ROOT, fingerprint
from .metrics import completion_data, validate_execution
from .warehouse_evaluation import EvaluationConfig, aggregate_rows, load_task, save_json, write_csv
from .warehouse_disappear_evaluation import DisappearTerminalMonitor, disappear_disk_collisions
from .warehouse_disappear_benchmark import METHODS, POLICY, reference_directory
from .warehouse_evaluation_report import audit_deadlock_geometry
from .warehouse_terminal_adjudication import certify_visible_join_deadlock
from .warehouse_preflight import check_paths, check_joint_certificate
from .warehouse_width_benchmark import sha
from .warehouse_cost_experiment_report import labels_and_order, table, plots, comparison_run


def audit_removed_snapshot(states,controls,agent,start,dt):
    """Check fixed absent poses and the velocity of the incoming interval.

    State columns 3:5 describe the displacement ending at that sample. On the
    removal sample they may retain the small incoming velocity allowed by the
    settled threshold; following samples have zero displacement and velocity.
    No pose tolerance or mutation of the saved trace is needed.
    """
    poses=states[start:,agent,:3]
    if not np.array_equal(poses,np.broadcast_to(poses[0],poses.shape)):
        raise ValueError('Removed snapshot pose moved')
    if np.any(controls[start:,agent]!=0):raise ValueError('Removed agent received controls')
    incoming=(states[start,agent,:2]-states[start-1,agent,:2])/dt
    if not np.array_equal(states[start,agent,3:5],incoming):
        raise ValueError('Removal-sample velocity differs from incoming displacement')
    if np.any(states[start+1:,agent,3:5]!=0):
        raise ValueError('Absent snapshot has nonzero velocity after removal')


def audit_deadlock_subgraph(task,world,paths,states,evidence,cfg,active,original_error):
    """Require a fleet-covering cycle graph using only corroborated raw edges.

    An unnecessary false edge must not invalidate other physical evidence.
    Every retained edge passes the original geometry auditor. Rejection is
    mandatory if removing unsupported edges breaks unfinished-fleet coverage.
    This changes neither the raw evidence nor any recorded outcome.
    """
    from scipy.sparse import csr_matrix
    from scipy.sparse.csgraph import connected_components
    present=set(map(int,np.flatnonzero(active[-1])))
    verified=[];rejected=[]
    for i,j in evidence['obstruction_edges']:
        if i not in present or j not in present:
            raise ValueError('Absent robot in deadlock subgraph')
        edge_evidence=evidence|dict(blocked_robot_ids=[i],obstruction_edges=[[i,j]])
        try:audit_deadlock_geometry(task,world,paths,states,edge_evidence,cfg)
        except ValueError as error:rejected.append(dict(edge=[i,j],reason=str(error)))
        else:verified.append([i,j])
    graph=np.zeros((task['robot_count'],task['robot_count']),dtype=bool)
    for i,j in verified:graph[i,j]=True
    np.fill_diagonal(graph,False)
    _,labels=connected_components(csr_matrix(graph),directed=True,connection='strong')
    groups=[np.flatnonzero(labels==value).tolist() for value in set(labels)
            if np.sum(labels==value)>1]
    blocked={i for group in groups for i in group}
    while True:
        extended=blocked|{i for i in present if any(graph[i,j] for j in blocked)}
        if extended==blocked:break
        blocked=extended
    if not present.issubset(blocked):
        raise ValueError(f'Corroborated deadlock edges do not cover robots {sorted(present-blocked)}')
    filtered=evidence|dict(blocked_robot_ids=sorted(present),obstruction_edges=verified,
                          mutual_obstruction_groups=groups)
    geometry=audit_deadlock_geometry(task,world,paths,states,filtered,cfg)
    return geometry|dict(raw_geometry_error=str(original_error),
        rejected_obstruction_edges=rejected,verified_obstruction_edges=verified,
        verified_mutual_obstruction_groups=groups,verified_blocked_robot_ids=sorted(present),
        raw_motion_and_evidence_unchanged=True)


def adjudicate_stall(task,world,paths,states,controls,evidence,cfg,active):
    ids=np.flatnonzero(active[-1]);window=round(cfg.deadlock_window_seconds/task['dt'])
    if not np.all(active[-window:,ids]):raise ValueError('Presence changed in the deadlock window')
    mapping={int(i):j for j,i in enumerate(ids)}
    reduced_task=task|dict(robot_count=len(ids),robots=[task['robots'][i] for i in ids])
    reduced_evidence=evidence|dict(unfinished_robot_ids=list(range(len(ids))),
        blocked_robot_ids=[mapping[i] for i in evidence['blocked_robot_ids']],
        obstruction_edges=[[mapping[i],mapping[j]] for i,j in evidence['obstruction_edges']])
    result=certify_visible_join_deadlock(reduced_task,world,[paths[i] for i in ids],
        states[:,ids],controls[:,ids],reduced_evidence,cfg)
    result['original_robot_indices']=ids.tolist()
    result['excluded_disappeared_robot_indices']=np.flatnonzero(~active[-1]).tolist()
    return result


def audit_trace(arguments):
    suite,output,job=arguments;suite,output=Path(suite),Path(output)
    method,task_id=job['method'],job['task_id'];directory=output/f'{task_id}__{method}__p42__c42'
    filename=directory/'run.json';run=json.loads(filename.read_text())
    source=reference_directory(task_id,method);reference=json.loads((source/'run.json').read_text())
    if sha(source/'run.json')!=job['reference_run_sha256'] or sha(source/'trajectory.npz')!=job['reference_trajectory_sha256']:
        raise ValueError('Historical execution changed')
    task,world,task_hash=load_task(suite,task_id);cfg=EvaluationConfig(**run['config'])
    if run['config']!=asdict(EvaluationConfig()) or run['controller_profiles']!=reference['controller_profiles']:
        raise ValueError('Controller/configuration differs')
    if run['scenario_sha256']!=task_hash or run['target_policy']!=POLICY or run['protocol_inputs']['target_policy']!=POLICY:
        raise ValueError('Frozen task or target policy differs')
    identity=fingerprint(dict(task=task_hash,config=run['config'],code=run['code_sha256'],planner=method,protocol_inputs=run['protocol_inputs']))
    if identity!=run['evaluation_identity']:raise ValueError('Evaluation identity mismatch')
    if run['joint_feasibility_certificate']!=check_joint_certificate(suite,task):raise ValueError('Joint proof differs')
    for name,key in (('paths.json','paths_sha256'),('route_choices.json','choices_sha256')):
        if sha(directory/name)!=job[key] or sha(source/name)!=job[key] or run['anchor_validation'][key]!=job[key]:
            raise ValueError('Paired anchor paths/choices changed')
    paths=json.loads((directory/'paths.json').read_text());check_paths(task,world,paths)
    with np.load(directory/'trajectory.npz',allow_pickle=False) as trace:
        states,controls,active=trace['states'],trace['controls'],trace['active']
    if len(states)!=len(controls)+1 or active.shape!=states.shape[:2] or active.dtype!=np.bool_:
        raise ValueError('Invalid trace/presence shape')
    if not np.isfinite(states).all() or not np.isfinite(controls).all() or not active[0].all():raise ValueError('Invalid trace')
    np.testing.assert_allclose(states[0,:,:3],[r['start'] for r in task['robots']],atol=1e-10)
    if not validate_execution(task,states,controls)['passed']:raise ValueError('Native physics violation')
    goals=np.array([r['goal']['pose'][:2] for r in task['robots']]);tolerance=np.array([r['goal']['radius'] for r in task['robots']])
    settled=(np.linalg.norm(states[1:,:,:2]-goals,axis=2)<=tolerance+1e-8)&(np.max(abs(controls),axis=2)<=1e-8)
    expected=np.vstack([np.ones(task['robot_count'],dtype=bool),~np.logical_or.accumulate(settled,axis=0)])
    if not np.array_equal(active,expected):raise ValueError('Removal did not match the first settled target timestep')
    arrivals=[]
    for i in range(task['robot_count']):
        absent=np.flatnonzero(~active[:,i]);arrival=float(absent[0]*task['dt']) if len(absent) else None
        arrivals.append(arrival)
        if len(absent):
            audit_removed_snapshot(states,controls,i,absent[0],task['dt'])
    if arrivals!=run['removal_seconds']:
        raise ValueError('Recorded removal measurements differ from presence transitions')
    _,_,stable_arrivals=completion_data(task,states,controls)
    if stable_arrivals!=run['individual_arrival_seconds']:
        raise ValueError('Legacy settled arrival measurements do not replay')
    if [a is not None for a in arrivals]!=[a is not None for a in stable_arrivals]:
        raise ValueError('Legacy completion counts differ from actual disappearances')
    # The legacy 1e-6 state-speed threshold may precede the stricter 1e-8
    # applied-control removal threshold. Retain both measurements explicitly;
    # the fleet makespan below is checked against actual disappearances.
    arrival_differences=[dict(agent=i,legacy_arrival_seconds=b,removal_seconds=a)
        for i,(a,b) in enumerate(zip(arrivals,stable_arrivals)) if a!=b]
    collision=disappear_disk_collisions(task,world,states,active)
    if collision['collisions'] or run['collisions']:raise ValueError('Collision while present')
    success=not active[-1].any()
    if run['success']!=success or run['all_robots_success']!=success:raise ValueError('Fleet success differs')
    np.testing.assert_allclose(run['completion_fraction'],np.mean(~active[-1]),atol=1e-12)
    if success:np.testing.assert_allclose(run['makespan_seconds'],max(arrivals),atol=1e-10)
    elif run['makespan_seconds'] is not None:raise ValueError('Failed makespan imputed')
    monitor=DisappearTerminalMonitor(task,paths,cfg);terminal=None
    for step,control in enumerate(controls,1):
        terminal,evidence=monitor.observe(step,states[step],control,active[step])
        if terminal:break
    if terminal!=run['status'] or step!=len(controls) or evidence!=run['termination_evidence']:
        raise ValueError('Terminal evidence does not replay')
    if evidence.get('parked_goal_blocker_robot_ids',[]):raise ValueError('Absent robot counted as blocker')
    if any(not active[-1,i] or not active[-1,j] for i,j in evidence.get('obstruction_edges',[])):
        raise ValueError('Absent robot in terminal obstruction')
    geometry=None
    if terminal=='deadlock':
        try:geometry=audit_deadlock_geometry(task,world,paths,states,evidence,cfg)
        except ValueError as error:
            geometry=audit_deadlock_subgraph(task,world,paths,states,evidence,cfg,active,error)
            save_json(directory/'deadlock_geometry_corroboration.json',geometry)
    adjudication,unresolved=None,None
    if terminal=='stalled':
        try:adjudication=adjudicate_stall(task,world,paths,states,controls,evidence,cfg,active)
        except ValueError as error:unresolved=str(error)
        if adjudication:save_json(directory/'terminal_adjudication.json',adjudication)
    # Before the first disappearance, the old and new simulations must match
    # bit for bit, including the interval on which the first robot settles.
    changed=np.flatnonzero(np.any(~active,axis=1));prefix=int(changed[0])+1 if len(changed) else len(states)
    with np.load(source/'trajectory.npz',allow_pickle=False) as old:
        if not np.array_equal(states[:prefix],old['states'][:prefix]) or not np.array_equal(controls[:prefix-1],old['controls'][:prefix-1]):
            raise ValueError('Motion changed before the first disappearance')
    return dict(run_id=run['run_id'],passed=True,run_sha256=sha(filename),trajectory_sha256=sha(directory/'trajectory.npz'),
        raw_status=terminal,status='deadlock' if adjudication else terminal,checked_agent_runs=32,checked_steps=len(controls),
        historical_prefix_steps_reproduced=prefix-1,collisions=0,removed_agents=int(np.sum(~active[-1])),
        deadlock_geometry=geometry,terminal_adjudication=adjudication,unresolved_stall_reason=unresolved,
        arrival_measurement_differences=arrival_differences)


def publish(suite,output,workers=8,full_suite=True):
    suite,output=Path(suite),Path(output);experiment=json.loads((output/'experiment.json').read_text());jobs=experiment['jobs']
    files=list(output.glob('*/run.json'))
    if len(files)!=len(jobs):raise ValueError('Incomplete experiment')
    common=json.loads(files[0].read_text())['code_sha256']
    for name,digest in common.items():
        if sha(ROOT/name)!=digest:raise ValueError(f'Executed source changed: {name}')
    for filename in files:
        if json.loads(filename.read_text())['code_sha256']!=common:raise ValueError('Mixed executed sources')
    records=[];errors=[]
    with ProcessPoolExecutor(max_workers=workers) as pool:
        futures={pool.submit(audit_trace,(str(suite),str(output),j)):j for j in jobs}
        for count,future in enumerate(as_completed(futures),1):
            try:records.append(future.result())
            except Exception as error:
                job=futures[future]
                errors.append(dict(task_id=job['task_id'],method=job['method'],error=str(error)))
                print(f'Audit failed for {job["task_id"]} / {job["method"]}: {error}',flush=True)
            if count%25==0:print(f'Audited {count}/{len(jobs)} disappearance traces',flush=True)
            if count%25==0 or errors:
                save_json(output/'audit_progress.json',dict(checked_runs=count,expected_runs=len(jobs),errors=errors))
    save_json(output/'audit_errors.json',errors)
    if errors:raise ValueError(f'{len(errors)} trace audits failed; see {output}/audit_errors.json')
    audit=dict(passed=True,runs=len(records),checked_agent_runs=sum(r['checked_agent_runs'] for r in records),
        target_policy=POLICY,auditor_sha256=sha(__file__),source_and_binary_hashes=common,
        raw_terminal_outcomes=dict(Counter(r['raw_status'] for r in records)),
        verified_terminal_outcomes=dict(Counter(r['status'] for r in records)),collisions=0,
        historical_reference_runs_rechecked=len(records),historical_prefixes_reproduced=len(records),
        unresolved_stalls=[r['run_id'] for r in records if r['unresolved_stall_reason']],
        legacy_arrival_differences=[dict(run_id=r['run_id'],**d) for r in records for d in r['arrival_measurement_differences']],
        pruned_graph_corroborations=[r['run_id'] for r in records if r['deadlock_geometry'] and
            r['deadlock_geometry'].get('rejected_obstruction_edges')],
        traces=sorted(records,key=lambda r:r['run_id']))
    save_json(output/'audit.json',audit)
    byid={r['run_id']:r for r in records};new=[];parked=[]
    historical_receipts={}
    for job in jobs:
        parent=reference_directory(job['task_id'],job['method']).parent
        if parent not in historical_receipts:
            historical_receipts[parent]={r['run_id']:r for r in json.loads((parent/'audit.json').read_text())['traces']}
    for job in jobs:
        run_id=f'{job["task_id"]}__{job["method"]}__p42__c42'
        run=json.loads((output/run_id/'run.json').read_text());run['raw_status']=run['status'];run['status']=byid[run_id]['status']
        new.append(comparison_run(run)|dict(target_policy='disappear'))
        ref=json.loads((reference_directory(job['task_id'],job['method'])/'run.json').read_text())
        receipt=historical_receipts[reference_directory(job['task_id'],job['method']).parent][ref['run_id']]
        ref['raw_status']=ref['status']
        ref['status']=receipt.get('verified_terminal_outcome',receipt.get('status',ref['status']))
        parked.append(comparison_run(ref)|dict(target_policy='park'))
    labels,order=labels_and_order(new)
    target=suite/'disappear_evaluation' if full_suite else output/'review';target.mkdir(parents=True,exist_ok=True)
    views=dict(overall=['planner'],by_map=['source_map','planner'],
        by_scenario=['source_map','scenario_id','diameter_meters','traffic_pattern','planner'],
        by_map_size=['source_map','diameter_meters','planner'],by_map_pattern=['source_map','traffic_pattern','planner'])
    tables={name:aggregate_rows(new,keys) for name,keys in views.items()}
    for name,keys in views.items():
        save_json(target/f'comparison_{name}.json',tables[name]);write_csv(tables[name],target/f'comparison_{name}.csv')
        save_json(target/f'disappear_{name}.json',tables[name]);write_csv(tables[name],target/f'disappear_{name}.csv')
        both=aggregate_rows(parked+new,['target_policy']+keys)
        save_json(target/f'policies_{name}.json',both);write_csv(both,target/f'policies_{name}.csv')
    write_csv(parked+new,target/'comparison_runs.csv')
    write_csv(new,target/'disappear_runs.csv')
    pairs=[]
    for name in ('both','warehouse','warehouse2'):
        for method in order:
            fresh={r['task_id']:r for r in new if r['planner']==method and (name=='both' or r['source_map']==name)}
            old={r['task_id']:r for r in parked if r['planner']==method and r['task_id'] in fresh}
            if not fresh:continue
            matched=[i for i in fresh if fresh[i]['success'] and old[i]['success']]
            pairs.append(dict(source_map=name,planner=method,tasks=len(fresh),both_successful_tasks=len(matched),
                gained_fleet_successes=sum(fresh[i]['success'] and not old[i]['success'] for i in fresh),
                lost_fleet_successes=sum(old[i]['success'] and not fresh[i]['success'] for i in fresh),
                fleet_success_rate_difference=float(np.mean([fresh[i]['success']-old[i]['success'] for i in fresh])),
                agent_completion_difference=float(np.mean([fresh[i]['completion_fraction']-old[i]['completion_fraction'] for i in fresh])),
                disappear_minus_park_makespan_seconds=float(np.mean([fresh[i]['makespan_seconds']-old[i]['makespan_seconds'] for i in matched])) if matched else None))
    save_json(target/'paired_target_policies.json',pairs);write_csv(pairs,target/'paired_target_policies.csv')
    save_json(target/'audit.json',audit);save_json(target/'comparison_summary.json',dict(runs=len(new),methods=order,labels=labels,tables=tables))
    if full_suite:
        plots(target,tables,labels,order)
        from . import warehouse_disappear_figures
        warehouse_disappear_figures.plots(target,tables,new,parked,pairs,order)
    overall,mapped=table(tables['overall'],labels,order),table(tables['by_map'],labels,order,mapped=True)
    sized=table([r|dict(source_map=f'{r["source_map"]} / {r["diameter_meters"]:.1f} m')
                 for r in tables['by_map_size']],labels,order,mapped=True)
    patterned=table([r|dict(source_map=f'{r["source_map"]} / {r["traffic_pattern"]}')
                     for r in tables['by_map_pattern']],labels,order,mapped=True)
    oldtable={r['planner']:r for r in aggregate_rows(parked,['planner'])}
    rows=['| Method | Park: fleet success | Disappear: fleet success | Change | Agent completion: park → disappear | Successful makespan: park → disappear (s) |',
        '| --- | ---: | ---: | ---: | ---: | ---: |']
    for r in sorted(tables['overall'],key=lambda r:order.index(r['planner'])):
        old=oldtable[r['planner']]
        fmt=lambda x:'—' if x is None else f'{x:.1f}'
        rows.append(f'| {labels[r["planner"]]} | {old["successes"]}/{old["runs"]} ({old["fleet_success_rate"]:.1%}) | {r["successes"]}/{r["runs"]} ({r["fleet_success_rate"]:.1%}) | {100*(r["fleet_success_rate"]-old["fleet_success_rate"]):+.1f} pp | {old["mean_agent_completion_fraction"]:.1%} → {r["mean_agent_completion_fraction"]:.1%} | {fmt(old["successful_makespan_mean_seconds"])} → {fmt(r["successful_makespan_mean_seconds"])} |')
    comparison='\n'.join(rows)
    lines=['# Warehouse benchmark: disappear at target','',
        '**1,250 executions: 250 tasks per method, 125 per map, 32 equal-sized agents per task. Each map/scenario has five start–goal variants.**','',
        'All five methods reuse their exact original anchors and route choices on the same frozen tasks. Normal MPPI/ORCA, 1.2 m/s speed bound, bounded dynamics, braking safety filter and termination windows stay fixed. The original K-route candidate sets are reused; the separate open-route case experiment is not part of these results.','',
        experiment['policy_definition'],'',
        'Fleet success means all 32 agents finish. Makespans include physically bounded deceleration to rest. Failed makespans are undefined. Means and sample standard deviations use successful tasks only; successful cohorts differ between methods and target policies.','',
        '## Disappear-at-target: all tasks','',overall,'',
        '![Disappear-at-target overview](disappear_overview.png)','',
        '## Disappear-at-target: each map','',mapped,'',
        '![Fleet completion over simulated time, with all tasks in the denominator](disappear_completion_over_time.png)','',
        '## Disappear-at-target: size averages on each map','',
        'Each row averages 25 tasks: five traffic scenarios × five start–goal variants. Agent sizes are diameters.','',sized,'',
        '![Fleet success by agent size](fleet_success_by_size.png)','',
        '![Individual-agent completion by size](agent_completion_by_size.png)','',
        '![Makespan by size, successful tasks only](successful_makespan_by_size.png)','',
        '## Disappear-at-target: traffic-pattern averages on each map','',
        'Each row averages 25 tasks: five sizes × five start–goal variants.','',patterned,'',
        '![Fleet success by traffic pattern](disappear_fleet_success_by_pattern.png)','',
        '![Individual-agent completion by traffic pattern](disappear_agent_completion_by_pattern.png)','',
        '## Disappear-at-target: every scenario, five variants','',
        'Every cell below averages exactly five start–goal variants for one scenario, agent size, map and method. Fleet success therefore changes in increments of 20 percentage points. '
        '[All 250 scenario/method averages](disappear_by_scenario.csv).','',
        '![Scenario fleet success](disappear_scenario_success.png)','',
        '![Scenario individual-agent completion](disappear_scenario_completion.png)','',
        '## Stay-at-target versus disappear-at-target','',comparison,'',
        '![Same-task target-policy comparison](stay_vs_disappear_overview.png)','',
        '## Audit and the original exception','',
        f'Audited {audit["runs"]} new runs / {audit["checked_agent_runs"]} agent executions, with zero collisions or motion-limit violations. All {audit["historical_prefixes_reproduced"]} runs reproduce the parked baseline exactly up to and including the first disappearance. Raw outcomes: `{audit["raw_terminal_outcomes"]}`. Verified outcomes: `{audit["verified_terminal_outcomes"]}`. Unresolved stalls remain failures.','',
        'The original final audit stopped with `Removed snapshot moved` for agent 3 in `warehouse_s04_v01 / rrt`. Its x, y and heading remain exactly constant after removal. On the removal sample, the recorded Cartesian velocity describes the last incoming interval (at most 1.873e-9 m/s in a component); the following zero-displacement samples have zero velocity. The corrected audit checks fixed poses exactly, incoming velocity against the measured incoming displacement, zero later velocity and zero outgoing controls. No trajectories, controls, arrival times or success flags were edited, and no simulations were rerun.','',
        f'The legacy individual-arrival metric uses a 1e-6 state-speed/yaw-rate threshold; disappearance uses a stricter 1e-8 applied-control threshold. The audit independently reconstructs both. {len(audit["legacy_arrival_differences"])} agent records have an earlier legacy arrival by 0.1 s; fleet success, completion counts and every reported fleet makespan agree with actual disappearance times. The differences are recorded in `audit.json`, and raw per-agent timestamps remain unchanged.','',
        f'{len(audit["pruned_graph_corroborations"])} deadlock record(s) contained a redundant edge not corroborated after choosing a footprint-clear visible route join. For these records, every raw edge is checked independently, unsupported edges are excluded from the certificate, and the remaining graph must still cover every unfinished agent through persistent cycles or chains into cycles. The raw detector evidence remains unchanged; certificates list both rejected and retained edges.','',
        '## Matched successful tasks','',
        'Negative differences below mean disappearance was faster on the same successful tasks.','',
        '| Method | Both policies successful | Disappear minus park makespan (s) | Gained successes | Lost successes |',
        '| --- | ---: | ---: | ---: | ---: |']
    for row in pairs:
        if row['source_map']=='both':
            d=row['disappear_minus_park_makespan_seconds'];value=f'{d:+.2f}' if d is not None else '—'
            lines.append(f'| {labels[row["planner"]]} | {row["both_successful_tasks"]} | {value} | {row["gained_fleet_successes"]} | {row["lost_fleet_successes"]} |')
    lines+=['','![Paired makespan change on the same successful tasks](stay_vs_disappear_paired_makespan.png)','',
        '## Downloadable data','',
        '[Disappear only: all 1,250 executions](disappear_runs.csv). '
        '[Overall table](disappear_overall.csv). [Per-map table](disappear_by_map.csv). '
        '[Five start–goal variant averages per scenario](disappear_by_scenario.csv). '
        '[Averages by size](disappear_by_map_size.csv). [Averages by traffic pattern](disappear_by_map_pattern.csv). '
        '[Both target policies, every run](comparison_runs.csv). [Paired policy changes](paired_target_policies.csv). [Audit](audit.json).','',
        'All figures are supplied as PNG and SVG.','',
        'Audit and rebuild this report without rerunning simulations: `OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 PYTHONPATH=benchmarks .venv/bin/python -m multi_agent.warehouse_disappear_report --workers 8`.','']
    (target/'REPORT.md').write_text('\n'.join(lines))
    if full_suite:
        marker='\n## Disappear-at-target benchmark\n'
        for filename,link in ((suite/'evaluation/REVIEW.md','../disappear_evaluation/REPORT.md'),
                (ROOT/'benchmarks/WAREHOUSE_BOTTLENECK_RESULTS.md','scenarios/warehouse_uniform/disappear_evaluation/REPORT.md')):
            text=filename.read_text().split(marker)[0]
            image_root=str(Path(link).parent)
            filename.write_text(text+marker+'\n### Disappear-at-target only\n\n'+overall+'\n\n'+mapped+
                f'\n\n![Disappear-at-target results]({image_root}/disappear_overview.png).\n'+
                '\n### Comparison with stay-at-target\n\n'+comparison+
                '\n\nRobots disappear after physically settling at their targets. Controller settings and sampled paths are unchanged. Successful-makespan means use different successful cohorts; matched-task comparisons are in the full report.\n\n'+
                f'[Complete results, per-map tables, five-variant averages, standalone figures and audit]({link}).\n')
    hashes={str(p.relative_to(target)):sha(p) for p in target.iterdir() if p.is_file() and p.name!='verification.json'}
    save_json(target/'verification.json',dict(passed=True,runs=len(new),historical_runs=len(parked),
        source_hashes=common,auditor_sha256=sha(__file__),artifact_sha256=hashes,
        complete_suite=full_suite,zero_collisions=True,physics_verified=True,presence_verified=True,
        figure_generator_sha256=sha(Path(__file__).with_name('warehouse_disappear_figures.py'))))
    print('Published disappear-at-target results:',target/'REPORT.md',flush=True)
    return tables


def main():
    import argparse
    import os
    parser=argparse.ArgumentParser(description='Audit saved traces and rebuild tables/figures without simulation.')
    parser.add_argument('--suite',type=Path,default=ROOT/'benchmarks/scenarios/warehouse_uniform')
    parser.add_argument('--output',type=Path,default=ROOT/'benchmarks/results/warehouse_disappear')
    parser.add_argument('--workers',type=int,default=8)
    args=parser.parse_args();output=args.output
    save_json(output/'execution_state.json',dict(status='auditing',pid=os.getpid(),expected_runs=1250,
        completed_runs=len(list(output.glob('*/run.json'))),audit_workers=args.workers,simulation_workers=0))
    try:publish(args.suite,output,workers=args.workers)
    except Exception as error:
        save_json(output/'execution_state.json',dict(status='audit_failed',completed_runs=len(list(output.glob('*/run.json'))),
            expected_runs=1250,error=str(error),simulation_workers=0))
        raise
    save_json(output/'execution_state.json',dict(status='complete',runs=1250,expected_runs=1250,
        audit_passed=True,active_processes=0,report=str(args.suite/'disappear_evaluation/REPORT.md')))
    save_json(output/'monitoring.json',dict(status='complete',completed_runs=1250,audit_passed=True,active_processes=0))


if __name__=='__main__':main()
