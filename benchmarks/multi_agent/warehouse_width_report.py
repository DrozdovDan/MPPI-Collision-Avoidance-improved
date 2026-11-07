"""Audit and publish the frozen width-weighted/uniform/RRT comparison."""
from collections import Counter
from concurrent.futures import ProcessPoolExecutor
from dataclasses import asdict
import hashlib
import json
import os
from pathlib import Path
import shutil

import numpy as np
import shapely

from .geometry import ROOT,fingerprint
from .metrics import validate_execution,completion_data
from .bottleneck_selection import CorridorWidths,draw_width_route,audit_lane_geometry
from .warehouse_width_benchmark import METHOD,JITTER_TAG,WIDTH_DEFINITION,sha,baseline_contract
from .warehouse_evaluation import load_task,EvaluationConfig,TerminalMonitor,disk_collisions,aggregate_rows,write_csv,save_json
from .warehouse_preflight import check_paths,check_joint_certificate,anchor_identity
from .warehouse_evaluation_report import audit_deadlock_geometry,format_time
from .warehouse_terminal_adjudication import certify_visible_join_deadlock

_WIDTH_CACHE={}
LABELS={'rrt':'RRT-Connect','k_routes':'K routes, uniform',METHOD:'K routes, bottleneck width'}
METHODS=('rrt','k_routes',METHOD)


def _audit_weighted(arguments):
    suite,baseline,output,filename=map(Path,arguments)
    run=json.loads(filename.read_text());directory=filename.parent
    task,world,task_hash=load_task(suite,run['task_id']);cfg=EvaluationConfig(**run['config'])
    if task_hash!=run['scenario_sha256']:raise ValueError('Weighted task changed')
    if fingerprint(dict(task=task_hash,config=run['config'],code=run['code_sha256'],planner=METHOD,
        protocol_inputs=run['protocol_inputs']))!=run['evaluation_identity']:raise ValueError('Weighted identity differs')
    if run['joint_feasibility_certificate']!=check_joint_certificate(suite,task):raise ValueError('Joint proof changed')
    uniform_dir=baseline/f'{task["id"]}__k_routes__p42__c42'
    old=json.loads((uniform_dir/'run.json').read_text())
    uniform_paths=json.loads((uniform_dir/'paths.json').read_text())
    uniform_choices=json.loads((uniform_dir/'route_choices.json').read_text())['robots']
    rrt=json.loads((baseline/f'{task["id"]}__rrt__p42__c42/run.json').read_text())
    for reference in (old,rrt):
        if run['controller_profiles']!=reference['controller_profiles'] or run['config']!=reference['config']:
            raise ValueError('Controller settings differ across methods')
        if run['scenario_sha256']!=reference['scenario_sha256']:raise ValueError('Assignments differ')
    record_file=output/'selection_inputs'/f'{task["id"]}.json'
    if sha(record_file)!=run['protocol_inputs']['selection_inputs_sha256']:raise ValueError('Selection inputs changed')
    prepared=json.loads(record_file.read_text())
    native_file=uniform_dir/'agents.routes.json'
    if sha(native_file)!=prepared['native_candidates_sha256']:raise ValueError('Frozen candidates changed')
    native=json.loads(native_file.read_text())
    paths=json.loads((directory/'paths.json').read_text());check_paths(task,world,paths)
    choices=json.loads((directory/'route_choices.json').read_text())
    anchors=run['anchor_validation']
    if anchors['identity']!=anchor_identity(task,METHOD,cfg.selection_seed,cfg.k):raise ValueError('Anchor identity differs')
    for name,key in [('paths.json','paths_sha256'),('route_choices.json','choices_sha256')]:
        if sha(directory/name)!=anchors[key]:raise ValueError('Saved weighted anchors changed')
    rng=np.random.default_rng(cfg.selection_seed)
    jitter=np.random.default_rng(np.random.SeedSequence([cfg.selection_seed,JITTER_TAG]))
    widths=_WIDTH_CACHE.setdefault(task['source_map'],None)
    if widths is None:widths=CorridorWidths(world);_WIDTH_CACHE[task['source_map']]=widths
    counts=Counter()
    for i,(robot,query,item,choice,path) in enumerate(zip(task['robots'],native['queries'],prepared['robots'],choices['robots'],paths)):
        if item!=choice or item['id']!=robot['id'] or query['name']!=robot['id']:raise ValueError('Robot/choice mismatch')
        if (uniform_choices[i]['available']!=len(query['routes']) or
            uniform_choices[i]['selected_index']!=item['baseline_uniform_index'] or
            uniform_paths[i]!=query['routes'][item['baseline_uniform_index']]['path']):
            raise ValueError('Candidate set is inconsistent with the preserved uniform execution')
        measured=[]
        for candidate,stored in zip(query['routes'],item['routes']):
            witness=widths.route(candidate);widths.check_witness(witness)
            if not np.isclose(witness['width'],stored['bottleneck_width'],atol=2e-7,rtol=1e-8):
                raise ValueError('Bottleneck width does not recompute')
            lane=audit_lane_geometry(candidate,robot['radius'],robot['safety_margin'])
            for key in ('split_sections','shared_corridor_sections','transition_sections'):counts[key]+=lane[key]
            counts['candidate_routes']+=1;measured.append(witness['width'])
        selected,p,u=draw_width_route(measured,int(rng.integers(len(measured))),float(jitter.random()))
        if selected!=item['selected_index']:raise ValueError('Width-proportional draw does not replay')
        np.testing.assert_allclose(p,item['probabilities'],atol=1e-12)
        np.testing.assert_allclose(u,item['uniform_variate'],atol=1e-12)
        np.testing.assert_allclose(p[selected],item['selection_probability'],atol=1e-12)
        if path!=query['routes'][selected]['path']:raise ValueError('Keep-right geometry was modified after selection')
    if len(choices['robots'])!=32 or len(prepared['robots'])!=32:raise ValueError('Incomplete weighted fleet')
    with np.load(directory/'trajectory.npz',allow_pickle=False) as d:
        states,controls=d['states'],d['controls']
        np.testing.assert_allclose(states[0,:,:3],[r['start'] for r in task['robots']],atol=1e-10)
        if len(states)!=len(controls)+1 or not np.isfinite(states).all() or not np.isfinite(controls).all():
            raise ValueError('Invalid weighted trace')
        if not validate_execution(task,states,controls)['passed']:raise ValueError('Weighted physics/limit violation')
        collision=disk_collisions(task,world,states)
        if collision['collisions'] or run['collisions']:raise ValueError('Weighted collision')
        _,_,arrivals=completion_data(task,states,controls);success=all(a is not None for a in arrivals)
        if run['success']!=success:raise ValueError('Weighted success differs from saved motion')
        np.testing.assert_allclose(run['completion_fraction'],np.mean([a is not None for a in arrivals]),atol=1e-12)
        if success:np.testing.assert_allclose(run['makespan_seconds'],max(arrivals),atol=1e-10)
        elif run['makespan_seconds'] is not None:raise ValueError('Deadlock has an imputed makespan')
        monitor=TerminalMonitor(task,paths,cfg);terminal=None
        for step,u in enumerate(controls,1):
            terminal,replayed_evidence=monitor.observe(step,states[step],u)
            if terminal:break
        if terminal!=run['status'] or step!=len(controls):raise ValueError('Weighted termination evidence differs')
        if replayed_evidence!=run['termination_evidence']:raise ValueError('Recorded terminal evidence does not replay exactly')
        geometry=audit_deadlock_geometry(task,world,paths,states,run['termination_evidence'],cfg) if terminal=='deadlock' else None
        adjudication=None
        if terminal=='stalled':
            adjudication=certify_visible_join_deadlock(task,world,paths,states,controls,replayed_evidence,cfg)
            adjudication.update(run_sha256=sha(filename),trajectory_sha256=sha(directory/'trajectory.npz'))
            save_json(directory/'terminal_adjudication.json',adjudication)
        identical=paths==uniform_paths
        if identical:
            with np.load(uniform_dir/'trajectory.npz',allow_pickle=False) as base:
                if not np.array_equal(states,base['states']) or not np.array_equal(controls,base['controls']):
                    raise ValueError('Identical anchor inputs did not reproduce the baseline motion')
    return dict(run_id=run['run_id'],passed=True,checked_steps=len(controls),checked_agent_runs=32,
        trajectory_sha256=sha(directory/'trajectory.npz'),run_sha256=sha(filename),
        width_and_lane_counts=dict(counts),deadlock_geometry=geometry,identical_to_uniform_anchors=identical,
        raw_terminal_status=terminal,verified_terminal_outcome='deadlock' if adjudication else terminal,
        terminal_adjudication=adjudication)


def audit_width_results(suite,baseline,output,workers=8):
    suite,baseline,output=map(Path,(suite,baseline,output));contract=baseline_contract(suite,baseline)
    experiment=json.loads((output/'experiment.json').read_text())
    files=sorted(output.glob(f'*__{METHOD}__*/run.json'));manifest=json.loads((suite/'manifest.json').read_text())
    expected={x['id'] for x in manifest['scenarios']}
    runs=[json.loads(p.read_text()) for p in files]
    if len(files)!=250 or {r['task_id'] for r in runs}!=expected or experiment['expected_runs']!=250:
        raise ValueError('Publication requires all 250 weighted tasks exactly once')
    common=runs[0]['code_sha256']
    for name,digest in common.items():
        if sha(ROOT/name)!=digest:raise ValueError(f'Weighted implementation changed during evaluation: {name}')
    for run in runs:
        if run['code_sha256']!=common or run['config']!=asdict(EvaluationConfig()):raise ValueError('Mixed weighted protocols')
        if run['status'] not in ('success','deadlock','stalled'):raise ValueError(f'Unresolved weighted outcome: {run["run_id"]}: {run["status"]}')
    historical=json.loads((baseline/'audit.json').read_text())
    for record in historical['traces']:
        folder=baseline/record['run_id']
        if sha(folder/'run.json')!=record['run_sha256'] or sha(folder/'trajectory.npz')!=record['trajectory_sha256']:
            raise ValueError('A historical baseline execution changed')
    with ProcessPoolExecutor(max_workers=workers) as pool:
        records=list(pool.map(_audit_weighted,[(str(suite),str(baseline),str(output),str(p)) for p in files]))
    report=dict(passed=True,weighted_runs=250,weighted_agent_runs=8000,baseline_runs_rechecked=500,
        baseline_contract=contract,weighted_source_and_binary_hashes=common,
        auditor_sha256=sha(__file__),candidate_routes_checked=sum(r['width_and_lane_counts']['candidate_routes'] for r in records),
        identical_anchor_runs_reproduced=sum(r['identical_to_uniform_anchors'] for r in records),collisions=0,
        raw_terminal_outcomes=dict(Counter(r['raw_terminal_status'] for r in records)),
        verified_terminal_outcomes=dict(Counter(r['verified_terminal_outcome'] for r in records)),
        visible_join_deadlock_certificates=[r['run_id'] for r in records if r['terminal_adjudication']],
        terminal_adjudicator_sha256=sha(ROOT/'benchmarks/multi_agent/warehouse_terminal_adjudication.py'),traces=records)
    save_json(output/'audit.json',report);return report


def all_runs(baseline,output):
    runs=[json.loads(p.read_text()) for root in (Path(baseline),Path(output)) for p in sorted(root.glob('*/run.json'))]
    audit=json.loads((Path(output)/'audit.json').read_text())
    if not audit['passed']:raise ValueError('Comparison requires a passed full weighted audit')
    records={r['run_id']:r for r in audit['traces']}
    for run in runs:
        run['raw_status']=run['status']
        if run['planner']==METHOD:
            record=records[run['run_id']]
            if sha(Path(output)/run['run_id']/'run.json')!=record['run_sha256']:
                raise ValueError('Weighted outcome changed after audit')
            run['status']=record['verified_terminal_outcome']
            run['terminal_adjudication']=bool(record['terminal_adjudication'])
    return runs


def comparison_tables(baseline,output):
    runs=all_runs(baseline,output)
    views=dict(overall=['planner'],by_map=['source_map','planner'],
        by_scenario=['source_map','scenario_id','diameter_meters','traffic_pattern','planner'],
        by_map_size=['source_map','diameter_meters','planner'],by_map_pattern=['source_map','traffic_pattern','planner'])
    tables={key:aggregate_rows(runs,group) for key,group in views.items()}
    for key,rows in tables.items():save_json(Path(output)/f'comparison_{key}.json',rows);write_csv(rows,Path(output)/f'comparison_{key}.csv')
    brief=[{k:r.get(k) for k in ('run_id','task_id','source_map','scenario_id','variant_index','diameter_meters',
        'traffic_pattern','planner','raw_status','status','terminal_adjudication','success','completion_fraction','makespan_seconds','collisions')} for r in runs]
    write_csv(brief,Path(output)/'comparison_runs.csv')
    pairs={r['task_id']:{s['planner']:s for s in runs if s['task_id']==r['task_id']} for r in runs}
    paired=[]
    for source_map in ('warehouse','warehouse2','both'):
        group=[p for p in pairs.values() if source_map=='both' or p[METHOD]['source_map']==source_map]
        for reference in ('rrt','k_routes'):
            both=[p for p in group if p[METHOD]['success'] and p[reference]['success']]
            paired.append(dict(source_map=source_map,reference=reference,tasks=len(group),
                weighted_fleet_success_rate=float(np.mean([p[METHOD]['success'] for p in group])),
                reference_fleet_success_rate=float(np.mean([p[reference]['success'] for p in group])),
                mean_paired_agent_completion_difference=float(np.mean([p[METHOD]['completion_fraction']-p[reference]['completion_fraction'] for p in group])),
                both_successful_tasks=len(both),paired_makespan_difference_seconds=float(np.mean([
                    p[METHOD]['makespan_seconds']-p[reference]['makespan_seconds'] for p in both])) if both else None))
    save_json(Path(output)/'paired_comparisons.json',paired);write_csv(paired,Path(output)/'paired_comparisons.csv')
    save_json(Path(output)/'comparison_summary.json',dict(runs=len(runs),methods=METHODS,tables=tables))
    return runs,tables


def plots(output,tables):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    output=Path(output);sizes=[.6,.7,.8,.9,1.]
    lookup={(r['source_map'],r['diameter_meters'],r['planner']):r for r in tables['by_map_size']}
    colors=['#3278a5','#6a9b58','#df8433']
    for key,title,filename in [('fleet_success_rate','All 32 agents finish (%)','fleet_success_by_size'),
        ('mean_agent_completion_fraction','Individual-agent completion (%)','agent_completion_by_size')]:
        fig,axes=plt.subplots(1,2,figsize=(13,4.8),layout='constrained',sharey=True)
        for ax,name in zip(axes,('warehouse','warehouse2')):
            for index,method in enumerate(METHODS):
                y=[100*lookup[(name,d,method)][key] for d in sizes]
                ax.bar(np.arange(5)+(index-1)*.25,y,.24,color=colors[index],label=LABELS[method])
            ax.set(title=name,xlabel='Agent diameter (m)',xticks=range(5),xticklabels=sizes,ylim=(0,105));ax.grid(axis='y',alpha=.2);ax.set_axisbelow(True)
        axes[0].set_ylabel(title);axes[1].legend(fontsize=8)
        fig.suptitle('25 tasks per size and map · same controller · 1.2 m/s')
        for suffix in ('png','svg'):fig.savefig(output/f'{filename}.{suffix}',dpi=160)
        plt.close(fig)
    fig,axes=plt.subplots(1,2,figsize=(13,4.8),layout='constrained')
    for ax,name in zip(axes,('warehouse','warehouse2')):
        for color,method in zip(colors,METHODS):
            y=[lookup[(name,d,method)]['successful_makespan_mean_seconds'] for d in sizes]
            ax.plot(sizes,[np.nan if v is None else v for v in y],'o-',color=color,label=LABELS[method])
        ax.set(title=name,xlabel='Agent diameter (m)',ylabel='Successful-run makespan only (s)',xticks=sizes);ax.grid(alpha=.2);ax.legend(fontsize=8)
    for suffix in ('png','svg'):fig.savefig(output/f'successful_makespan_by_size.{suffix}',dpi=160)
    plt.close(fig)


def selection_summary(output):
    rows=[];details=[]
    for name in ('warehouse','warehouse2'):
        uniform=[];weighted=[];expected_uniform=[];expected_weighted=[];changed=0;counts=Counter()
        for path in sorted((Path(output)/'selection_inputs').glob(f'{name}_s*.json')):
            record=json.loads(path.read_text())
            for robot in record['robots']:
                widths=np.array([r['bottleneck_width'] for r in robot['routes']])
                uniform.append(widths[robot['baseline_uniform_index']]);weighted.append(widths[robot['selected_index']])
                expected_uniform.append(float(widths.mean()));expected_weighted.append(float(widths@robot['probabilities']))
                if expected_weighted[-1]<expected_uniform[-1]-1e-9:raise ValueError('Expected width did not increase under proportional sampling')
                changed+=robot['selected_index']!=robot['baseline_uniform_index'];counts[robot['available']]+=1
                details.append(dict(task_id=record['task_id'],source_map=name,robot_id=robot['id'],
                    available_routes=robot['available'],baseline_uniform_index=robot['baseline_uniform_index'],
                    selected_index=robot['selected_index'],selection_probability=robot['selection_probability'],
                    selected_bottleneck_width_meters=float(widths[robot['selected_index']]),
                    bottleneck_widths_meters=json.dumps(widths.tolist()),probabilities=json.dumps(robot['probabilities']),
                    uniform_variate=robot['uniform_variate']))
        rows.append(dict(source_map=name,agents=len(uniform),changed_selected_routes=changed,
            uniform_sampled_bottleneck_mean_meters=float(np.mean(uniform)),
            weighted_sampled_bottleneck_mean_meters=float(np.mean(weighted)),
            expected_uniform_bottleneck_mean_meters=float(np.mean(expected_uniform)),
            expected_weighted_bottleneck_mean_meters=float(np.mean(expected_weighted)),
            returned_route_counts=dict(counts)))
    save_json(Path(output)/'selection_summary.json',rows);write_csv(rows,Path(output)/'selection_summary.csv')
    write_csv(details,Path(output)/'weighted_route_choices.csv')
    return rows


def lane_figure(suite,baseline,output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from corridor_graph.traffic_visualization import _map
    chosen={}
    for path in sorted((Path(output)/'selection_inputs').glob('warehouse_s*.json')):
        record=json.loads(path.read_text());native=json.loads((Path(baseline)/f'{record["task_id"]}__k_routes__p42__c42/agents.routes.json').read_text())
        for query,robot in zip(native['queries'],record['robots']):
            route=query['routes'][robot['selected_index']]
            for section in route['segments']:
                key=('forward' if section['forward'] else 'reverse') if section['state']=='SPLIT' else ('shared' if section['state']=='SHARED' and section['kind']=='corridor' else None)
                if key and key not in chosen and shapely.LineString(section['reference_geometry']).length>2.:
                    chosen[key]=(record['task_id'],query['name'],section)
        if len(chosen)==3:break
    fig,axes=plt.subplots(1,3,figsize=(15,4.8),layout='constrained')
    titles=dict(forward='Forward: travel-right lane',reverse='Reverse: travel-right lane',shared='Shared: reference centerline')
    for ax,key in zip(axes,('forward','reverse','shared')):
        task_id,robot,section=chosen[key];_,world,_=load_task(suite,task_id);_map(ax,world)
        reference=np.asarray(section['reference_geometry']);geometry=np.asarray(section['geometry']);line=shapely.LineString(reference)
        center=np.array(line.interpolate(.5*line.length).coords)[0];ax.plot(*reference.T,'k--',lw=1.5,label='Physical reference')
        ax.plot(*geometry.T,color='#276ea8',lw=2,label='Selected anchor')
        if section['state']=='SPLIT':
            opposite=np.asarray(section['opposing_geometry']);ax.plot(*opposite.T,':',color='#d68130',lw=1.5,label='Opposing lane')
        index=int(np.argmin(np.linalg.norm(geometry-center,axis=1)));index=min(index,len(geometry)-2)
        direction=geometry[index+1]-geometry[index];direction=direction/max(np.linalg.norm(direction),1e-12)*.7
        ax.annotate('',geometry[index]+direction,geometry[index],arrowprops=dict(arrowstyle='->',color='#276ea8',lw=2))
        ax.set(xlim=(center[0]-3.5,center[0]+3.5),ylim=(center[1]-3.5,center[1]+3.5),xlabel='x (m)',ylabel='y (m)',title=titles[key]+'\n'+task_id+' / '+robot)
        ax.legend(fontsize=7)
    fig.suptitle('Actual selected native geometry · direction-relative right side · validated center fallback')
    for suffix in ('png','svg'):fig.savefig(Path(output)/f'lane_policy.{suffix}',dpi=150)
    plt.close(fig)


def render_width_examples(suite,baseline,output):
    from .visualization import animate_run
    output=Path(output);target=output/'animations';target.mkdir(exist_ok=True)
    runs=[json.loads(p.read_text()) for p in sorted(output.glob(f'*__{METHOD}__*/run.json'))];selected=[]
    for name in ('warehouse','warehouse2'):
        for pattern in ('distributed','opposing_bottleneck','intersection','symmetric_swap','competing_bottlenecks'):
            group=[r for r in runs if r['source_map']==name and r['traffic_pattern']==pattern]
            if group:selected.append(next((r for r in group if r['success']),group[0]))
    index=[]
    for run in selected:
        task,world,_=load_task(suite,run['task_id']);filename=target/f'{run["run_id"]}__{run["status"]}.gif'
        if not filename.exists():
            with np.load(output/run['run_id']/'trajectory.npz') as data:states=data['states']
            groups=(run.get('termination_evidence') or {}).get('mutual_obstruction_groups',[])
            focus=(states[-1,groups[0],:2].mean(axis=0),4.) if groups else None
            anchors=json.loads((output/run['run_id']/'paths.json').read_text())
            animate_run(task,world,states,filename,result=run,frame_count=64,focus=focus,anchor_paths=anchors)
        index.append(dict(source_map=run['source_map'],traffic_pattern=run['traffic_pattern'],
            run_id=run['run_id'],status=run['status'],file=filename.name))
        print('Recorded weighted rollout:',filename,flush=True)
    save_json(target/'index.json',index)
    lines=['# Recorded width-weighted K-route motion','','Actual saved poses; playback compresses simulated time. Dashed lines are the selected keep-right anchors; solid traces are actual motion. Examples select the first success for each map/traffic-pattern, or the first saved run when no success exists. All tasks remain in the reported denominator.','',
        '| Map | Traffic pattern | Outcome | GIF |','| --- | --- | --- | --- |']
    lines += [f'| {r["source_map"]} | {r["traffic_pattern"]} | {r["status"]} | [animation]({r["file"]}) |' for r in index]
    (target/'README.md').write_text('\n'.join(lines)+'\n')


def write_width_report(output,tables):
    output=Path(output);preflight=json.loads((output/'selection_preflight.json').read_text())
    audit=json.loads((output/'audit.json').read_text())
    lines=['# Bottleneck-width-proportional K-route benchmark','',
        '**250 new weighted-selection runs, compared with the same 250 uniform-K and 250 RRT tasks.** Each task has 32 equal-sized disk robots. Every scenario average contains five start–goal variants.','',
        'The candidate sets are exactly the frozen keep-right sets used by the uniform baseline: requested K=5, with the actual available count recorded per agent. Only selection changes: **p_i = w_i / sum_j w_j**. One path is sampled once per agent and held for the run.','',
        WIDTH_DEFINITION+' Polygon-vertex events and corridor endpoints are checked, including bottlenecks between reference vertices. Independent GEOS checks verify the reported wall contacts. Widths are not the clearance of the displaced right-side lane.','',
        'Both policies use seed 42. The weighted draw couples to the original uniform draw using an independent within-bin jitter, followed by inverse-CDF sampling. This has the requested width-proportional marginal probabilities and reproduces the original choice when widths are equal.','',
        '**Normal MPPI/ORCA**, not the uncertainty-aware controller, is used with the identical native binary and controller profiles: maximum forward speed 1.2 m/s, dt=0.1 s, 128 rollouts, 31 horizon states, the same shared braking safety filter and waiting policy. Observation noise is zero. Original dynamics, acceleration and yaw limits are retained. The map calibration is 1 native map unit = 1 m.','',
        'Historical baseline source versions are preserved; adding the anchor-provider interface does not change simulation, controller or stopping logic. The baseline execution checksums and unchanged controller/dynamics are verified separately from the new weighted source identity. Native candidate-generation timings are reused and explicitly recorded; makespan is simulated task-completion time.','',
        '## Right-side lane placement','',
        'On certified **SPLIT** corridor sections the anchor is strictly on the travel-right side; reverse travel swaps physical sides. `LeftOneWay` denotes reverse travel relative to canonical graph orientation and still means travel-right. On **SHARED** corridor sections the anchor equals the reference centerline. Smooth transitions and start/goal/junction connectors join these sections. The preferred offset uses half of the available footprint-safe displacement on that side, with bounded adjustments for corners, clearance and connections; it is not always exactly one quarter of the raw wall-to-wall width.','',
        'Splitting requires a certified pair of opposing, continuously clear lanes and valid connections. Conservative certification can fall back to center even where a different geometric construction might fit. These statements concern planned anchors; MPPI may leave an anchor during avoidance.','',
        f"All **{preflight['candidate_routes']:,} candidate paths** passed the independent right-side/shared-center geometry audit. Native wide-straight, reverse-direction and narrow-center tests also pass.",'',
        '![Actual selected right-side and center anchor geometry](lane_policy.png)','',
        '## Fleet success and individual completion','',
        'Fleet success requires all 32 agents physically stopped in their goal regions, valid dynamics/limits and zero continuous footprint collisions. Agent completion counts each finished robot, including runs where others deadlock. Failed makespans are undefined; means below use successful runs only.','',
        f"Raw weighted terminal labels: **{audit['raw_terminal_outcomes']}**. Verified outcomes: **{audit['verified_terminal_outcomes']}**. {len(audit['visible_join_deadlock_certificates'])} raw stalled run(s) required an additional visible-join blocking certificate: the nearest geometric anchor projection lies behind shelving, while a footprint-clear rejoin feeds persistent traffic obstruction. The raw labels, states, controls, success flags and completion counts remain unchanged. Every unfinished robot must be covered by an independently checked blocking cycle, a parked-goal blocker, or a chain into either. `comparison_runs.csv` retains both raw and verified labels; certificates are included in the audit. Uncertified stalls reject publication.",'',
        '![Recorded state and independently checked visible rejoin](visible_join_deadlock.png)' if audit['visible_join_deadlock_certificates'] else '', '',
        '| Method | Tasks | Fleet successes | Fleet success | Agent completion | Successful makespan (s) |',
        '| --- | ---: | ---: | ---: | ---: | --- |']
    for method in METHODS:
        r=next(x for x in tables['overall'] if x['planner']==method)
        lines.append(f'| {LABELS[method]} | {r["runs"]} | {r["successes"]} | {r["fleet_success_rate"]:.1%} | {r["mean_agent_completion_fraction"]:.1%} | {format_time(r)} |')
    lines+=['','![Fleet success by size](fleet_success_by_size.png)','','![Agent completion by size](agent_completion_by_size.png)','','![Conditional makespan](successful_makespan_by_size.png)','']
    for title,key,labels in [('Per map','by_map',['source_map']),('Across traffic patterns, per size and map','by_map_size',['source_map','diameter_meters']),('Across sizes, per traffic pattern and map','by_map_pattern',['source_map','traffic_pattern'])]:
        lines += [f'## {title}','','| Cohort | Method | Runs | Fleet success | Agent completion | Successful makespan (s) |','| --- | --- | ---: | ---: | ---: | --- |']
        for r in tables[key]:
            label=' / '.join(str(r[k]) for k in labels)
            lines.append(f'| {label} | {LABELS[r["planner"]]} | {r["runs"]} | {r["successes"]}/{r["runs"]} ({r["fleet_success_rate"]:.1%}) | {r["mean_agent_completion_fraction"]:.1%} | {format_time(r)} |')
        lines+=['']
    lines+=['## Evidence and data','',
        '- [Five-variant scenario averages](comparison_by_scenario.csv)',
        '- [Per-map averages](comparison_by_map.csv)',
        '- [Size averages](comparison_by_map_size.csv)',
        '- [Traffic-pattern averages](comparison_by_map_pattern.csv)',
        '- [Paired task comparisons](paired_comparisons.csv)',
        '- [Sampled and expected bottleneck widths](selection_summary.csv)',
        '- [All 8,000 weighted route draws, bottleneck widths and probabilities](weighted_route_choices.csv)',
        '- [All 750 planner–task outcomes](comparison_runs.csv)',
        '- [Independent weighted trace, width, probability and lane audit](audit.json)',
        '- [Width and lane preflight](selection_preflight.json)',
        '- [Actual recorded weighted GIFs](animations/README.md)','',
        'Paired makespan differences use only tasks on which both compared methods finished; the sample count is recorded. Baseline traces are preserved and their prior audit checksums reverified. New raw states, controls, route probabilities and width witnesses remain under `benchmarks/results/warehouse_bottleneck_width/`.','',
        '```bash','.venv/bin/python benchmarks/multi_agent_bench.py evaluate-warehouse-width --workers 20','```','']
    (output/'REPORT.md').write_text('\n'.join(lines))


def publish_width_results(suite,baseline,output):
    suite,baseline,output=map(Path,(suite,baseline,output))
    audit_width_results(suite,baseline,output)
    runs,tables=comparison_tables(baseline,output)
    if len(runs)!=750:raise ValueError('Comparison must contain 750 planner-task outcomes')
    selection_summary(output);plots(output,tables);lane_figure(suite,baseline,output)
    render_width_examples(suite,baseline,output);write_width_report(output,tables)
    target=ROOT/'benchmarks/scenarios/warehouse_uniform/width_evaluation';target.mkdir(exist_ok=True)
    for path in output.iterdir():
        if path.is_file() and path.suffix in ('.json','.csv','.png','.svg','.md'):shutil.copyfile(path,target/path.name)
    shutil.copytree(output/'animations',target/'animations',dirs_exist_ok=True)
    primary=ROOT/'benchmarks/WAREHOUSE_BOTTLENECK_RESULTS.md'
    import re
    def relocate(match):
        value=match.group(1)
        return match.group(0) if value.startswith(('http:','https:','#')) else ']('+os.path.relpath(target/value,primary.parent)+')'
    primary.write_text(re.sub(r'\]\(([^)]+)\)',relocate,(target/'REPORT.md').read_text()))
    return primary
