"""Audit and compare the five open-candidate warehouse case executions."""
from dataclasses import asdict
import gzip
import json
from pathlib import Path
import numpy as np

from .geometry import ROOT,fingerprint
from .warehouse_evaluation import EvaluationConfig,load_task,save_json,write_csv
from .warehouse_disappear_evaluation import DisappearTerminalMonitor,disappear_disk_collisions
from .warehouse_disappear_report import adjudicate_stall
from .warehouse_evaluation_report import audit_deadlock_geometry
from .warehouse_preflight import anchor_identity,check_paths,check_joint_certificate
from .warehouse_width_benchmark import sha
from .metrics import completion_data,validate_execution
from .bottleneck_selection import CorridorWidths,audit_lane_geometry
from .warehouse_open_routes_case import TASK,SUITE,OLD,METHODS

LABELS={'rrt':'RRT-Connect','k_routes':'K routes, uniform','k_routes_width':'K routes, bottleneck',
    'k_routes_edge_cost':'K routes, lambda=1, tau=1','k_routes_edge_cost_l2_t10':'K routes, lambda=2, tau=10'}


def audit_selection(output,method,task,world,experiment):
    prepared_file=output/f'selection_inputs/{method}.json'
    if sha(prepared_file)!=experiment['selection_inputs'][method]['sha256']:raise ValueError('Prepared inputs changed')
    prepared=json.loads(prepared_file.read_text());directory=output/f'{TASK}__{method}__p42__c42'
    run=json.loads((directory/'run.json').read_text())
    if sha(prepared_file)!=run['protocol_inputs']['selection_inputs_sha256']:raise ValueError('Selection identity differs')
    paths=json.loads((directory/'paths.json').read_text());choices=json.loads((directory/'route_choices.json').read_text())
    if paths!=prepared['paths'] or choices!=prepared['choices']:raise ValueError('Executed anchors differ')
    if run['anchor_validation']['identity']!=anchor_identity(task,method,42,5):raise ValueError('Wrong anchor identity')
    for name,key in [('paths.json','paths_sha256'),('route_choices.json','choices_sha256')]:
        if sha(directory/name)!=run['anchor_validation'][key]:raise ValueError('Anchors changed')
    check_paths(task,world,paths)
    if method=='rrt':
        if paths!=json.loads((OLD/f'{TASK}__rrt__p42__c42/paths.json').read_text()):raise ValueError('RRT anchors differ')
        return prepared
    pool=json.loads((output/'candidates.routes.json').read_text())
    if sha(output/'candidates.routes.json')!=experiment['identity']['candidate_pool_sha256']:raise ValueError('Pool changed')
    graph=json.loads(gzip.decompress((SUITE/'geometry_cache/warehouse2.resources.json.gz').read_bytes()))
    edges={e['id']:e for e in graph['routing_geometry']['edges']};measure=CorridorWidths(world)
    common_file=ROOT/f'benchmarks/results/warehouse_bottleneck_width/selection_inputs/{TASK}.json'
    if sha(common_file)!=experiment['identity']['common_random_inputs_sha256']:raise ValueError('Common variates changed')
    common=json.loads(common_file.read_text())
    for robot,query,item,old,path in zip(task['robots'],pool['queries'],choices['robots'],common['robots'],paths):
        if item['id']!=robot['id'] or item['available']!=5 or len(query['routes'])!=5:raise ValueError('Incomplete candidate set')
        if item['uniform_variate']!=old['uniform_variate']:raise ValueError('Common random number differs')
        widths=[];costs=[]
        all_widths=[edges[e]['min_width'] for route in query['routes'] for e in route['edge_ids']]
        maximum=np.longdouble(max(all_widths)) if all_widths else None
        for route,saved in zip(query['routes'],item['routes']):
            if fingerprint(route['path'])!=saved['path_sha256']:raise ValueError('Native geometry differs')
            lane=audit_lane_geometry(route,robot['radius'],robot['safety_margin'])
            if lane!=saved['lane_audit']:raise ValueError('Lane certificate differs')
            witness=measure.route(route);measure.check_witness(witness);widths.append(witness['width'])
            np.testing.assert_allclose(witness['width'],saved['bottleneck_width'],rtol=1e-12,atol=1e-10)
            lengths=np.array([edges[e]['length'] for e in route['edge_ids']],dtype=np.longdouble)
            ew=np.array([edges[e]['min_width'] for e in route['edge_ids']],dtype=np.longdouble)
            q=np.clip(1-(ew-2*np.longdouble(robot['radius']))/(2*maximum),0,1) if len(ew) else np.array([],dtype=np.longdouble)
            weight=1 if method=='k_routes_edge_cost' else 2
            costs.append(lengths.sum()+weight*np.dot(lengths,q))
        if method=='k_routes':probabilities=np.repeat(np.longdouble(1)/5,5)
        elif method=='k_routes_width':
            w=np.array(widths,dtype=np.longdouble);probabilities=w/w.sum()
        else:
            costs=np.array(costs);temperature=1 if method=='k_routes_edge_cost' else 10
            w=np.exp(-(costs-costs.min())/temperature);probabilities=w/w.sum()
            np.testing.assert_allclose(costs.astype(float),[c['cost'] for c in item['distribution']['routes']],rtol=1e-12,atol=1e-11)
        np.testing.assert_allclose(probabilities.astype(float),item['probabilities'],rtol=1e-10,atol=1e-13)
        cumulative=np.cumsum(item['probabilities']);cumulative[-1]=1.
        selected=int(np.searchsorted(cumulative,item['uniform_variate'],side='right'))
        if selected!=item['selected_index'] or path!=query['routes'][selected]['path']:raise ValueError('Route draw differs')
    return prepared


def audit_trace(output,method,experiment):
    task,world,task_hash=load_task(SUITE,TASK);directory=output/f'{TASK}__{method}__p42__c42'
    run=json.loads((directory/'run.json').read_text());old=json.loads((OLD/directory.name/'run.json').read_text())
    if run['config']!=asdict(EvaluationConfig()) or run['controller_profiles']!=old['controller_profiles']:
        raise ValueError('Controller or physics configuration differs')
    if run['scenario_sha256']!=task_hash or run['joint_feasibility_certificate']!=check_joint_certificate(SUITE,task):
        raise ValueError('Task or feasibility proof differs')
    identity=fingerprint(dict(task=task_hash,config=run['config'],code=run['code_sha256'],planner=method,protocol_inputs=run['protocol_inputs']))
    if identity!=run['evaluation_identity']:raise ValueError('Invalid execution identity')
    if sha(output/'experiment.json')!=run['protocol_inputs']['experiment_sha256']:raise ValueError('Experiment changed')
    prepared=audit_selection(output,method,task,world,experiment);paths=prepared['paths'];cfg=EvaluationConfig(**run['config'])
    with np.load(directory/'trajectory.npz',allow_pickle=False) as data:states,controls,active=data['states'],data['controls'],data['active']
    if len(states)!=len(controls)+1 or active.shape!=states.shape[:2] or active.dtype!=np.bool_:raise ValueError('Bad trace shape')
    if not np.isfinite(states).all() or not np.isfinite(controls).all() or not active[0].all():raise ValueError('Bad trace')
    np.testing.assert_allclose(states[0,:,:3],[r['start'] for r in task['robots']],atol=1e-10)
    if not validate_execution(task,states,controls)['passed']:raise ValueError('Physics violation')
    goals=np.array([r['goal']['pose'][:2] for r in task['robots']]);tolerance=np.array([r['goal']['radius'] for r in task['robots']])
    settled=(np.linalg.norm(states[1:,:,:2]-goals,axis=2)<=tolerance+1e-8)&(np.max(abs(controls),axis=2)<=1e-8)
    expected=np.vstack([np.ones(32,dtype=bool),~np.logical_or.accumulate(settled,axis=0)])
    if not np.array_equal(expected,active):raise ValueError('Removal timing differs')
    _,_,arrivals=completion_data(task,states,controls)
    if arrivals!=run['individual_arrival_seconds'] or arrivals!=run['removal_seconds']:raise ValueError('Measured arrivals differ')
    for i in range(32):
        removed=np.flatnonzero(~active[:,i])
        if len(removed):
            j=removed[0]
            if np.any(controls[j:,i]!=0) or not np.array_equal(states[j:,i],np.broadcast_to(states[j,i],states[j:,i].shape)):
                raise ValueError('Removed snapshot changed')
    collision=disappear_disk_collisions(task,world,states,active)
    if collision['collisions'] or run['collisions']:raise ValueError('Collision while present')
    success=not active[-1].any()
    if run['success']!=success:raise ValueError('Wrong fleet success')
    np.testing.assert_allclose(run['completion_fraction'],np.mean(~active[-1]),atol=1e-12)
    if success:np.testing.assert_allclose(run['makespan_seconds'],max(arrivals),atol=1e-10)
    elif run['makespan_seconds'] is not None:raise ValueError('Failed makespan imputed')
    monitor=DisappearTerminalMonitor(task,paths,cfg);terminal=None
    for step,control in enumerate(controls,1):
        terminal,evidence=monitor.observe(step,states[step],control,active[step])
        if terminal:break
    if terminal!=run['status'] or step!=len(controls) or evidence!=run['termination_evidence']:raise ValueError('Terminal evidence differs')
    if any(not active[-1,i] or not active[-1,j] for i,j in evidence.get('obstruction_edges',[])):raise ValueError('Removed blocker')
    geometry=audit_deadlock_geometry(task,world,paths,states,evidence,cfg) if terminal=='deadlock' else None
    adjudication=None;unresolved=None
    if terminal=='stalled':
        try:adjudication=adjudicate_stall(task,world,paths,states,controls,evidence,cfg,active)
        except ValueError as error:unresolved=str(error)
    previous_paths=json.loads((OLD/directory.name/'paths.json').read_text())
    same_anchors=paths==previous_paths
    repeated_rrt=False
    if same_anchors:
        with np.load(OLD/directory.name/'trajectory.npz',allow_pickle=False) as reference:
            if not all(np.array_equal(actual,reference[key]) for actual,key in [(states,'states'),(controls,'controls'),(active,'active')]):
                raise ValueError('Identical anchors did not reproduce the preserved trace')
        repeated_rrt=method=='rrt'
    return dict(run_id=run['run_id'],passed=True,run_sha256=sha(directory/'run.json'),trajectory_sha256=sha(directory/'trajectory.npz'),
        raw_status=run['status'],status='deadlock' if adjudication else run['status'],collisions=0,
        checked_steps=len(controls),checked_agent_runs=32,deadlock_geometry=geometry,
        terminal_adjudication=adjudication,unresolved_stall_reason=unresolved,rrt_bitwise_reproduced=repeated_rrt,identical_old_anchors=same_anchors,
        identical_anchor_motion_reproduced=same_anchors,changed_anchor_count=sum(a!=b for a,b in zip(paths,previous_paths)))


def publish(output):
    output=Path(output);experiment=json.loads((output/'experiment.json').read_text());records=[]
    for method in METHODS:
        record=audit_trace(output,method,experiment);records.append(record)
        print('Audited',method,record['status'],flush=True)
    common=json.loads((output/f'{TASK}__rrt__p42__c42/run.json').read_text())['code_sha256']
    for name,digest in common.items():
        if sha(ROOT/name)!=digest:raise ValueError('Executed source changed: '+name)
    rows=[]
    for method,record in zip(METHODS,records):
        run=json.loads((output/f'{TASK}__{method}__p42__c42/run.json').read_text());old=json.loads((OLD/f'{TASK}__{method}__p42__c42/run.json').read_text())
        if run['code_sha256']!=common:raise ValueError('Mixed executed sources')
        rows.append(dict(planner=method,old_finished_agents=round(32*old['completion_fraction']),old_status=old['status'],old_makespan_seconds=old['makespan_seconds'],
            finished_agents=round(32*run['completion_fraction']),completion_fraction=run['completion_fraction'],status=record['status'],raw_status=record['raw_status'],
            changed_anchor_count=record['changed_anchor_count'],identical_anchor_motion_reproduced=record['identical_anchor_motion_reproduced'],
            makespan_seconds=run['makespan_seconds'],simulated_seconds=run['simulated_seconds'],collisions=run['collisions'],control_limits_valid=run['control_limits_valid']))
    audit=dict(passed=True,runs=5,checked_agent_runs=160,collisions=0,source_and_binary_hashes=common,auditor_sha256=sha(__file__),traces=records)
    save_json(output/'audit.json',audit);save_json(output/'comparison.json',rows);write_csv(rows,output/'comparison.csv')
    target=SUITE/f'disappear_evaluation/open_routes/{TASK}';target.mkdir(parents=True,exist_ok=True)
    for name in ['audit.json','comparison.json','comparison.csv','experiment.json']:save_json(target/name,json.loads((output/name).read_text())) if name.endswith('.json') else (target/name).write_bytes((output/name).read_bytes())
    table=['| Method | Old finished | New finished | New result | New makespan |','| --- | ---: | ---: | --- | ---: |']
    for row in rows:
        seconds='undefined' if row['makespan_seconds'] is None else f'{row["makespan_seconds"]:.1f} s'
        table.append(f'| {LABELS[row["planner"]]} | {row["old_finished_agents"]}/32 | {row["finished_agents"]}/32 | {row["status"]} | {seconds} |')
    text='\n'.join(table)
    lines=[f'# Open-route case: {TASK}','',
        'Five methods ran concurrently, one execution per method. The same frozen 32-agent task, normal MPPI/ORCA, 1.2 m/s bound, original bounded physics, safety filter, seed 42 and disappear-after-settled-target policy are retained.','',
        'All four K methods use the same 160 native candidates: K=5 for every agent, max_candidates=128, and no binding length-ratio filter. The existing binary uses a numerical cap of 1e12; the recorded conservative bound on every possible loop-free graph walk proves this cap cannot discard a candidate. RRT uses its original independent static-map anchors and is physically rerun as a control.','',
        'The original per-agent uniform random numbers are reused across the old and new route sets. Uniform sampling selects floor(5U); bottleneck and both graph-edge-cost methods use their own probability CDF with the same U. Graph-edge cost retains full stored edge lengths/min_width, excludes connectors, and uses physical radius 0.3 m.','',
        text,'',
        'All five traces passed independent checks of geometry, sampling probabilities, original differential-drive equations, rate bounds, presence masks, collisions while present, arrivals and terminal evidence. The new RRT states, controls and presence mask reproduce the old disappearance run bit for bit. Makespans are defined only for all-32 successful executions. This is one seed on one task; no success rate across tasks or seeds is inferred.','',
        '[Audit](audit.json). [All metrics](comparison.csv). [Frozen experiment](experiment.json).','',
        'Reproduce or resume: `OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 PYTHONPATH=benchmarks .venv/bin/python -m multi_agent.warehouse_open_routes_case --workers 5`.','']
    if any(r['unresolved_stall_reason'] for r in records):lines+=['Some raw stalls could not be certified as mutual deadlock and remain failures; see audit.json.','']
    if (output/'selection_diagnostics.json').exists():
        save_json(target/'selection_diagnostics.json',json.loads((output/'selection_diagnostics.json').read_text()))
        lines += ['## Selected paths','',
            'Uniform sampling changed 14 anchors, bottleneck sampling changed 16. Both edge-cost parameterisations selected exactly the same 32 anchors as before; those complete traces, as well as RRT, must reproduce the previous motions bit for bit. Availability of wider alternatives does not force the cost distribution to choose them.','',
            '[Selected-path lengths and bottleneck crossing IDs](selection_diagnostics.json).','']
    (target/'REPORT.md').write_text('\n'.join(lines))
    review=SUITE/'evaluation/REVIEW.md';content=review.read_text();marker='\n## Open-route rerun: warehouse2_s02_v02\n'
    block=marker+'\n'+text+'\n\nSame task and controller; new K=5 pool per agent, no binding length cutoff, 128 candidate attempts, paired random numbers. One execution per method.\n\n[Complete report and audit](../disappear_evaluation/open_routes/warehouse2_s02_v02/REPORT.md).\n'
    if marker not in content:
        anchor='\n## Disappear-at-target benchmark\n'
        if anchor in content:content=content.replace(anchor,block+anchor,1)
        else:content+=block
        review.write_text(content)
    save_json(target/'verification.json',dict(passed=True,runs=5,agent_executions=160,auditor_sha256=sha(__file__),
        source_and_binary_hashes=common,artifacts={p.name:sha(p) for p in target.iterdir() if p.is_file() and p.name!='verification.json'}))
    print(text,flush=True);print('Published',target/'REPORT.md',flush=True)
    return rows
