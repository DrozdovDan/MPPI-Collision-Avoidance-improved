"""Audit parameterised edge-cost runs and preserve every previous benchmark."""

from collections import Counter
from concurrent.futures import ProcessPoolExecutor, as_completed
from dataclasses import asdict
import gzip
import json
from pathlib import Path
import re

import numpy as np

from .geometry import ROOT, fingerprint
from .metrics import completion_data, validate_execution
from .warehouse_evaluation import (EvaluationConfig, TerminalMonitor, aggregate_rows,
    disk_collisions, load_task, save_json, write_csv)
from .warehouse_evaluation_report import audit_deadlock_geometry, format_time
from .warehouse_preflight import anchor_identity, check_joint_certificate, check_paths
from .warehouse_terminal_adjudication import certify_visible_join_deadlock
from .warehouse_width_benchmark import baseline_contract, sha


def audit_trace(arguments):
    suite, baseline, output, filename = map(Path, arguments)
    run = json.loads(filename.read_text()); directory = filename.parent
    experiment = json.loads((output/'experiment.json').read_text())
    METHOD = experiment['method']
    LAMBDA, TAU = experiment['lambda_weight'], experiment['tau']
    if (run['planner'] != METHOD or run['protocol_inputs']['lambda_weight'] != LAMBDA
            or run['protocol_inputs']['tau'] != TAU):
        raise ValueError('Run parameters differ from the frozen experiment')
    task, world, task_hash = load_task(suite, run['task_id']); cfg = EvaluationConfig(**run['config'])
    if run['scenario_sha256'] != task_hash or fingerprint(dict(task=task_hash, config=run['config'],
            code=run['code_sha256'], planner=METHOD, protocol_inputs=run['protocol_inputs'])) != run['evaluation_identity']:
        raise ValueError('Edge-cost evaluation identity differs')
    if run['joint_feasibility_certificate'] != check_joint_certificate(suite, task):
        raise ValueError('Joint feasibility proof differs')
    old_dir = baseline/f'{task["id"]}__k_routes__p42__c42'
    for method in ('rrt','k_routes'):
        reference = json.loads((baseline/f'{task["id"]}__{method}__p42__c42/run.json').read_text())
        if (reference['controller_profiles'] != run['controller_profiles'] or reference['config'] != run['config']
                or reference['scenario_sha256'] != run['scenario_sha256']):
            raise ValueError('Controller, configuration or assignments differ from baseline')
    record_file = output/f'selection_inputs/{task["id"]}.json'
    if sha(record_file) != run['protocol_inputs']['selection_inputs_sha256']:
        raise ValueError('Prepared selection inputs changed')
    prepared = json.loads(record_file.read_text())
    preflight = json.loads((output/'selection_preflight.json').read_text())
    reference_file = Path(preflight['previous_directory'])/f'selection_inputs/{task["id"]}.json'
    if sha(reference_file) != prepared['reference_inputs_sha256']:
        raise ValueError('Common random variates changed')
    reference_data = json.loads(reference_file.read_text())
    reference_choices = reference_data['robots']
    for current, reference in zip(prepared['robots'], reference_choices):
        for key in ('id','uniform_variate','baseline_uniform_index','jitter'):
            if current[key] != reference[key]:
                raise ValueError('Selection did not reuse the common seed-42 variates')
    if prepared['lambda_weight'] != LAMBDA or prepared['tau'] != TAU:
        raise ValueError('Edge-cost parameters differ')
    native_file = old_dir/'agents.routes.json'
    if sha(native_file) != prepared['native_candidates_sha256']:
        raise ValueError('Frozen native candidates changed')
    native = json.loads(native_file.read_text())
    graph_file = suite/f'geometry_cache/{task["source_map"]}.resources.json.gz'
    if sha(graph_file) != prepared['graph_bundle_sha256']:
        raise ValueError('Graph edges changed')
    graph = json.loads(gzip.decompress(graph_file.read_bytes()))
    edges = {edge['id']: edge for edge in graph['routing_geometry']['edges']}
    paths = json.loads((directory/'paths.json').read_text()); check_paths(task,world,paths)
    choices = json.loads((directory/'route_choices.json').read_text())
    anchors = run['anchor_validation']
    if anchors['identity'] != anchor_identity(task,METHOD,cfg.selection_seed,cfg.k):
        raise ValueError('Anchor identity differs')
    for name,key in (('paths.json','paths_sha256'),('route_choices.json','choices_sha256')):
        if sha(directory/name) != anchors[key]:
            raise ValueError('Saved anchors changed')
    if paths != prepared['paths'] or choices['robots'] != prepared['robots'] or len(paths) != 32:
        raise ValueError('Prepared inputs differ from executed anchors')
    candidate_count = 0
    for robot,query,item,path in zip(task['robots'],native['queries'],prepared['robots'],paths):
        if robot['id'] != query['name'] or item['id'] != robot['id']:
            raise ValueError('Robot mapping differs')
        all_widths = [edges[e]['min_width'] for r in query['routes'] for e in r['edge_ids']]
        w_max = np.longdouble(max(all_widths)) if all_widths else None
        costs = []
        for route,stored in zip(query['routes'],item['routes']):
            lengths = np.array([edges[e]['length'] for e in route['edge_ids']],dtype=np.longdouble)
            widths = np.array([edges[e]['min_width'] for e in route['edge_ids']],dtype=np.longdouble)
            q = np.clip(1-(widths-2*np.longdouble(robot['radius']))/(2*w_max),0,1) if len(widths) else np.array([],dtype=np.longdouble)
            cost = lengths.sum()+LAMBDA*np.dot(lengths,q)
            np.testing.assert_allclose(stored['cost'],float(cost),rtol=1e-12,atol=1e-12)
            if fingerprint(route['path']) != stored['path_sha256']:
                raise ValueError('Keep-right candidate geometry changed')
            costs.append(cost); candidate_count += 1
        if len(costs) != item['available']:
            raise ValueError('Candidate count differs')
        costs = np.array(costs); p = np.exp(-(costs-costs.min())/TAU); p /= p.sum()
        np.testing.assert_allclose(p.astype(float),item['probabilities'],rtol=1e-10,atol=1e-13)
        # Use the recorded float probability CDF for exact boundary reproduction;
        # the probabilities were independently checked above at higher precision.
        cumulative = np.cumsum(item['probabilities']); cumulative[-1] = 1.
        selected = int(np.searchsorted(cumulative,item['uniform_variate'],side='right'))
        if selected != item['selected_index'] or path != query['routes'][selected]['path']:
            raise ValueError('Sampled route or geometry differs')
    with np.load(directory/'trajectory.npz',allow_pickle=False) as data:
        states,controls = data['states'],data['controls']
    np.testing.assert_allclose(states[0,:,:3],[r['start'] for r in task['robots']],atol=1e-10)
    if len(states) != len(controls)+1 or not np.isfinite(states).all() or not np.isfinite(controls).all():
        raise ValueError('Invalid saved trace')
    if not validate_execution(task,states,controls)['passed']:
        raise ValueError('Physics or motion-limit violation')
    if disk_collisions(task,world,states)['collisions'] or run['collisions']:
        raise ValueError('Collision in edge-cost run')
    _,_,arrivals = completion_data(task,states,controls)
    success = all(a is not None for a in arrivals)
    if run['success'] != success or run['individual_arrival_seconds'] != arrivals:
        raise ValueError('Success or arrivals differ from recorded motion')
    np.testing.assert_allclose(run['completion_fraction'],np.mean([a is not None for a in arrivals]),atol=1e-12)
    if success:
        np.testing.assert_allclose(run['makespan_seconds'],max(arrivals),atol=1e-10)
    elif run['makespan_seconds'] is not None:
        raise ValueError('Failed run has an imputed makespan')
    monitor = TerminalMonitor(task,paths,cfg); terminal = None
    for step,control in enumerate(controls,1):
        terminal,evidence = monitor.observe(step,states[step],control)
        if terminal:
            break
    if terminal != run['status'] or step != len(controls) or evidence != run['termination_evidence']:
        raise ValueError('Terminal evidence does not replay exactly')
    geometry = audit_deadlock_geometry(task,world,paths,states,evidence,cfg) if terminal=='deadlock' else None
    adjudication,unresolved = None,None
    if terminal == 'stalled':
        try:
            adjudication = certify_visible_join_deadlock(task,world,paths,states,controls,evidence,cfg)
        except ValueError as error:
            unresolved = str(error)
        if adjudication:
            adjudication.update(run_sha256=sha(filename),trajectory_sha256=sha(directory/'trajectory.npz'))
            save_json(directory/'terminal_adjudication.json',adjudication)
    identical = paths == json.loads((old_dir/'paths.json').read_text())
    if identical:
        with np.load(old_dir/'trajectory.npz',allow_pickle=False) as old:
            if not np.array_equal(states,old['states']) or not np.array_equal(controls,old['controls']):
                raise ValueError('Identical anchors failed to reproduce baseline motion')
    same_reference = paths == reference_data['paths']
    if same_reference:
        reference_root = Path(preflight['previous_directory'])
        reference_method = json.loads((reference_root/'experiment.json').read_text())['method']
        reference_trace = reference_root/f'{task["id"]}__{reference_method}__p42__c42/trajectory.npz'
        with np.load(reference_trace,allow_pickle=False) as old:
            if not np.array_equal(states,old['states']) or not np.array_equal(controls,old['controls']):
                raise ValueError('Identical anchors failed to reproduce the previous parameter setting')
    return dict(run_id=run['run_id'],passed=True,run_sha256=sha(filename),
                trajectory_sha256=sha(directory/'trajectory.npz'),checked_steps=len(controls),
                checked_agent_runs=32,candidate_routes_checked=candidate_count,collisions=0,
                raw_status=terminal,status='deadlock' if adjudication else terminal,
                deadlock_geometry=geometry,terminal_adjudication=adjudication,
                unresolved_stall_reason=unresolved,identical_to_uniform_anchors=identical,
                identical_to_reference_anchors=same_reference)


def comparison_run(run):
    """Keep scoring fields; avoid copying 32 controller profiles into every worker."""
    keys = ('run_id','task_id','source_map','scenario_id','variant_index','diameter_meters',
            'traffic_pattern','planner','raw_status','status','success','completion_fraction',
            'makespan_seconds','collisions','robot_count','protocol_inputs')
    return {key:run[key] for key in keys if key in run}


def historical_runs(baseline, weighted, histories):
    runs, receipts = [], []
    for root in (Path(baseline),Path(weighted),*histories):
        receipt = json.loads((root/'audit.json').read_text())
        if not receipt['passed']:
            raise ValueError(f'Historical audit failed: {root}')
        for record in receipt['traces']:
            filename = root/record['run_id']/'run.json'
            if sha(filename) != record['run_sha256'] or sha(filename.parent/'trajectory.npz') != record['trajectory_sha256']:
                raise ValueError(f'Historical execution changed: {record["run_id"]}')
            run = json.loads(filename.read_text())
            run['raw_status'] = run['status']
            run['status'] = record.get('verified_terminal_outcome',record.get('status',run['status']))
            runs.append(comparison_run(run))
        receipts.append(dict(directory=str(root),audit_sha256=sha(root/'audit.json'),runs=len(receipt['traces'])))
    return runs,receipts


def audit(suite,baseline,weighted,previous,output,*,workers=8):
    suite,baseline,weighted,previous,output = map(Path,(suite,baseline,weighted,previous,output))
    contract = baseline_contract(suite,baseline)
    histories = sorted({p.parent.resolve() for p in output.parent.glob('warehouse_edge_cost*/audit.json')
                        if p.parent.resolve()!=output.resolve()})
    if previous.resolve() not in histories:
        raise ValueError('Missing preserved lambda=1, tau=1 experiment')
    historical, receipts = historical_runs(baseline,weighted,histories)
    experiment = json.loads((output/'experiment.json').read_text())
    files = sorted(output.glob('*/run.json')); runs = [json.loads(p.read_text()) for p in files]
    expected = {e['id'] for e in json.loads((suite/'manifest.json').read_text())['scenarios']}
    if len(runs)!=250 or {r['task_id'] for r in runs}!=expected:
        raise ValueError('All 250 tasks are required for publication')
    common = runs[0]['code_sha256']
    for filename,digest in common.items():
        if sha(ROOT/filename)!=digest:
            raise ValueError(f'Executed source or binary changed: {filename}')
    for run in runs:
        if run['code_sha256']!=common or run['config']!=asdict(EvaluationConfig()) or run['planner']!=experiment['method']:
            raise ValueError('Mixed execution protocols')
        if run['status'] not in ('success','deadlock','stalled'):
            raise ValueError(f'Unexpected outcome: {run["run_id"]}: {run["status"]}')
    records = []
    with ProcessPoolExecutor(max_workers=workers) as pool:
        for count,future in enumerate(as_completed([pool.submit(audit_trace,(str(suite),str(baseline),str(output),str(p))) for p in files]),1):
            records.append(future.result())
            if count%25==0:
                print(f'Audited {count}/250 trajectories',flush=True)
    result = dict(passed=True,runs=250,checked_agent_runs=8000,method=experiment['method'],
        lambda_weight=experiment['lambda_weight'],tau=experiment['tau'],source_and_binary_hashes=common,
        baseline_contract=contract,historical_runs_rechecked=len(historical),historical_audits=receipts,
        candidate_routes_checked=sum(r['candidate_routes_checked'] for r in records),auditor_sha256=sha(__file__),
        raw_terminal_outcomes=dict(Counter(r['raw_status'] for r in records)),
        verified_terminal_outcomes=dict(Counter(r['status'] for r in records)),
        unresolved_stalls=[r['run_id'] for r in records if r['unresolved_stall_reason']],collisions=0,
        identical_reference_anchor_runs_reproduced=sum(r['identical_to_reference_anchors'] for r in records),
        traces=sorted(records,key=lambda r:r['run_id']))
    save_json(output/'audit.json',result)
    for record in result['traces']:
        run = json.loads((output/record['run_id']/'run.json').read_text())
        run['raw_status']=run['status'];run['status']=record['status'];historical.append(comparison_run(run))
    return result,historical


def labels_and_order(runs):
    labels = {'rrt':'RRT-Connect','k_routes':'K routes, uniform','k_routes_width':'K routes, bottleneck width'}
    cost_params = {}
    for r in runs:
        if r['planner'] in labels:
            continue
        protocol = r['protocol_inputs']; weight,temperature = protocol['lambda_weight'],protocol['tau']
        labels[r['planner']] = f'K routes, edge cost (λ={weight:g}, τ={temperature:g})'
        cost_params[r['planner']] = (weight,temperature)
    order = ['rrt','k_routes','k_routes_width']+sorted(cost_params,key=lambda m:cost_params[m])
    return labels,order


def table(rows,labels,order,*,mapped=False):
    lines = ['| '+('Map | ' if mapped else '')+'Method | Tasks | Fleet successes | Fleet success | Agent completion | Successful makespan (s) |',
             '| '+('--- | ' if mapped else '')+'--- | ---: | ---: | ---: | ---: | --- |']
    for r in sorted(rows,key=lambda r:(r.get('source_map',''),order.index(r['planner']))):
        lines.append('| '+(r['source_map']+' | ' if mapped else '')+
            f'{labels[r["planner"]]} | {r["runs"]} | {r["successes"]} | {r["fleet_success_rate"]:.1%} | '
            f'{r["mean_agent_completion_fraction"]:.2%} | {format_time(r)} |')
    return '\n'.join(lines)


def paired(runs,method,order):
    tasks = {}
    for run in runs:
        tasks.setdefault(run['task_id'],{})[run['planner']] = run
    rows = []
    for name in ('both','warehouse','warehouse2'):
        group = [t for t in tasks.values() if name=='both' or t[method]['source_map']==name]
        if not group:
            continue
        for reference in order:
            if reference==method:
                continue
            common = [t for t in group if t[method]['success'] and t[reference]['success']]
            rows.append(dict(source_map=name,reference=reference,tasks=len(group),
                both_successful_tasks=len(common),
                agent_completion_difference=float(np.mean([t[method]['completion_fraction']-t[reference]['completion_fraction'] for t in group])),
                new_minus_reference_makespan_seconds=float(np.mean([t[method]['makespan_seconds']-t[reference]['makespan_seconds'] for t in common])) if common else None))
    return rows


def plots(target,tables,labels,order):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    width = .8/len(order)
    colors = ['#3278a5','#6a9b58','#df8433','#8b62b5','#c14b52','#797340']
    for key,title,filename in (('fleet_success_rate','All 32 agents finish (%)','fleet_success_by_size'),
        ('mean_agent_completion_fraction','Individual-agent completion (%)','agent_completion_by_size'),
        ('successful_makespan_mean_seconds','Mean successful makespan (s)','successful_makespan_by_size')):
        percent = key!='successful_makespan_mean_seconds'
        fig,axes = plt.subplots(1,2,figsize=(15,5),layout='constrained',sharey=True)
        for ax,name in zip(axes,('warehouse','warehouse2')):
            for index,method in enumerate(order):
                rows = sorted([r for r in tables['by_map_size'] if r['source_map']==name and r['planner']==method],key=lambda r:r['diameter_meters'])
                values = [(100 if percent else 1)*r[key] if r[key] is not None else np.nan for r in rows]
                ax.bar(np.arange(5)+(index-(len(order)-1)/2)*width,values,width*.95,
                       color=colors[index%len(colors)],label=labels[method])
            ax.set(title=name,xlabel='Agent diameter (m)',xticks=range(5),xticklabels=['0.6','0.7','0.8','0.9','1.0'])
            if percent:
                ax.set_ylim(0,105)
            else:
                ax.set_ylim(bottom=0)
            ax.grid(axis='y',alpha=.2);ax.set_axisbelow(True)
        axes[0].set_ylabel(title);axes[1].legend(fontsize=8)
        if not percent:
            fig.suptitle('Successful tasks only; methods can have different successful cohorts')
        fig.savefig(target/f'{filename}.png',dpi=150);fig.savefig(target/f'{filename}.svg');plt.close(fig)


def publish(suite,baseline,weighted,previous,output,*,workers=8):
    suite,output = Path(suite),Path(output)
    audit_result,runs = audit(suite,baseline,weighted,previous,output,workers=workers)
    labels,order = labels_and_order(runs)
    if len(runs)!=250*len(order) or len({(r['task_id'],r['planner']) for r in runs})!=len(runs):
        raise ValueError('Comparison must contain every task exactly once per method')
    experiment = json.loads((output/'experiment.json').read_text());params=experiment['params'];method=params['method']
    target = suite/f'edge_cost_{params["tag"]}_evaluation';target.mkdir(parents=True,exist_ok=True)
    views = dict(overall=['planner'],by_map=['source_map','planner'],
        by_scenario=['source_map','scenario_id','diameter_meters','traffic_pattern','planner'],
        by_map_size=['source_map','diameter_meters','planner'],by_map_pattern=['source_map','traffic_pattern','planner'])
    tables = {name:aggregate_rows(runs,keys) for name,keys in views.items()}
    for name,rows in tables.items():
        save_json(target/f'comparison_{name}.json',rows);write_csv(rows,target/f'comparison_{name}.csv')
    brief = [{key:r.get(key) for key in ('run_id','task_id','source_map','scenario_id','variant_index','diameter_meters',
        'traffic_pattern','planner','raw_status','status','success','completion_fraction','makespan_seconds','collisions')} for r in runs]
    write_csv(brief,target/'comparison_runs.csv')
    pairs=paired(runs,method,order);save_json(target/'paired_comparisons.json',pairs);write_csv(pairs,target/'paired_comparisons.csv')
    save_json(target/'audit.json',audit_result)
    save_json(target/'comparison_summary.json',dict(runs=len(runs),methods=order,labels=labels,tables=tables))
    plots(target,tables,labels,order)
    if (output/'selection_summary.json').exists():
        save_json(target/'selection_summary.json',json.loads((output/'selection_summary.json').read_text()))
    overall,mapped=table(tables['overall'],labels,order),table(tables['by_map'],labels,order,mapped=True)
    lines=[f'# Warehouse edge-cost experiment: λ = {params["lambda_weight"]:g}, temperature τ = {params["tau"]:g}','',
        'All methods use the same 250 frozen tasks, 32 agents per task, normal MPPI/ORCA, 1.2 m/s speed bound, original physics, braking safety filter and termination rules. Each scenario has five start–goal variants. All historical runs remain preserved.','',
        'Only the path-selection parameters change. Stored graph edge lengths and widths are used exactly, with connectors excluded and full stored lengths charged for partial-edge entries. The 680 candidates with no graph edges retain zero cost. Every sampled anchor still includes the complete start-to-goal geometry. Random variates and controller seed remain paired with the previous experiment.','',
        'Fleet success requires all 32 robots to finish. Agent completion counts each finished robot, including failed fleets. Makespan is averaged over successful tasks only; failed makespans remain undefined. The sample count n is included.','',
        overall,'',mapped,'',
        f'New-method raw terminal outcomes: `{audit_result["raw_terminal_outcomes"]}`. Verified outcomes: `{audit_result["verified_terminal_outcomes"]}`. '
        f'Zero collisions or motion-limit violations. Rechecked {audit_result["historical_runs_rechecked"]} historical execution checksums.','',
        '## Paired makespan comparison','',
        'Use only the same tasks successfully completed by both methods. Negative differences favor the new parameters.','',
        '| Compared with | Both successful tasks | New minus reference makespan (s) |','| --- | ---: | ---: |']
    for row in pairs:
        if row['source_map']=='both':
            delta=row['new_minus_reference_makespan_seconds']
            lines.append(f'| {labels[row["reference"]]} | {row["both_successful_tasks"]} | '+(f'{delta:.2f}' if delta is not None else 'undefined')+' |')
    lines += ['', '## Detailed data','',
        '- [Five-variant scenario averages](comparison_by_scenario.csv)',
        '- [Size averages by map](comparison_by_map_size.csv)',
        '- [Traffic-pattern averages by map](comparison_by_map_pattern.csv)',
        '- [All executions](comparison_runs.csv)',
        '- [Paired comparisons](paired_comparisons.csv)',
        '- [New-method audit](audit.json)',
        '- [Fleet success by size](fleet_success_by_size.png)',
        '- [Agent completion by size](agent_completion_by_size.png)',
        '- [Successful makespan by size](successful_makespan_by_size.png)','',
        f'Reproduce with `OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 PYTHONPATH=benchmarks .venv/bin/python -m multi_agent.warehouse_cost_experiment --lambda-weight {params["lambda_weight"]:g} --tau {params["tau"]:g} --workers 20`.','']
    if audit_result['unresolved_stalls']:
        lines += [f'Unresolved stationary stalls: {len(audit_result["unresolved_stalls"])}. These remain failed tasks and are not relabeled without a certificate.','']
    (target/'REPORT.md').write_text('\n'.join(lines))
    root_report=ROOT/'benchmarks/WAREHOUSE_BOTTLENECK_RESULTS.md'
    text=root_report.read_text()
    pattern=r'\| Method \| Tasks \| Fleet successes \| Fleet success \| Agent completion \| Successful makespan \(s\) \|\n(?:\|[^\n]*\|\n)+'
    text,count=re.subn(pattern,overall+'\n',text,count=1)
    if count!=1:
        raise ValueError('Could not locate existing overall table')
    block=('\n## Edge-cost benchmark added\n\n'+mapped+'\n\n'
        f'The latest experiment uses λ = {params["lambda_weight"]:g} and temperature τ = {params["tau"]:g}, with the same graph-only cost. '
        f'[All-method report, paired makespans, scenario averages and audit](scenarios/warehouse_uniform/{target.name}/REPORT.md). '
        'The historical sections below retain their original benchmark cohorts.\n\n')
    text=re.sub(r'\n## Edge-cost benchmark added\n.*?(?=## Per map\n)','',text,flags=re.S)
    text=text.replace('## Per map\n',block+'## Per map\n',1)
    for figure in ('fleet_success_by_size','agent_completion_by_size','successful_makespan_by_size'):
        text=re.sub(r'(?:width_evaluation|edge_cost(?:_[a-z0-9_]+)?_evaluation)/'+figure+r'\.png',target.name+'/'+figure+'.png',text)
    root_report.write_text(text)
    review=suite/'evaluation/REVIEW.md';text=review.read_text()
    rows=['| Map | Routing | Finished agents | Agent completion | All-32 fleet successes |','| --- | --- | ---: | ---: | ---: |']
    for name in ('warehouse','warehouse2','both maps'):
        for candidate in order:
            cohort=[r for r in runs if r['planner']==candidate and (name=='both maps' or r['source_map']==name)]
            finished=round(sum(r['robot_count']*r['completion_fraction'] for r in cohort));assigned=sum(r['robot_count'] for r in cohort)
            successes=sum(r['success'] for r in cohort)
            rows.append(f'| {name} | {labels[candidate]} | {finished}/{assigned} | {finished/assigned:.2%} | {successes}/{len(cohort)} ({successes/len(cohort):.1%}) |')
    pattern=r'\| Map \| Routing \| Finished agents \| Agent completion \| All-32 fleet successes \|\n(?:\|[^\n]*\|\n)+'
    text,count=re.subn(pattern,'\n'.join(rows)+'\n',text,count=1)
    if count!=1:
        raise ValueError('Could not locate main Review table')
    marker='\n## Parameterised edge-cost experiments\n'
    if marker in text:
        text=text.split(marker)[0]
    text+=marker+'\n'+overall+'\n\n'+mapped+'\n\n'+f'[Complete comparison and audit](../{target.name}/REPORT.md).\n'
    review.write_text(text)
    print('Published all-method results:',target/'REPORT.md',flush=True)
    return tables
