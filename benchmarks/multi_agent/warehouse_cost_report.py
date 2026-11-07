"""Audit edge-cost executions and add them to the three preserved baselines."""

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
from .warehouse_cost_benchmark import METHOD, LAMBDA, TAU, sha
from .warehouse_evaluation import (EvaluationConfig, TerminalMonitor, aggregate_rows,
                                   disk_collisions, load_task, save_json, write_csv)
from .warehouse_evaluation_report import audit_deadlock_geometry, format_time
from .warehouse_preflight import anchor_identity, check_joint_certificate, check_paths
from .warehouse_terminal_adjudication import certify_visible_join_deadlock
from .warehouse_width_benchmark import baseline_contract

METHODS = ('rrt', 'k_routes', 'k_routes_width', METHOD)
LABELS = dict(rrt='RRT-Connect', k_routes='K routes, uniform',
              k_routes_width='K routes, bottleneck width', k_routes_edge_cost='K routes, edge cost (λ=τ=1)')


def _audit_cost(arguments):
    suite, baseline, output, filename = map(Path, arguments)
    run = json.loads(filename.read_text()); directory = filename.parent
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
    return dict(run_id=run['run_id'],passed=True,run_sha256=sha(filename),
                trajectory_sha256=sha(directory/'trajectory.npz'),checked_steps=len(controls),
                checked_agent_runs=32,candidate_routes_checked=candidate_count,collisions=0,
                raw_status=terminal,status='deadlock' if adjudication else terminal,
                deadlock_geometry=geometry,terminal_adjudication=adjudication,
                unresolved_stall_reason=unresolved,identical_to_uniform_anchors=identical)


def audit(suite,baseline,weighted,output,*,workers=8):
    suite,baseline,weighted,output = map(Path,(suite,baseline,weighted,output))
    contract = baseline_contract(suite,baseline)
    width_audit = json.loads((weighted/'audit.json').read_text())
    if not width_audit['passed'] or width_audit['weighted_runs'] != 250:
        raise ValueError('Complete bottleneck baseline audit required')
    files = sorted(output.glob(f'*__{METHOD}__*/run.json'))
    runs = [json.loads(p.read_text()) for p in files]
    expected = {e['id'] for e in json.loads((suite/'manifest.json').read_text())['scenarios']}
    if len(runs) != 250 or {r['task_id'] for r in runs} != expected:
        raise ValueError('All 250 edge-cost tasks are required')
    common = runs[0]['code_sha256']
    for filename,digest in common.items():
        if sha(ROOT/filename) != digest:
            raise ValueError(f'Executed source or binary changed: {filename}')
    for run in runs:
        if run['code_sha256'] != common or run['config'] != asdict(EvaluationConfig()):
            raise ValueError('Mixed execution protocols')
        if run['status'] not in ('success','deadlock','stalled'):
            raise ValueError(f'Unexpected terminal outcome: {run["run_id"]}: {run["status"]}')
    historical = json.loads((baseline/'audit.json').read_text())
    for root,records in ((baseline,historical['traces']),(weighted,width_audit['traces'])):
        for record in records:
            folder = root/record['run_id']
            if sha(folder/'run.json') != record['run_sha256'] or sha(folder/'trajectory.npz') != record['trajectory_sha256']:
                raise ValueError('Preserved baseline execution changed')
    records = []
    with ProcessPoolExecutor(max_workers=workers) as pool:
        futures = [pool.submit(_audit_cost,(str(suite),str(baseline),str(output),str(p))) for p in files]
        for count,future in enumerate(as_completed(futures),1):
            records.append(future.result())
            if count%25 == 0:
                print(f'Audited {count}/250 edge-cost trajectories',flush=True)
    result = dict(passed=True,runs=250,checked_agent_runs=8000,baseline_runs_rechecked=750,
                  candidate_routes_checked=sum(r['candidate_routes_checked'] for r in records),
                  source_and_binary_hashes=common,baseline_contract=contract,
                  baseline_width_audit_sha256=sha(weighted/'audit.json'),auditor_sha256=sha(__file__),
                  raw_terminal_outcomes=dict(Counter(r['raw_status'] for r in records)),
                  verified_terminal_outcomes=dict(Counter(r['status'] for r in records)),
                  unresolved_stalls=[r['run_id'] for r in records if r['unresolved_stall_reason']],
                  collisions=0,traces=sorted(records,key=lambda r:r['run_id']))
    save_json(output/'audit.json',result)
    return result


def combine(baseline,weighted,output):
    from .warehouse_width_report import all_runs
    runs = all_runs(baseline,weighted)
    audit_result = json.loads((Path(output)/'audit.json').read_text())
    for record in audit_result['traces']:
        filename = Path(output)/record['run_id']/'run.json'
        if sha(filename) != record['run_sha256']:
            raise ValueError('Audited edge-cost run changed')
        run = json.loads(filename.read_text())
        run['raw_status'] = run['status']; run['status'] = record['status']
        run['terminal_adjudication'] = bool(record['terminal_adjudication'])
        runs.append(run)
    if len(runs) != 1000 or len({(r['task_id'],r['planner']) for r in runs}) != 1000:
        raise ValueError('Comparison must have exactly four methods on all 250 tasks')
    return runs


def paired(runs):
    tasks = {}
    for r in runs:
        tasks.setdefault(r['task_id'],{})[r['planner']] = r
    rows = []
    for name in ('both','warehouse','warehouse2'):
        cohort = [t for t in tasks.values() if name=='both' or t[METHOD]['source_map']==name]
        if not cohort:
            continue
        for reference in METHODS[:-1]:
            common = [t for t in cohort if t[METHOD]['success'] and t[reference]['success']]
            rows.append(dict(source_map=name,reference=reference,tasks=len(cohort),
                        both_successful_tasks=len(common),
                        agent_completion_difference=float(np.mean([t[METHOD]['completion_fraction']-t[reference]['completion_fraction'] for t in cohort])),
                        edge_cost_minus_reference_makespan_seconds=float(np.mean([t[METHOD]['makespan_seconds']-t[reference]['makespan_seconds'] for t in common])) if common else None))
    return rows


def table(rows,per_map=False):
    headers = ['| '+('Map | ' if per_map else '')+'Method | Tasks | Fleet successes | Fleet success | Agent completion | Successful makespan (s) |',
               '| '+('--- | ' if per_map else '')+'--- | ---: | ---: | ---: | ---: | --- |']
    for row in sorted(rows,key=lambda r:(r.get('source_map',''),METHODS.index(r['planner']))):
        headers.append('| '+(row['source_map']+' | ' if per_map else '')+
                       f'{LABELS[row["planner"]]} | {row["runs"]} | {row["successes"]} | '
                       f'{row["fleet_success_rate"]:.1%} | {row["mean_agent_completion_fraction"]:.2%} | {format_time(row)} |')
    return '\n'.join(headers)


def make_plots(folder,tables):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    colors = ['#3278a5','#6a9b58','#df8433','#8b62b5']
    for key,label,filename in (('fleet_success_rate','All 32 agents finish (%)','fleet_success_by_size'),
                              ('mean_agent_completion_fraction','Agent completion (%)','agent_completion_by_size'),
                              ('successful_makespan_mean_seconds','Mean successful makespan (s)','successful_makespan_by_size')):
        percent = key != 'successful_makespan_mean_seconds'
        fig,axes = plt.subplots(1,2,figsize=(14,5),layout='constrained',sharey=True)
        for ax,name in zip(axes,('warehouse','warehouse2')):
            for index,method in enumerate(METHODS):
                rows = sorted([r for r in tables['by_map_size'] if r['source_map']==name and r['planner']==method],key=lambda r:r['diameter_meters'])
                values = [(100 if percent else 1)*r[key] if r[key] is not None else np.nan for r in rows]
                ax.bar(np.arange(5)+(index-1.5)*.2,values,.19,color=colors[index],label=LABELS[method])
            ax.set(title=name,xlabel='Agent diameter (m)',xticks=range(5),xticklabels=['0.6','0.7','0.8','0.9','1.0'])
            ax.set_ylim(0,105) if percent else ax.set_ylim(bottom=0)
            ax.grid(axis='y',alpha=.2); ax.set_axisbelow(True)
        axes[0].set_ylabel(label);axes[1].legend(fontsize=8)
        if not percent:
            fig.suptitle('Successful tasks only; methods can have different successful cohorts')
        fig.savefig(folder/f'{filename}.png',dpi=150);fig.savefig(folder/f'{filename}.svg');plt.close(fig)


def publish_results(suite,baseline,weighted,output,*,workers=8):
    audit_result = audit(suite,baseline,weighted,output,workers=workers)
    runs = combine(baseline,weighted,output)
    target = Path(suite)/'edge_cost_evaluation';target.mkdir(parents=True,exist_ok=True)
    views = dict(overall=['planner'],by_map=['source_map','planner'],
                 by_scenario=['source_map','scenario_id','diameter_meters','traffic_pattern','planner'],
                 by_map_size=['source_map','diameter_meters','planner'],by_map_pattern=['source_map','traffic_pattern','planner'])
    tables = {name:aggregate_rows(runs,keys) for name,keys in views.items()}
    for name,rows in tables.items():
        save_json(target/f'comparison_{name}.json',rows);write_csv(rows,target/f'comparison_{name}.csv')
    brief = [{key:r.get(key) for key in ('run_id','task_id','source_map','scenario_id','variant_index','diameter_meters','traffic_pattern','planner','raw_status','status','success','completion_fraction','makespan_seconds','collisions')} for r in runs]
    write_csv(brief,target/'comparison_runs.csv')
    pairs = paired(runs);save_json(target/'paired_comparisons.json',pairs);write_csv(pairs,target/'paired_comparisons.csv')
    save_json(target/'audit.json',audit_result)
    save_json(target/'comparison_summary.json',dict(runs=1000,methods=METHODS,tables=tables))
    make_plots(target,tables)
    overall_table, map_table = table(tables['overall']),table(tables['by_map'],True)
    lines = ['# Warehouse benchmark with edge-cost sampling','',
             'All four methods use the same 250 tasks, 32 agents per task, normal MPPI/ORCA, 1.2 m/s speed limit, physics, safety filter and stopping rules. Each scenario has five start–goal variants.','',
             'The new method samples one fixed route per agent with λ = 1 and temperature τ = 1. It uses stored graph edge lengths and widths exactly: connectors are excluded, full stored edge lengths are charged for partial traversals, and the 680 zero-edge candidates retain zero cost. The sampled anchor still covers the complete start-to-goal path.','',
             'Fleet success means all 32 agents finish. Agent completion counts finished individuals, including failed fleets. Makespan is simulated completion time and is averaged over successful tasks only; failed makespans remain undefined. The sample count n is shown.','',
             overall_table,'',map_table,'',
             f'Raw new-method terminal outcomes: `{audit_result["raw_terminal_outcomes"]}`. Verified outcomes: `{audit_result["verified_terminal_outcomes"]}`. Zero collisions or motion-limit violations. All 750 historical execution checksums were rechecked.','',
             '## Paired makespan comparison','',
             'These differences use only the same tasks successfully completed by both methods. Negative values favor edge-cost sampling.','',
             '| Compared with | Both successful tasks | Edge-cost minus reference makespan (s) |',
             '| --- | ---: | ---: |']
    for row in pairs:
        if row['source_map']=='both':
            delta = row['edge_cost_minus_reference_makespan_seconds']
            lines.append(f'| {LABELS[row["reference"]]} | {row["both_successful_tasks"]} | '+(f'{delta:.2f}' if delta is not None else 'undefined')+' |')
    lines += ['', '## Detailed data','',
              '- [Five-variant scenario averages](comparison_by_scenario.csv)',
              '- [Size averages by map](comparison_by_map_size.csv)',
              '- [Traffic-pattern averages by map](comparison_by_map_pattern.csv)',
              '- [All 1,000 run outcomes](comparison_runs.csv)',
              '- [Paired comparisons](paired_comparisons.csv)',
              '- [Full new-method audit](audit.json)',
              '- [Fleet success by size](fleet_success_by_size.png)',
              '- [Agent completion by size](agent_completion_by_size.png)','',
              'Reproduce with `OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 PYTHONPATH=benchmarks .venv/bin/python -m multi_agent.warehouse_cost_benchmark --workers 20`. Existing runs are reused only when their full evaluation identity matches.','']
    if audit_result['unresolved_stalls']:
        lines += [f'Unresolved stationary stalls: {len(audit_result["unresolved_stalls"])}. These remain failed tasks and are not relabeled as deadlocks without a certificate.','']
    (target/'REPORT.md').write_text('\n'.join(lines))
    # Add the new row to the existing comparison table without overwriting raw data.
    old_report = ROOT/'benchmarks/WAREHOUSE_BOTTLENECK_RESULTS.md'
    content = old_report.read_text()
    pattern = r'\| Method \| Tasks \| Fleet successes \| Fleet success \| Agent completion \| Successful makespan \(s\) \|\n(?:\|[^\n]*\|\n)+'
    content,count = re.subn(pattern,overall_table+'\n',content,count=1)
    if count != 1:
        raise ValueError('Could not locate the existing overall comparison table')
    note = ('\n## Edge-cost benchmark added\n\n'+map_table+'\n\n'
            'The edge-cost method uses λ = τ = 1 and the requested graph-only cost. '
            'See the [four-method report](scenarios/warehouse_uniform/edge_cost_evaluation/REPORT.md) '
            'for paired makespan comparisons, all five-variant scenario averages and the audit. '
            'The remaining historical sections below describe the original bottleneck benchmark.\n\n')
    content = re.sub(r'\n## Edge-cost benchmark added\n.*?(?=## Per map\n)','',content,flags=re.S)
    content = content.replace('## Per map\n',note+'## Per map\n',1)
    for figure in ('fleet_success_by_size','agent_completion_by_size','successful_makespan_by_size'):
        content = content.replace(f'width_evaluation/{figure}.png',f'edge_cost_evaluation/{figure}.png')
    old_report.write_text(content)
    review = Path(suite)/'evaluation/REVIEW.md'
    text = review.read_text()
    top_rows = ['| Map | Routing | Finished agents | Agent completion | All-32 fleet successes |',
                '| --- | --- | ---: | ---: | ---: |']
    for name in ('warehouse','warehouse2','both maps'):
        for method in METHODS:
            cohort = [r for r in runs if r['planner']==method and (name=='both maps' or r['source_map']==name)]
            finished = round(sum(r['robot_count']*r['completion_fraction'] for r in cohort))
            assigned = sum(r['robot_count'] for r in cohort); successes = sum(r['success'] for r in cohort)
            top_rows.append(f'| {name} | {LABELS[method]} | {finished}/{assigned} | {finished/assigned:.2%} | '
                            f'{successes}/{len(cohort)} ({successes/len(cohort):.1%}) |')
    top_pattern = r'\| Map \| Routing \| Finished agents \| Agent completion \| All-32 fleet successes \|\n(?:\|[^\n]*\|\n)+'
    text,count = re.subn(top_pattern,'\n'.join(top_rows)+'\n',text,count=1)
    if count != 1:
        raise ValueError('Could not locate the main Review comparison table')
    marker = '\n## Edge-cost sampling benchmark (λ = τ = 1)\n'
    if marker in text:
        text = text.split(marker)[0]
    text += marker+'\n'+overall_table+'\n\n'+map_table+'\n\n'
    text += '[Four-method results, scenario averages and audit](../edge_cost_evaluation/REPORT.md). '
    text += 'Makespan means use successful runs only; paired comparisons are provided in the linked report.\n'
    review.write_text(text)
    print('Published audited four-method results:',target/'REPORT.md',flush=True)
    return tables
