"""Run all five frozen anchor methods with disappearance after settled arrival."""
import argparse
from collections import Counter
from concurrent.futures import ProcessPoolExecutor, as_completed
from dataclasses import asdict
from functools import partial
import json
import os
from pathlib import Path
import shutil

from .geometry import ROOT, fingerprint
from .warehouse_evaluation import EvaluationConfig, aggregate_rows, load_task, save_json, write_csv
from .warehouse_disappear_evaluation import run_disappear_evaluation
from . import warehouse_disappear_runtime
from .warehouse_width_benchmark import baseline_contract, sha

METHODS = ('rrt', 'k_routes', 'k_routes_width', 'k_routes_edge_cost', 'k_routes_edge_cost_l2_t10')
SOURCE_ROOTS = dict(rrt='warehouse_uniform_corrected', k_routes='warehouse_uniform_corrected',
    k_routes_width='warehouse_bottleneck_width', k_routes_edge_cost='warehouse_edge_cost',
    k_routes_edge_cost_l2_t10='warehouse_edge_cost_l2_t10')
POLICY = 'disappear_after_settled_arrival'


def reference_directory(task_id, method):
    return ROOT/'benchmarks/results'/SOURCE_ROOTS[method]/f'{task_id}__{method}__p42__c42'


def prepare(suite, output, task_ids=None, methods=METHODS):
    contract = baseline_contract(suite, ROOT/'benchmarks/results/warehouse_uniform_corrected')
    ids = [e['id'] for e in json.loads((suite/'manifest.json').read_text())['scenarios']]
    if task_ids:
        if set(task_ids)-set(ids):raise ValueError('Unknown task IDs')
        ids = [i for i in ids if i in task_ids]
    if set(methods)-set(METHODS):raise ValueError('Unknown method')
    receipts = {}
    for name in set(SOURCE_ROOTS[m] for m in methods):
        filename = ROOT/'benchmarks/results'/name/'audit.json'
        data = json.loads(filename.read_text())
        if not data['passed']:raise ValueError('Unaudited reference')
        receipts[name] = {t['run_id']:t for t in data['traces']}
    jobs = []
    for task_id in sorted(ids,key=lambda x:(x.split('_',1)[1],x.split('_',1)[0])):
        task,world,task_hash = load_task(suite,task_id)
        for method in methods:
            directory = reference_directory(task_id,method)
            run_file = directory/'run.json'; run = json.loads(run_file.read_text())
            receipt = receipts[SOURCE_ROOTS[method]][run['run_id']]
            if sha(run_file)!=receipt['run_sha256'] or sha(directory/'trajectory.npz')!=receipt['trajectory_sha256']:
                raise ValueError('Audited reference changed')
            if run['scenario_sha256']!=task_hash or run['config']!=asdict(EvaluationConfig()):
                raise ValueError('Reference assignments/configuration differ')
            for name,key in (('paths.json','paths_sha256'),('route_choices.json','choices_sha256')):
                if sha(directory/name)!=run['anchor_validation'][key]:raise ValueError('Reference anchors changed')
            jobs.append(dict(task_id=task_id,method=method,reference_run_sha256=sha(run_file),
                paths_sha256=sha(directory/'paths.json'),choices_sha256=sha(directory/'route_choices.json'),
                reference_trajectory_sha256=receipt['trajectory_sha256']))
    sources = [Path(__file__),Path(warehouse_disappear_runtime.__file__),ROOT/'benchmarks/multi_agent/warehouse_disappear_evaluation.py']
    identity = fingerprint(dict(jobs=jobs,config=asdict(EvaluationConfig()),policy=POLICY,
        code={str(p.relative_to(ROOT)):sha(p) for p in sources},contract=contract))
    record = dict(identity=identity,target_policy=POLICY,expected_runs=len(jobs),jobs=jobs,
        config=asdict(EvaluationConfig()),baseline_contract=contract,
        policy_definition='Remove a disk at the first completed timestep inside its goal tolerance with both applied controls <=1e-8; the incoming interval remains collision-checked. Exclude it from neighbors, joint safety and obstructions starting with the next interval. Keep its terminal snapshot and explicit monotone presence mask in the trace.')
    filename = output/'experiment.json'
    if filename.exists() and json.loads(filename.read_text())!=record:raise ValueError('Experiment identity differs')
    save_json(filename,record)
    return record


def anchors(task, world, planner, selection_seed, destination, k, *, suite_folder):
    source = reference_directory(task['id'],planner)
    old = json.loads((source/'run.json').read_text())
    for name in ('paths.json','route_choices.json'):
        shutil.copyfile(source/name,Path(destination)/name)
    return json.loads((source/'paths.json').read_text()),old['total_global_planning_seconds'],old['per_robot_global_planning_seconds']


def run_task(suite,output,job):
    reference = json.loads((reference_directory(job['task_id'],job['method'])/'run.json').read_text())
    protocol = dict(target_policy=POLICY, **job)
    for key in ('lambda_weight','tau'):
        if key in reference.get('protocol_inputs',{}):protocol[key]=reference['protocol_inputs'][key]
    result = run_disappear_evaluation(suite,job['task_id'],job['method'],output,
        anchor_provider=anchors,execution_sources=[Path(__file__),Path(warehouse_disappear_runtime.__file__),
            ROOT/'benchmarks/multi_agent/warehouse_evaluation.py'],protocol_inputs=protocol)
    if result['controller_profiles']!=reference['controller_profiles']:
        raise ValueError('Controller differs from parked reference')
    return {k:result[k] for k in ('run_id','task_id','planner','status','completion_fraction','makespan_seconds')}


def summarize(output, expected):
    runs = [json.loads(p.read_text()) for p in sorted(output.glob('*/run.json'))]
    for name,keys in dict(overall=['planner'],by_map=['source_map','planner'],
        by_scenario=['source_map','scenario_id','diameter_meters','traffic_pattern','planner'],
        by_map_size=['source_map','diameter_meters','planner'],by_map_pattern=['source_map','traffic_pattern','planner']).items():
        rows = aggregate_rows(runs,keys);save_json(output/f'{name}.json',rows);write_csv(rows,output/f'{name}.csv')
    save_json(output/'progress_summary.json',dict(completed_runs=len(runs),expected_runs=expected,
        outcomes=dict(Counter(r['status'] for r in runs)),by_method=dict(Counter(r['planner'] for r in runs))))
    return runs


def run_suite(suite,output,workers=20,audit_workers=8,task_ids=None,methods=METHODS,prepare_only=False):
    suite,output=Path(suite).resolve(),Path(output).resolve();output.mkdir(parents=True,exist_ok=True)
    experiment=prepare(suite,output,task_ids,methods);expected=experiment['expected_runs']
    if prepare_only:return experiment
    save_json(output/'execution_state.json',dict(status='running',pid=os.getpid(),expected_runs=expected,workers=workers))
    with ProcessPoolExecutor(max_workers=workers) as pool:
        futures=[pool.submit(run_task,str(suite),str(output),job) for job in experiment['jobs']]
        for count,future in enumerate(as_completed(futures),1):
            run=future.result()
            print(f'[{count}/{expected}] {run["run_id"]}: {run["status"]}, agents={round(32*run["completion_fraction"])}/32, makespan={run["makespan_seconds"]}',flush=True)
            if count%10==0:summarize(output,expected)
    summarize(output,expected)
    from .warehouse_disappear_report import publish
    result=publish(suite,output,workers=audit_workers,full_suite=expected==1250)
    save_json(output/'execution_state.json',dict(status='complete',pid=os.getpid(),expected_runs=expected,runs=expected,workers=workers))
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--suite',type=Path,default=ROOT/'benchmarks/scenarios/warehouse_uniform')
    parser.add_argument('--output',type=Path,default=ROOT/'benchmarks/results/warehouse_disappear')
    parser.add_argument('--workers',type=int,default=20)
    parser.add_argument('--audit-workers',type=int,default=8)
    parser.add_argument('--task-ids',nargs='+')
    parser.add_argument('--methods',nargs='+',default=METHODS)
    parser.add_argument('--prepare-only',action='store_true')
    run_suite(**vars(parser.parse_args()))


if __name__=='__main__':main()
