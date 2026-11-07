"""Parameterised edge-cost experiments on the immutable warehouse benchmark."""

import argparse
from collections import Counter
from concurrent.futures import ProcessPoolExecutor, as_completed
from dataclasses import asdict
from functools import partial
import gzip
import json
import math
from pathlib import Path
import time

from . import edge_cost_selection
from .edge_cost_selection import draw_edge_cost_route, graph_route_edges
from .geometry import ROOT, fingerprint
from .warehouse_evaluation import EvaluationConfig, aggregate_rows, load_task, run_evaluation, save_json, write_csv
from .warehouse_preflight import check_paths
from .warehouse_width_benchmark import baseline_contract, sha


def settings(lambda_weight, tau):
    if not math.isfinite(lambda_weight) or lambda_weight < 0 or not math.isfinite(tau) or tau <= 0:
        raise ValueError('Require finite lambda >= 0 and temperature > 0')
    slug = lambda x: format(x, '.12g').replace('.', 'p').replace('+', '')
    tag = f'l{slug(lambda_weight)}_t{slug(tau)}'
    return dict(lambda_weight=float(lambda_weight), tau=float(tau), tag=tag,
                method=f'k_routes_edge_cost_{tag}')


def _prepare(arguments):
    suite, baseline, previous, output, task_id, params, expected_sha = arguments
    suite, baseline, previous, output = map(Path, (suite, baseline, previous, output))
    task, world, task_hash = load_task(suite, task_id)
    started = time.perf_counter()
    source = previous / f'selection_inputs/{task_id}.json'
    if sha(source) != expected_sha:
        raise ValueError('Preserved lambda=1, tau=1 selection inputs changed')
    old = json.loads(source.read_text())
    directory = baseline / f'{task_id}__k_routes__p42__c42'
    native_file = directory / 'agents.routes.json'
    if sha(native_file) != old['native_candidates_sha256'] or task_hash != old['scenario_sha256']:
        raise ValueError('Frozen candidate set or assignments changed')
    native = json.loads(native_file.read_text())
    graph_file = suite / f'geometry_cache/{task["source_map"]}.resources.json.gz'
    if sha(graph_file) != old['graph_bundle_sha256']:
        raise ValueError('Stored graph changed')
    graph = json.loads(gzip.decompress(graph_file.read_bytes()))
    edges = {edge['id']: edge for edge in graph['routing_geometry']['edges']}
    paths, robots = [], []
    for robot, query, reference in zip(task['robots'], native['queries'], old['robots']):
        tick = time.perf_counter()
        if robot['id'] != query['name'] or robot['id'] != reference['id'] or not query['validation_passed']:
            raise ValueError('Robot/candidate mapping differs')
        if len(query['routes']) != len(reference['routes']):
            raise ValueError('Returned K differs')
        route_edges = [graph_route_edges(route, edges) for route in query['routes']]
        index, distribution = draw_edge_cost_route(route_edges, robot['radius'],
            lambda_weight=params['lambda_weight'], tau=params['tau'], uniform_variate=reference['uniform_variate'])
        candidates = []
        for j, (route, edge_list, cost) in enumerate(zip(query['routes'], route_edges, distribution['routes'])):
            digest = fingerprint(route['path'])
            if digest != reference['routes'][j]['path_sha256']:
                raise ValueError('Certified keep-right geometry changed')
            candidates.append(dict(index=j, route_id=route['id'], edges=edge_list, path_sha256=digest, **cost))
        paths.append(query['routes'][index]['path'])
        robots.append(dict(id=robot['id'], available=len(candidates), routes=candidates,
            w_max=distribution['w_max'], probabilities=distribution['probabilities'], selected_index=index,
            selection_probability=distribution['probabilities'][index], uniform_variate=reference['uniform_variate'],
            baseline_uniform_index=reference['baseline_uniform_index'], jitter=reference['jitter'],
            preparation_seconds=time.perf_counter()-tick))
    if len(paths) != 32 or len(robots) != 32:
        raise ValueError('Incomplete fleet')
    check_paths(task, world, paths)
    record = dict(task_id=task_id, source_map=task['source_map'], scenario_sha256=task_hash,
        selection_seed=42, requested_k=5, **params,
        edge_definition=old['edge_definition'], native_candidates_sha256=sha(native_file),
        baseline_run_sha256=sha(directory/'run.json'), graph_bundle_sha256=sha(graph_file),
        reference_inputs_sha256=expected_sha, original_global_planning_seconds=old['original_global_planning_seconds'],
        original_robot_planning_seconds=old['original_robot_planning_seconds'],
        preparation_seconds=time.perf_counter()-started, paths=paths, robots=robots)
    filename = output/f'selection_inputs/{task_id}.json'
    save_json(filename, record)
    return dict(task_id=task_id, sha256=sha(filename), source_map=task['source_map'])


def prepare(suite, baseline, previous, output, params):
    contract = baseline_contract(suite, baseline)
    old = json.loads((previous/'selection_preflight.json').read_text())
    old_audit = json.loads((previous/'audit.json').read_text())
    if old['tasks'] != 250 or not old_audit['passed'] or old_audit['runs'] != 250:
        raise ValueError('Complete audited previous edge-cost experiment is required')
    identity = fingerprint(dict(params=params, contract=contract, previous_preflight=sha(previous/'selection_preflight.json'),
        code={str(p.relative_to(ROOT)):sha(p) for p in (Path(__file__),Path(edge_cost_selection.__file__))}))
    output.mkdir(parents=True, exist_ok=True); (output/'selection_inputs').mkdir(exist_ok=True)
    preflight = output/'selection_preflight.json'
    if preflight.exists():
        cached = json.loads(preflight.read_text())
        if cached['identity'] != identity:
            raise ValueError('Existing experiment has different settings or sources')
        for r in cached['selection_inputs']:
            if sha(output/f'selection_inputs/{r["task_id"]}.json') != r['sha256']:
                raise ValueError('Prepared selection inputs changed')
        return cached
    jobs = [(str(suite),str(baseline),str(previous),str(output),r['task_id'],params,r['sha256']) for r in old['selection_inputs']]
    records = []
    with ProcessPoolExecutor(max_workers=4) as pool:
        for future in as_completed([pool.submit(_prepare, job) for job in jobs]):
            records.append(future.result())
            if len(records)%25 == 0:
                print(f'Prepared and clearance-checked {len(records)}/250 tasks', flush=True)
    result = dict(passed=True, identity=identity, tasks=len(records), params=params, baseline_contract=contract,
        previous_directory=str(previous), previous_audit_sha256=sha(previous/'audit.json'),
        selection_inputs=sorted(records,key=lambda r:r['task_id']))
    save_json(preflight, result)
    return result


def anchors(task, world, planner, selection_seed, folder, k=5, *, suite_folder=None, prepared):
    record = json.loads((Path(prepared)/f'selection_inputs/{task["id"]}.json').read_text())
    if (planner != record['method'] or selection_seed != record['selection_seed'] or k != record['requested_k']
            or fingerprint({key:value for key,value in task.items() if key!='generation_seconds'}) != record['scenario_sha256']):
        raise ValueError('Prepared anchors belong to another experiment')
    save_json(Path(folder)/'paths.json',record['paths'])
    save_json(Path(folder)/'route_choices.json',dict(planner=planner,seed=selection_seed,
        lambda_weight=record['lambda_weight'],tau=record['tau'],edge_definition=record['edge_definition'],robots=record['robots']))
    timings = [old+robot['preparation_seconds'] for old,robot in zip(record['original_robot_planning_seconds'],record['robots'])]
    return record['paths'],record['original_global_planning_seconds']+record['preparation_seconds'],timings


def run_task(arguments):
    suite, baseline, output, task_id = arguments
    filename = Path(output)/f'selection_inputs/{task_id}.json'
    record = json.loads(filename.read_text())
    protocol = {key:record[key] for key in ('lambda_weight','tau','method','native_candidates_sha256','graph_bundle_sha256','baseline_run_sha256')}
    protocol['selection_inputs_sha256'] = sha(filename)
    result = run_evaluation(suite,task_id,record['method'],output,
        anchor_provider=partial(anchors,prepared=output),execution_sources=[Path(__file__),Path(edge_cost_selection.__file__)],
        protocol_inputs=protocol)
    old = json.loads((Path(baseline)/f'{task_id}__k_routes__p42__c42/run.json').read_text())
    if result['controller_profile_sha256'] != old['controller_profile_sha256']:
        raise ValueError('Controller differs from the preserved baseline')
    return result


def summarize(output):
    runs = [json.loads(p.read_text()) for p in sorted(Path(output).glob('*/run.json'))]
    for name,keys in dict(overall=['planner'],by_map=['source_map','planner'],
        by_scenario=['source_map','scenario_id','diameter_meters','traffic_pattern','planner'],
        by_map_size=['source_map','diameter_meters','planner'],by_map_pattern=['source_map','traffic_pattern','planner']).items():
        rows = aggregate_rows(runs,keys); save_json(Path(output)/f'{name}.json',rows); write_csv(rows,Path(output)/f'{name}.csv')
    save_json(Path(output)/'progress_summary.json',dict(completed_runs=len(runs),outcomes=dict(Counter(r['status'] for r in runs))))
    return runs


def run_suite(suite,baseline,previous,output,*,lambda_weight,tau,workers=20,audit_workers=8,prepare_only=False):
    params = settings(lambda_weight,tau)
    suite,baseline,previous = [Path(p).resolve() for p in (suite,baseline,previous)]
    output = Path(output).resolve() if output else ROOT/f'benchmarks/results/warehouse_edge_cost_{params["tag"]}'
    preflight = prepare(suite,baseline,previous,output,params)
    if prepare_only:
        return preflight
    save_json(output/'experiment.json',dict(expected_runs=250,workers=workers,method=params['method'],
        params=params,lambda_weight=lambda_weight,tau=tau,config=asdict(EvaluationConfig()),normal_mppi=True,
        selection_preflight_sha256=sha(output/'selection_preflight.json'),baseline_contract=preflight['baseline_contract']))
    ids = sorted([r['task_id'] for r in preflight['selection_inputs']],key=lambda x:(x.split('_',1)[1],x.split('_',1)[0]))
    with ProcessPoolExecutor(max_workers=workers) as pool:
        futures = [pool.submit(run_task,(str(suite),str(baseline),str(output),task_id)) for task_id in ids]
        for count,future in enumerate(as_completed(futures),1):
            run = future.result()
            print(f'[{count}/250] {run["run_id"]}: {run["status"]}, agents={round(32*run["completion_fraction"])}/32, makespan={run["makespan_seconds"]}',flush=True)
            if count%5 == 0:
                summarize(output)
    summarize(output)
    from .warehouse_cost_experiment_report import publish
    return publish(suite,baseline,ROOT/'benchmarks/results/warehouse_bottleneck_width',previous,output,workers=audit_workers)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--suite',type=Path,default=ROOT/'benchmarks/scenarios/warehouse_uniform')
    parser.add_argument('--baseline',type=Path,default=ROOT/'benchmarks/results/warehouse_uniform_corrected')
    parser.add_argument('--previous',type=Path,default=ROOT/'benchmarks/results/warehouse_edge_cost')
    parser.add_argument('--output',type=Path)
    parser.add_argument('--lambda-weight',type=float,required=True)
    parser.add_argument('--tau',type=float,required=True)
    parser.add_argument('--workers',type=int,default=20)
    parser.add_argument('--audit-workers',type=int,default=8)
    parser.add_argument('--prepare-only',action='store_true')
    run_suite(**vars(parser.parse_args()))


if __name__ == '__main__':
    main()
