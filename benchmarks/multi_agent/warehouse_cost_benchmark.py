"""Run the frozen warehouse tasks with graph-edge-cost softmax route sampling."""

import argparse
from collections import Counter, defaultdict
from concurrent.futures import ProcessPoolExecutor, as_completed
import csv
from dataclasses import asdict
from functools import partial
import json
from pathlib import Path
import time

import numpy as np

from . import edge_cost_selection
from .edge_cost_selection import edge_cost_distribution, graph_route_edges
from .geometry import ROOT, fingerprint
from .warehouse_evaluation import (EvaluationConfig, aggregate_rows, load_task,
                                   run_evaluation, save_json, write_csv)
from .warehouse_preflight import check_paths
from .warehouse_width_benchmark import baseline_contract, sha

METHOD = 'k_routes_edge_cost'
LAMBDA = 1.
TAU = 1.
RULE = 'p_k = softmax(-(sum(l_e) + lambda*sum(l_e*clamp(1-(w_e-2*r)/(2*W_max),0,1)))/tau)'


def _prepare_task(arguments):
    suite, baseline, weighted, output, task_id, rows, source = arguments
    suite, baseline, weighted, output = map(Path, (suite, baseline, weighted, output))
    task, world, task_hash = load_task(suite, task_id)
    tick = time.perf_counter()
    old_file = weighted / f'selection_inputs/{task_id}.json'
    if sha(old_file) != source['bottleneck_input_sha256']:
        raise ValueError('Bottleneck input changed after the probability comparison')
    old = json.loads(old_file.read_text())
    directory = baseline / f'{task_id}__k_routes__p42__c42'
    native_file = directory / 'agents.routes.json'
    native_sha = sha(native_file)
    if native_sha != source['native_candidates_sha256'] or task_hash != source['scenario_sha256']:
        raise ValueError('Probability comparison used different candidates or assignments')
    native = json.loads(native_file.read_text())
    import gzip
    graph_file = suite / f'geometry_cache/{task["source_map"]}.resources.json.gz'
    graph = json.loads(gzip.decompress(graph_file.read_bytes()))
    edges = {e['id']: e for e in graph['routing_geometry']['edges']}
    by_robot = defaultdict(list)
    for row in rows:
        by_robot[row['agent_id']].append(row)
    paths, robots = [], []
    for robot, query, previous in zip(task['robots'], native['queries'], old['robots']):
        start = time.perf_counter()
        if robot['id'] != query['name'] or robot['id'] != previous['id'] or not query['validation_passed']:
            raise ValueError('Robot/candidate mapping differs')
        records = sorted(by_robot[robot['id']], key=lambda r: int(r['route_index']))
        if len(records) != len(query['routes']):
            raise ValueError('Candidate count differs from the probability report')
        route_edges = [graph_route_edges(route, edges) for route in query['routes']]
        distribution = edge_cost_distribution(route_edges, robot['radius'], lambda_weight=LAMBDA, tau=TAU)
        p = np.asarray(distribution['probabilities'])
        np.testing.assert_allclose(p, [float(r['cost_probability']) for r in records], rtol=1e-13, atol=1e-15)
        u = previous['uniform_variate']
        cumulative = np.cumsum(p); cumulative[-1] = 1.
        selected = int(np.searchsorted(cumulative, u, side='right'))
        candidates = []
        for index, (route, record, edge_list, cost) in enumerate(zip(query['routes'], records, route_edges, distribution['routes'])):
            if int(record['route_id']) != route['id'] or previous['routes'][index]['path_sha256'] != fingerprint(route['path']):
                raise ValueError('Candidate geometry differs from the preserved set')
            candidates.append(dict(index=index, route_id=route['id'], edges=edge_list,
                                   path_sha256=fingerprint(route['path']), **cost))
        paths.append(query['routes'][selected]['path'])
        robots.append(dict(id=robot['id'], available=len(p), routes=candidates,
                           w_max=distribution['w_max'], probabilities=p.tolist(), selected_index=selected,
                           selection_probability=float(p[selected]), uniform_variate=u,
                           baseline_uniform_index=previous['baseline_uniform_index'], jitter=previous['jitter'],
                           preparation_seconds=time.perf_counter()-start))
    if len(paths) != 32 or len(robots) != 32:
        raise ValueError('Incomplete fleet')
    check_paths(task, world, paths)
    original = json.loads((directory / 'run.json').read_text())
    record = dict(task_id=task_id, source_map=task['source_map'], scenario_sha256=task_hash,
                  selection_seed=42, requested_k=5, rule=RULE, lambda_weight=LAMBDA, tau=TAU,
                  edge_definition='Stored graph edge.length and edge.min_width; connectors excluded; full edge lengths for partial traversals.',
                  native_candidates_sha256=native_sha, baseline_run_sha256=sha(directory/'run.json'),
                  graph_bundle_sha256=sha(graph_file), bottleneck_input_sha256=sha(old_file),
                  original_global_planning_seconds=original['total_global_planning_seconds'],
                  original_robot_planning_seconds=original['per_robot_global_planning_seconds'],
                  preparation_seconds=time.perf_counter()-tick, paths=paths, robots=robots)
    filename = output / f'selection_inputs/{task_id}.json'
    save_json(filename, record)
    return dict(task_id=task_id, sha256=sha(filename), source_map=task['source_map'],
                agents=32, candidate_routes=sum(len(r['routes']) for r in robots))


def prepare_inputs(suite, baseline, weighted, probabilities, output, task_ids=None):
    suite, baseline, weighted, probabilities, output = map(Path, (suite, baseline, weighted, probabilities, output))
    output.mkdir(parents=True, exist_ok=True); (output/'selection_inputs').mkdir(exist_ok=True)
    contract = baseline_contract(suite, baseline)
    summary = json.loads((probabilities/'summary.json').read_text())
    verification = json.loads((probabilities/'verification.json').read_text())
    if (summary['lambda_weight'] != LAMBDA or summary['tau'] != TAU or not verification['passed']
            or sha(probabilities/'probabilities.csv') != verification['output_sha256']['probabilities.csv']):
        raise ValueError('Probability comparison has different parameters or changed data')
    implementation = {str(p.relative_to(ROOT)): sha(p) for p in (Path(__file__), Path(edge_cost_selection.__file__))}
    identity = fingerprint(dict(contract=contract, probabilities=sha(probabilities/'summary.json'),
                                implementation=implementation, task_ids=task_ids))
    preflight_file = output/'selection_preflight.json'
    if preflight_file.exists():
        saved = json.loads(preflight_file.read_text())
        if saved['identity'] != identity:
            raise ValueError('Existing edge-cost experiment has different inputs')
        for record in saved['selection_inputs']:
            if sha(output/f'selection_inputs/{record["task_id"]}.json') != record['sha256']:
                raise ValueError('Prepared edge-cost anchors changed')
        return saved
    manifest = json.loads((suite/'manifest.json').read_text())
    ids = [e['id'] for e in manifest['scenarios'] if task_ids is None or e['id'] in task_ids]
    if task_ids and set(task_ids)-set(ids):
        raise ValueError('Unknown task IDs')
    sources = {s['task_id']: s for s in json.loads((probabilities/'inputs.json').read_text())}
    rows = defaultdict(list)
    with (probabilities/'probabilities.csv').open() as stream:
        for row in csv.DictReader(stream):
            if row['task_id'] in ids:
                rows[row['task_id']].append(row)
    jobs = [(str(suite), str(baseline), str(weighted), str(output), task_id, rows[task_id], sources[task_id]) for task_id in ids]
    records = []
    with ProcessPoolExecutor(max_workers=4) as pool:
        for future in as_completed([pool.submit(_prepare_task, job) for job in jobs]):
            records.append(future.result())
            if len(records)%25 == 0:
                print(f'Prepared and clearance-checked {len(records)}/{len(ids)} tasks', flush=True)
    result = dict(passed=True, identity=identity, tasks=len(records), baseline_contract=contract,
                  probability_summary_sha256=sha(probabilities/'summary.json'),
                  probability_csv_sha256=sha(probabilities/'probabilities.csv'),
                  lambda_weight=LAMBDA, tau=TAU, selection_inputs=sorted(records, key=lambda r:r['task_id']))
    save_json(preflight_file, result)
    return result


def cost_anchors(task, world, planner, selection_seed, folder, k=5, *, suite_folder=None, prepared):
    if planner != METHOD or selection_seed != 42 or k != 5:
        raise ValueError('Unexpected edge-cost selection settings')
    record = json.loads((Path(prepared)/f'selection_inputs/{task["id"]}.json').read_text())
    if record['scenario_sha256'] != fingerprint({key:value for key,value in task.items() if key!='generation_seconds'}):
        raise ValueError('Edge-cost inputs used another task')
    save_json(Path(folder)/'paths.json', record['paths'])
    save_json(Path(folder)/'route_choices.json', dict(planner=METHOD, seed=selection_seed, sampling=RULE,
              lambda_weight=LAMBDA, tau=TAU, edge_definition=record['edge_definition'], robots=record['robots']))
    timings = [old+r['preparation_seconds'] for old,r in zip(record['original_robot_planning_seconds'],record['robots'])]
    return record['paths'], record['original_global_planning_seconds']+record['preparation_seconds'], timings


def run_task(arguments):
    suite, baseline, output, task_id = arguments
    output = Path(output)
    record_file = output/f'selection_inputs/{task_id}.json'
    record = json.loads(record_file.read_text())
    inputs = dict(selection_inputs_sha256=sha(record_file), native_candidates_sha256=record['native_candidates_sha256'],
                  graph_bundle_sha256=record['graph_bundle_sha256'], baseline_run_sha256=record['baseline_run_sha256'],
                  lambda_weight=LAMBDA, tau=TAU, selection_rule=RULE)
    result = run_evaluation(suite, task_id, METHOD, output,
              anchor_provider=partial(cost_anchors, prepared=output),
              execution_sources=[Path(__file__), Path(edge_cost_selection.__file__)], protocol_inputs=inputs)
    old = json.loads((Path(baseline)/f'{task_id}__k_routes__p42__c42/run.json').read_text())
    if result['controller_profile_sha256'] != old['controller_profile_sha256']:
        raise ValueError('Edge-cost controller differs from baseline')
    return result


def summarize(output):
    output = Path(output)
    runs = [json.loads(p.read_text()) for p in sorted(output.glob(f'*__{METHOD}__*/run.json'))]
    for name, keys in dict(overall=['planner'], by_map=['source_map','planner'],
            by_scenario=['source_map','scenario_id','diameter_meters','traffic_pattern','planner'],
            by_map_size=['source_map','diameter_meters','planner'],
            by_map_pattern=['source_map','traffic_pattern','planner']).items():
        table = aggregate_rows(runs, keys); save_json(output/f'{name}.json',table); write_csv(table,output/f'{name}.csv')
    save_json(output/'progress_summary.json', dict(completed_runs=len(runs), outcomes=dict(Counter(r['status'] for r in runs))))
    return runs


def run_suite(suite, baseline, weighted, probabilities, output, *, workers=20, audit_workers=8,
              task_ids=None, prepare_only=False, publish=True):
    suite, baseline, weighted, probabilities, output = [Path(p).resolve() for p in (suite,baseline,weighted,probabilities,output)]
    preflight = prepare_inputs(suite,baseline,weighted,probabilities,output,task_ids)
    if prepare_only:
        return preflight
    save_json(output/'experiment.json',dict(expected_runs=preflight['tasks'], workers=workers,
              method=METHOD, config=asdict(EvaluationConfig()), lambda_weight=LAMBDA, tau=TAU,
              baseline_contract=preflight['baseline_contract'], selection_preflight_sha256=sha(output/'selection_preflight.json'),
              normal_mppi=True, sampling='Same coupled uniform variates as the bottleneck policy; inverse CDF of edge-cost probabilities.'))
    ids = [r['task_id'] for r in preflight['selection_inputs']]
    # Interleave maps so partial progress does not consist of only one map.
    ids = sorted(ids,key=lambda x:(x.split('_',1)[1],x.split('_',1)[0]))
    with ProcessPoolExecutor(max_workers=workers) as pool:
        futures = [pool.submit(run_task,(str(suite),str(baseline),str(output),task_id)) for task_id in ids]
        for count, future in enumerate(as_completed(futures),1):
            run = future.result()
            print(f'[{count}/{len(ids)}] {run["run_id"]}: {run["status"]}, agents={round(32*run["completion_fraction"])}/32, makespan={run["makespan_seconds"]}',flush=True)
            if count%5 == 0:
                summarize(output)
    summarize(output)
    if publish and preflight['tasks']==250:
        from .warehouse_cost_report import publish_results
        return publish_results(suite,baseline,weighted,output,workers=audit_workers)
    return output


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--suite',type=Path,default=ROOT/'benchmarks/scenarios/warehouse_uniform')
    parser.add_argument('--baseline',type=Path,default=ROOT/'benchmarks/results/warehouse_uniform_corrected')
    parser.add_argument('--weighted',type=Path,default=ROOT/'benchmarks/results/warehouse_bottleneck_width')
    parser.add_argument('--probabilities',type=Path,default=ROOT/'benchmarks/scenarios/warehouse_uniform/edge_cost_probabilities')
    parser.add_argument('--output',type=Path,default=ROOT/'benchmarks/results/warehouse_edge_cost')
    parser.add_argument('--workers',type=int,default=20)
    parser.add_argument('--audit-workers',type=int,default=8)
    parser.add_argument('--task',action='append',dest='task_ids')
    parser.add_argument('--prepare-only',action='store_true')
    parser.add_argument('--no-publish',action='store_false',dest='publish')
    run_suite(**vars(parser.parse_args()))


if __name__ == '__main__':
    main()
