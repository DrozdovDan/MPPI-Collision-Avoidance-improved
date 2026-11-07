"""Paired five-method replay of the warehouse case with unrestricted route length."""
import argparse
from concurrent.futures import ProcessPoolExecutor, as_completed
from dataclasses import asdict
from functools import partial
import gzip
import json
import math
import os
from pathlib import Path
import shutil
import time
import numpy as np

from .geometry import ROOT, fingerprint
from . import bottleneck_selection, edge_cost_selection, warehouse_disappear_runtime
from .bottleneck_selection import CorridorWidths, audit_lane_geometry, proportional_probabilities
from .edge_cost_selection import edge_cost_distribution, graph_route_edges
from .warehouse_evaluation import EvaluationConfig, load_task, save_json
from .warehouse_disappear_evaluation import run_disappear_evaluation
from .warehouse_disappear_benchmark import METHODS, POLICY
from .warehouse_width_benchmark import baseline_contract, sha
from .warehouse_preflight import check_paths

TASK='warehouse2_s02_v02'
SUITE=ROOT/'benchmarks/scenarios/warehouse_uniform'
POOL=ROOT/f'benchmarks/results/warehouse_disappear_route_diagnosis/{TASK}/no_length_filter/budget128.routes.json'
OLD=ROOT/'benchmarks/results/warehouse_disappear'


def preparation_identity():
    contract=baseline_contract(SUITE,ROOT/'benchmarks/results/warehouse_uniform_corrected')
    common=ROOT/f'benchmarks/results/warehouse_bottleneck_width/selection_inputs/{TASK}.json'
    return dict(task=TASK,candidate_pool_sha256=sha(POOL),common_random_inputs_sha256=sha(common),
        graph_bundle_sha256=sha(SUITE/'geometry_cache/warehouse2.resources.json.gz'),
        config=asdict(EvaluationConfig()),baseline_contract=contract,
        implementation_sha256={str(p.relative_to(ROOT)):sha(p) for p in [Path(__file__),
            Path(bottleneck_selection.__file__),Path(edge_cost_selection.__file__),Path(warehouse_disappear_runtime.__file__)]})


def prepare(output):
    output=Path(output);output.mkdir(parents=True,exist_ok=True);(output/'selection_inputs').mkdir(exist_ok=True)
    identity=preparation_identity();filename=output/'experiment.json'
    if filename.exists():
        cached=json.loads(filename.read_text())
        if cached['identity']!=identity:raise ValueError('Existing experiment identity differs')
        for method,record in cached['selection_inputs'].items():
            if sha(output/f'selection_inputs/{method}.json')!=record['sha256']:raise ValueError('Selection inputs changed')
        if sha(output/'candidates.routes.json')!=identity['candidate_pool_sha256']:raise ValueError('Candidate pool changed')
        return cached
    task,world,task_hash=load_task(SUITE,TASK)
    pool=json.loads(POOL.read_text());config=pool['config'];queries=pool['queries']
    if config['max_candidates']!=128 or config['max_detour_ratio']!=1e12 or len(queries)!=32:
        raise ValueError('Wrong diagnostic candidate set')
    graph=json.loads(gzip.decompress((SUITE/'geometry_cache/warehouse2.resources.json.gz').read_bytes()))
    edges={e['id']:e for e in graph['routing_geometry']['edges']}
    # A positive-weight shortest-path search is loop-free. Twice all reference
    # edge lengths plus every possible source/target connector is a conservative
    # upper bound on every search candidate, proving 1e12 cannot prune a path.
    bounds=np.array(world['bounds']);diagonal=float(np.linalg.norm(bounds[2:]-bounds[:2]))
    length_upper=2*sum(float(e['reference_distances'][-1]) for e in edges.values())+(2*config['max_attachments']+1)*diagonal
    reference=json.loads((ROOT/f'benchmarks/results/warehouse_bottleneck_width/selection_inputs/{TASK}.json').read_text())
    widths=CorridorWidths(world);all_candidates=[];minimum_baseline=float('inf')
    for robot,query,old in zip(task['robots'],queries,reference['robots']):
        if robot['id']!=query['name'] or robot['id']!=old['id'] or not query['validation_passed'] or len(query['routes'])!=5:
            raise ValueError('Every agent must have five certified candidates')
        if query['radius']!=robot['radius'] or query['safety_margin']!=robot['safety_margin']:raise ValueError('Radius differs')
        np.testing.assert_allclose(query['start'],robot['start'][:2],atol=1e-12)
        np.testing.assert_allclose(query['goal'],robot['goal']['pose'][:2],atol=1e-12)
        minimum_baseline=min(minimum_baseline,query['routes'][0]['physical_metrics']['length'])
        routes=[]
        for index,route in enumerate(query['routes']):
            witness=widths.route(route);widths.check_witness(witness)
            lane=audit_lane_geometry(route,robot['radius'],robot['safety_margin'])
            routes.append(dict(index=index,route_id=route['id'],path_sha256=fingerprint(route['path']),
                bottleneck_width=witness['width'],width_witness=witness,lane_audit=lane,
                edges=graph_route_edges(route,edges)))
        all_candidates.append(routes)
    if not 1e12*minimum_baseline>length_upper:raise ValueError('Diagnostic length cap could bind')
    # Check every native path, not just the sampled anchors.
    for index in range(5):check_paths(task,world,[q['routes'][index]['path'] for q in queries])
    shutil.copyfile(POOL,output/'candidates.routes.json')
    selection_inputs={}
    for method in METHODS:
        tick=time.perf_counter();robots=[];paths=[]
        if method=='rrt':
            directory=OLD/f'{TASK}__rrt__p42__c42';old_run=json.loads((directory/'run.json').read_text())
            paths=json.loads((directory/'paths.json').read_text());choices=json.loads((directory/'route_choices.json').read_text())
            planning=old_run['total_global_planning_seconds'];timings=old_run['per_robot_global_planning_seconds']
        else:
            for robot,query,routes,old in zip(task['robots'],queries,all_candidates,reference['robots']):
                u=old['uniform_variate'];distribution=None
                if method=='k_routes':probabilities=[.2]*5
                elif method=='k_routes_width':probabilities=proportional_probabilities([r['bottleneck_width'] for r in routes]).tolist()
                else:
                    weight,temperature=(1.,1.) if method=='k_routes_edge_cost' else (2.,10.)
                    distribution=edge_cost_distribution([r['edges'] for r in routes],robot['radius'],lambda_weight=weight,tau=temperature)
                    probabilities=distribution['probabilities']
                cumulative=np.cumsum(probabilities);cumulative[-1]=1.
                index=int(np.searchsorted(cumulative,u,side='right'))
                robots.append(dict(id=robot['id'],available=5,routes=routes,probabilities=probabilities,
                    selected_index=index,selection_probability=probabilities[index],uniform_variate=u,
                    distribution=distribution))
                paths.append(query['routes'][index]['path'])
            choices=dict(planner=method,selection_seed=42,robots=robots,common_uniform_variates_reused=True)
            timings=[q['planning_seconds'] for q in queries]
            planning=pool['map_load_seconds']+sum(timings)
        check_paths(task,world,paths)
        record=dict(task_id=TASK,method=method,scenario_sha256=task_hash,paths=paths,choices=choices,
            per_robot_global_planning_seconds=timings,total_global_planning_seconds=planning,
            preparation_seconds=time.perf_counter()-tick,selection_seed=42,controller_seed=42)
        save_json(output/f'selection_inputs/{method}.json',record)
        selection_inputs[method]=dict(sha256=sha(output/f'selection_inputs/{method}.json'))
    experiment=dict(identity=identity,expected_runs=5,methods=list(METHODS),task_id=TASK,target_policy=POLICY,
        candidate_generation_config=config,returned_k_per_agent=[5]*32,
        no_length_pruning_proof=dict(candidate_reference_length_upper_bound_meters=length_upper,
            minimum_shortest_reference_length_meters=minimum_baseline,cap_value=1e12,
            minimum_cap_meters=1e12*minimum_baseline,cap_cannot_bind=True),
        common_random_policy='Reuse each saved per-agent uniform variate U across old/new pools and all four K sampling policies; uniform selects floor(5*U). No retries or route switching.',
        selection_inputs=selection_inputs,normal_mppi=True,physics_and_controller_unchanged=True)
    save_json(filename,experiment)
    return experiment


def anchors(task,world,planner,selection_seed,destination,k,*,suite_folder,prepared):
    record=json.loads((Path(prepared)/f'selection_inputs/{planner}.json').read_text())
    save_json(Path(destination)/'paths.json',record['paths']);save_json(Path(destination)/'route_choices.json',record['choices'])
    return record['paths'],record['total_global_planning_seconds'],record['per_robot_global_planning_seconds']


def run_method(output,method):
    output=Path(output);experiment=json.loads((output/'experiment.json').read_text())
    protocol=dict(target_policy=POLICY,task_id=TASK,method=method,
        candidate_pool_sha256=experiment['identity']['candidate_pool_sha256'],
        selection_inputs_sha256=sha(output/f'selection_inputs/{method}.json'),
        experiment_sha256=sha(output/'experiment.json'),max_candidates=128,length_filter_disabled=True)
    if method.startswith('k_routes_edge_cost'):
        protocol.update(lambda_weight=1. if method=='k_routes_edge_cost' else 2.,tau=1. if method=='k_routes_edge_cost' else 10.)
    run=run_disappear_evaluation(SUITE,TASK,method,output,anchor_provider=partial(anchors,prepared=output),
        execution_sources=[Path(__file__),Path(bottleneck_selection.__file__),Path(edge_cost_selection.__file__),
            Path(warehouse_disappear_runtime.__file__),ROOT/'benchmarks/multi_agent/warehouse_evaluation.py'],
        protocol_inputs=protocol)
    old=json.loads((OLD/f'{TASK}__{method}__p42__c42/run.json').read_text())
    if run['controller_profiles']!=old['controller_profiles'] or run['config']!=old['config']:
        raise ValueError('Controller or configuration changed')
    return {key:run[key] for key in ('run_id','planner','status','completion_fraction','makespan_seconds','simulated_seconds')}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,default=ROOT/f'benchmarks/results/warehouse_open_routes_{TASK}')
    parser.add_argument('--workers',type=int,default=5)
    parser.add_argument('--prepare-only',action='store_true')
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    experiment=prepare(args.output)
    if args.prepare_only:return
    save_json(args.output/'execution_state.json',dict(status='running',pid=os.getpid(),expected_runs=5,workers=args.workers))
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        futures=[pool.submit(run_method,str(args.output),method) for method in METHODS]
        for count,future in enumerate(as_completed(futures),1):
            run=future.result();print(f'[{count}/5] {run["planner"]}: {run["status"]}, agents={round(32*run["completion_fraction"])}/32, makespan={run["makespan_seconds"]}',flush=True)
    from .warehouse_open_routes_report import publish
    publish(args.output)
    save_json(args.output/'execution_state.json',dict(status='complete',pid=os.getpid(),runs=5,workers=args.workers))


if __name__=='__main__':main()
