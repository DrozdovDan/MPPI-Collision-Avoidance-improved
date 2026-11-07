"""Paired width-proportional selection from the frozen keep-right candidate sets."""
from collections import Counter
from concurrent.futures import ProcessPoolExecutor,as_completed
from dataclasses import asdict
from functools import partial
import hashlib
import json
from pathlib import Path
import time

import numpy as np

from . import bottleneck_selection
from .bottleneck_selection import CorridorWidths,audit_lane_geometry,draw_width_route
from .geometry import ROOT,fingerprint
from .warehouse_evaluation import EvaluationConfig,load_task,run_evaluation,save_json,aggregate_rows,write_csv

METHOD='k_routes_width'
JITTER_TAG=0x57494454
WIDTH_DEFINITION='Minimum full wall-to-wall free cross-section normal to the physical reference path, before lane displacement; no radius subtraction.'


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def baseline_contract(suite,baseline):
    suite,baseline=Path(suite).resolve(),Path(baseline).resolve()
    audit=json.loads((baseline/'audit.json').read_text())
    experiment=json.loads((baseline/'experiment.json').read_text())
    if not audit['passed'] or audit['runs']!=500:raise ValueError('A complete audited baseline is required')
    if sha(suite/'manifest.json')!=experiment['frozen_manifest_sha256']:raise ValueError('Frozen tasks changed')
    if experiment['config']!=asdict(EvaluationConfig()):raise ValueError('Baseline controller/configuration differs')
    historical=[]
    for name,digest in audit['source_and_binary_hashes'].items():
        if sha(ROOT/name)==digest:continue
        snapshot=baseline/'source_snapshots'/f'warehouse_evaluation.{digest}.py'
        if name!='benchmarks/multi_agent/warehouse_evaluation.py' or not snapshot.exists() or sha(snapshot)!=digest:
            raise ValueError(f'Baseline controller, simulator or routing implementation changed: {name}')
        historical.append(dict(source=name,executed_sha256=digest,snapshot=str(snapshot.relative_to(ROOT))))
    return dict(baseline_audit_sha256=sha(baseline/'audit.json'),
        baseline_experiment_sha256=sha(baseline/'experiment.json'),
        frozen_manifest_sha256=sha(suite/'manifest.json'),historical_harness_sources=historical,
        controller_library_sha256=sha(ROOT/'build/release/libcontroller_library.so'),
        normal_mppi_module_sha256=sha(ROOT/'build/release/mppi_ca_cpp.so'))


def _prepare_map(arguments):
    suite,baseline,output,name,task_ids,cached_hashes=arguments
    suite,baseline,output=map(Path,(suite,baseline,output))
    worlds=json.loads((suite/'maps.json').read_text())['worlds'];widths=CorridorWidths(worlds[name])
    records=[];counts=Counter();minimum=None;max_center=0.
    implementation_sha256=sha(Path(bottleneck_selection.__file__))
    for ordinal,task_id in enumerate(task_ids,1):
        task,world,task_hash=load_task(suite,task_id)
        directory=baseline/f'{task_id}__k_routes__p42__c42'
        native_file=directory/'agents.routes.json';native=json.loads(native_file.read_text())
        old=json.loads((directory/'route_choices.json').read_text())
        old_run=json.loads((directory/'run.json').read_text())
        filename=output/'selection_inputs'/f'{task_id}.json'
        if filename.exists() and cached_hashes.get(task_id)==sha(filename):
            cached=json.loads(filename.read_text())
            if (cached['selection_implementation_sha256']==implementation_sha256 and
                cached['native_candidates_sha256']==sha(native_file) and cached['scenario_sha256']==task_hash and
                cached['baseline_run_sha256']==sha(directory/'run.json')):
                for robot in cached['robots']:
                    for route in robot['routes']:
                        lane=route['lane_audit'];counts['candidate_routes']+=1
                        for key in ('split_sections','shared_corridor_sections','transition_sections'):counts[key]+=lane[key]
                        if lane['minimum_right_offset'] is not None:
                            minimum=lane['minimum_right_offset'] if minimum is None else min(minimum,lane['minimum_right_offset'])
                        max_center=max(max_center,lane['maximum_shared_deviation'])
                records.append(dict(task_id=task_id,sha256=sha(filename)))
                continue
        rng=np.random.default_rng(42);jitter=np.random.default_rng(np.random.SeedSequence([42,JITTER_TAG]))
        prepared=[];start=time.perf_counter()
        for robot,query,choice in zip(task['robots'],native['queries'],old['robots']):
            if robot['id']!=query['name'] or robot['id']!=choice['id'] or not query['validation_passed']:
                raise ValueError('Candidate/robot mapping or native validation differs')
            if query['radius']!=robot['radius'] or query['safety_margin']!=robot['safety_margin']:
                raise ValueError('Candidate radius differs from the physical robot')
            np.testing.assert_allclose(query['start'],robot['start'][:2],atol=1e-10)
            np.testing.assert_allclose(query['goal'],robot['goal']['pose'][:2],atol=1e-10)
            tick=time.perf_counter();routes=[]
            for index,route in enumerate(query['routes']):
                witness=widths.route(route);widths.check_witness(witness)
                lane=audit_lane_geometry(route,robot['radius'],robot['safety_margin'])
                counts['candidate_routes']+=1
                for key in ('split_sections','shared_corridor_sections','transition_sections'):counts[key]+=lane[key]
                if lane['minimum_right_offset'] is not None:
                    minimum=lane['minimum_right_offset'] if minimum is None else min(minimum,lane['minimum_right_offset'])
                max_center=max(max_center,lane['maximum_shared_deviation'])
                routes.append(dict(index=index,route_id=route['id'],bottleneck_width=witness['width'],
                    width_witness=witness,path_sha256=fingerprint(route['path']),lane_audit=lane))
            uniform_index=int(rng.integers(len(routes)));noise=float(jitter.random())
            if uniform_index!=choice['selected_index']:raise ValueError('Original uniform draw does not replay')
            selected,p,u=draw_width_route([r['bottleneck_width'] for r in routes],uniform_index,noise)
            prepared.append(dict(id=robot['id'],available=len(routes),routes=routes,
                probabilities=p.tolist(),selected_index=selected,selection_probability=float(p[selected]),
                baseline_uniform_index=uniform_index,jitter=noise,uniform_variate=u,
                preparation_seconds=time.perf_counter()-tick))
        if len(prepared)!=32:raise ValueError('Incomplete full-fleet candidate set')
        record=dict(task_id=task_id,source_map=name,scenario_sha256=task_hash,selection_seed=42,requested_k=5,
            rule='p_i = bottleneck_width_i / sum_j bottleneck_width_j',width_definition=WIDTH_DEFINITION,
            selection_implementation_sha256=implementation_sha256,
            native_candidates_sha256=sha(native_file),baseline_run_sha256=sha(directory/'run.json'),
            original_global_planning_seconds=old_run['total_global_planning_seconds'],
            original_robot_planning_seconds=old_run['per_robot_global_planning_seconds'],
            preparation_seconds=time.perf_counter()-start,robots=prepared)
        save_json(filename,record)
        records.append(dict(task_id=task_id,sha256=sha(filename)))
        if ordinal%5==0:print(f'Prepared {name}: {ordinal}/{len(task_ids)} tasks; {len(widths.cache)} cached physical chords',flush=True)
    return dict(source_map=name,passed=True,tasks=len(records),selection_inputs=records,
        counts=dict(counts),minimum_right_offset=minimum,maximum_shared_deviation=max_center)


def prepare_inputs(suite,baseline,output,task_ids=None):
    suite,baseline,output=(Path(p).resolve() for p in (suite,baseline,output));output.mkdir(parents=True,exist_ok=True)
    (output/'selection_inputs').mkdir(exist_ok=True)
    contract=baseline_contract(suite,baseline)
    entries=json.loads((suite/'manifest.json').read_text())['scenarios']
    if task_ids is not None and set(task_ids)-{e['id'] for e in entries}:raise ValueError('Unknown warehouse task')
    entries=[e for e in entries if task_ids is None or e['id'] in task_ids]
    cached={}
    if (output/'selection_preflight.json').exists():
        previous=json.loads((output/'selection_preflight.json').read_text())
        cached={r['task_id']:r['sha256'] for m in previous['maps'] for r in m['selection_inputs']}
    groups={m:[e['id'] for e in entries if e['source_map']==m] for m in ('warehouse','warehouse2')}
    jobs=[(str(suite),str(baseline),str(output),m,ids,cached) for m,ids in groups.items() if ids]
    with ProcessPoolExecutor(max_workers=2) as pool:maps=list(pool.map(_prepare_map,jobs))
    report=dict(passed=True,tasks=len(entries),baseline_contract=contract,maps=maps,
        candidate_routes=sum(m['counts']['candidate_routes'] for m in maps),
        width_definition=WIDTH_DEFINITION,rule='p_i = w_i / sum_j w_j',jitter_seed_tag=JITTER_TAG)
    save_json(output/'selection_preflight.json',report)
    return report


def weighted_anchors(task,world,planner,selection_seed,folder,k=5,*,suite_folder=None,baseline,prepared):
    if planner!=METHOD:raise ValueError('Unexpected weighted-selection method')
    baseline,prepared,folder=map(Path,(baseline,prepared,folder))
    record=json.loads((prepared/'selection_inputs'/f'{task["id"]}.json').read_text())
    if record['scenario_sha256']!=fingerprint({a:b for a,b in task.items() if a!='generation_seconds'}):
        raise ValueError('Width selection used another task')
    if record['selection_seed']!=selection_seed or record['requested_k']!=k:raise ValueError('Selection settings changed')
    source=baseline/f'{task["id"]}__k_routes__p42__c42/agents.routes.json'
    if sha(source)!=record['native_candidates_sha256']:raise ValueError('Frozen K-route candidates changed')
    native=json.loads(source.read_text());paths=[];robots=[];timings=[]
    for robot,query,item in zip(task['robots'],native['queries'],record['robots']):
        index=item['selected_index'];path=query['routes'][index]['path']
        if item['id']!=robot['id'] or fingerprint(path)!=item['routes'][index]['path_sha256']:
            raise ValueError('Selected keep-right geometry changed')
        paths.append(path);robots.append(item)
        timings.append(record['original_robot_planning_seconds'][len(timings)]+item['preparation_seconds'])
    choices=dict(planner=METHOD,seed=selection_seed,sampling=record['rule'],width_definition=WIDTH_DEFINITION,
        jitter_seed_tag=JITTER_TAG,candidate_generation_reused_from=str(source.relative_to(ROOT)),robots=robots)
    save_json(folder/'route_choices.json',choices);save_json(folder/'paths.json',paths)
    return paths,record['original_global_planning_seconds']+record['preparation_seconds'],timings


def run_weighted_task(arguments):
    suite,baseline,output,task_id=arguments
    suite,baseline,output=(Path(p).resolve() for p in (suite,baseline,output))
    record=output/'selection_inputs'/f'{task_id}.json';data=json.loads(record.read_text())
    inputs=dict(selection_inputs_sha256=sha(record),native_candidates_sha256=data['native_candidates_sha256'],
        baseline_run_sha256=data['baseline_run_sha256'],selection_rule=data['rule'],width_definition=WIDTH_DEFINITION)
    provider=partial(weighted_anchors,baseline=baseline,prepared=output)
    result=run_evaluation(suite,task_id,METHOD,output,anchor_provider=provider,
        execution_sources=[Path(__file__),Path(bottleneck_selection.__file__)],protocol_inputs=inputs)
    old=json.loads((baseline/f'{task_id}__k_routes__p42__c42/run.json').read_text())
    if result['controller_profile_sha256']!=old['controller_profile_sha256']:
        raise ValueError('Weighted controller profile differs from the frozen uniform baseline')
    return result


def summarize_width_runs(output):
    output=Path(output);runs=[json.loads(p.read_text()) for p in sorted(output.glob(f'*__{METHOD}__*/run.json'))]
    views=dict(by_scenario=['source_map','scenario_id','diameter_meters','traffic_pattern','planner'],
        by_map_size=['source_map','diameter_meters','planner'],by_map_pattern=['source_map','traffic_pattern','planner'],
        by_map=['source_map','planner'],overall=['planner'])
    for name,keys in views.items():
        table=aggregate_rows(runs,keys);save_json(output/f'{name}.json',table);write_csv(table,output/f'{name}.csv')
    save_json(output/'progress_summary.json',dict(completed_runs=len(runs),outcomes=dict(Counter(r['status'] for r in runs))))
    return runs


def run_width_suite(suite,baseline,output,*,workers=20,task_ids=None,prepare_only=False,publish=True):
    suite,baseline,output=(Path(p).resolve() for p in (suite,baseline,output))
    preflight=prepare_inputs(suite,baseline,output,task_ids)
    if prepare_only:return preflight
    ids=[r['task_id'] for m in preflight['maps'] for r in m['selection_inputs']]
    save_json(output/'experiment.json',dict(expected_runs=len(ids),workers=workers,config=asdict(EvaluationConfig()),
        method=METHOD,baseline_directory=str(baseline.relative_to(ROOT)),normal_mppi=True,
        baseline_contract=preflight['baseline_contract'],selection_preflight_sha256=sha(output/'selection_preflight.json')))
    with ProcessPoolExecutor(max_workers=workers) as pool:
        futures=[pool.submit(run_weighted_task,(str(suite),str(baseline),str(output),i)) for i in ids]
        for completed,future in enumerate(as_completed(futures),1):
            r=future.result();print(f"[{completed}/{len(ids)}] {r['run_id']}: {r['status']}, agents={r['completion_fraction']:.3f}, makespan={r['makespan_seconds']}",flush=True)
            if completed%10==0:summarize_width_runs(output)
    summarize_width_runs(output)
    if publish and len(ids)==250:
        from .warehouse_width_report import publish_width_results
        return publish_width_results(suite,baseline,output)
    return output
