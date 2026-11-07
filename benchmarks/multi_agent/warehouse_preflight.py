"""Check every frozen assignment before running paired local controllers."""

from concurrent.futures import ProcessPoolExecutor, as_completed
import hashlib
import json
from pathlib import Path
import time

import numpy as np
import shapely

from .generation import save_json
from .geometry import ROOT, fingerprint
from .runner import global_routes


def anchor_identity(task, planner, seed, k):
    sources = [ROOT/'benchmarks/multi_agent/runner.py', ROOT/'benchmarks/rrt_connect.py',
               ROOT/'benchmarks/k_routes.py', ROOT/'build/routes/corridor_plan', Path(__file__)]
    return fingerprint(dict(task=task, planner=planner, seed=seed, k=k,
        code={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sources}))


def check_paths(task, world, paths):
    if len(paths) != len(task['robots']): raise ValueError('Incomplete fleet anchors')
    obstacle = shapely.union_all([shapely.Polygon(p) for p in world['obstacles']])
    boundary = shapely.box(*world['bounds']).boundary
    for robot, path in zip(task['robots'],paths):
        np.testing.assert_allclose(path[0],robot['start'][:2],atol=1e-8)
        np.testing.assert_allclose(path[-1],robot['goal']['pose'][:2],atol=1e-8)
        line=shapely.LineString(path); radius=robot['radius']+robot['safety_margin']
        if not line.is_simple: raise ValueError(f"Self-intersecting anchor for {robot['id']}")
        if (line.distance(obstacle) < radius-1e-7 or
            line.distance(boundary) < radius-1e-7 or
            not shapely.box(*world['bounds']).covers(line)):
            raise ValueError(f"Anchor lacks swept-disk clearance for {robot['id']}")


def check_joint_certificate(folder,task):
    destination=Path(folder)/'joint_feasibility'/task['id']
    path=destination/'validation.json'
    if not path.exists():raise ValueError(f"Missing joint feasibility certificate for {task['id']}; run certify-warehouse first")
    record=json.loads(path.read_text())
    identity=fingerprint({k:v for k,v in task.items() if k!='generation_seconds'})
    if not record['passed'] or record['task_sha256']!=identity or record['checked_robots']!=task['robot_count']:
        raise ValueError(f"Invalid joint feasibility certificate for {task['id']}")
    arrays=destination/'joint_witness.npz'
    if hashlib.sha256(arrays.read_bytes()).hexdigest()!=record['joint_witness_sha256']:
        raise ValueError(f"Joint feasibility trace changed for {task['id']}")
    return dict(validation_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                witness_sha256=record['joint_witness_sha256'])


def plan_checked(folder, task_id, planner, output, seed=42, k=5):
    folder, output = Path(folder), Path(output)
    task=json.loads((folder/'tasks'/f'{task_id}.json').read_text())
    world=json.loads((folder/'maps.json').read_text())['worlds'][task['source_map']]
    destination=output/f'{task_id}__{planner}__p{seed}__c42';destination.mkdir(parents=True,exist_ok=True)
    identity=anchor_identity(task,planner,seed,k)
    metadata=destination/'anchors_validation.json'
    if metadata.exists():
        saved=json.loads(metadata.read_text())
        if saved['identity']==identity and saved['passed']:
            paths=json.loads((destination/'paths.json').read_text());check_paths(task,world,paths)
            return saved
    tick=time.monotonic()
    try:
        paths,seconds,timings=global_routes(task,world,planner,seed,destination,k,suite_folder=folder)
        check_paths(task,world,paths)
        result=dict(passed=True,identity=identity,task_id=task_id,planner=planner,
            total_global_planning_seconds=seconds,per_robot_global_planning_seconds=timings,
            paths_sha256=hashlib.sha256((destination/'paths.json').read_bytes()).hexdigest(),
            choices_sha256=hashlib.sha256((destination/'route_choices.json').read_bytes()).hexdigest())
    except (ValueError,RuntimeError,AssertionError) as error:
        result=dict(passed=False,identity=identity,task_id=task_id,planner=planner,error=str(error))
    result['preflight_seconds']=time.monotonic()-tick
    save_json(metadata,result)
    return result


def preflight_suite(folder,output,workers=12,task_ids=None):
    folder,output=Path(folder),Path(output);output.mkdir(parents=True,exist_ok=True)
    manifest=json.loads((folder/'manifest.json').read_text())
    ids=[e['id'] for e in manifest['scenarios'] if task_ids is None or e['id'] in task_ids]
    results=[]
    with ProcessPoolExecutor(max_workers=workers) as pool:
        futures=[pool.submit(plan_checked,folder,i,m,output) for i in ids for m in ('rrt','k_routes')]
        for f in as_completed(futures):
            r=f.result();results.append(r)
            print(f"[{len(results)}/{2*len(ids)}] {r['task_id']} {r['planner']}: anchors {r['passed']} {r.get('error','')}",flush=True)
    report=dict(expected_runs=2*len(ids),passed_runs=sum(r['passed'] for r in results),
        passed=all(r['passed'] for r in results),runs=sorted(results,key=lambda r:(r['task_id'],r['planner'])))
    save_json(output/'preflight.json',report)
    return report
