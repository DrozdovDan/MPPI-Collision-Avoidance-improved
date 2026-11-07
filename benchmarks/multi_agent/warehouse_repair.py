"""Replace uncertified endpoint variants using joint feasibility, never MPPI outcomes."""

import copy
import json
from pathlib import Path
import shutil
import time

import numpy as np

from .generation import save_json
from .geometry import fingerprint,validate_task
from .warehouse_generation import (UniformWarehouseGenerator,WAREHOUSE_MAPS,_traffic_snapshot,
    construct_variant,scenario_definitions)
from .warehouse_joint_feasibility import joint_certificate,replay_joint_certificate


def repair_task(folder,task_id,certificates,workspace):
    folder,certificates,workspace=Path(folder),Path(certificates),Path(workspace)
    old=json.loads((folder/'tasks'/f'{task_id}.json').read_text())
    worlds=json.loads((folder/'maps.json').read_text())['worlds']
    design=next(d for d in scenario_definitions() if d['id']==old['scenario_id'])
    workspace.mkdir(parents=True,exist_ok=True);(workspace/'tasks').mkdir(exist_ok=True)
    (workspace/'witnesses').mkdir(exist_ok=True);(workspace/'geometry_cache').mkdir(exist_ok=True)
    shutil.copy2(folder/'maps.json',workspace/'maps.json')
    traffic=_traffic_snapshot(folder,old['source_map'])
    notes=[]
    for attempt in range(12):
        tick=time.monotonic()
        config=copy.deepcopy(old['generation']['config'])|dict(focused_core_robot_count=8,
            seed=old['generation']['config']['seed']+attempt*10_000_000)
        generator=UniformWarehouseGenerator(old['source_map'],worlds[old['source_map']],traffic,
            old['physical_scale']['diameter_meters'],config)
        others=[json.loads(p.read_text()) for p in (folder/'tasks').glob(old['scenario_id']+'_v*.json')
                if p.stem!=task_id]
        task,arrays,audit=construct_variant(generator,design,old['variant_index'],
            WAREHOUSE_MAPS.index(old['source_map']),config,others)
        task['witness_file']=f'witnesses/{task_id}.npz'
        task['validation']=validate_task(task,worlds[task['source_map']],arrays)
        task['generation_repair']=dict(reason='joint completion not certified for the original endpoint assignment',
            original_task_sha256=fingerprint({k:v for k,v in old.items() if k!='generation_seconds'}),
            focused_core_robot_count=8,selection='first individual and joint certified candidate; no evaluated planner or controller outcomes',
            repair_attempt=attempt,generation_attempts=audit)
        task['joint_feasibility_file']=f'joint_feasibility/{task_id}/validation.json'
        task['generation_seconds']=time.monotonic()-tick
        save_json(workspace/'tasks'/f'{task_id}.json',task)
        np.savez_compressed(workspace/task['witness_file'],**arrays)
        result=joint_certificate(workspace,task_id,workspace/'certificates')
        notes.append(dict(attempt=attempt,passed=result['passed'],assignment_sha256=task['assignment_sha256']))
        save_json(workspace/'repair_attempts.json',notes)
        print(f"{task_id} repair candidate {attempt}: joint certificate {result['passed']}",flush=True)
        if not result['passed']:continue
        replay_joint_certificate(workspace,task_id,workspace/'certificates')
        archive=folder/'revisions/individual_only_v1';(archive/'tasks').mkdir(parents=True,exist_ok=True)
        (archive/'witnesses').mkdir(exist_ok=True)
        for relative in [Path('tasks')/f'{task_id}.json',Path(old['witness_file'])]:
            target=archive/relative
            if not target.exists():shutil.copy2(folder/relative,target)
        shutil.copy2(workspace/'tasks'/f'{task_id}.json',folder/'tasks'/f'{task_id}.json')
        shutil.copy2(workspace/task['witness_file'],folder/task['witness_file'])
        shutil.copytree(workspace/'certificates'/task_id,certificates/task_id,dirs_exist_ok=True)
        return dict(task_id=task_id,passed=True,original_task_sha256=task['generation_repair']['original_task_sha256'],
            new_task_sha256=result['task_sha256'],attempts=notes)
    raise RuntimeError(f'No joint-certified replacement for {task_id}; original preserved')


def update_manifest(folder,repairs):
    folder=Path(folder);path=folder/'manifest.json';manifest=json.loads(path.read_text())
    archive=folder/'revisions/individual_only_v1';archive.mkdir(parents=True,exist_ok=True)
    if not (archive/'manifest.json').exists():shutil.copy2(path,archive/'manifest.json')
    for item in manifest['scenarios']:
        task=json.loads((folder/item['file']).read_text())
        item['scenario_sha256']=fingerprint({k:v for k,v in task.items() if k!='generation_seconds'})
    manifest['joint_feasibility_revision']=2
    save_json(path,manifest)
    save_json(folder/'revisions/joint_feasibility_repairs.json',dict(
        reason='Require a constructive joint completion schedule, independent of both evaluated planners and MPPI',
        repairs=repairs))
