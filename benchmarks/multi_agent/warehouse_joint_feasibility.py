"""Construct and replay joint feasibility certificates independently of MPPI.

One robot moves at a time while all other disks remain at their actual poses.
Temporary holding moves resolve occupied goals (including symmetric swaps).
Routes come from a deterministic configuration-space triangulation, not either
evaluated anchor planner. The certificate is evidence, not an MPPI fallback.
"""

from collections import deque
from concurrent.futures import ProcessPoolExecutor, as_completed
import hashlib
import json
from pathlib import Path
import time

import numpy as np
import shapely

from rrt_connect import _sampling_mesh
from .geometry import WorldGeometry, diff_drive_witness, fingerprint, validate_witness
from .generation import save_json


def mesh_path(free, start, goal):
    # The mesh must include all stationary disk holes, not just its rectangle.
    blocked = shapely.box(*free.bounds).difference(free)
    triangles, tree, adjacency = _sampling_mesh(blocked.wkb, tuple(free.bounds))
    roots = tree.query(shapely.Point(start), predicate='intersects')
    targets = set(tree.query(shapely.Point(goal), predicate='intersects'))
    queue = deque(map(int, roots)); parent = {int(i): None for i in roots}
    found = None
    while queue:
        i = queue.popleft()
        if i in targets: found = i; break
        for j in adjacency[i]:
            if j not in parent: parent[j] = i; queue.append(j)
    if found is None: return None
    walk = []
    while found is not None: walk.append(found); found = parent[found]
    walk.reverse()
    path = [np.asarray(start)]
    for a, b in zip(walk, walk[1:]):
        shared = shapely.intersection(triangles[a], triangles[b])
        if shared.geom_type != 'LineString': raise ValueError('Invalid triangle adjacency')
        path.append(np.asarray(shared.interpolate(.5, normalized=True).coords[0]))
    path.append(np.asarray(goal))
    result = [path[0]]; i = 0
    while i < len(path)-1:
        j = len(path)-1
        while j > i+1 and not free.covers(shapely.LineString([path[i], path[j]])): j -= 1
        if np.linalg.norm(result[-1]-path[j]) > 1e-10: result.append(path[j])
        i = j
    return np.asarray(result)


def joint_certificate(folder, task_id, output, attempts=4):
    folder, output = Path(folder), Path(output)
    task = json.loads((folder/'tasks'/f'{task_id}.json').read_text())
    world = json.loads((folder/'maps.json').read_text())['worlds'][task['source_map']]
    sha = fingerprint({k:v for k,v in task.items() if k != 'generation_seconds'})
    destination = output/task_id; destination.mkdir(parents=True, exist_ok=True)
    if (destination/'validation.json').exists():
        saved = json.loads((destination/'validation.json').read_text())
        if saved['task_sha256'] == sha and saved['passed']: return saved
    tick = time.monotonic()
    robots = task['robots']; n = len(robots); dt = task['dt']
    starts = np.array([r['start'] for r in robots]); goals = np.array([r['goal']['pose'][:2] for r in robots])
    # The task's pair clearance is one safety margin between physical bodies;
    # split it between the two disks. Static clearance uses the full margin.
    radius = np.array([r['radius']+.5*r['safety_margin']+.002 for r in robots])
    # Circumscribe every circular obstacle, retaining clearance at polygon chords.
    polygon_factor = 1/np.cos(np.pi/64)
    geometry = WorldGeometry(world)
    static_radius = robots[0]['radius']+robots[0]['safety_margin']+1e-6
    bound = np.array(world['bounds'])+np.array([1, 1, -1, -1])*static_radius
    base = shapely.box(*bound).difference(shapely.union_all(
        [shapely.Polygon(p) for p in world['obstacles']]).buffer(static_radius/np.cos(np.pi/128), quad_segs=32))
    best = 0; attempt_notes = []
    for attempt in range(attempts):
        rng = np.random.default_rng([1731, attempt])
        poses = starts.copy(); done = np.zeros(n, dtype=bool); staged = np.zeros(n, dtype=int)
        moves = []; arrays = {}; minimum = float('inf'); elapsed = 0
        for number in range(4*n):
            if done.all(): break
            selected = None
            order = rng.permutation(np.flatnonzero(~done))
            for temporary in (False, True):
                for i in order:
                    if temporary and staged[i] >= 2: continue
                    others = np.arange(n) != i
                    clearance = radius[i]+radius[others]
                    if not temporary and np.any(np.linalg.norm(poses[others,:2]-goals[i],axis=1) <= clearance): continue
                    blocked = shapely.union_all(shapely.buffer(shapely.points(poses[others,:2]),
                        clearance*polygon_factor, quad_segs=16))
                    # A portal midpoint lies on a mesh edge. Replay roundoff
                    # can put it a few ulps outside that edge on the next move.
                    # This tolerance is far below the explicit 2 mm guard.
                    free = base.difference(blocked)
                    if not free.covers(shapely.Point(poses[i,:2])) and free.distance(shapely.Point(poses[i,:2])) < 1e-8:
                        free = free.buffer(1e-8)
                    component = next((p for p in shapely.get_parts(free)
                        if p.geom_type == 'Polygon' and p.covers(shapely.Point(poses[i,:2]))), None)
                    if component is None: raise ValueError(f'Current certificate pose outside free space: {task_id} robot {i}, move {number}')
                    if temporary:
                        # Hold away from every goal and existing robot. Prefer
                        # nearby open space, so a buffer does not seal an aisle.
                        triangles = shapely.get_parts(shapely.constrained_delaunay_triangles(component))
                        candidates = shapely.get_coordinates(shapely.centroid(triangles))
                        gaps = np.linalg.norm(candidates[:,None]-goals[None],axis=-1)
                        mask = (gaps > 2*radius[i]+.3).all(axis=1) & (np.linalg.norm(candidates-poses[i,:2],axis=1)>2*radius[i])
                        candidates = candidates[mask]
                        if not len(candidates): continue
                        points = shapely.points(candidates)
                        room = shapely.distance(points, base.boundary)
                        # A holding disk must leave room to pass. Near-wall
                        # buffers can create a new one-lane blockage and make
                        # every greedy attempt fail in the same way.
                        spacious = room > 2*radius[i]+.15
                        holding = (~done) & (staged>0)
                        holding[i] = False
                        if holding.any():
                            spacious &= (np.linalg.norm(candidates[:,None]-poses[holding,:2][None],axis=-1)>4*radius[i]+.1).all(axis=1)
                        if spacious.any():
                            candidates=candidates[spacious];room=room[spacious]
                        score = (.2*np.linalg.norm(candidates-poses[i,:2],axis=1)+
                                 np.linalg.norm(candidates-goals[i],axis=1)+5/np.maximum(room,.05))
                        target = candidates[np.argmin(score)]
                    else:
                        target = goals[i]
                        if not component.covers(shapely.Point(target)): continue
                        # Do not permanently park across the only escape of
                        # an unfinished robot. Otherwise a greedy certificate
                        # can strand its last agent despite a feasible task.
                        parked = done.copy(); parked[i] = True
                        parking = poses[parked,:2].copy()
                        parking[np.flatnonzero(np.flatnonzero(parked)==i)[0]] = target
                        remaining = np.flatnonzero(~parked)
                        locked = base.difference(shapely.union_all(shapely.buffer(shapely.points(parking),
                            2*radius[i]*polygon_factor,quad_segs=16)))
                        pieces = [p for p in shapely.get_parts(locked) if p.geom_type=='Polygon']
                        if any(not any(p.covers(shapely.Point(poses[j,:2])) and
                            p.covers(shapely.Point(goals[j])) for p in pieces) for j in remaining): continue
                        # Connectivity alone is insufficient: parking across
                        # a doorway can trap several robots in a single-file
                        # pocket where their order cannot be changed.
                        if any(sum(p.covers(shapely.Point(poses[j,:2])) for j in remaining)>1 and
                               p.buffer(-(2*radius[i]+.1)).is_empty for p in pieces): continue
                    path = mesh_path(component, poses[i,:2], target)
                    if path is None: continue
                    selected = (int(i), path, temporary); break
                if selected is not None: break
            if selected is None: break
            i, path, temporary = selected
            states, controls = diff_drive_witness(path, robots[i], dt, initial_heading=poses[i,2])
            validate_witness(states, controls, robots[i], geometry, dt)
            np.testing.assert_allclose(states[0,:3], poses[i], atol=1e-8)
            segments = shapely.linestrings(np.stack([states[:-1,:2], states[1:,:2]],axis=1))
            stationary = shapely.points(poses[np.arange(n)!=i,:2])
            gaps = shapely.distance(segments[:,None], stationary[None])-robots[i]['radius']-np.array(
                [r['radius'] for j,r in enumerate(robots) if j!=i])[None]
            if np.min(gaps) < robots[i]['safety_margin']-1e-8: raise ValueError('Joint witness contact')
            minimum = min(minimum, float(gaps.min()))
            stem = f'move_{number:03d}'
            arrays[stem+'_states'] = states; arrays[stem+'_controls'] = controls
            moves.append(dict(robot_id=robots[i]['id'], robot_index=i, arrays=stem,
                start_step=elapsed, steps=len(controls), temporary=bool(temporary)))
            elapsed += len(controls); poses[i] = states[-1,:3]
            if temporary: staged[i] += 1
            done[i] = np.linalg.norm(poses[i,:2]-goals[i]) < 1e-8
            if number%10==0:
                save_json(destination/'progress.json',dict(attempt=attempt,moves=len(moves),
                    finished=int(done.sum()),seconds=time.monotonic()-tick))
        best = max(best, int(done.sum()))
        attempt_notes.append(dict(attempt=attempt, finished=int(done.sum()), moves=len(moves)))
        if not done.all():
            save_json(destination/f'attempt_{attempt:02d}_stalled.json',dict(
                poses=poses.tolist(),done=done.tolist(),staged=staged.tolist(),
                moves=[dict(m,terminal_pose=arrays[m['arrays']+'_states'][-1,:3].tolist()) for m in moves]))
        if done.all():
            witness = destination/'joint_witness.npz'; np.savez_compressed(witness, **arrays)
            result = dict(passed=True, task_id=task_id, task_sha256=sha,
                method='sequential swept-disk mesh routes with temporary holding, bounded unicycle controls',
                joint_witness_sha256=hashlib.sha256(witness.read_bytes()).hexdigest(),
                checked_robots=n, move_sequence=moves, joint_makespan_seconds=elapsed*dt,
                minimum_inter_robot_clearance_meters=minimum, attempts=attempt_notes,
                validation_seconds=time.monotonic()-tick)
            save_json(destination/'validation.json', result); return result
    result = dict(passed=False, task_id=task_id, task_sha256=sha,
        reason='No joint certificate constructed within the documented attempt budget; infeasibility is not proved',
        best_finished=best, attempts=attempt_notes, validation_seconds=time.monotonic()-tick)
    save_json(destination/'validation.json', result); return result


def certify_suite(folder, output, workers=12, task_ids=None):
    folder, output = Path(folder), Path(output); output.mkdir(parents=True, exist_ok=True)
    manifest = json.loads((folder/'manifest.json').read_text())
    ids = [x['id'] for x in manifest['scenarios'] if task_ids is None or x['id'] in task_ids]
    results = []
    with ProcessPoolExecutor(max_workers=workers) as pool:
        futures = [pool.submit(joint_certificate, folder, i, output) for i in ids]
        for f in as_completed(futures):
            r = f.result(); results.append(r)
            print(f"[{len(results)}/{len(ids)}] {r['task_id']}: joint certificate {r['passed']} ({r['validation_seconds']:.1f}s)",flush=True)
    report = dict(expected_tasks=len(ids), certified_tasks=sum(r['passed'] for r in results),
        passed=all(r['passed'] for r in results), tasks=sorted(results,key=lambda r:r['task_id']))
    save_json(output/'summary.json', report)
    return report


def replay_joint_certificate(folder, task_id, output):
    """Independently replay the saved certificate, including every parked disk."""
    folder, output=Path(folder),Path(output)
    task=json.loads((folder/'tasks'/f'{task_id}.json').read_text())
    world=json.loads((folder/'maps.json').read_text())['worlds'][task['source_map']]
    report=json.loads((output/task_id/'validation.json').read_text())
    if not report['passed']:raise ValueError('No successful joint certificate')
    if report['task_sha256']!=fingerprint({k:v for k,v in task.items() if k!='generation_seconds'}):
        raise ValueError('Joint certificate belongs to another task')
    witness=output/task_id/'joint_witness.npz'
    if report['joint_witness_sha256']!=hashlib.sha256(witness.read_bytes()).hexdigest():
        raise ValueError('Joint certificate arrays have changed')
    robots=task['robots'];poses=np.array([r['start'] for r in robots]);geometry=WorldGeometry(world)
    radii=np.array([r['radius'] for r in robots]);elapsed=0;minimum=float('inf')
    with np.load(witness,allow_pickle=False) as arrays:
        for move in report['move_sequence']:
            i=move['robot_index'];stem=move['arrays'];states=arrays[stem+'_states'];controls=arrays[stem+'_controls']
            if move['start_step']!=elapsed or move['steps']!=len(controls):raise ValueError('Nonsequential certificate schedule')
            np.testing.assert_allclose(states[0,:3],poses[i],atol=1e-8)
            validate_witness(states,controls,robots[i],geometry,task['dt'])
            if np.max(abs(controls[-1]))>1e-8:raise ValueError('Witness switch before rest')
            lines=shapely.linestrings(np.stack([states[:-1,:2],states[1:,:2]],axis=1))
            other=np.arange(len(robots))!=i
            gap=shapely.distance(lines[:,None],shapely.points(poses[other,:2])[None])-radii[i]-radii[other]
            if gap.min()<robots[i]['safety_margin']-1e-8:raise ValueError('Joint witness collision or clearance violation')
            minimum=min(minimum,float(gap.min()));poses[i]=states[-1,:3];elapsed+=len(controls)
    goals=np.array([r['goal']['pose'][:2] for r in robots])
    if np.any(np.linalg.norm(poses[:,:2]-goals,axis=1)>1e-8):raise ValueError('Joint witness did not finish all agents')
    return dict(passed=True,task_id=task_id,checked_moves=len(report['move_sequence']),
        checked_robots=len(robots),joint_makespan_seconds=elapsed*task['dt'],
        minimum_inter_robot_clearance_meters=minimum)
