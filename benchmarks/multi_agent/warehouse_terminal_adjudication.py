"""Certify missed traffic deadlocks from immutable, stationary saved motion.

The frozen online monitor uses a geometric nearest anchor projection, which can
lie behind shelving. This postprocessing never changes a raw outcome or motion.
It can corroborate a stalled fleet only when every unfinished robot belongs to
or feeds a persistent blocking cycle or a parked-goal blocker.
"""
import hashlib
from pathlib import Path

import numpy as np
import shapely
from shapely.ops import substring
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import connected_components

from .warehouse_evaluation_report import audit_deadlock_geometry


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def certify_visible_join_deadlock(task, world, paths, states, controls, evidence, cfg):
    """Require physical visible joins, persistent blocking and complete coverage."""
    window = round(cfg.deadlock_window_seconds / task['dt'])
    if len(controls) < window:
        raise ValueError('Insufficient stationary history for deadlock adjudication')
    xy = states[-1, :, :2]
    goals = np.array([r['goal']['pose'][:2] for r in task['robots']])
    goal_radii = np.array([r['goal']['radius'] for r in task['robots']])
    radii = np.array([r['radius'] for r in task['robots']])
    finished = ((np.linalg.norm(xy-goals, axis=1) <= goal_radii+1e-8) &
                (np.max(abs(controls[-1]), axis=1) <= 1e-8))
    unfinished = set(map(int, np.flatnonzero(~finished)))
    if unfinished != set(evidence['unfinished_robot_ids']):
        raise ValueError('Stall evidence does not cover the actual unfinished fleet')
    recent = states[-window:]
    stationary = np.mean(np.linalg.norm(recent[:, :, 3:5], axis=2), axis=0) <= cfg.stationary_speed_mps
    excursion = np.linalg.norm(np.ptp(recent[:, :, :2], axis=0), axis=1)
    if not np.all((stationary & (excursion <= cfg.progress_threshold_meters)) | finished):
        raise ValueError('The uncompleted fleet is not persistently stationary')
    # Preserve and independently corroborate the original detector's edges.
    original = audit_deadlock_geometry(task, world, paths, states, evidence, cfg)
    obstruction = np.zeros((len(xy), len(xy)), dtype=bool)
    for source, target in evidence['obstruction_edges']:
        obstruction[source, target] = True
    obstacles = shapely.union_all([shapely.Polygon(p) for p in world['obstacles']])
    bounds = np.array(world['bounds'])
    additions = []
    missing = sorted(unfinished-set(evidence['blocked_robot_ids']))
    if not missing:
        raise ValueError('No missing blocking relationship to adjudicate')
    lookahead = cfg.speed_mps*(cfg.horizon_states-1)*task['dt']
    for i in missing:
        path = np.asarray(paths[i]); line = shapely.LineString(path)
        nearest = np.asarray(line.interpolate(line.project(shapely.Point(xy[i]))).coords)[0]
        nearest_clearance = shapely.LineString([xy[i], nearest]).distance(obstacles)-radii[i]
        margin = task['robots'][i]['safety_margin']
        if nearest_clearance >= margin-1e-9:
            raise ValueError('Unclassified stall is not a hidden-projection deadlock')
        delta = np.diff(path, axis=0); squared = np.sum(delta*delta, axis=1)
        fractions = np.clip(np.divide(np.sum((xy[i]-path[:-1])*delta, axis=1), squared,
            out=np.zeros(len(delta)), where=squared>0), 0., 1.)
        points = path[:-1]+fractions[:, None]*delta
        joins = shapely.linestrings(np.stack([np.tile(xy[i], (len(points), 1)), points], axis=1))
        clearances = shapely.distance(joins, obstacles)-radii[i]
        in_bounds = ((points[:, 0] >= bounds[0]+radii[i]+margin-1e-9) &
                     (points[:, 0] <= bounds[2]-radii[i]-margin+1e-9) &
                     (points[:, 1] >= bounds[1]+radii[i]+margin-1e-9) &
                     (points[:, 1] <= bounds[3]-radii[i]-margin+1e-9))
        visible = np.flatnonzero((squared>1e-20) & (clearances>=margin-1e-9) & in_bounds)
        if not len(visible):
            raise ValueError('No footprint-clear visible join for stalled robot')
        lengths = np.sqrt(squared)
        stations = np.r_[0., np.cumsum(lengths)][:-1]+fractions*lengths
        distances = np.sum((points-xy[i])**2, axis=1)
        j = visible[np.lexsort((stations[visible], distances[visible]))[0]]
        tail = substring(line, float(stations[j]), float(stations[j])+lookahead)
        coordinates = [xy[i].tolist()]+list(tail.coords)
        # Clip the join AND subsequent anchor to the actual 3 s travel horizon.
        section = substring(shapely.LineString(coordinates), 0., lookahead)
        distances = shapely.distance(section, shapely.points(recent[:, :, :2]))
        ratios = np.mean(distances < radii[i]+radii[None, :]+cfg.obstruction_clearance_meters, axis=0)
        blockers = np.flatnonzero((ratios>=cfg.persistence_fraction) & (stationary | finished))
        blockers = [int(j) for j in blockers if j != i]
        if not blockers:
            raise ValueError('Visible rejoin has no persistent physical blocker')
        obstruction[i, blockers] = True
        additions.append(dict(robot_index=i, hidden_nearest_projection=nearest.tolist(),
            hidden_join_clearance_meters=float(nearest_clearance), visible_join=points[j].tolist(),
            visible_join_clearance_meters=float(clearances[j]), visible_anchor_station_meters=float(stations[j]),
            checked_forward_section=list(section.coords), persistent_blocker_ratios={str(j):float(ratios[j]) for j in blockers}))
    np.fill_diagonal(obstruction, False)
    _, labels = connected_components(csr_matrix(obstruction), directed=True, connection='strong')
    groups = [np.flatnonzero(labels==label).tolist() for label in set(labels)
              if np.count_nonzero(labels==label)>=2]
    blocked = {i for group in groups for i in group}
    blocked |= {i for i in unfinished if np.any(obstruction[i] & finished)}
    while True:
        extended = blocked | {i for i in unfinished if any(obstruction[i, j] for j in blocked)}
        if extended == blocked:
            break
        blocked = extended
    if not unfinished.issubset(blocked):
        raise ValueError('Visible-join evidence does not establish fleet-wide deadlock')
    return dict(passed=True, raw_status='stalled', verified_terminal_outcome='deadlock',
        evidence_window_seconds=cfg.deadlock_window_seconds, observed_seconds=len(controls)*task['dt'],
        unfinished_robot_ids=sorted(unfinished), blocked_robot_ids=sorted(unfinished & blocked),
        mutual_obstruction_groups=groups, obstruction_edges=np.argwhere(obstruction).tolist(),
        original_edge_corroboration=original, added_visible_join_evidence=additions,
        mean_state_speed_and_yaw_rate= np.mean(np.linalg.norm(recent[:, :, 3:5], axis=2), axis=0).tolist(),
        position_excursion_meters=excursion.tolist(), raw_motion_unchanged=True,
        adjudicator_sha256=sha(__file__))
