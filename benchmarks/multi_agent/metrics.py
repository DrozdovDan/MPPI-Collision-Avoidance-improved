"""Censoring-aware metrics and evidence-based operational deadlock detection."""

from dataclasses import asdict, dataclass
import math

import numpy as np
import shapely
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import connected_components
from shapely.geometry import LineString, Point

from .geometry import WorldGeometry, footprints, swept_footprints


@dataclass(frozen=True)
class DeadlockConfig:
    stationary_speed: float = .03
    slow_speed: float = .15
    progress_distance: float = .10
    congestion_seconds: float = 2.
    no_progress_seconds: float = 4.
    deadlock_seconds: float = 8.
    proximity_margin: float = .35
    persistent_fraction: float = .85
    forward_cone_cosine: float = .25
    minimum_group: int = 2

    def __post_init__(self):
        values = asdict(self)
        if any(not np.isfinite(v) or v <= 0 for v in values.values()):
            raise ValueError("Detection thresholds must be finite and positive")
        if self.persistent_fraction > 1 or self.forward_cone_cosine >= 1 or self.minimum_group < 2:
            raise ValueError("Invalid persistence, cone, or group threshold")
        if not self.congestion_seconds <= self.no_progress_seconds <= self.deadlock_seconds:
            raise ValueError("Detection windows must be ordered")


def intervals(mask, dt, minimum_seconds=0.):
    mask = np.asarray(mask, dtype=bool)
    edges = np.diff(np.r_[False, mask, False].astype(int))
    return [dict(start_seconds=float(a*dt), end_seconds=float(b*dt), duration_seconds=float((b-a)*dt))
            for a, b in zip(np.flatnonzero(edges == 1), np.flatnonzero(edges == -1))
            if (b-a)*dt + 1e-9 >= minimum_seconds]


def completion_data(task, states, controls):
    states = np.asarray(states)
    goals = np.array([r["goal"]["pose"][:2] for r in task["robots"]])
    errors = np.linalg.norm(states[:, :, :2] - goals, axis=2)
    radii = np.array([r["goal"]["radius"] for r in task["robots"]])
    if states.shape[2] == 6:
        rest = (abs(states[:, :, 3]) < 1e-6) & (abs(states[:, :, 5]) < 1e-6)
    else:
        rest = np.linalg.norm(states[:, :, 3:5], axis=2) < 1e-6
        if len(controls):
            rest[1:] &= abs(np.asarray(controls)[:, :, 1]) < 1e-6
    at_goal = (errors <= radii + 1e-8) & rest
    # Completion is a settled terminal visit, rather than an early goal touch
    # followed by departure. This matches the simulator's physical parking.
    stable = np.flip(np.logical_and.accumulate(np.flip(at_goal, axis=0), axis=0), axis=0)
    arrival = [float(np.flatnonzero(stable[:, i])[0] * task["dt"]) if stable[-1, i] else None
               for i in range(task["robot_count"])]
    return errors, stable, arrival


def detect_events(task, states, stable, paths, config=None):
    cfg = config or DeadlockConfig()
    dt, n = task["dt"], task["robot_count"]
    xy = np.asarray(states)[:, :, :2]
    speed = np.r_[np.zeros((1, n)), np.linalg.norm(np.diff(xy, axis=0), axis=2) / dt]
    goals = np.array([r["goal"]["pose"][:2] for r in task["robots"]])
    goal_distance = np.linalg.norm(xy-goals, axis=2)
    progress = np.zeros((len(xy), n))
    for i, path in enumerate(paths):
        progress[:, i] = np.maximum.accumulate(shapely.line_locate_point(LineString(path), shapely.points(xy[:, i])))
    radii = np.array([r["radius"] for r in task["robots"]])
    gaps = np.linalg.norm(xy[:, :, None] - xy[:, None, :], axis=-1) - radii[:, None] - radii[None, :]
    near = gaps < cfg.proximity_margin
    for i in range(n): near[:, i, i] = False
    unfinished = ~stable
    slow = (speed < cfg.slow_speed) & unfinished
    no_progress = np.zeros_like(slow)
    congestion = np.zeros_like(slow)
    group_evidence = {}
    for window_seconds, target in ((cfg.congestion_seconds, congestion), (cfg.no_progress_seconds, no_progress)):
        window = max(1, int(math.ceil(window_seconds / dt)))
        for t in range(window, len(xy)):
            stagnant = ((progress[t]-progress[t-window] <= cfg.progress_distance) &
                        (goal_distance[t-window]-goal_distance[t] <= cfg.progress_distance))
            if target is congestion:
                target[t] = stagnant & slow[t] & near[t].any(axis=1)
            else:
                target[t] = stagnant & unfinished[t]
    window = max(1, int(math.ceil(cfg.deadlock_seconds / dt)))
    for t in range(window, len(xy)):
        stationary = (speed[t-window+1:t+1].mean(axis=0) <= cfg.stationary_speed)
        stagnant = ((progress[t]-progress[t-window] <= cfg.progress_distance) &
                    (goal_distance[t-window]-goal_distance[t] <= cfg.progress_distance))
        eligible = stationary & stagnant & unfinished[t]
        desired = goals - xy[t]
        norm = np.linalg.norm(desired, axis=1)
        relative = xy[t, None, :, :] - xy[t, :, None, :]
        cosine = np.sum(desired[:, None, :] * relative, axis=2) / np.maximum(1e-12, norm[:, None] * np.linalg.norm(relative, axis=2))
        persistent = near[t-window+1:t+1].mean(axis=0) >= cfg.persistent_fraction
        obstruction = persistent & (cosine > cfg.forward_cone_cosine) & eligible[:, None] & eligible[None, :]
        _, labels = connected_components(csr_matrix(obstruction), directed=True, connection="strong")
        for label in set(labels):
            group = tuple(np.flatnonzero(labels == label).tolist())
            if len(group) >= cfg.minimum_group:
                group_evidence.setdefault(group, np.zeros(len(xy), dtype=bool))[t] = True
    deadlocks = []
    for group, evidence in sorted(group_evidence.items()):
        for event in intervals(evidence, dt):
            deadlocks.append(dict(event, robots=[task["robots"][i]["id"] for i in group],
                                  detected_seconds=event["start_seconds"],
                                  evidence_window_start_seconds=max(0., event["start_seconds"]-cfg.deadlock_seconds),
                                  definition="operational: stationary, no progress, persistent directed mutual-obstruction cycle"))
    per_robot = []
    for i, r in enumerate(task["robots"]):
        per_robot.append(dict(id=r["id"],
            temporary_slowing=[e for e in intervals(slow[:, i], dt) if e["duration_seconds"] < cfg.congestion_seconds],
            congestion=intervals(congestion[:, i], dt), no_progress=intervals(no_progress[:, i], dt),
            total_stopped_seconds=float(np.sum((speed[1:, i] < cfg.stationary_speed) & unfinished[1:, i]) * dt)))
    return dict(config=asdict(cfg), per_robot=per_robot, deadlocks=deadlocks,
                deadlock_count=len(deadlocks), no_progress_event_count=sum(len(r["no_progress"]) for r in per_robot))


def collision_metrics(task, world, states):
    """Actual footprints plus continuous disk/envelope lower bounds.

    Rectangle swept hulls are broad-phase only. Potential contacts are refined
    at synchronized intermediate poses; unresolved broad-phase contacts are
    explicitly reported, never counted as demonstrated collisions.
    """
    robots = task["robots"]; xy = states[:, :, :2]; dt = task["dt"]
    geometry = WorldGeometry(world)
    bodies = [footprints(states[:, i], r) for i, r in enumerate(robots)]
    collision_records, unresolved = [], []
    minimum_pair, minimum_envelope = float("inf"), float("inf")
    for i, r in enumerate(robots):
        swept = swept_footprints(states[:, i], r)
        suspect = (shapely.intersects(swept, geometry.obstacles) | ~shapely.covers(geometry.workspace, swept))
        for k in np.flatnonzero(suspect):
            interpolation = np.linspace(states[k, i], states[k+1, i], 33)
            poses = footprints(interpolation, r)
            hit = bool(np.any(shapely.intersects(poses, geometry.obstacles) | ~shapely.covers(geometry.workspace, poses)))
            (collision_records if hit else unresolved).append(dict(step=int(k), kind="static", robots=[r["id"]]))
        for j in range(i+1, len(robots)):
            sampled = shapely.distance(bodies[i], bodies[j])
            minimum_pair = min(minimum_pair, float(np.min(sampled)))
            relative = xy[:, i] - xy[:, j]; change = np.diff(relative, axis=0)
            squared = np.sum(change**2, axis=1)
            fraction = np.divide(-np.sum(relative[:-1] * change, axis=1), squared,
                                 out=np.zeros_like(squared), where=squared > 0)
            closest = np.linalg.norm(relative[:-1] + np.clip(fraction, 0, 1)[:, None] * change, axis=1)
            envelope = closest - r["radius"] - robots[j]["radius"]
            minimum_envelope = min(minimum_envelope, float(np.min(envelope)))
            if r["footprint"] == robots[j]["footprint"] == "disk":
                minimum_pair = min(minimum_pair, float(np.min(envelope)))
            for k in np.flatnonzero(envelope < -1e-9):
                a = np.linspace(states[k, i], states[k+1, i], 33)
                b = np.linspace(states[k, j], states[k+1, j], 33)
                if r["footprint"] == robots[j]["footprint"] == "disk":
                    hit = True
                else:
                    hit = bool(np.any(shapely.intersects(footprints(a, r), footprints(b, robots[j]))))
                (collision_records if hit else unresolved).append(dict(step=int(k), kind="robot_robot", robots=[r["id"], robots[j]["id"]]))
    # Count contiguous contact episodes rather than counting each timestep anew.
    grouped = {}
    for record in collision_records:
        grouped.setdefault((record["kind"], tuple(record["robots"])), np.zeros(len(states)-1, dtype=bool))[record["step"]] = True
    events = [dict(event, kind=kind, robots=list(ids)) for (kind, ids), mask in grouped.items() for event in intervals(mask, dt)]
    return dict(collisions=len(events), collision_events=events,
                minimum_inter_robot_clearance=minimum_pair if np.isfinite(minimum_pair) else None,
                minimum_swept_envelope_clearance=minimum_envelope if np.isfinite(minimum_envelope) else None,
                unresolved_swept_contacts=unresolved,
                collision_definition="oriented rectangles for cars, disks for differential-drive; synchronized swept-contact refinement")


def validate_execution(task, states, controls):
    """Check measured motion and bounded commands, independently of the planner."""
    dt = task["dt"]
    violations = []
    for i, robot in enumerate(task["robots"]):
        s, u, lim = states[:, i], controls[:, i], robot["motion_limits"]
        if robot["model"] == "car_like":
            speed = s[1:, 3]
            np.testing.assert_allclose(np.diff(s[:, 3]), s[:-1, 5]*dt, atol=1e-8)
            np.testing.assert_allclose(np.diff(s[:, 5]), u[:, 0]*dt, atol=1e-8)
            np.testing.assert_allclose(np.diff(s[:, 4]), u[:, 1]*dt, atol=1e-8)
            np.testing.assert_allclose(np.diff(s[:, 2]), speed*np.tan(s[1:,4])/robot["wheelbase"]*dt, atol=1e-8)
            if (s[:,3].min() < lim["v_min"]-1e-8 or s[:,3].max() > lim["v_max"]+1e-8 or
                s[:,5].min() < lim["a_min"]-1e-8 or s[:,5].max() > lim["a_max"]+1e-8 or
                s[:,4].min() < lim["w_min"]-1e-8 or s[:,4].max() > lim["w_max"]+1e-8 or
                u[:,0].min() < lim["j_min"]-1e-8 or u[:,0].max() > lim["j_max"]+1e-8 or
                abs(u[:,1]).max() > lim["steering_rate_max"]+1e-8):
                violations.append(dict(robot=robot["id"], kind="car_state_or_control_limit"))
        else:
            speed = u[:,0]
            np.testing.assert_allclose(np.diff(s[:,2]),u[:,1]*dt,atol=1e-8)
            delta = np.diff(np.vstack([np.zeros(2),u]),axis=0)/dt
            if (u[:,0].min() < lim["v_min"]-1e-8 or u[:,0].max() > lim["v_max"]+1e-8 or
                abs(u[:,1]).max() > lim["w_max"]+1e-8 or
                delta[:,0].max() > lim["v_accel_max"]+1e-8 or -delta[:,0].min() > lim["v_brake_max"]+1e-8):
                violations.append(dict(robot=robot["id"], kind="diff_drive_speed_or_linear_acceleration"))
            exceeded = abs(delta[:,1]) > lim["w_accel_max"]+1e-8
            if exceeded.any():
                violations.append(dict(robot=robot["id"], kind="angular_acceleration", bound=lim["w_accel_max"],
                                       peak=float(abs(delta[:,1]).max()), steps=np.flatnonzero(exceeded).tolist(),
                                       cause="existing controller fallback may reset yaw rate immediately"))
        expected = speed[:,None]*np.c_[np.cos(s[:-1,2]),np.sin(s[:-1,2])]*dt
        np.testing.assert_allclose(np.diff(s[:,:2],axis=0),expected,atol=1e-8)
    return dict(passed=not violations, model_equations_valid=True, violations=violations,
                checked_steps=len(controls), checked_robots=task["robot_count"],
                state_resets_or_trace_filters=False)


def evaluate_run(task, world, states, controls, paths, *, timeout, deadlock_config=None):
    states, controls = np.asarray(states), np.asarray(controls)
    dt = task["dt"]
    physical_validation = validate_execution(task, states, controls)
    errors, stable, arrivals = completion_data(task, states, controls)
    complete = [a is not None for a in arrivals]
    all_finished = all(complete)
    path_lengths = np.linalg.norm(np.diff(states[:, :, :2], axis=0), axis=2).sum(axis=0)
    events = detect_events(task, states, stable, paths, deadlock_config)
    collisions = collision_metrics(task, world, states)
    occupancy = {}
    for resource in task["resources"]:
        center, radius = np.asarray(resource["center"]), resource["radius"]
        occupied = np.linalg.norm(states[:, :, :2]-center, axis=2) <= radius + np.array([r["radius"] for r in task["robots"]])
        occupancy[str(resource["id"])] = dict(counts=occupied.sum(axis=1).tolist(),
            per_robot_seconds=(occupied[1:].sum(axis=0)*dt).tolist(), region=dict(center=center.tolist(), radius=radius))
    elapsed = (len(states)-1) * dt
    valid_finish = all_finished and physical_validation["passed"]
    return dict(completion_fraction=float(np.mean(complete)), all_robots_success=valid_finish and collisions["collisions"] == 0,
                makespan_seconds=max(arrivals) if valid_finish else None,
                individual_travel_times=arrivals, sum_travel_times=sum(arrivals) if valid_finish else None,
                completed_travel_time_sum=sum(a for a in arrivals if a is not None),
                restricted_total_time=sum(a if a is not None else elapsed for a in arrivals),
                censoring="Unfinished arrival times and failed makespans are null; restricted time charges unfinished robots the observation horizon.",
                per_robot_path_lengths=path_lengths.tolist(), final_goal_errors=errors[-1].tolist(),
                total_stopped_seconds=sum(r["total_stopped_seconds"] for r in events["per_robot"]),
                no_progress_event_count=events["no_progress_event_count"], deadlock_events=events["deadlocks"],
                deadlock_count=events["deadlock_count"], event_diagnostics=events,
                timeout=bool(timeout and not all_finished), bottleneck_occupancy=occupancy,
                simulated_seconds=elapsed, control_limits_valid=physical_validation["passed"],
                physical_validation=physical_validation, **collisions)
