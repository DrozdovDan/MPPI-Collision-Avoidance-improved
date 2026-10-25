"""Geometry, existing-model feasibility witnesses, and independent validation.

Distances are native Bench-MR map units. A rectangle is centered at the same
reference point used by the existing notebook drawings. Planning/control use
its circumscribed circle; evaluation also checks the actual rectangle sweep.
"""

from collections import OrderedDict
import hashlib
import json
import math
from pathlib import Path
import sys

import numpy as np
import shapely
from shapely.geometry import LineString, Point, Polygon, box
from shapely.ops import unary_union

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "example"))
from dyn_models import CarLikeDyn
from corridor_graph.geometry import FreeSpace, sample_polyline
from rrt_connect import RRTConnectPlanner


def fingerprint(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"),
                                     allow_nan=False).encode()).hexdigest()


def load_suite(folder):
    folder = Path(folder)
    manifest = json.loads((folder / "manifest.json").read_text())
    if manifest["schema_version"] != 1:
        raise ValueError("Unsupported multi-agent scenario schema")
    worlds = json.loads((folder / "maps.json").read_text())["worlds"]
    tasks = [json.loads((folder / item["file"]).read_text()) for item in manifest["scenarios"]]
    return manifest, worlds, tasks


def profiles_for(world_name, traffic):
    """Choose dimensions from traversable interior widths, excluding wall tips."""
    widths = [e["min_width"] for e in traffic["routing_geometry"]["edges"]
              if .45 < e["min_width"] < 2.0]
    if not widths:
        raise ValueError("No usable interior width scale")
    if world_name.startswith("parking"):
        reference = float(np.quantile(widths, .25))
        # The largest car's *width*, not just its planning circle, exceeds
        # the reference neck. Wide driving aisles still admit its envelope.
        radii = [reference * x for x in (.40, .55, 1.30)]
    else:
        reference = float(np.median(widths))
        radii = [reference * x for x in (.18, .32, .55)]
    profiles = []
    for label, radius in zip(("small", "medium", "large"), radii):
        car = world_name.startswith("parking")
        # A length/width ratio of 2.2; dimensions are map-scale, not meters.
        length = 2 * radius / math.sqrt(1 + (1 / 2.2)**2)
        width = length / 2.2
        limits = dict(v_min=0., v_max=.65 if car else 1.2,
                      w_min=-.6 if car else -1.8, w_max=.6 if car else 1.8,
                      v_accel_max=.6 if car else .8, v_brake_max=1.2,
                      w_accel_max=2., a_min=-1.2, a_max=.6,
                      j_min=-1., j_max=1., steering_rate_max=.6)
        profiles.append(dict(size_class=label, model="car_like" if car else "diff_drive",
                             radius=radius, length=length if car else 2 * radius,
                             width=width if car else 2 * radius,
                             wheelbase=.65 * length if car else None,
                             footprint="rectangle" if car else "disk",
                             motion_limits=limits, safety_margin=.02 if car else .04,
                             size_reference_width=reference))
    return profiles


def agent_parameters(robot):
    return dict(robot["motion_limits"], size=robot["radius"], r_vis=30.)


def rectangles(poses, robot):
    poses = np.asarray(poses)
    local = np.array([[-1, -1], [1, -1], [1, 1], [-1, 1]]) * [robot["length"] / 2, robot["width"] / 2]
    c, s = np.cos(poses[:, 2]), np.sin(poses[:, 2])
    x = poses[:, 0, None] + c[:, None] * local[:, 0] - s[:, None] * local[:, 1]
    y = poses[:, 1, None] + s[:, None] * local[:, 0] + c[:, None] * local[:, 1]
    return np.stack([x, y], axis=-1)


def footprints(poses, robot):
    poses = np.atleast_2d(poses)
    if robot["footprint"] == "rectangle":
        return shapely.polygons(rectangles(poses, robot))
    # Circumscribed polygons avoid understating disk size.
    return shapely.buffer(shapely.points(poses[:, :2]), robot["radius"] / math.cos(math.pi / 64), quad_segs=16)


def swept_footprints(states, robot):
    """Conservative continuous sweeps for linear pose interpolation per timestep.

    The rotation sagitta bounds deviation of every rectangle vertex from its
    endpoint chord. This includes turning footprints, not just sampled poses.
    """
    states = np.asarray(states, dtype=float)
    if robot["footprint"] == "rectangle":
        corners = rectangles(states, robot)
        pairs = np.concatenate([corners[:-1], corners[1:]], axis=1)
        hulls = shapely.convex_hull(shapely.multipoints(pairs))
        angle = np.abs(np.diff(states[:, 2]))
        if np.any(angle > math.pi):
            raise ValueError("Sweep requires angular increments <= pi")
        sagitta = robot["radius"] * (1 - np.cos(angle / 2)) + 1e-10
        return shapely.buffer(hulls, sagitta)
    lines = shapely.linestrings(np.stack([states[:-1, :2], states[1:, :2]], axis=1))
    return shapely.buffer(lines, robot["radius"] / math.cos(math.pi / 64), quad_segs=16)


class WorldGeometry:
    def __init__(self, world):
        self.free = FreeSpace(world)
        self.world = world
        self.workspace = self.free.workspace
        self.obstacles = self.free.obstacles
        self.planners = {}

    def planner(self, robot):
        key = (robot["radius"], robot["safety_margin"])
        if key not in self.planners:
            self.planners[key] = RRTConnectPlanner(
                self.world["obstacles"], self.world["bounds"], key[0], key[1],
                smoothing_iterations=0, seed=0)
        return self.planners[key]

    def poses_free(self, poses, robot):
        bodies = footprints(poses, robot)
        return (np.asarray(shapely.covers(self.workspace, bodies)) &
                ~np.asarray(shapely.intersects(self.obstacles, bodies)))

    def witness_clearance(self, states, robot):
        swept = swept_footprints(states, robot)
        inside = np.asarray(shapely.covers(self.workspace, swept))
        gaps = shapely.distance(swept, self.workspace.boundary)
        if not self.obstacles.is_empty:
            gaps = np.minimum(shapely.distance(swept, self.obstacles), gaps)
        if not inside.all() or np.any(shapely.intersects(swept, self.obstacles)):
            return -1.
        return float(np.min(gaps))


def _braking_control(model, state, dt):
    next_v = state[3] + state[5] * dt
    target_a = -min(-model.a_min, model.acceleration_headroom(max(0., next_v), model.j_max, dt))
    return model.limit_control(state, [(target_a - state[5]) / dt, 0.], dt)


def car_witness(path, robot, geometry, *, dt=.1, max_steps=1000):
    """Bounded feasibility check, not a new global planner.

    Follow a geometry-analysis reference with two fixed preview settings using
    the existing CarLikeDyn. Accept only an actual collision-free control/state
    trace that stops near its target. No MPPI or K-route result selects tasks.
    """
    path = np.asarray(path, dtype=float)
    reference = LineString(path)
    target = path[-1]
    initial_heading = math.atan2(*(path[1] - path[0])[::-1])
    params = agent_parameters(robot)
    for preview_factor in (1.6, 2.5, 1.0):
        model = CarLikeDyn(robot["wheelbase"])
        model.configure_limits(params)
        state = np.r_[path[0], initial_heading, 0., 0., 0.]
        trace, controls = [state.copy()], []
        preview = max(.18, preview_factor * robot["length"])
        progress = 0.
        braking = False
        for step in range(max_steps):
            distance = np.linalg.norm(state[:2] - target)
            progress = max(progress, reference.project(Point(state[:2])))
            ahead = np.asarray(reference.interpolate(min(reference.length, progress + preview)).coords[0])
            dx, dy = ahead - state[:2]
            lateral = -dx * math.sin(state[2]) + dy * math.cos(state[2])
            longitudinal = dx * math.cos(state[2]) + dy * math.sin(state[2])
            if longitudinal < -.05 and distance > .3:
                break
            steering = math.atan(robot["wheelbase"] * 2 * lateral / max(.0025, dx * dx + dy * dy))
            speed = min(.45, robot["motion_limits"]["v_max"], .8 * distance)
            desired_a = np.clip((speed - state[3]) / .6, model.a_min, model.a_max)
            control = model.limit_control(state, [(desired_a - state[5]) / .4,
                                                   (steering - state[4]) / .3], dt)
            if distance < .18:
                braking = True
            if braking:
                control = _braking_control(model, state, dt)
            state = model.state_shift(state, dt) + model.dynamic_model(state, control, dt)
            trace.append(state.copy())
            controls.append(control)
            if step % 12 == 0 and not geometry.poses_free([state], robot)[0]:
                break
            if braking and abs(state[3]) < 1e-8 and abs(state[5]) < 1e-8:
                if np.linalg.norm(state[:2] - target) < .28:
                    states = np.asarray(trace)
                    if geometry.witness_clearance(states, robot) >= robot["safety_margin"]:
                        return states, np.asarray(controls)
                break
    return None


def diff_drive_witness(path, robot, dt=.1):
    """Existing unicycle equations: turn in place, then traverse each segment."""
    from dyn_models import DiffDriveDyn
    model = DiffDriveDyn()
    path = np.asarray(path)
    heading = math.atan2(*(path[1] - path[0])[::-1])
    state = np.r_[path[0], heading, 0., 0.]
    trace, controls = [state.copy()], []
    def bounded_pulse(distance, maximum, acceleration):
        count = max(3, math.ceil(2 * abs(distance) / (maximum * dt)))
        while True:
            pulse = np.sin(np.pi * np.arange(1, count+1) / (count+1))**2
            pulse *= distance / (pulse.sum() * dt)
            if max(abs(pulse)) <= maximum + 1e-10 and max(abs(np.diff(np.r_[0., pulse, 0.]))) <= acceleration * dt + 1e-10:
                return np.r_[pulse, 0.]
            count += 1
    # In-place rotation is supported. Pulses also honor configured speed and
    # angular acceleration bounds, including every start and stop.
    for a, b in zip(path, path[1:]):
        direction = math.atan2(*(b - a)[::-1])
        error = math.atan2(math.sin(direction-state[2]), math.cos(direction-state[2]))
        length = np.linalg.norm(b - a)
        lim = robot["motion_limits"]
        for axis, pulse in ((1, bounded_pulse(error, lim["w_max"], lim["w_accel_max"])),
                            (0, bounded_pulse(length, lim["v_max"], min(lim["v_accel_max"], lim["v_brake_max"])))):
            for value in pulse:
                control = np.zeros(2); control[axis] = value
                pose = model.state_shift(state[:3], dt) + model.dynamic_model(state[:3], control, dt)
                following = np.r_[pose, (pose[:2] - state[:2]) / dt]
                trace.append(following)
                controls.append(control.copy())
                state = following
    trace.append(state.copy()); trace[-1][3:] = 0.; controls.append(np.zeros(2))
    return np.asarray(trace, dtype=float), np.asarray(controls, dtype=float)


def validate_witness(states, controls, robot, geometry, dt):
    from dyn_models import DiffDriveDyn
    states, controls = np.asarray(states), np.asarray(controls)
    if not np.isfinite(states).all() or not np.isfinite(controls).all() or len(states) != len(controls)+1:
        raise ValueError("Invalid witness arrays")
    lim = robot["motion_limits"]
    if robot["model"] == "car_like":
        model = CarLikeDyn(robot["wheelbase"]); model.configure_limits(agent_parameters(robot))
        for index, command in enumerate(controls):
            bounded = model.limit_control(states[index], command, dt)
            np.testing.assert_allclose(bounded, command, atol=1e-8)
            following = states[index] + model.dynamic_model(states[index], command, dt)
            np.testing.assert_allclose(states[index+1], following, atol=1e-8)
        if max(abs(states[-1, 3]), abs(states[-1, 5])) > 1e-8:
            raise ValueError("Car feasibility witness must stop physically")
    else:
        if np.any(controls[:, 0] < lim["v_min"]-1e-8) or np.any(controls[:, 0] > lim["v_max"]+1e-8) or np.any(abs(controls[:, 1]) > lim["w_max"]+1e-8):
            raise ValueError("Differential-drive witness exceeds kinematic limits")
        expected = controls[:, 0, None] * np.c_[np.cos(states[:-1, 2]), np.sin(states[:-1, 2])] * dt
        np.testing.assert_allclose(np.diff(states[:, :2], axis=0), expected, atol=1e-8)
        np.testing.assert_allclose(np.diff(states[:, 2]), controls[:, 1] * dt, atol=1e-8)
        delta = np.diff(np.vstack([np.zeros(2), controls]), axis=0) / dt
        if np.max(abs(delta[:, 0])) > max(lim["v_accel_max"], lim["v_brake_max"]) + 1e-8 or np.max(abs(delta[:, 1])) > lim["w_accel_max"] + 1e-8:
            raise ValueError("Differential-drive witness exceeds acceleration bounds")
    clearance = geometry.witness_clearance(states, robot)
    if clearance < robot["safety_margin"] - 1e-8:
        raise ValueError("Actual swept footprint violates map or safety margin")
    return clearance


def validate_task(task, world, witnesses, *, replay=True):
    if task["schema_version"] != 1:
        raise ValueError("Unsupported scenario schema version")
    if fingerprint(world) != task["map_sha256"]:
        raise ValueError("Scenario/map fingerprint mismatch")
    if task["units"] != "native_map_units" or task["transformation"] != {"kind": "identity", "scale": 1., "crop": None}:
        raise ValueError("Unsupported units or map transformation")
    robots = task["robots"]
    if len(robots) != task["robot_count"] or not robots:
        raise ValueError("Invalid robot count")
    geometry = WorldGeometry(world)
    gaps = []
    for r in robots:
        if r["radius"] <= 0 or not np.isfinite(r["start"] + r["goal"]["pose"]).all():
            raise ValueError("Invalid dimensions or poses")
        if r["model"] != ("car_like" if task["source_map"].startswith("parking") else "diff_drive"):
            raise ValueError("Map/model mismatch")
        if r["footprint"] == "rectangle" and not math.isclose(math.hypot(r["length"], r["width"]) / 2, r["radius"], abs_tol=1e-8):
            raise ValueError("Planning radius does not enclose vehicle footprint consistently")
        poses = [r["start"], r["goal"]["pose"]]
        if not geometry.poses_free(poses, r).all():
            raise ValueError("Start or goal footprint collision")
        if not all(geometry.planner(r).point_is_free(p[:2]) for p in poses):
            raise ValueError("Start or goal fails conservative planning clearance")
        states, controls = witnesses[r["id"] + "_states"], witnesses[r["id"] + "_controls"]
        np.testing.assert_allclose(states[0, :3], r["start"], atol=1e-8)
        if np.linalg.norm(states[-1, :2] - r["goal"]["pose"][:2]) > r["goal"]["radius"]:
            raise ValueError("Witness does not reach goal")
        if replay:
            gaps.append(validate_witness(states, controls, r, geometry, task["dt"]))
    minimum_start_gap, minimum_goal_gap = float("inf"), float("inf")
    for i, a in enumerate(robots):
        for b in robots[i+1:]:
            radius = a["radius"] + b["radius"]
            start_gap = np.linalg.norm(np.subtract(a["start"][:2], b["start"][:2])) - radius
            goal_gap = np.linalg.norm(np.subtract(a["goal"]["pose"][:2], b["goal"]["pose"][:2])) - radius
            minimum_start_gap = min(minimum_start_gap, float(start_gap))
            minimum_goal_gap = min(minimum_goal_gap, float(goal_gap))
            if min(start_gap, goal_gap) < max(a["safety_margin"], b["safety_margin"]):
                raise ValueError("Overlapping initial or terminal planning envelopes")
    return dict(passed=True, checked_robots=len(robots), individually_feasible=True,
                car_feasibility="bounded-control bicycle replay with swept rectangles" if robots[0]["model"] == "car_like" else "unicycle controls with swept disks",
                minimum_start_envelope_gap=minimum_start_gap, minimum_goal_envelope_gap=minimum_goal_gap,
                minimum_witness_footprint_clearance=min(gaps) if gaps else None,
                jointly_solvable_claim=False)
