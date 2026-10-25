"""Scenario contracts, actual-body reachability, and deadlock counterexamples."""

import copy
import gzip
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

import numpy as np
from shapely.geometry import Polygon

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "benchmarks"))
sys.path.insert(0, str(ROOT / "example"))
sys.path.insert(0, str(ROOT / "build/release"))
from multi_agent import CATEGORIES, MAP_NAMES
from multi_agent.geometry import (WorldGeometry, agent_parameters, diff_drive_witness, fingerprint,
                                  footprints, load_suite, swept_footprints, validate_task, validate_witness)
from multi_agent.analysis import AnalysisGraph
from multi_agent.generation import DEFAULT_CONFIG, Generator, validate_suite
from multi_agent.metrics import DeadlockConfig, collision_metrics, completion_data, detect_events, evaluate_run, validate_execution
from multi_agent.runner import controller_profile, serializable_profiles, global_routes

DATA = ROOT / "benchmarks/scenarios/multi_agent"


def disk_robot(index, start, goal):
    return dict(id=f"r{index:02d}", model="diff_drive", footprint="disk", radius=.2, length=.4, width=.4,
                wheelbase=None, safety_margin=.02, size_class="small", start=[*start, 0.],
                goal=dict(pose=[*goal, 0.], radius=.2, heading_required=False),
                motion_limits=dict(v_min=0., v_max=1., w_min=-2., w_max=2.,
                                   v_accel_max=1., v_brake_max=1., w_accel_max=2.))


def stationary_case(*, finished=False, queued=False, duration=12.):
    robots = [disk_robot(0, [2., 2.], [8., 2.]), disk_robot(1, [2.55, 2.], [1., 2.])]
    if queued: robots[1]["goal"]["pose"] = [9., 2., 0.]
    if finished: robots[1]["goal"]["pose"] = robots[1]["start"].copy()
    task = dict(robots=robots, robot_count=2, dt=.1, resources=[])
    states = np.tile(np.c_[[r["start"] for r in robots], np.zeros((2, 2))], (int(duration/.1)+1, 1, 1))
    commands = np.zeros((len(states)-1, 2, 2))
    paths = [[r["start"][:2], r["goal"]["pose"][:2]] for r in robots]
    # A zero-length goal path is not needed by the detector for a finished car.
    if finished: paths[1] = [[2.55, 2.], [2.56, 2.]]
    _, stable, _ = completion_data(task, states, commands)
    return task, states, commands, stable, paths


class SavedSuiteTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.manifest, cls.worlds, cls.tasks = load_suite(DATA)

    def test_exact_category_count_and_requested_fleets(self):
        self.assertEqual(len(self.tasks), 50)
        self.assertEqual(len({t["id"] for t in self.tasks}), 50)
        for world in MAP_NAMES:
            tasks = [t for t in self.tasks if t["source_map"] == world]
            self.assertEqual(len(tasks), 10)
            for task, (category, count, difficulty) in zip(tasks, CATEGORIES):
                self.assertEqual((task["category"], task["robot_count"], task["intended_difficulty"]), (category, count, difficulty))
                self.assertFalse(task["difficulty_is_measured"])

    def test_units_identity_models_and_heterogeneous_dimensions(self):
        for task in self.tasks:
            self.assertEqual(task["units"], "native_map_units")
            self.assertEqual(task["transformation"], dict(kind="identity", scale=1., crop=None))
            self.assertEqual(task["map_sha256"], fingerprint(self.worlds[task["source_map"]]))
            car = task["source_map"].startswith("parking")
            self.assertTrue(all(r["model"] == ("car_like" if car else "diff_drive") for r in task["robots"]))
            if task["category"] == "heterogeneous_congestion":
                self.assertEqual({r["size_class"] for r in task["robots"]}, {"small", "medium", "large"})
                if car:
                    self.assertEqual(len({r["wheelbase"] for r in task["robots"]}), 3)
            for r in task["robots"]:
                if car:
                    self.assertAlmostEqual(r["length"] / r["width"], 2.2)
                    self.assertAlmostEqual(math.hypot(r["length"], r["width"]) / 2, r["radius"])

    def test_all_saved_control_witnesses_and_footprints(self):
        report = validate_suite(DATA)
        self.assertTrue(report["passed"])
        self.assertEqual(report["robots_checked"], 760)

    def test_symmetric_tasks_swap_actual_endpoint_regions(self):
        for task in self.tasks:
            if task["category"] != "symmetric_swap": continue
            for a, b in zip(task["robots"][::2], task["robots"][1::2]):
                np.testing.assert_array_equal(a["start"][:2], b["goal"]["pose"][:2])
                np.testing.assert_array_equal(b["start"][:2], a["goal"]["pose"][:2])

    def test_parking_topology_limitation_and_warehouse_alternatives_are_explicit(self):
        for task in self.tasks:
            if task["category"] != "competing_bottlenecks": continue
            has_alternative = any(r["geometry_evidence"]["alternative_length"] is not None for r in task["robots"])
            if task["source_map"].startswith("parking"):
                self.assertFalse(has_alternative)
                self.assertTrue(any("zero holes" in s for s in task["limitations"]))
            else:
                self.assertTrue(has_alternative)
                robot = next(r for r in task["robots"] if r["geometry_evidence"]["alternative_path"] is not None)
                geometry = WorldGeometry(self.worlds[task["source_map"]])
                alternate = robot["geometry_evidence"]["alternative_path"]
                self.assertTrue(all(geometry.planner(robot).segment_is_free(a,b) for a,b in zip(alternate, alternate[1:])))

    def test_profiles_do_not_depend_on_routing_strategy(self):
        task = next(t for t in self.tasks if t["id"] == "parking1_09")
        profiles = serializable_profiles(controller_profile(task))
        self.assertEqual(len(profiles), task["robot_count"])
        for robot, profile in zip(task["robots"], profiles):
            self.assertEqual(profile["dist_between_axles"], robot["wheelbase"])
        self.assertEqual(len({fingerprint({k:v for k,v in p.items() if k != "dist_between_axles"}) for p in profiles}), 1)

    def test_parking_size_conflicts_include_actual_body_width(self):
        for task in self.tasks:
            if task["category"] != "heterogeneous_congestion" or not task["source_map"].startswith("parking"): continue
            evidence = task["size_accessibility"]
            self.assertTrue(evidence)
            self.assertTrue(any(e["vehicle_minimum_widths"]["small"] < e["local_neck_width"] < e["vehicle_minimum_widths"]["large"]
                                and e["small_witness_robot_ids"] for e in evidence))

    def test_corrupted_car_dynamics_certificate_is_rejected(self):
        task = next(t for t in self.tasks if t["id"] == "parking1_01")
        with np.load(DATA / task["witness_file"], allow_pickle=False) as data:
            robot = task["robots"][0]
            states = data[robot["id"]+"_states"].copy()
            controls = data[robot["id"]+"_controls"].copy()
        states[3, 5] += .2
        with self.assertRaises(AssertionError):
            validate_witness(states, controls, robot, WorldGeometry(self.worlds["parking1"]), task["dt"])

    def test_generation_seed_reproduces_geometry_and_witness(self):
        task = self.tasks[0]
        traffic_file = DATA / "geometry_cache/parking1.resources.json.gz"
        generator = Generator("parking1", self.worlds["parking1"], json.loads(gzip.decompress(traffic_file.read_bytes())), DEFAULT_CONFIG)
        regenerated = generator.candidate(1, 4, task["generation_seed"], task["generation"]["candidate_index"])
        self.assertIsNotNone(regenerated)
        other, witness = regenerated
        self.assertEqual(fingerprint(other["robots"]), fingerprint(task["robots"]))
        with np.load(DATA / task["witness_file"], allow_pickle=False) as saved:
            for key in witness: np.testing.assert_array_equal(witness[key], saved[key])


class DetectionTests(unittest.TestCase):
    def test_persistent_opposing_stationary_group_is_operational_deadlock(self):
        task, states, commands, stable, paths = stationary_case()
        events = detect_events(task, states, stable, paths)
        self.assertEqual(events["deadlock_count"], 1)
        self.assertEqual(events["deadlocks"][0]["robots"], ["r00", "r01"])
        self.assertGreaterEqual(events["deadlocks"][0]["detected_seconds"], 8.)

    def test_finished_robot_does_not_form_deadlocked_group(self):
        task, states, commands, stable, paths = stationary_case(finished=True)
        self.assertEqual(detect_events(task, states, stable, paths)["deadlock_count"], 0)

    def test_short_wait_is_not_deadlock(self):
        task, states, commands, stable, paths = stationary_case(duration=3.)
        self.assertEqual(detect_events(task, states, stable, paths)["deadlock_count"], 0)

    def test_same_direction_queue_without_cycle_is_not_deadlock(self):
        task, states, commands, stable, paths = stationary_case(queued=True)
        events = detect_events(task, states, stable, paths)
        self.assertEqual(events["deadlock_count"], 0)
        self.assertGreater(events["no_progress_event_count"], 0)

    def test_isolated_stalled_robots_are_no_progress_not_deadlock(self):
        task, states, commands, stable, paths = stationary_case()
        states[:, 1, 0] = 6.
        self.assertEqual(detect_events(task, states, stable, paths)["deadlock_count"], 0)

    def test_timeout_is_censored_and_independent_of_deadlock(self):
        task, states, commands, stable, paths = stationary_case(queued=True)
        world = dict(bounds=[0., 0., 10., 5.], obstacles=[])
        measured = evaluate_run(task, world, states, commands, paths, timeout=True)
        self.assertTrue(measured["timeout"])
        self.assertEqual(measured["deadlock_count"], 0)
        self.assertIsNone(measured["makespan_seconds"])
        self.assertIsNone(measured["sum_travel_times"])
        self.assertEqual(measured["individual_travel_times"], [None, None])

    def test_invalid_detection_windows_are_rejected(self):
        with self.assertRaises(ValueError): DeadlockConfig(deadlock_seconds=2.)
        with self.assertRaises(ValueError): DeadlockConfig(proximity_margin=-1.)


class ActualFootprintTests(unittest.TestCase):
    def test_rate_limit_violations_are_reported_without_editing_motion(self):
        robot = disk_robot(0,[1,1],[2,2]); task=dict(robots=[robot],robot_count=1,dt=.1)
        commands=np.array([[[0.,1.]],[[0.,0.]]])
        original=commands.copy()
        states=np.zeros((3,1,5));states[:,:,:2]=[1.,1.];states[1:,0,2]=.1
        report=validate_execution(task,states,commands)
        self.assertFalse(report["passed"])
        self.assertEqual(report["violations"][0]["kind"],"angular_acceleration")
        np.testing.assert_array_equal(commands,original)

    def test_rotation_sweep_covers_corner_not_seen_in_endpoint_poses(self):
        robot = dict(model="car_like", footprint="rectangle", length=2., width=.2, radius=math.hypot(1., .1))
        poses = np.array([[3., 3., 0.], [3., 3., math.pi/2]])
        obstacle = Polygon([[3.65,3.65],[3.75,3.65],[3.75,3.75],[3.65,3.75]])
        self.assertTrue(all(not p.intersects(obstacle) for p in footprints(poses, robot)))
        self.assertTrue(swept_footprints(poses, robot)[0].intersects(obstacle))

    def test_crossing_swept_regions_at_different_times_are_not_collision(self):
        robots = [disk_robot(0, [1,2], [3,2]), disk_robot(1, [2,1], [2,2])]
        robots[0]["radius"] = robots[1]["radius"] = .1
        states = np.zeros((2, 2, 5)); states[0, :, :2] = [[1,2],[2,1]]; states[1, :, :2] = [[3,2],[2,2]]
        task = dict(robots=robots, robot_count=2, dt=.1)
        world = dict(bounds=[0.,0.,5.,5.], obstacles=[])
        self.assertEqual(collision_metrics(task, world, states)["collisions"], 0)
        states[1,1,:2] = [2,3]
        self.assertEqual(collision_metrics(task, world, states)["collisions"], 1)

    def test_unicycle_witness_respects_speed_and_acceleration(self):
        robot = disk_robot(0, [1,1], [3,3])
        world = dict(bounds=[0.,0.,5.,5.], obstacles=[])
        states, controls = diff_drive_witness([[1,1],[3,1],[3,3]], robot)
        gap = validate_witness(states, controls, robot, WorldGeometry(world), .1)
        self.assertGreater(gap, 0.)
        np.testing.assert_allclose(states[-1,:2], [3,3], atol=1e-8)


class SimulatorInstrumentationTests(unittest.TestCase):
    def test_observer_can_end_run_without_changing_dynamics(self):
        import utils
        from dyn_models import DiffDriveDyn
        from bench_mr import parameters
        from params_names import EXP_TIME_STEP, XY_GOAL_TOLERANCE, MAX_EXP_STEPS
        robot = disk_robot(0,[0,0],[5,0]); profile = parameters() | dict(batch_size=32,time_steps=11)
        seen = []
        def observe(step, states, commands, seconds):
            seen.append((step, states.copy(), commands.copy(), seconds.copy()))
            return step < 3
        result = utils.launch_mppi(np.zeros((1,5)),np.array([[5.,0,0,0,0]]),1,
            [agent_parameters(robot)],profile,{EXP_TIME_STEP:.1,XY_GOAL_TOLERANCE:.2,MAX_EXP_STEPS:100},
            [DiffDriveDyn()], seed=42, anchor_paths=[[[0,0],[5,0]]],on_step=observe)
        self.assertFalse(result[0]);self.assertEqual(result[2],3)
        states = np.array(result[-1])
        for k, (_, measured, commands, seconds) in enumerate(seen):
            np.testing.assert_allclose(measured, states[k+1])
            self.assertTrue(np.all(seconds >= 0))
            np.testing.assert_allclose(np.linalg.norm(states[k+1,:,:2]-states[k,:,:2],axis=1),abs(commands[:,0])*.1,atol=1e-10)

    def test_differential_drive_parks_with_bounded_control_changes(self):
        import utils
        from dyn_models import DiffDriveDyn
        from bench_mr import parameters
        from params_names import EXP_TIME_STEP, XY_GOAL_TOLERANCE, MAX_EXP_STEPS
        robot = disk_robot(0,[0,0],[2,0]);profile=parameters() | dict(batch_size=96,time_steps=21,anchor_path_weight=2000.)
        commands=[]
        def observe(step, states, controls, seconds):commands.append(controls[0].copy())
        result=utils.launch_mppi(np.zeros((1,5)),np.array([[2.,0,0,0,0]]),1,
            [agent_parameters(robot)],profile,{EXP_TIME_STEP:.1,XY_GOAL_TOLERANCE:.3,MAX_EXP_STEPS:180},
            [DiffDriveDyn()],seed=42,anchor_paths=[[[0,0],[2,0]]],park_at_goal=True,on_step=observe)
        self.assertTrue(result[0])
        states=np.array(result[-1]);commands=np.array(commands)
        self.assertLessEqual(np.linalg.norm(states[-1,0,:2]-[2,0]),.3)
        np.testing.assert_allclose(commands[-1],0,atol=1e-8)
        np.testing.assert_allclose(states[-1,0,3:],0,atol=1e-8)
        self.assertLessEqual(abs(np.diff(np.r_[0.,commands[:,0]])).max(),.1+1e-8)
        self.assertLessEqual(abs(np.diff(np.r_[0.,commands[:,1]])).max(),.2+1e-8)


if __name__ == "__main__":
    unittest.main()
