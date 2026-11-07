"""Structural, physical and reproduction checks for the revised warehouse suite."""

from collections import Counter, defaultdict
import copy
import gzip
import json
from pathlib import Path
import sys
import unittest
from unittest.mock import Mock

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "benchmarks"))
from multi_agent.geometry import WorldGeometry, fingerprint, load_suite, validate_witness
from multi_agent.warehouse_generation import (DIAMETERS_METERS, PATTERNS, WAREHOUSE_CONFIG,
    UniformWarehouseGenerator, assignment_fingerprint, construct_variant,
    scenario_definitions, uniform_profile)

DATA = ROOT / "benchmarks/scenarios/warehouse_uniform"


class UniformWarehouseTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.manifest, cls.worlds, cls.tasks = load_suite(DATA)
        cls.groups = defaultdict(list)
        for task in cls.tasks:
            cls.groups[task["scenario_id"]].append(task)

    def test_exact_250_tasks_50_designs_and_8000_assignments(self):
        self.assertEqual(len(self.tasks), 250)
        self.assertEqual(len({t["id"] for t in self.tasks}), 250)
        self.assertEqual(Counter(t["source_map"] for t in self.tasks),
                         {"warehouse": 125, "warehouse2": 125})
        self.assertEqual(len(self.groups), 50)
        self.assertEqual(sum(t["robot_count"] for t in self.tasks), 8000)
        self.assertEqual(self.manifest["scenario_design_count"], 50)
        self.assertEqual(self.manifest["task_count"], 250)

    def test_full_size_pattern_product_on_each_map(self):
        self.assertEqual(json.loads((DATA / "scenario_definitions.json").read_text()), scenario_definitions())
        expected = {(d, pattern) for d in DIAMETERS_METERS for _, pattern in PATTERNS}
        for name in self.worlds:
            actual = {(group[0]["physical_scale"]["diameter_meters"], group[0]["traffic_pattern"])
                      for group in self.groups.values() if group[0]["source_map"] == name}
            self.assertEqual(actual, expected)

    def test_homogeneous_real_sized_agents_and_unmodified_maps(self):
        source = json.loads((ROOT / "benchmarks/bench_mr_tasks.json").read_text())
        metadata = json.loads((DATA / "maps.json").read_text())
        self.assertEqual(metadata["meters_per_map_unit"], 1.)
        self.assertIn("benchmark calibration", metadata["physical_scale_source"])
        for name, world in self.worlds.items():
            self.assertEqual(world, source["worlds"][name])
        for task in self.tasks:
            self.assertEqual(task["robot_count"], 32)
            self.assertEqual(task["requested_robot_count"], 32)
            self.assertEqual(task["map_sha256"], fingerprint(self.worlds[task["source_map"]]))
            profiles = [{k:v for k,v in r.items() if k not in ("id", "start", "goal", "geometry_evidence")}
                        for r in task["robots"]]
            self.assertEqual(len({fingerprint(p) for p in profiles}), 1)
            robot = task["robots"][0]
            diameter = robot["diameter_meters"]
            self.assertGreaterEqual(diameter, .6)
            self.assertLessEqual(diameter, 1.)
            self.assertAlmostEqual(robot["radius"]*2, diameter)
            self.assertEqual((robot["model"], robot["footprint"]), ("diff_drive", "disk"))
            self.assertFalse(task["generation"]["controller_or_k_used_for_selection"])
            self.assertFalse(task["validation"]["jointly_solvable_claim"])

    def test_five_different_starts_and_goals_for_every_agent(self):
        for group in self.groups.values():
            self.assertEqual({t["variant_index"] for t in group}, set(range(1,6)))
            self.assertEqual(len({t["assignment_sha256"] for t in group}), 5)
            self.assertEqual(len({t["generation"]["fixed_focus_index"] for t in group}), 1)
            for task in group:
                self.assertEqual(task["assignment_sha256"], assignment_fingerprint(task))
            for i in range(32):
                self.assertEqual(len({tuple(t["robots"][i]["start"][:2]) for t in group}), 5)
                self.assertEqual(len({tuple(t["robots"][i]["goal"]["pose"][:2]) for t in group}), 5)

    def test_endpoints_do_not_overlap(self):
        for task in self.tasks:
            for endpoint in ("start", "goal"):
                centers = np.array([r["start"][:2] if endpoint == "start" else r["goal"]["pose"][:2]
                                    for r in task["robots"]])
                distances = np.linalg.norm(centers[:,None]-centers[None,:], axis=2)
                distances[np.diag_indices(32)] = np.inf
                minimum = 2*task["robots"][0]["radius"] + task["robots"][0]["safety_margin"]
                self.assertGreaterEqual(distances.min(), minimum)

    def test_symmetric_swap_pairs_still_exchange_endpoints(self):
        for task in self.tasks:
            if task["traffic_pattern"] != "symmetric_swap": continue
            for a,b in zip(task["robots"][::2], task["robots"][1::2]):
                np.testing.assert_array_equal(a["start"][:2], b["goal"]["pose"][:2])
                np.testing.assert_array_equal(b["start"][:2], a["goal"]["pose"][:2])

    def test_extreme_size_witnesses_replay(self):
        for name in self.worlds:
            for diameter in (.6, 1.):
                task = next(t for t in self.tasks if t["source_map"] == name
                            and t["physical_scale"]["diameter_meters"] == diameter)
                geometry = WorldGeometry(self.worlds[name])
                with np.load(DATA / task["witness_file"], allow_pickle=False) as saved:
                    for robot in task["robots"]:
                        validate_witness(saved[robot["id"]+"_states"], saved[robot["id"]+"_controls"],
                                         robot, geometry, task["dt"])

    def test_seed_reproduces_saved_first_variant(self):
        task = self.tasks[0]
        name = task["source_map"]
        traffic = json.loads(gzip.decompress((DATA / f"geometry_cache/{name}.resources.json.gz").read_bytes()))
        config = copy.deepcopy(task["generation"]["config"])
        generator = UniformWarehouseGenerator(name, self.worlds[name], traffic,
                                              task["physical_scale"]["diameter_meters"], config)
        other, arrays, _ = construct_variant(generator, scenario_definitions()[0], 1, 0, config)
        self.assertEqual(other["assignment_sha256"], task["assignment_sha256"])
        self.assertEqual(fingerprint(other["robots"]), fingerprint(task["robots"]))
        with np.load(DATA / task["witness_file"], allow_pickle=False) as saved:
            for key, value in arrays.items(): np.testing.assert_array_equal(value, saved[key])

    def test_failed_packing_never_reduces_fleet(self):
        generator = Mock()
        generator.candidate.return_value = None
        with self.assertRaisesRegex(RuntimeError, "no reduced-count task"):
            construct_variant(generator, scenario_definitions()[0], 1, 0,
                              WAREHOUSE_CONFIG | dict(max_task_attempts=2))
        self.assertEqual(generator.candidate.call_count, 2)
        self.assertTrue(all(call.args[1] == 32 for call in generator.candidate.call_args_list))

    def test_rejects_subminiature_or_oversized_profiles(self):
        for diameter in (.2, .59, 1.01, 2.):
            with self.assertRaises(ValueError): uniform_profile("warehouse", {}, diameter)


if __name__ == "__main__":
    unittest.main()
