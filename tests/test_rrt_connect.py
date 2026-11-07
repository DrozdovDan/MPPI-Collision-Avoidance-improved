import sys
import json
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "benchmarks"))
from rrt_connect import RRTConnectPlanner


class RRTConnectTests(unittest.TestCase):
    obstacle = [[4., 2.], [6., 2.], [6., 8.], [4., 8.]]

    def test_repaired_warehouse_assignments_fit_the_common_search_budget(self):
        root=Path(__file__).resolve().parents[1]/'benchmarks/scenarios/warehouse_uniform'
        world=json.loads((root/'maps.json').read_text())['worlds']['warehouse']
        for name,index in [('warehouse_s12_v02',1),('warehouse_s12_v03',7)]:
            with self.subTest(task=name):
                task=json.loads((root/'tasks'/f'{name}.json').read_text());robot=task['robots'][index]
                clearance=robot['radius']+robot['safety_margin']
                planner=RRTConnectPlanner(world['obstacles'],np.array(world['bounds'])+np.array([1,1,-1,-1])*clearance,
                    robot['radius'],robot['safety_margin'],step_size=1.5,max_iterations=100000,
                    smoothing_iterations=800,seed=42+index,guided_sampling=True)
                result=planner.plan(robot['start'][:2],robot['goal']['pose'][:2])
                self.assertLessEqual(result.iterations,100000)
                self.assertTrue(all(planner.segment_is_free(a,b) for a,b in zip(result.path,result.path[1:])))

    def test_guided_sampling_resolves_the_recorded_warehouse_query(self):
        root=Path(__file__).resolve().parents[1]/'benchmarks/scenarios/warehouse_uniform'
        task=json.loads((root/'tasks/warehouse_s01_v01.json').read_text())
        world=json.loads((root/'maps.json').read_text())['worlds']['warehouse']
        robot=task['robots'][11];c=robot['radius']+robot['safety_margin']
        planner=RRTConnectPlanner(world['obstacles'],np.array(world['bounds'])+np.array([1,1,-1,-1])*c,
            robot['radius'],robot['safety_margin'],step_size=1.5,max_iterations=15000,seed=53,guided_sampling=True)
        result=planner.plan(robot['start'][:2],robot['goal']['pose'][:2])
        self.assertLessEqual(result.iterations,15000)
        self.assertTrue(all(planner.segment_is_free(a,b) for a,b in zip(result.path,result.path[1:])))

    def test_plans_and_smooths_around_polygon(self):
        planner = RRTConnectPlanner(
            [self.obstacle], [0., 0., 10., 10.], robot_radius=.25,
            clearance=.05, step_size=.5, max_iterations=5000,
            smoothing_iterations=200, seed=5,
        )
        result = planner.plan([1., 5.], [9., 5.])
        self.assertEqual(result.path[0], [1., 5.])
        self.assertEqual(result.path[-1], [9., 5.])
        self.assertLessEqual(len(result.path), len(result.raw_path))
        self.assertGreater(len(result.path), 2)
        for start, end in zip(result.path, result.path[1:]):
            self.assertTrue(planner.segment_is_free(start, end))

    def test_direct_path_and_repeatability(self):
        def plan():
            planner = RRTConnectPlanner(
                [], [0., 0., 2., 2.], robot_radius=.1, seed=11
            )
            return planner.plan([.2, .2], [1.8, 1.8])

        first, second = plan(), plan()
        self.assertEqual(first.path, [[.2, .2], [1.8, 1.8]])
        self.assertEqual(first, second)

        unsmoothed = RRTConnectPlanner(
            [self.obstacle], [0., 0., 10., 10.], robot_radius=.25,
            step_size=.5, smoothing_iterations=0, seed=5,
        ).plan([1., 5.], [9., 5.])
        self.assertEqual(unsmoothed.path, unsmoothed.raw_path)

    def test_rejects_configuration_space_collision(self):
        planner = RRTConnectPlanner(
            [self.obstacle], [0., 0., 10., 10.], robot_radius=.25, seed=1
        )
        with self.assertRaisesRegex(ValueError, "start"):
            planner.plan([3.8, 5.], [9., 5.])
        with self.assertRaises(ValueError):
            planner.plan([1., np.nan], [9., 5.])


if __name__ == "__main__":
    unittest.main()
