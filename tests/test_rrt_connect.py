import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "benchmarks"))
from rrt_connect import RRTConnectPlanner


class RRTConnectTests(unittest.TestCase):
    obstacle = [[4., 2.], [6., 2.], [6., 8.], [4., 8.]]

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
