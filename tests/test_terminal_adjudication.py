"""Visible joins must prove a traffic deadlock, never merely rename a stall."""
import sys
from pathlib import Path
import unittest

import numpy as np
import shapely

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'benchmarks'))
from multi_agent.warehouse_evaluation import EvaluationConfig
from multi_agent.warehouse_terminal_adjudication import certify_visible_join_deadlock


class VisibleJoinDeadlockTest(unittest.TestCase):
    def setUp(self):
        self.world = dict(bounds=[-10., -10., 10., 10.],
                          obstacles=[[[-4., -1.], [4., -1.], [4., 0.], [-4., 0.]]])
        self.paths = [[[5., 1.], [6., 1.], [6., -2.], [-5., -2.]],
                      [[.8, 1.], [4., 1.]], [[1.6, 1.], [-5., 1.]]]
        self.task = dict(dt=.1, robot_count=3, robots=[dict(radius=.3, safety_margin=.05,
                         goal=dict(pose=p[-1]+[0.], radius=.1)) for p in self.paths])
        self.states = np.zeros((451, 3, 5)); self.states[:, :, :2] = [[0., 1.], [.8, 1.], [1.6, 1.]]
        self.controls = np.zeros((450, 3, 2))
        self.evidence = dict(unfinished_robot_ids=[0, 1, 2], blocked_robot_ids=[1, 2],
                             obstruction_edges=[[1, 2], [2, 1], [2, 0]])

    def certify(self):
        return certify_visible_join_deadlock(self.task, self.world, self.paths,
            self.states, self.controls, self.evidence, EvaluationConfig())

    def test_hidden_projection_adds_proven_edge_into_existing_cycle(self):
        before = self.states.copy(); controls = self.controls.copy()
        result = self.certify()
        self.assertTrue(result['passed']); self.assertEqual(result['blocked_robot_ids'], [0, 1, 2])
        witness = result['added_visible_join_evidence'][0]
        self.assertLess(witness['hidden_join_clearance_meters'], .05)
        self.assertGreaterEqual(witness['visible_join_clearance_meters'], .05)
        self.assertLessEqual(shapely.LineString(witness['checked_forward_section']).length, 3.6+1e-9)
        self.assertEqual(witness['persistent_blocker_ratios'], {'1': 1., '2': 1.})
        np.testing.assert_array_equal(before, self.states)
        np.testing.assert_array_equal(controls, self.controls)

    def test_robots_beyond_horizon_do_not_certify_a_stall(self):
        self.states[:, 1, :2] = [7., 1.]; self.states[:, 2, :2] = [8., 1.]
        self.paths[1] = [[7., 1.], [9., 1.]]; self.paths[2] = [[8., 1.], [5., 1.]]
        with self.assertRaisesRegex(ValueError, 'no persistent physical blocker'):
            self.certify()

    def test_clear_nearest_projection_does_not_qualify(self):
        self.paths[0] = [[0., 1.], [4., 1.]]
        with self.assertRaisesRegex(ValueError, 'not a hidden-projection'):
            self.certify()

    def test_moving_fleet_does_not_qualify(self):
        self.states[-300:, 0, 3] = .2
        with self.assertRaisesRegex(ValueError, 'not persistently stationary'):
            self.certify()

    def test_partial_unfinished_coverage_does_not_qualify(self):
        self.evidence['unfinished_robot_ids'] = [1, 2]
        with self.assertRaisesRegex(ValueError, 'actual unfinished fleet'):
            self.certify()


if __name__ == '__main__':
    unittest.main()
