import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'build/release'))
import mppi_ca_cpp as mppi


class PolygonTests(unittest.TestCase):
    square = [(0., 0.), (2., 0.), (2., 2.), (0., 2.)]

    def test_segments_and_radius(self):
        for polygon in (self.square, self.square[::-1], self.square + [self.square[0]]):
            for a, b, radius, expected in [
                ((-1, 1), (3, 1), 0, True),
                ((1, 1), (1, 1), 0, True),
                ((-1, 0), (0, 0), 0, True),
                ((-2, 0), (-1, 0), 0, False),
                ((-1, -.2), (3, -.2), .3, True),
                ((-1, -.4), (3, -.4), .3, False),
            ]:
                self.assertEqual(mppi.polygon_collision(a, b, polygon, radius), expected)

    def test_concave(self):
        polygon = [(0, 0), (2, 0), (2, 1), (1, 1), (1, 2), (0, 2)]
        self.assertFalse(mppi.polygon_collision((1.5, 1.5), (2, 2), polygon, 0))
        self.assertTrue(mppi.polygon_collision((.5, 1.5), (2, 1.5), polygon, 0))

    def test_validation_and_all_colliding(self):
        controller = mppi.Controller()
        for polygon in ([(0, 0)], [(0, 0), (1, 1), (2, 2)], [(0, 0), (1, 0), (0, float('nan'))]):
            with self.assertRaises(ValueError):
                controller.set_obstacles([polygon])
        controller.set_obstacles([self.square])
        control, _ = controller.next_step(np.array([1., 1., 0., 0., 0.]),
                                         np.empty((0, 4)), np.empty(0), np.array([4., 1., 0.]))
        np.testing.assert_array_equal(control, [0., 0.])
        controller.set_obstacles([])

    def test_anchor_path_validation_and_execution(self):
        controller = mppi.Controller()
        for path in (
            [(0., 0.)],
            [(0., 0.), (0., 0.)],
            [(0., 0.), (float('inf'), 1.)],
        ):
            with self.assertRaises(ValueError):
                controller.set_anchor_path(path)

        controller.set_anchor_path([(0., 0.), (0., -2.), (3., -2.), (3., 0.)])
        mppi.set_random_seed(42)
        control, trajectories = controller.next_step(
            np.zeros(5), np.empty((0, 4)), np.empty(0), np.array([3., 0., 0.])
        )
        self.assertTrue(np.isfinite(control).all())
        self.assertTrue(np.isfinite(trajectories).all())
        controller.set_anchor_path([])


if __name__ == '__main__':
    unittest.main()
