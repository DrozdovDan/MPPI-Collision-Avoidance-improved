import json
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'benchmarks'))
from bench_mr import ROOT, load_svg, tasks_for_results


class ImportTests(unittest.TestCase):
    def test_translation_flip_and_hull(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'scene.svg'
            path.write_text('<svg><g transform="translate(10,-2)">'
                            '<path d="M 0,0 h 2 v 2 l -1,-1 -1,1 z"/></g></svg>')
            polygon, = load_svg(path)
        self.assertEqual(set(map(tuple, polygon)), {(10., 2.), (12., 2.), (12., 0.), (10., 0.)})

    def test_curves_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'scene.svg'
            path.write_text('<svg><path d="M 0,0 Q 1,2 2,0 Z"/></svg>')
            with self.assertRaises(ValueError):
                load_svg(path)

    def test_published_task_counts_and_coordinates(self):
        suite = json.loads((ROOT / 'benchmarks/bench_mr_tasks.json').read_text())
        self.assertEqual(len(suite['tasks']), 25)
        self.assertEqual(len(suite['worlds']), 5)
        self.assertEqual(suite['tasks'][0]['start'], [0, -2.27, 0])
        self.assertEqual(suite['tasks'][0]['goal'], [7.72, -7.72, -1.58])
        self.assertEqual(len(suite['worlds']['parking1']['obstacles']), 13)

    def test_native_sampler_repeatable(self):
        sys.path.insert(0, str(ROOT / 'build/release'))
        import mppi_ca_cpp as mppi

        def sample():
            controller = mppi.Controller()
            mppi.set_random_seed(123)
            return controller.next_step(np.zeros(5), np.empty((0, 4)), np.empty(0), np.array([3., 0., 0.]))

        a, ta = sample()
        b, tb = sample()
        np.testing.assert_array_equal(a, b)
        np.testing.assert_array_equal(ta, tb)

        # Reset the actual rollout sampler without constructing another controller.
        # A seed applied only in the binding's DSO could otherwise appear to work
        # because both constructors seed their own generator within the same second.
        first, second = mppi.Controller(), mppi.Controller()
        mppi.set_random_seed(123)
        a, ta = first.next_step(np.zeros(5), np.empty((0, 4)), np.empty(0), np.array([3., 0., 0.]))
        mppi.set_random_seed(123)
        b, tb = second.next_step(np.zeros(5), np.empty((0, 4)), np.empty(0), np.array([3., 0., 0.]))
        np.testing.assert_array_equal(a, b)
        np.testing.assert_array_equal(ta, tb)

    def test_custom_task_is_available_to_result_renderers(self):
        suite = {'tasks': []}
        run = {'task': 'custom_map', 'world': 'map', 'group': 'custom',
               'start': [1, 2, 0], 'goal': [3, 4, 1]}
        task, = tasks_for_results(suite, [run])
        self.assertEqual(task['start'], [1, 2, 0])
        self.assertEqual(task['goal'], [3, 4, 1])


if __name__ == '__main__':
    unittest.main()
