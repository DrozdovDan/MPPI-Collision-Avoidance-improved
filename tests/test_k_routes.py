"""Exercise the notebook adapter against real offline and native planners."""

import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

import numpy as np
from shapely.geometry import LineString, Polygon, box

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'benchmarks'))

from k_routes import plan_k_routes, prepare_corridor_map


class KRouteNotebookAdapterTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.directory = tempfile.TemporaryDirectory()
        cls.addClassCleanup(cls.directory.cleanup)
        cls.folder = Path(cls.directory.name)
        cls.map_file = cls.folder / 'map.json'
        cls.world = dict(bounds=[0., 0., 12., 10.],
                         obstacles=[[[5., 3.], [7., 3.], [7., 7.], [5., 7.]]])
        cls.bundle = prepare_corridor_map(cls.world, cls.map_file)
        cls.binary = ROOT / 'build/routes/corridor_plan'

    def test_cache_reuse_and_map_mismatch(self):
        with patch('corridor_graph.build_corridor_graph', side_effect=AssertionError('rebuilt')):
            cached = prepare_corridor_map(self.world, self.map_file)
        self.assertEqual(cached['metadata'], self.bundle['metadata'])
        changed = self.world | {'bounds': [0., 0., 13., 10.]}
        with self.assertRaisesRegex(ValueError, 'differs from world'):
            prepare_corridor_map(changed, self.map_file)

    def test_cache_round_trip_with_two_digit_metadata_ids(self):
        world = dict(bounds=[0., 0., 24., 20.], obstacles=[
            [[5., 3.], [7., 3.], [7., 7.], [5., 7.]],
            [[12., 3.], [14., 3.], [14., 7.], [12., 7.]],
            [[12., 12.], [14., 12.], [14., 16.], [12., 16.]],
        ])
        filename = self.folder / 'many_nodes.json'
        original = prepare_corridor_map(world, filename)
        mapping = original['routing_geometry']['metadata']['working_to_routing']
        self.assertGreater(max(map(int, mapping)), 9)
        reloaded = prepare_corridor_map(world, filename)
        self.assertEqual(original['metadata'], reloaded['metadata'])

    def test_native_batch_preserves_radius_and_complete_safe_paths(self):
        if not self.binary.exists():
            self.skipTest('Build build/routes/corridor_plan to run native integration')
        queries = [dict(name=f'radius{radius}', start=[1., 5.], goal=[11., 5.],
                        radius=radius, safety_margin=.05, k=5) for radius in (.2, .7)]
        report = plan_k_routes(self.binary, self.map_file, queries, self.folder, stem='batch')
        obstacle = Polygon(self.world['obstacles'][0])
        boundary = box(*self.world['bounds']).boundary
        for query, result in zip(queries, report['queries']):
            self.assertEqual(result['name'], query['name'])
            self.assertEqual(result['radius'], query['radius'])
            self.assertEqual(result['safety_margin'], query['safety_margin'])
            self.assertTrue(result['validation_passed'])
            self.assertGreater(len(result['routes']), 0)
            self.assertLessEqual(len(result['routes']), query['k'])
            for route in result['routes']:
                np.testing.assert_allclose(route['path'][0], query['start'])
                np.testing.assert_allclose(route['path'][-1], query['goal'])
                path = LineString(route['path'])
                clearance = query['radius'] + query['safety_margin']
                self.assertGreaterEqual(path.distance(obstacle), clearance - 1e-7)
                self.assertGreaterEqual(path.distance(boundary), clearance - 1e-7)
        saved = json.loads((self.folder / 'batch.queries.json').read_text())
        self.assertEqual(saved['queries'], queries)

    def test_empty_route_set_stops_before_inference(self):
        if not self.binary.exists():
            self.skipTest('Build build/routes/corridor_plan to run native integration')
        # The requested start lies inside the shelf.
        query = dict(name='blocked', start=[6., 5.], goal=[11., 5.],
                     radius=.2, safety_margin=.05, k=5)
        with self.assertRaisesRegex(RuntimeError, 'No keep-right route returned for blocked'):
            plan_k_routes(self.binary, self.map_file, [query], self.folder, stem='blocked')


if __name__ == '__main__':
    unittest.main()
