"""Analytical checks of the requested edge cost law; no controller simulations."""

from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'benchmarks'))
from multi_agent.edge_cost_selection import (
    draw_edge_cost_route, edge_cost_distribution, graph_route_edges,
)


def edge(length, width):
    return dict(length=length, width=width)


class EdgeCostSelectionTests(unittest.TestCase):
    def test_hand_calculated_costs_and_temperature_one(self):
        routes = [[edge(2, 2), edge(3, 4)], [edge(6, 4)]]
        result = edge_cost_distribution(routes, .5, lambda_weight=1, tau=1)
        self.assertEqual(result['w_max'], 4)
        np.testing.assert_allclose(result['routes'][0]['q'], [.875, .625])
        np.testing.assert_allclose([r['cost'] for r in result['routes']], [8.625, 9.75])
        expected = np.exp([0., -1.125]); expected /= expected.sum()
        np.testing.assert_allclose(result['probabilities'], expected)

    def test_maximum_is_over_all_candidates_and_radius_is_not_safety_inflated(self):
        a = edge_cost_distribution([[edge(2, 2)]], .5, lambda_weight=1, tau=1)
        b = edge_cost_distribution([[edge(2, 2)], [edge(1, 10)]], .5, lambda_weight=1, tau=1)
        self.assertEqual(a['routes'][0]['q'], [.75])
        np.testing.assert_allclose(b['routes'][0]['q'], [.95])

    def test_clamp_and_repeated_edge_length_exposure(self):
        r = edge_cost_distribution([[edge(2, .5), edge(2, .5)], [edge(1, 4)]],
                                   .5, lambda_weight=1, tau=1)
        self.assertEqual(r['routes'][0]['q'], [1., 1.])
        self.assertEqual(r['routes'][0]['cost'], 8.)

    def test_lambda_zero_is_length_only_and_equal_costs_are_uniform(self):
        routes = [[edge(3, 1)], [edge(3, 10)]]
        r = edge_cost_distribution(routes, .4, lambda_weight=0, tau=1)
        np.testing.assert_array_equal(r['probabilities'], [.5, .5])

    def test_short_narrow_exposure_is_distinct_from_long_narrow_exposure(self):
        routes = [[edge(1, 1), edge(9, 5)], [edge(9, 1), edge(1, 5)]]
        r = edge_cost_distribution(routes, .4, lambda_weight=1, tau=1)
        self.assertGreater(r['probabilities'][0], r['probabilities'][1])

    def test_stable_softmax_with_large_costs_and_cold_temperature(self):
        r = edge_cost_distribution([[edge(10000, 3)], [edge(10001, 3)]], .4,
                                   lambda_weight=1, tau=1)
        self.assertAlmostEqual(sum(r['probabilities']), 1)
        self.assertGreater(r['probabilities'][0], .8)
        r = edge_cost_distribution([[edge(10000, 3)], [edge(10001, 3)]], .4,
                                   lambda_weight=1, tau=1e-300)
        self.assertEqual(r['probabilities'], [1., 0.])

    def test_exact_graph_fields_ignore_connectors_and_partial_traversal_length(self):
        graph = {8: dict(length=12., min_width=3.)}
        route = dict(edge_ids=[8, 8], physical_metrics=dict(length=99),
                     edge_traversals=[dict(first_distance=1, last_distance=2)] * 2)
        self.assertEqual(graph_route_edges(route, graph),
                         [dict(edge_id=8, length=12., width=3.)] * 2)

    def test_zero_edge_candidates_have_zero_cost_and_all_empty_have_uniform_probability(self):
        r = edge_cost_distribution([[], [edge(1, 2)]], .5, lambda_weight=1, tau=1)
        self.assertEqual(r['routes'][0]['cost'], 0)
        self.assertAlmostEqual(r['probabilities'][0], 1 / (1 + np.exp(-1.75)))
        r = edge_cost_distribution([[], []], .5, lambda_weight=1, tau=1)
        self.assertIsNone(r['w_max'])
        self.assertEqual(r['probabilities'], [.5, .5])

    def test_inverse_cdf_and_invalid_parameters(self):
        routes = [[edge(1, 2)], [edge(1, 2)]]
        self.assertEqual(draw_edge_cost_route(routes, .5, lambda_weight=1, tau=1,
                                             uniform_variate=0.)[0], 0)
        self.assertEqual(draw_edge_cost_route(routes, .5, lambda_weight=1, tau=1,
                                             uniform_variate=.5)[0], 1)
        for radius, weight, tau in ((0, 1, 1), (.4, -1, 1), (.4, 1, 0),
                                     (.4, np.inf, 1), (.4, 1, np.nan)):
            with self.assertRaises(ValueError):
                edge_cost_distribution(routes, radius, lambda_weight=weight, tau=tau)
        for routes in ([], [[edge(-1, 2)]], [[edge(1, 0)]], [[edge(np.inf, 2)]]):
            with self.assertRaises(ValueError):
                edge_cost_distribution(routes, .4, lambda_weight=1, tau=1)
        with self.assertRaises(ValueError):
            draw_edge_cost_route([[edge(1, 2)]], .4, lambda_weight=1, tau=1, uniform_variate=1)


if __name__ == '__main__':
    unittest.main()
