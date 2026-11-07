"""Length/width Gibbs distribution over a single robot's existing K routes.

Lengths and widths are in meters, radius is the physical radius (no safety
margin added), lambda_weight is dimensionless, and tau is in meters. W_max is
local to this candidate set, never global to the map or fleet. This module
does not plan paths, run a controller, or modify the candidate geometry.
"""

import math

import numpy as np


def edge_cost_distribution(routes, radius, *, lambda_weight, tau):
    """Evaluate the requested law on lists of {length, width} edge records.

    Every route must include every edge traversal it is to be charged for.
    Repeated edges count repeatedly in sums, but not in W_max. Empty routes
    have zero length and cost. When every route is empty, W_max is None (no q
    needs evaluating), and the distribution is uniform over zero costs.
    Returned costs are absolute; probabilities use cost differences to avoid
    overflow/underflow of all exponential weights at small temperature.
    """
    if not math.isfinite(radius) or radius <= 0:
        raise ValueError('radius must be positive and finite')
    if not math.isfinite(lambda_weight) or lambda_weight < 0:
        raise ValueError('lambda_weight must be nonnegative and finite')
    if not math.isfinite(tau) or tau <= 0:
        raise ValueError('tau must be positive and finite')
    routes = list(routes)
    if not routes:
        raise ValueError('At least one candidate route is required')
    arrays = []
    for route in routes:
        values = np.asarray([[edge['length'], edge['width']] for edge in route], dtype=float).reshape(-1, 2)
        if (not np.isfinite(values).all() or np.any(values[:, 0] < 0)
                or np.any(values[:, 1] <= 0)):
            raise ValueError('Edges need finite nonnegative lengths and positive widths')
        arrays.append(values)
    widths = [float(values[:, 1].max()) for values in arrays if len(values)]
    w_max = max(widths, default=None)
    records = []
    for values in arrays:
        # Divide first to keep 2*W_max from overflowing for very large widths.
        with np.errstate(over='ignore'):
            q = (np.clip(1. - .5 * (values[:, 1] / w_max) + radius / w_max, 0., 1.)
                 if len(values) else np.array([], dtype=float))
        with np.errstate(over='ignore', invalid='ignore'):
            length = float(np.sum(values[:, 0]))
            exposure = float(np.dot(values[:, 0], q))
            cost = length + lambda_weight * exposure
        if not all(map(math.isfinite, (length, exposure, cost))):
            raise ValueError('Route length or cost exceeds the finite numeric range')
        records.append(dict(length=length, width_exposure=exposure, cost=cost, q=q.tolist()))
    costs = np.array([record['cost'] for record in records])
    with np.errstate(over='ignore', under='ignore'):
        weights = np.exp(-(costs - costs.min()) / tau)
    probabilities = weights / weights.sum()
    for record, probability in zip(records, probabilities):
        record['probability'] = float(probability)
    return dict(w_max=w_max, radius=float(radius), lambda_weight=float(lambda_weight),
                tau=float(tau), probabilities=probabilities.tolist(), routes=records)


def graph_route_edges(route, graph_edges):
    """Use stored graph edge lengths and min_width, exactly as requested.

    Start/goal/direct connectors (no graph ID) are excluded. Even for a partial
    traversal the full stored graph edge length is used, not the traversed
    arclength. Preserve multiplicity and route order; do not deduplicate edges.
    """
    return [dict(edge_id=edge_id, length=float(graph_edges[edge_id]['length']),
                 width=float(graph_edges[edge_id]['min_width']))
            for edge_id in route['edge_ids']]


def draw_edge_cost_route(routes, radius, *, lambda_weight, tau, uniform_variate):
    """Sample one path by inverse CDF, using a supplied uniform variate in [0,1)."""
    if not math.isfinite(uniform_variate) or not 0 <= uniform_variate < 1:
        raise ValueError('uniform_variate must be in [0, 1)')
    distribution = edge_cost_distribution(routes, radius, lambda_weight=lambda_weight, tau=tau)
    cumulative = np.cumsum(distribution['probabilities'])
    cumulative[-1] = 1.
    index = int(np.searchsorted(cumulative, uniform_variate, side='right'))
    return index, distribution
