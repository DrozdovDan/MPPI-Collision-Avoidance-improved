"""Analytic width-class intervals on reference chords, independent of sampling.

Distance to a polygon side is a capsule: an interior strip and endpoint circles.
Intersect those three primitives with each chord and union their arclength
intervals. No discretized buffer, robot footprint, or width interpolation is used.
"""

import math
import numpy as np
from shapely.geometry import LineString, box
from shapely.strtree import STRtree


def _linear_band(offset, slope, low, high, interval):
    if abs(slope) <= np.finfo(float).eps:
        return interval if low <= offset <= high else None
    a, b = (low - offset) / slope, (high - offset) / slope
    lo, hi = max(interval[0], min(a, b)), min(interval[1], max(a, b))
    return (lo, hi) if hi > lo else None


def _capsule_intervals(a, b, start, end, radius):
    delta, side = b - a, end - start
    d2, s2 = np.dot(delta, delta), np.dot(side, side)
    if d2 == 0 or s2 == 0:
        return []
    offset = a - start
    strip = _linear_band(np.dot(offset, side) / s2, np.dot(delta, side) / s2, 0., 1., (0., 1.))
    if strip:
        normal = np.array([-side[1], side[0]]) / math.sqrt(s2)
        strip = _linear_band(np.dot(offset, normal), np.dot(delta, normal), -radius, radius, strip)
    result = [strip] if strip else []
    for center in (start, end):
        relative = a - center
        projection = -np.dot(relative, delta) / d2
        cross = relative[0] * delta[1] - relative[1] * delta[0]
        residual = radius**2 - cross**2 / d2
        if residual <= 0:
            continue
        span = math.sqrt(residual / d2)
        lo, hi = max(0., projection - span), min(1., projection + span)
        if hi > lo:
            result.append((lo, hi))
    return result


def merge_intervals(intervals, tolerance):
    result = []
    for start, end in sorted(intervals):
        if end <= start:
            continue
        if result and start <= result[-1][1] + tolerance:
            result[-1] = (result[-1][0], max(end, result[-1][1]))
        else:
            result.append((float(start), float(end)))
    return result


class WidthClassIntervals:
    def __init__(self, auditor):
        self.auditor = auditor
        self.tree = STRtree([s for s, _ in auditor.segments])

    def extract(self, profile, width):
        distances, points, _ = profile
        radius = width / 2
        intervals = []
        for i, (a, b) in enumerate(zip(points[:-1], points[1:])):
            length = float(distances[i + 1] - distances[i])
            if length <= 0:
                continue
            bounds = (*list(np.minimum(a, b) - radius), *list(np.maximum(a, b) + radius))
            for feature_id in self.tree.query(box(*bounds)):
                start = self.auditor.starts[feature_id]
                end = start + self.auditor.deltas[feature_id]
                for lo, hi in _capsule_intervals(a, b, start, end, radius):
                    intervals.append((float(distances[i] + lo * length), float(distances[i] + hi * length)))
        return merge_intervals(intervals, self.auditor.precision)
