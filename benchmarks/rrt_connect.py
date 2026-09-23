"""Bidirectional RRT-Connect for point anchors in polygonal workspaces."""

from dataclasses import dataclass
import math

import numpy as np
from shapely.geometry import LineString, Point, Polygon, box
from shapely.ops import unary_union


TRAPPED = 0
ADVANCED = 1
REACHED = 2


@dataclass(frozen=True)
class RRTConnectResult:
    """Raw RRT tree path and its collision-checked shortcut smoothing."""

    raw_path: list
    path: list
    iterations: int


@dataclass
class _Node:
    point: np.ndarray
    parent: int | None


class _Tree:
    def __init__(self, root, rooted_at_start):
        self.nodes = [_Node(np.asarray(root, dtype=float), None)]
        self.rooted_at_start = rooted_at_start

    def add(self, point, parent):
        self.nodes.append(_Node(np.asarray(point, dtype=float), parent))
        return len(self.nodes) - 1

    def nearest(self, point):
        point = np.asarray(point, dtype=float)
        return min(
            range(len(self.nodes)),
            key=lambda index: float(np.sum((self.nodes[index].point - point) ** 2)),
        )

    def branch(self, index):
        result = []
        while index is not None:
            node = self.nodes[index]
            result.append(node.point.copy())
            index = node.parent
        return result[::-1]


class RRTConnectPlanner:
    """Plan a collision-free 2-D polyline using bidirectional RRT-Connect.

    Obstacles are expanded by ``robot_radius + clearance``. Workspace bounds
    constrain the robot reference point, matching the Bench-MR adapter.
    """

    def __init__(
        self,
        obstacles,
        bounds,
        robot_radius,
        clearance=0.05,
        step_size=0.75,
        max_iterations=10000,
        goal_sample_rate=0.05,
        smoothing_iterations=250,
        seed=None,
    ):
        bounds = np.asarray(bounds, dtype=float)
        if bounds.shape != (4,) or not np.isfinite(bounds).all():
            raise ValueError("bounds must contain finite [min_x, min_y, max_x, max_y]")
        if bounds[0] >= bounds[2] or bounds[1] >= bounds[3]:
            raise ValueError("bounds must have positive width and height")
        if not math.isfinite(robot_radius) or robot_radius < 0:
            raise ValueError("robot_radius must be finite and nonnegative")
        if not math.isfinite(clearance) or clearance < 0:
            raise ValueError("clearance must be finite and nonnegative")
        if not math.isfinite(step_size) or step_size <= 0:
            raise ValueError("step_size must be finite and positive")
        if max_iterations <= 0 or smoothing_iterations < 0:
            raise ValueError("iteration limits must be nonnegative and max_iterations positive")
        if not 0 <= goal_sample_rate <= 1:
            raise ValueError("goal_sample_rate must be between zero and one")

        polygons = []
        for vertices in obstacles:
            polygon = Polygon(vertices)
            if not polygon.is_valid or polygon.is_empty or polygon.area <= 0:
                raise ValueError("obstacles must be valid, nonzero-area polygons")
            polygons.append(polygon)

        self.bounds = bounds
        self.workspace = box(*bounds)
        obstacle_union = unary_union(polygons) if polygons else Polygon()
        self.blocked = obstacle_union.buffer(robot_radius + clearance)
        self.step_size = float(step_size)
        self.max_iterations = int(max_iterations)
        self.goal_sample_rate = float(goal_sample_rate)
        self.smoothing_iterations = int(smoothing_iterations)
        self.rng = np.random.default_rng(seed)

    @staticmethod
    def _point(value, name):
        point = np.asarray(value, dtype=float)
        if point.shape != (2,) or not np.isfinite(point).all():
            raise ValueError(f"{name} must contain two finite coordinates")
        return point

    def point_is_free(self, point):
        geometry = Point(np.asarray(point, dtype=float))
        return self.workspace.covers(geometry) and not self.blocked.intersects(geometry)

    def segment_is_free(self, start, end):
        segment = LineString([np.asarray(start, dtype=float), np.asarray(end, dtype=float)])
        return self.workspace.covers(segment) and not self.blocked.intersects(segment)

    def _sample(self, other_root):
        if self.rng.random() < self.goal_sample_rate:
            return other_root.copy()
        return self.rng.uniform(self.bounds[:2], self.bounds[2:])

    def _extend(self, tree, target):
        nearest_index = tree.nearest(target)
        nearest = tree.nodes[nearest_index].point
        delta = np.asarray(target, dtype=float) - nearest
        distance = float(np.linalg.norm(delta))
        if distance <= 1e-12:
            return REACHED, nearest_index
        new_point = nearest + delta * min(1.0, self.step_size / distance)
        if not self.segment_is_free(nearest, new_point):
            return TRAPPED, nearest_index
        new_index = tree.add(new_point, nearest_index)
        return (REACHED if distance <= self.step_size else ADVANCED), new_index

    def _connect(self, tree, target):
        while True:
            status, index = self._extend(tree, target)
            if status != ADVANCED:
                return status, index

    @staticmethod
    def _joined_path(tree_a, index_a, tree_b, index_b):
        branch_a = tree_a.branch(index_a)
        branch_b = tree_b.branch(index_b)
        if tree_a.rooted_at_start:
            start_branch, goal_branch = branch_a, branch_b
        else:
            start_branch, goal_branch = branch_b, branch_a
        return start_branch + list(reversed(goal_branch))[1:]

    def smooth(self, path):
        """Remove unnecessary vertices with randomized line-of-sight shortcuts."""
        smoothed = [np.asarray(point, dtype=float) for point in path]
        if self.smoothing_iterations == 0:
            return [point.tolist() for point in smoothed]
        for _ in range(self.smoothing_iterations):
            if len(smoothed) <= 2:
                break
            first, second = sorted(self.rng.choice(len(smoothed), size=2, replace=False))
            if second <= first + 1:
                continue
            if self.segment_is_free(smoothed[first], smoothed[second]):
                smoothed[first + 1:second] = []

        # Finish deterministically so the result does not retain any vertex
        # that has direct visibility to a later one.
        index = 0
        result = [smoothed[0]]
        while index < len(smoothed) - 1:
            next_index = len(smoothed) - 1
            while next_index > index + 1 and not self.segment_is_free(
                smoothed[index], smoothed[next_index]
            ):
                next_index -= 1
            result.append(smoothed[next_index])
            index = next_index
        return [point.tolist() for point in result]

    def plan(self, start, goal):
        start = self._point(start, "start")
        goal = self._point(goal, "goal")
        if not self.point_is_free(start):
            raise ValueError("start is outside the free configuration space")
        if not self.point_is_free(goal):
            raise ValueError("goal is outside the free configuration space")
        if np.linalg.norm(goal - start) <= 1e-12:
            point = start.tolist()
            return RRTConnectResult([point], [point], 0)
        if self.segment_is_free(start, goal):
            direct = [start.tolist(), goal.tolist()]
            return RRTConnectResult(direct, direct.copy(), 0)

        tree_a = _Tree(start, rooted_at_start=True)
        tree_b = _Tree(goal, rooted_at_start=False)
        for iteration in range(1, self.max_iterations + 1):
            sample = self._sample(tree_b.nodes[0].point)
            status_a, index_a = self._extend(tree_a, sample)
            if status_a != TRAPPED:
                status_b, index_b = self._connect(tree_b, tree_a.nodes[index_a].point)
                if status_b == REACHED:
                    raw = self._joined_path(tree_a, index_a, tree_b, index_b)
                    raw = [point.tolist() for point in raw]
                    return RRTConnectResult(raw, self.smooth(raw), iteration)
            tree_a, tree_b = tree_b, tree_a

        raise RuntimeError(f"RRT-Connect failed after {self.max_iterations} iterations")


def plan_anchor_path(start, goal, obstacles, bounds, **kwargs):
    """Convenience wrapper returning an :class:`RRTConnectResult`."""
    return RRTConnectPlanner(obstacles, bounds, **kwargs).plan(start, goal)
