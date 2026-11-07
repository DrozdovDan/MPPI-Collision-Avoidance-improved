"""Bidirectional RRT-Connect for point anchors in polygonal workspaces."""

from dataclasses import dataclass
from functools import lru_cache
from collections import deque
import math

import numpy as np
import shapely
from shapely.geometry import LineString, Point, Polygon, box
from shapely.ops import unary_union


TRAPPED = 0
ADVANCED = 1
REACHED = 2


@lru_cache(maxsize=16)
def _sampling_mesh(blocked_wkb, bounds):
    """A configuration-space mesh guides samples, never supplies the RRT path."""
    free = box(*bounds).difference(shapely.from_wkb(blocked_wkb))
    triangles = shapely.get_parts(shapely.constrained_delaunay_triangles(free))
    tree = shapely.STRtree(triangles)
    adjacency = [[] for _ in triangles]
    edges = {}
    for i, triangle in enumerate(triangles):
        vertices = list(triangle.exterior.coords)
        for a, b in zip(vertices, vertices[1:]):
            key = tuple(sorted((a, b)))
            if key in edges:
                j = edges[key]; adjacency[i].append(j); adjacency[j].append(i)
            else: edges[key] = i
    return triangles, tree, adjacency


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
        self.points = np.empty((1024, 2))
        self.points[0] = root
        self.rooted_at_start = rooted_at_start

    def add(self, point, parent):
        if len(self.nodes) == len(self.points):
            self.points = np.resize(self.points, (2 * len(self.points), 2))
        self.points[len(self.nodes)] = point
        self.nodes.append(_Node(np.asarray(point, dtype=float), parent))
        return len(self.nodes) - 1

    def nearest(self, point):
        point = np.asarray(point, dtype=float)
        delta = self.points[:len(self.nodes)] - point
        return int(np.argmin(np.einsum('ij,ij->i', delta, delta)))

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
        guided_sampling=False,
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
        # GEOS round buffers inscribe circular arcs. Circumscribe them so a
        # shortcut cannot cut a corner inside the requested disk clearance.
        self.blocked = obstacle_union.buffer((robot_radius + clearance) / math.cos(math.pi/128), quad_segs=32)
        self.step_size = float(step_size)
        self.max_iterations = int(max_iterations)
        self.goal_sample_rate = float(goal_sample_rate)
        self.smoothing_iterations = int(smoothing_iterations)
        self.rng = np.random.default_rng(seed)
        self.guided_sampling = guided_sampling
        self.sampling_triangles = None

    def _guide_samples(self, start, goal):
        triangles, tree, adjacency = _sampling_mesh(self.blocked.wkb, tuple(self.bounds))
        starts = tree.query(Point(start), predicate="intersects")
        goals = set(tree.query(Point(goal), predicate="intersects"))
        queue = deque(map(int, starts)); parent = {int(i): None for i in starts}
        reached = None
        while queue:
            node = queue.popleft()
            if node in goals:
                reached = node; break
            for other in adjacency[node]:
                if other not in parent:
                    parent[other] = node; queue.append(other)
        if reached is None:
            raise RuntimeError("Endpoints are disconnected in the free configuration space")
        route = []
        while reached is not None:
            route.append(reached); reached = parent[reached]
        self.sampling_triangles = np.array([np.asarray(triangles[i].exterior.coords)[:3] for i in route])

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
        choice = self.rng.random()
        if choice < self.goal_sample_rate:
            return other_root.copy()
        if self.sampling_triangles is not None and choice < .8:
            triangle = self.sampling_triangles[self.rng.integers(len(self.sampling_triangles))]
            a, b = self.rng.random(2)
            if a + b > 1: a, b = 1-a, 1-b
            return triangle[0] + a*(triangle[1]-triangle[0]) + b*(triangle[2]-triangle[0])
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

        if self.guided_sampling:
            self._guide_samples(start, goal)

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
