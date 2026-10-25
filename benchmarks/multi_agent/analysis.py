"""Radius-filtered physical graph analysis; never calls the proposed K planner."""

from collections import OrderedDict
import math

import numpy as np
import shapely
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import connected_components, dijkstra
from scipy.spatial import cKDTree
from shapely.geometry import LineString, Point

from corridor_graph.geometry import sample_polyline


class AnalysisGraph:
    def __init__(self, traffic, geometry, robot, spacing):
        self.geometry, self.robot = geometry, robot
        routing = traffic["routing_geometry"]
        points = [n["position"] for n in routing["nodes"]]
        indices = {n["id"]: i for i, n in enumerate(routing["nodes"])}
        links, self.road_links = [], {}
        threshold = robot["radius"] + robot["safety_margin"]
        # Subdivide references so terminal tapers do not discard an otherwise
        # useful corridor. Continuous segment clearance is checked below.
        for edge in sorted(routing["edges"], key=lambda e: e["id"]):
            coords = sample_polyline(edge["reference_geometry"], spacing)
            ids = [indices[edge["u"]]]
            ids.extend(range(len(points), len(points)+len(coords)-2))
            points.extend(coords[1:-1].tolist())
            ids.append(indices[edge["v"]])
            edge_links = []
            for a, b in zip(ids, ids[1:]):
                if a != b:
                    edge_links.append((a, b))
            self.road_links[edge["id"]] = edge_links
            links.extend((a, b, edge["id"]) for a, b in edge_links)
        self.points = np.asarray(points)
        links = np.asarray(links, dtype=int)
        segments = shapely.linestrings(self.points[links[:, :2]])
        clearances = shapely.distance(segments, geometry.free.boundary)
        valid = (clearances > threshold + 1e-7) & shapely.contains(geometry.free.geometry, segments)
        self.links = links[valid]
        self.weights = np.linalg.norm(self.points[self.links[:, 0]] - self.points[self.links[:, 1]], axis=1)
        # Starts/goals occupy two-dimensional rooms, not just a one-dimensional
        # medial axis. Attach deterministic free-space samples as single leaves;
        # this adds staging capacity without creating fake topological routes.
        active = np.unique(self.links[:, :2])
        rng = np.random.default_rng(1731)
        bounds = np.asarray(geometry.world["bounds"])
        samples = rng.uniform(bounds[:2], bounds[2:], (7000, 2))
        mask = geometry.free.contains_points(samples) & (geometry.free.clearances(samples) > threshold + 1e-6)
        samples = samples[mask][:1200 if spacing < .5 else 2200]
        if len(samples):
            distance, nearest = cKDTree(self.points[active]).query(samples)
            segments = shapely.linestrings(np.stack([samples, self.points[active[nearest]]], axis=1))
            mask = ((distance < (3. if spacing < .5 else 7.)) &
                    (shapely.distance(segments, geometry.free.boundary) > threshold + 1e-7) &
                    shapely.contains(geometry.free.geometry, segments))
            samples, nearest = samples[mask], active[nearest[mask]]
            extra_ids = np.arange(len(self.points), len(self.points)+len(samples))
            extra = np.c_[extra_ids, nearest, np.full(len(samples), -1)]
            extra_weights = np.linalg.norm(samples-self.points[nearest], axis=1)
            self.points = np.concatenate([self.points, samples])
            self.links = np.concatenate([self.links, extra])
            self.weights = np.r_[self.weights, extra_weights]
        self.matrix = self._matrix()
        _, labels = connected_components(self.matrix, directed=False)
        active = np.unique(self.links[:, :2])
        groups = [(sum(labels[active] == k), int(k)) for k in set(labels[active])]
        self.main_component = max(groups)[1]
        self.labels = labels
        self.pool = active[labels[active] == self.main_component]
        self.tree = cKDTree(self.points[self.pool])
        self.cache = OrderedDict()
        self.resource_features = []
        for resource in traffic["resources"]:
            if resource["min_width"] <= 2 * threshold or resource["length"] < .25:
                continue
            self.resource_features.append(self._feature(resource))
        self.junction_features = []
        for resource in traffic["junction_regions"]:
            node_id = resource.get("junction_node_id")
            if node_id not in indices:
                continue
            node = indices[node_id]
            if node not in self.pool or self.matrix.getrow(node).nnz < 3:
                continue
            self.junction_features.append(self._feature(resource, self.points[node]))

    def _matrix(self, excluded_road=None):
        keep = np.ones(len(self.links), dtype=bool) if excluded_road is None else self.links[:, 2] != excluded_road
        links, weights = self.links[keep], self.weights[keep]
        return csr_matrix((np.r_[weights, weights],
                           (np.r_[links[:, 0], links[:, 1]], np.r_[links[:, 1], links[:, 0]])),
                          shape=(len(self.points), len(self.points)))

    def shortest(self, start, goal, excluded_road=None):
        if start == goal:
            return None
        if excluded_road is None and start in self.cache:
            distances, previous = self.cache.pop(start)
            self.cache[start] = (distances, previous)
        else:
            distances, previous = dijkstra(self.matrix if excluded_road is None else self._matrix(excluded_road),
                                           directed=False, indices=int(start), return_predecessors=True)
            if excluded_road is None:
                self.cache[start] = (distances, previous)
                if len(self.cache) > 32:
                    self.cache.popitem(last=False)
        if not np.isfinite(distances[goal]):
            return None
        ids = [int(goal)]
        while ids[-1] != start:
            node = int(previous[ids[-1]])
            if node < 0 or len(ids) > len(self.points):
                return None
            ids.append(node)
        return self.points[ids[::-1]].copy()

    def _feature(self, resource, position=None):
        chain = max(resource["geometry"], key=len)
        center = np.asarray(position if position is not None else resource["min_width_position"])
        chain = np.asarray(chain)
        index = min(max(1, int(np.argmin(np.linalg.norm(chain-center, axis=1)))), len(chain)-1)
        tangent = chain[index] - chain[index-1]
        tangent /= max(1e-12, np.linalg.norm(tangent))
        return dict(id=int(resource["id"]), kind=resource["kind"], center=center.tolist(),
                    tangent=tangent.tolist(), width=float(resource["min_width"]),
                    representative_width=float(resource["representative_width"]),
                    radius=max(float(resource["min_width"]) / 2, 2 * self.robot["radius"]),
                    road_ids=resource["source_edge_ids"], geometry=resource["geometry"])

    def feature_pairs(self, feature, rng, count=300):
        positions = self.points[self.pool]
        delta = positions - feature["center"]
        distance = np.linalg.norm(delta, axis=1)
        side = delta @ feature["tangent"]
        reach = max(6 * feature["width"], 12 * self.robot["radius"], 5.)
        first = self.pool[(side < -max(feature["width"]*.5, self.robot["radius"]*2)) & (distance < reach)]
        second = self.pool[(side > max(feature["width"]*.5, self.robot["radius"]*2)) & (distance < reach)]
        if not len(first) or not len(second):
            return
        for _ in range(count):
            yield int(rng.choice(first)), int(rng.choice(second))

    def features_for_path(self, path, features):
        line = LineString(path)
        found = []
        for f in features:
            if line.distance(Point(f["center"])) < min(f["radius"], max(self.robot["radius"] * 1.5, f["width"]*.3)):
                at = line.project(Point(f["center"]))
                before = np.asarray(line.interpolate(max(0, at-.2)).coords[0])
                after = np.asarray(line.interpolate(min(line.length, at+.2)).coords[0])
                direction = 1 if np.dot(after-before, f["tangent"]) >= 0 else -1
                found.append((f["id"], direction))
        return found

    def alternative(self, path, feature):
        start = int(self.tree.query(path[0])[1]); start = int(self.pool[start])
        goal = int(self.tree.query(path[-1])[1]); goal = int(self.pool[goal])
        candidates = []
        for road in feature["road_ids"]:
            alternate = self.shortest(start, goal, int(road))
            if alternate is not None and LineString(alternate).distance(Point(feature["center"])) > max(self.robot["radius"], feature["width"]*.3):
                candidates.append(alternate)
        return min(candidates, key=lambda p: LineString(p).length) if candidates else None
