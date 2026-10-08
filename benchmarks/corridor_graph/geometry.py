"""Polygon-boundary Voronoi construction and collision-checked discretization."""

from dataclasses import dataclass
import math

import numpy as np
import pyvoronoi
import shapely
from shapely.geometry import LineString, MultiPolygon, Polygon, box
from shapely.ops import unary_union


@dataclass(frozen=True)
class GeometryConfig:
    coordinate_scale: int = 1_000_000
    sample_spacing: float = 0.15
    boundary_trim: float = 0.0001
    merge_tolerance: float = 1e-8

    def __post_init__(self):
        if (not isinstance(self.coordinate_scale, int) or self.coordinate_scale <= 0
                or self.coordinate_scale > 2**31 - 1):
            raise ValueError("coordinate_scale must be a positive int32")
        for name in ("sample_spacing", "boundary_trim", "merge_tolerance"):
            value = getattr(self, name)
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be finite and positive")
        if self.merge_tolerance >= self.boundary_trim:
            raise ValueError("merge_tolerance must be smaller than boundary_trim")


def polygon_parts(geometry):
    if isinstance(geometry, Polygon):
        return [geometry] if not geometry.is_empty else []
    return [part for part in getattr(geometry, "geoms", ())
            if isinstance(part, Polygon) and not part.is_empty]


class FreeSpace:
    """Physical free space from an existing Bench-MR world dictionary.

    Clearance measures distance to the exposed obstacle union AND outer wall.
    Obstacles overlapping/touching each other or the wall are unioned first.
    No robot footprint or obstacle inflation is used.
    """

    def __init__(self, world):
        bounds = np.asarray(world["bounds"], dtype=float)
        if (bounds.shape != (4,) or not np.isfinite(bounds).all()
                or np.any(bounds[:2] >= bounds[2:])):
            raise ValueError("bounds must be finite [min_x, min_y, max_x, max_y]")
        polygons = []
        for vertices in world["obstacles"]:
            coordinates = np.asarray(vertices, dtype=float)
            if (coordinates.ndim != 2 or coordinates.shape[1] != 2
                    or not np.isfinite(coordinates).all()):
                raise ValueError("obstacle coordinates must be finite 2D points")
            polygon = Polygon(coordinates)
            if not polygon.is_valid or polygon.is_empty or polygon.area <= 0:
                raise ValueError("obstacles must be valid nonzero-area polygons")
            polygons.append(polygon)
        self.bounds = tuple(bounds)
        self.world = {"bounds": list(map(float, bounds)),
                      "obstacles": [list(map(list, p.exterior.coords[:-1])) for p in polygons]}
        self.workspace = box(*bounds)
        self.obstacles = unary_union(polygons)
        self.geometry = self.workspace.difference(self.obstacles)
        self.components = polygon_parts(self.geometry)
        self.boundary = self.geometry.boundary

    def clearances(self, points):
        return np.asarray(shapely.distance(shapely.points(points), self.boundary))

    def contains_points(self, points):
        return np.asarray(shapely.contains(self.geometry, shapely.points(points)))

    def line_is_free(self, points):
        line = LineString(points)
        return (self.workspace.covers(line) and not self.obstacles.intersects(line)
                and not self.workspace.boundary.intersects(line))


def sample_polyline(points, spacing):
    """Retain every bend and subdivide chords to at most spacing map units."""
    points = np.asarray(points, dtype=float)
    result = [points[0]]
    for start, end in zip(points[:-1], points[1:]):
        distance = float(np.linalg.norm(end - start))
        if distance == 0:
            continue
        count = max(1, math.ceil(distance / spacing))
        result.extend(start + (end - start) * (i / count)
                      for i in range(1, count + 1))
    return np.asarray(result)


@dataclass(frozen=True)
class SkeletonEdge:
    u: int
    v: int
    geometry: tuple[tuple[float, float], ...]


@dataclass
class Skeleton:
    positions: dict[int, tuple[float, float]]
    edges: list[SkeletonEdge]
    effective_boundary_trim: float | None = None
    effective_merge_tolerance: float | None = None


def _independent_holes(polygon):
    """Count obstacle boundary groups separated from the outer boundary.

    GEOS may store holes touching the outer ring (or one another) at a point.
    Those contacts close the intervening passage; raw ring counts would mistake
    them for independent alternatives and incorrectly reject every positive inset.
    """
    rings = [LineString(polygon.exterior), *map(LineString, polygon.interiors)]
    parent = list(range(len(rings)))
    def root(index):
        while parent[index] != index:
            parent[index] = parent[parent[index]]
            index = parent[index]
        return index
    for i, ring in enumerate(rings):
        for j, other in enumerate(rings[:i]):
            if ring.intersects(other):
                parent[root(i)] = root(j)
    return len({root(i) for i in range(1, len(rings))} - {root(0)})


def _topology_preserving_interior(space, trim):
    targets = [_independent_holes(p) for p in space.components]
    floor = 64 * np.finfo(float).eps * max(1., *map(abs, space.bounds))
    for _ in range(32):
        parts = [polygon_parts(p.buffer(-trim)) for p in space.components]
        if all(len(p) == 1 and _independent_holes(p[0]) == target for p, target in zip(parts, targets)):
            return unary_union([p[0] for p in parts]), trim
        trim /= 10
        if trim <= floor:
            break
    raise ValueError("numerical boundary trim cannot preserve free-space topology at float precision")


def build_skeleton(space, config):
    """Compute primary segment-site GVD edges, retaining only valid free space.

    Boost constructs continuous line/parabola bisectors of polygon SIDES,
    including endpoint sites. Infinite edges lie outside the bounded domain.
    Curves are discretized with Pyvoronoi; every retained chord is clipped to
    free space. A tiny numerical inset trims zero-clearance corner endpoints;
    it is not a robot-radius offset and is never used to compute widths.
    """
    if not space.components:
        return Skeleton({}, [])
    origin = np.asarray(space.bounds[:2])
    extent = max(np.asarray(space.bounds[2:]) - origin)
    if extent * config.coordinate_scale >= 2**31 - 1:
        raise ValueError("coordinate_scale overflows Boost int32; reduce it")
    voronoi = pyvoronoi.Pyvoronoi(config.coordinate_scale)
    quantized = []
    for component in space.components:
        rings = []
        for ring in [component.exterior, *component.interiors]:
            points = np.asarray(ring.coords) - origin
            snapped = np.rint(points * config.coordinate_scale) / config.coordinate_scale
            # Boolean unions may insert consecutive vertices a few ulps apart.
            # Remove only duplicates on the configured integer lattice.
            snapped = snapped[np.r_[True, np.any(np.diff(snapped, axis=0) != 0, axis=1)]]
            if len(snapped) < 4:
                raise ValueError("boundary ring collapsed during quantization; increase scale")
            rings.append(snapped)
            for start, end in zip(snapped[:-1], snapped[1:]):
                voronoi.AddSegment([start.tolist(), end.tolist()])
        quantized.append(Polygon(rings[0], rings[1:]))
    if not MultiPolygon(quantized).is_valid:
        raise ValueError("quantization changed boundary topology; increase coordinate_scale")
    voronoi.Construct()
    vertices, raw_edges = voronoi.GetVertices(), voronoi.GetEdges()
    interior, effective_trim = _topology_preserving_interior(space, config.boundary_trim)
    effective_merge = min(config.merge_tolerance, effective_trim / 10)
    positions, edges, buckets = {}, [], {}

    def node_at(point):
        # Neighbor buckets avoid arbitrary rounding cuts at cell boundaries.
        point = tuple(map(float, point))
        key = tuple(math.floor(x / effective_merge) for x in point)
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for node in buckets.get((key[0] + dx, key[1] + dy), ()):
                    if math.dist(positions[node], point) <= effective_merge:
                        return node
        node = len(positions)
        positions[node] = point
        buckets.setdefault(key, []).append(node)
        return node

    def lines(geometry):
        if isinstance(geometry, LineString):
            yield geometry
        else:
            for part in getattr(geometry, "geoms", ()):
                yield from lines(part)

    for index, edge in enumerate(raw_edges):
        if (index > edge.twin or not edge.is_primary
                or edge.start < 0 or edge.end < 0):
            continue
        start = [vertices[edge.start].X, vertices[edge.start].Y]
        end = [vertices[edge.end].X, vertices[edge.end].Y]
        if math.dist(start, end) <= effective_merge:
            continue
        if edge.is_linear:
            points = np.asarray([start, end])
        else:
            points = np.asarray(list(voronoi.DiscretizeCurvedEdge(
                index, config.sample_spacing, 1 / config.coordinate_scale)))
            # Use the exact shared Boost vertices at both ends.
            points[0], points[-1] = start, end
        clipped = LineString(points + origin).intersection(interior)
        for line in lines(clipped):
            if line.length <= effective_merge:
                continue
            samples = sample_polyline(line.coords, config.sample_spacing)
            u, v = node_at(samples[0]), node_at(samples[-1])
            samples[0], samples[-1] = positions[u], positions[v]
            if not space.line_is_free(samples):
                raise RuntimeError("discretized medial edge intersects a boundary")
            edges.append(SkeletonEdge(u, v, tuple(map(tuple, samples))))
    used = {node for edge in edges for node in (edge.u, edge.v)}
    return Skeleton({node: positions[node] for node in used}, edges, effective_trim, effective_merge)
