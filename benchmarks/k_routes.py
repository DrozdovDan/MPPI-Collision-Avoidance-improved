"""Notebook adapter for the native, radius-aware keep-right K-route planner."""

import json
from pathlib import Path
import subprocess

import numpy as np


def prepare_corridor_map(world, filename, *, rebuild=False):
    """Build the offline graph once; reject a cached bundle for another map."""
    from corridor_graph import (
        build_corridor_graph, extract_traffic_resources, load_traffic_resources,
        simplify_corridor_graph,
    )
    from corridor_graph.traffic_geometry import geometry_hash

    filename = Path(filename)
    if filename.exists() and not rebuild:
        traffic = load_traffic_resources(filename)
        if traffic.metadata['map_sha256'] != geometry_hash(world):
            raise ValueError('Cached corridor map differs from world; set REBUILD_GRAPH = True')
        return traffic.to_dict()

    print('Building detailed corridor graph, routing graph, and traffic resources...')
    detailed = build_corridor_graph(world)
    routing = simplify_corridor_graph(detailed, world)
    # JSON turns integer metadata keys into strings. Normalize before extraction
    # computes hashes, so their ordering also matches the reloaded bundle.
    routing.metadata = json.loads(json.dumps(routing.metadata, allow_nan=False))
    traffic = extract_traffic_resources(detailed, routing, world)
    filename.parent.mkdir(parents=True, exist_ok=True)
    filename.write_text(json.dumps(traffic.to_dict(), allow_nan=False) + '\n')
    return traffic.to_dict()


def plan_k_routes(binary, map_file, queries, output_dir, *, stem='agents',
                  max_candidates=24, max_detour_ratio=1.8, diversity_threshold=0.2):
    """Batch queries in one native process so map and lane caches are shared.

    Each query has name, start, goal, radius, safety_margin, and k. Native
    verification checks every returned lane-aware path. An empty route set is
    an error for these notebooks: no controller receives an unplanned fallback.
    """
    binary = Path(binary).resolve()
    if not binary.is_file():
        raise FileNotFoundError(
            f'Native planner missing: {binary}. From the project root run:\n'
            'cmake -S . -B build/routes -DBUILD_MPPI_CONTROLLER=OFF '
            '-DCMAKE_BUILD_TYPE=Release\n'
            'cmake --build build/routes --target corridor_plan -j2'
        )
    if not queries:
        raise ValueError('Provide at least one route query')
    for query in queries:
        if isinstance(query['k'], bool) or not isinstance(query['k'], int) or query['k'] < 1:
            raise ValueError('k must be a positive integer')
        if (not np.isfinite(query['radius']) or query['radius'] <= 0 or
                not np.isfinite(query['safety_margin']) or query['safety_margin'] < 0):
            raise ValueError('Radius must be positive and safety margin nonnegative')
        for key in ('start', 'goal'):
            point = np.asarray(query[key], dtype=float)
            if point.shape != (2,) or not np.isfinite(point).all():
                raise ValueError(f'{key} must be a finite [x, y] point')

    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    query_file = output_dir / f'{stem}.queries.json'
    result_file = output_dir / f'{stem}.routes.json'
    query_file.write_text(json.dumps({'queries': queries}, indent=2, allow_nan=False) + '\n')
    command = [
        str(binary), '--map', str(Path(map_file).resolve()),
        '--queries', str(query_file.resolve()), '--output', str(result_file.resolve()),
        '--max-candidates', str(max_candidates), '--max-detour-ratio', str(max_detour_ratio),
        '--diversity-threshold', str(diversity_threshold), '--verify',
    ]
    completed = subprocess.run(command, capture_output=True, text=True)
    if completed.returncode:
        raise RuntimeError(f'Native route planning failed:\n{completed.stderr}\n{completed.stdout}')
    report = json.loads(result_file.read_text())
    if len(report['queries']) != len(queries):
        raise RuntimeError('Native planner returned an unexpected query count')
    for query, result in zip(queries, report['queries']):
        if not result['validation_passed']:
            raise RuntimeError(f"Native route verification failed for {query['name']}")
        if not result['routes']:
            raise RuntimeError(
                f"No keep-right route returned for {query['name']} "
                f"(radius={query['radius']}, margin={query['safety_margin']}). "
                f'Inspect {result_file} for native search/lane diagnostics.'
            )
    return report
