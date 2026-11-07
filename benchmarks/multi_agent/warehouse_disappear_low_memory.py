"""Memory-efficient orchestration of the unchanged disappear benchmark.

Only preparation cleanup and intermediate aggregation differ. Frozen simulation
functions, experiment identities, anchor selections and final audits are reused.
"""

from collections import Counter
import ctypes
import gc
import json
from pathlib import Path

from . import warehouse_disappear_benchmark as benchmark


VIEWS = dict(
    overall=['planner'],
    by_map=['source_map', 'planner'],
    by_scenario=['source_map', 'scenario_id', 'diameter_meters', 'traffic_pattern', 'planner'],
    by_map_size=['source_map', 'diameter_meters', 'planner'],
    by_map_pattern=['source_map', 'traffic_pattern', 'planner'],
)
METRIC_FIELDS = tuple(dict.fromkeys(
    [field for fields in VIEWS.values() for field in fields]
    + ['status', 'success', 'completion_fraction', 'makespan_seconds']
))


def release_unused_memory():
    """Return freed allocator pages without changing any live simulation data."""
    gc.collect()
    allocator = ctypes.CDLL(None)
    trim = getattr(allocator, 'malloc_trim', None)
    if trim is not None:
        trim.argtypes = [ctypes.c_size_t]
        trim.restype = ctypes.c_int
        trim(0)


def summarize(output, expected):
    """Produce the original tables without retaining full run diagnostics."""
    output = Path(output)
    runs = []
    for filename in sorted(output.glob('*/run.json')):
        record = json.loads(filename.read_text())
        runs.append({field: record[field] for field in METRIC_FIELDS})
        del record
    for name, fields in VIEWS.items():
        rows = benchmark.aggregate_rows(runs, fields)
        benchmark.save_json(output / f'{name}.json', rows)
        benchmark.write_csv(rows, output / f'{name}.csv')
    benchmark.save_json(output / 'progress_summary.json', dict(
        completed_runs=len(runs), expected_runs=expected,
        outcomes=dict(Counter(run['status'] for run in runs)),
        by_method=dict(Counter(run['planner'] for run in runs)),
    ))
    del runs
    release_unused_memory()


def main():
    original_prepare = benchmark.prepare

    def prepare(*args, **kwargs):
        experiment = original_prepare(*args, **kwargs)
        # Trim before the worker fork so unused preparation allocations are
        # neither retained in the parent nor inherited by the workers.
        release_unused_memory()
        return experiment

    benchmark.prepare = prepare
    benchmark.summarize = summarize
    benchmark.main()


if __name__ == '__main__':
    main()
