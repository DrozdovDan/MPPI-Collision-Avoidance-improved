# Warehouse benchmark with edge-cost sampling

All four methods use the same 250 tasks, 32 agents per task, normal MPPI/ORCA, 1.2 m/s speed limit, physics, safety filter and stopping rules. Each scenario has five start–goal variants.

The new method samples one fixed route per agent with λ = 1 and temperature τ = 1. It uses stored graph edge lengths and widths exactly: connectors are excluded, full stored edge lengths are charged for partial traversals, and the 680 zero-edge candidates retain zero cost. The sampled anchor still covers the complete start-to-goal path.

Fleet success means all 32 agents finish. Agent completion counts finished individuals, including failed fleets. Makespan is simulated completion time and is averaged over successful tasks only; failed makespans remain undefined. The sample count n is shown.

| Method | Tasks | Fleet successes | Fleet success | Agent completion | Successful makespan (s) |
| --- | ---: | ---: | ---: | ---: | --- |
| RRT-Connect | 250 | 43 | 17.2% | 71.91% | 94.6 ± 21.2 (n=43) |
| K routes, uniform | 250 | 10 | 4.0% | 65.22% | 156.5 ± 27.1 (n=10) |
| K routes, bottleneck width | 250 | 35 | 14.0% | 78.06% | 134.2 ± 35.4 (n=35) |
| K routes, edge cost (λ=τ=1) | 250 | 43 | 17.2% | 67.46% | 96.1 ± 22.2 (n=43) |

| Map | Method | Tasks | Fleet successes | Fleet success | Agent completion | Successful makespan (s) |
| --- | --- | ---: | ---: | ---: | ---: | --- |
| warehouse | RRT-Connect | 125 | 9 | 7.2% | 61.38% | 85.2 ± 28.8 (n=9) |
| warehouse | K routes, uniform | 125 | 0 | 0.0% | 58.23% | — (0 successes) |
| warehouse | K routes, bottleneck width | 125 | 6 | 4.8% | 70.25% | 127.4 ± 11.7 (n=6) |
| warehouse | K routes, edge cost (λ=τ=1) | 125 | 12 | 9.6% | 66.38% | 85.1 ± 17.9 (n=12) |
| warehouse2 | RRT-Connect | 125 | 34 | 27.2% | 82.45% | 97.1 ± 18.5 (n=34) |
| warehouse2 | K routes, uniform | 125 | 10 | 8.0% | 72.22% | 156.5 ± 27.1 (n=10) |
| warehouse2 | K routes, bottleneck width | 125 | 29 | 23.2% | 85.88% | 135.6 ± 38.5 (n=29) |
| warehouse2 | K routes, edge cost (λ=τ=1) | 125 | 31 | 24.8% | 68.55% | 100.4 ± 22.4 (n=31) |

Raw new-method terminal outcomes: `{'success': 43, 'deadlock': 207}`. Verified outcomes: `{'success': 43, 'deadlock': 207}`. Zero collisions or motion-limit violations. All 750 historical execution checksums were rechecked.

## Paired makespan comparison

These differences use only the same tasks successfully completed by both methods. Negative values favor edge-cost sampling.

| Compared with | Both successful tasks | Edge-cost minus reference makespan (s) |
| --- | ---: | ---: |
| RRT-Connect | 24 | 5.97 |
| K routes, uniform | 9 | -53.07 |
| K routes, bottleneck width | 23 | -43.23 |

## Detailed data

- [Five-variant scenario averages](comparison_by_scenario.csv)
- [Size averages by map](comparison_by_map_size.csv)
- [Traffic-pattern averages by map](comparison_by_map_pattern.csv)
- [All 1,000 run outcomes](comparison_runs.csv)
- [Paired comparisons](paired_comparisons.csv)
- [Full new-method audit](audit.json)
- [Fleet success by size](fleet_success_by_size.png)
- [Agent completion by size](agent_completion_by_size.png)

Reproduce with `OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 PYTHONPATH=benchmarks .venv/bin/python -m multi_agent.warehouse_cost_benchmark --workers 20`. Existing runs are reused only when their full evaluation identity matches.
