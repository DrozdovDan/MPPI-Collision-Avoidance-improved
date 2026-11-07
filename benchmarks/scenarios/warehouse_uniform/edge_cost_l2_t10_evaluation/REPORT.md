# Warehouse edge-cost experiment: λ = 2, temperature τ = 10

All methods use the same 250 frozen tasks, 32 agents per task, normal MPPI/ORCA, 1.2 m/s speed bound, original physics, braking safety filter and termination rules. Each scenario has five start–goal variants. All historical runs remain preserved.

Only the path-selection parameters change. Stored graph edge lengths and widths are used exactly, with connectors excluded and full stored lengths charged for partial-edge entries. The 680 candidates with no graph edges retain zero cost. Every sampled anchor still includes the complete start-to-goal geometry. Random variates and controller seed remain paired with the previous experiment.

Fleet success requires all 32 robots to finish. Agent completion counts each finished robot, including failed fleets. Makespan is averaged over successful tasks only; failed makespans remain undefined. The sample count n is included.

| Method | Tasks | Fleet successes | Fleet success | Agent completion | Successful makespan (s) |
| --- | ---: | ---: | ---: | ---: | --- |
| RRT-Connect | 250 | 43 | 17.2% | 71.91% | 94.6 ± 21.2 (n=43) |
| K routes, uniform | 250 | 10 | 4.0% | 65.22% | 156.5 ± 27.1 (n=10) |
| K routes, bottleneck width | 250 | 35 | 14.0% | 78.06% | 134.2 ± 35.4 (n=35) |
| K routes, edge cost (λ=1, τ=1) | 250 | 43 | 17.2% | 67.46% | 96.1 ± 22.2 (n=43) |
| K routes, edge cost (λ=2, τ=10) | 250 | 50 | 20.0% | 70.31% | 99.2 ± 21.4 (n=50) |

| Map | Method | Tasks | Fleet successes | Fleet success | Agent completion | Successful makespan (s) |
| --- | --- | ---: | ---: | ---: | ---: | --- |
| warehouse | RRT-Connect | 125 | 9 | 7.2% | 61.38% | 85.2 ± 28.8 (n=9) |
| warehouse | K routes, uniform | 125 | 0 | 0.0% | 58.23% | — (0 successes) |
| warehouse | K routes, bottleneck width | 125 | 6 | 4.8% | 70.25% | 127.4 ± 11.7 (n=6) |
| warehouse | K routes, edge cost (λ=1, τ=1) | 125 | 12 | 9.6% | 66.38% | 85.1 ± 17.9 (n=12) |
| warehouse | K routes, edge cost (λ=2, τ=10) | 125 | 15 | 12.0% | 67.60% | 93.0 ± 17.9 (n=15) |
| warehouse2 | RRT-Connect | 125 | 34 | 27.2% | 82.45% | 97.1 ± 18.5 (n=34) |
| warehouse2 | K routes, uniform | 125 | 10 | 8.0% | 72.22% | 156.5 ± 27.1 (n=10) |
| warehouse2 | K routes, bottleneck width | 125 | 29 | 23.2% | 85.88% | 135.6 ± 38.5 (n=29) |
| warehouse2 | K routes, edge cost (λ=1, τ=1) | 125 | 31 | 24.8% | 68.55% | 100.4 ± 22.4 (n=31) |
| warehouse2 | K routes, edge cost (λ=2, τ=10) | 125 | 35 | 28.0% | 73.02% | 101.8 ± 22.4 (n=35) |

New-method raw terminal outcomes: `{'success': 50, 'deadlock': 200}`. Verified outcomes: `{'success': 50, 'deadlock': 200}`. Zero collisions or motion-limit violations. Rechecked 1000 historical execution checksums.

## Paired makespan comparison

Use only the same tasks successfully completed by both methods. Negative differences favor the new parameters.

| Compared with | Both successful tasks | New minus reference makespan (s) |
| --- | ---: | ---: |
| RRT-Connect | 29 | 7.70 |
| K routes, uniform | 9 | -50.17 |
| K routes, bottleneck width | 24 | -42.10 |
| K routes, edge cost (λ=1, τ=1) | 38 | 1.96 |

## Detailed data

- [Five-variant scenario averages](comparison_by_scenario.csv)
- [Size averages by map](comparison_by_map_size.csv)
- [Traffic-pattern averages by map](comparison_by_map_pattern.csv)
- [All executions](comparison_runs.csv)
- [Paired comparisons](paired_comparisons.csv)
- [New-method audit](audit.json)
- [Fleet success by size](fleet_success_by_size.png)
- [Agent completion by size](agent_completion_by_size.png)
- [Successful makespan by size](successful_makespan_by_size.png)

Reproduce with `OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 PYTHONPATH=benchmarks .venv/bin/python -m multi_agent.warehouse_cost_experiment --lambda-weight 2 --tau 10 --workers 20`.
