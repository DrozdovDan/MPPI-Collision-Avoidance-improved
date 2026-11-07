# Open-route case: warehouse2_s02_v02

Five methods ran concurrently, one execution per method. The same frozen 32-agent task, normal MPPI/ORCA, 1.2 m/s bound, original bounded physics, safety filter, seed 42 and disappear-after-settled-target policy are retained.

All four K methods use the same 160 native candidates: K=5 for every agent, max_candidates=128, and no binding length-ratio filter. The existing binary uses a numerical cap of 1e12; the recorded conservative bound on every possible loop-free graph walk proves this cap cannot discard a candidate. RRT uses its original independent static-map anchors and is physically rerun as a control.

The original per-agent uniform random numbers are reused across the old and new route sets. Uniform sampling selects floor(5U); bottleneck and both graph-edge-cost methods use their own probability CDF with the same U. Graph-edge cost retains full stored edge lengths/min_width, excludes connectors, and uses physical radius 0.3 m.

| Method | Old finished | New finished | New result | New makespan |
| --- | ---: | ---: | --- | ---: |
| RRT-Connect | 32/32 | 32/32 | success | 53.9 s |
| K routes, uniform | 5/32 | 10/32 | deadlock | undefined |
| K routes, bottleneck | 17/32 | 16/32 | deadlock | undefined |
| K routes, lambda=1, tau=1 | 8/32 | 8/32 | deadlock | undefined |
| K routes, lambda=2, tau=10 | 10/32 | 10/32 | deadlock | undefined |

All five traces passed independent checks of geometry, sampling probabilities, original differential-drive equations, rate bounds, presence masks, collisions while present, arrivals and terminal evidence. The new RRT states, controls and presence mask reproduce the old disappearance run bit for bit. Makespans are defined only for all-32 successful executions. This is one seed on one task; no success rate across tasks or seeds is inferred.

[Audit](audit.json). [All metrics](comparison.csv). [Frozen experiment](experiment.json).

Reproduce or resume: `OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 PYTHONPATH=benchmarks .venv/bin/python -m multi_agent.warehouse_open_routes_case --workers 5`.

## Selected paths

Uniform sampling changed 14 anchors, bottleneck sampling changed 16. Both edge-cost parameterisations selected exactly the same 32 anchors as before; those complete traces, as well as RRT, must reproduce the previous motions bit for bit. Availability of wider alternatives does not force the cost distribution to choose them.

[Selected-path lengths and bottleneck crossing IDs](selection_diagnostics.json).
