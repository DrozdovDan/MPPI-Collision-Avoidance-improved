# Full recorded comparison: warehouse_s13_v02

Same frozen starts, goals, 32 robots (diameter 0.8 m), physics, and normal MPPI/ORCA settings.

[Open full GIF](warehouse_s13_v02__rrt_vs_both_k_routes__full.gif)

| Method | Finished agents | Outcome | Recording ends | Makespan |
| --- | ---: | --- | ---: | ---: |
| RRT-Connect | 32/32 | success | 39.2 s | 39.2 s |
| K routes: uniform | 25/32 | deadlock | 61.0 s | undefined |
| K routes: bottleneck width | 28/32 | deadlock | 69.0 s | undefined |

The GIF shows the entire map above and the same blocking region below. Every selected anchor is dashed; every executed trail is solid. Filled circles are actual-size robot footprints with IDs; stars are goals. Agent IDs/colors match across methods.

All 691 recorded time steps from 0 to 69.0 s appear at 10 fps (1x playback), with a 2 s initial hold and 5 s final hold. A method that has terminated holds its last saved pose while the other recordings continue. Actual trace samples are used directly; there is no resimulation or pose interpolation.

Shortest full recording among tasks where RRT succeeds and both K policies fail. It is one example from the existing measurements.

[Final frame](final.png) · [Start frame](start.png) · [Audited source hashes and metadata](summary.json)

Reproduce from the project root:

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 PYTHONPATH=benchmarks .venv/bin/python -m multi_agent.warehouse_replay --task warehouse_s13_v02
```
