# Full recorded comparison: warehouse2_s02_v02

Same frozen starts, goals, 32 robots (diameter 0.6 m), physics, and normal MPPI/ORCA settings.

[Open full GIF](warehouse2_s02_v02__disappear_all_five__full.gif)

| Method | Finished agents | Outcome | Recording ends | Makespan |
| --- | ---: | --- | ---: | ---: |
| RRT-Connect | 32/32 | success | 53.9 s | 53.9 s |
| K routes: uniform | 5/32 | deadlock | 104.0 s | undefined |
| K routes: bottleneck | 17/32 | deadlock | 104.0 s | undefined |
| K routes: lambda=1, tau=1 | 8/32 | deadlock | 91.0 s | undefined |
| K routes: lambda=2, tau=10 | 10/32 | deadlock | 86.0 s | undefined |

The GIF shows the complete selected-path extent above and the same blocking region below. Every selected anchor is dashed; every executed trail is solid. Filled circles are present robot footprints with IDs; stars are goals. Removed robots have no circle, heading or ID; their executed trails remain. Agent IDs/colors match across methods.

All 1041 recorded time steps from 0 to 104.0 s appear at 10 fps (1x playback), with a 2 s initial hold and 5 s final hold. A method that has terminated holds its last presence mask and trails while other recordings continue; the successful RRT panel contains no remaining robots. Actual trace samples are used directly; there is no resimulation or pose interpolation.

Selected from completed disappearance runs: RRT succeeds, all four K-route variants deadlock. Choose the shortest maximum recorded duration among such tasks. It is one example from the existing measurements.

[Final frame](final.png) · [Start frame](start.png) · [Audited source hashes and metadata](summary.json)

Reproduce from the project root:

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 PYTHONPATH=benchmarks .venv/bin/python -m multi_agent.warehouse_disappear_replay --task warehouse2_s02_v02
```
