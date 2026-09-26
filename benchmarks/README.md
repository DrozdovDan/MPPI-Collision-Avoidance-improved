# Bench-MR polygon evaluation

This adapter runs the existing differential-drive MPPI controller on polygon
scenes and start/goal pairs from [Bench-MR](https://github.com/robot-motion/bench-mr).
It does not build Bench-MR's other planners or add obstacle constraints to ORCA.

## Run

Open `benchmarks/bench_mr.ipynb` for the notebook workflow. It contains task
selection, run configuration, an inline metrics table, trajectory plots, and
an animated GIF for a configurable task and seed.
By default it runs the five tasks whose coordinates are consistent with the
bundled SVG maps. The same workflow remains available from the command line.
For manual coordinates, set `CUSTOM_WORLD`, `CUSTOM_START`, and `CUSTOM_GOAL`
in the configuration cell. Poses use `[x, y, theta]`, with theta in radians.

From the project root, with the C++ module already built:

```bash
.venv/bin/pip install -r benchmarks/requirements.txt
cmake --build build/release --target mppi_all -j2
.venv/bin/python benchmarks/bench_mr.py run --repeats 3
```

The suite is bundled, so no Bench-MR installation is needed to run it. Results
are written to `benchmarks/results/current/`: full states and controls in
`results.json`, per-run metrics in `summary.csv`, and a map/path overview in
`trajectories.png`, plus a readable `REPORT.md`. Runs are sequential and results are saved after each run.
Use a separate `--output` directory to preserve a previous experiment.

## RRT-Connect anchor paths

`rrt_connect.py` provides a reusable bidirectional RRT-Connect planner for the
polygon worlds. It expands obstacles by the circular robot radius plus a
configurable clearance, constrains samples and edges to the world bounds, and
applies collision-checked shortcut smoothing before returning the anchor path.

```python
from rrt_connect import RRTConnectPlanner

planner = RRTConnectPlanner(
    world['obstacles'], world['bounds'], robot_radius=0.3,
    clearance=0.08, step_size=0.6, seed=42,
)
result = planner.plan(task['start'][:2], task['goal'][:2])
controller.set_anchor_path(result.path)
```

`rrt_connect_multiagent.ipynb` runs the published `parking1_2` and
`testing_parking1` tasks concurrently in their shared Bench-MR world. It shows
the raw RRT paths, smoothed anchor paths, executed trajectories, and an animated
multi-agent run. This is an integration example, not a Bench-MR score: the two
source tasks are combined into a concurrent scenario and ORCA is consequently
active.

`rrt_connect_warehouse2_multiagent.ipynb` is the large-map example. It exposes
editable `CUSTOM_STARTS` and `CUSTOM_GOALS` arrays and runs four agents through
the central bottleneck. The corner routes include the published
`testing_warehouse2` top-left pose and bottom-right goal; two additional
middle-lane routes create close interaction without an exact head-on deadlock.
The notebook reports the closest center distance and surface clearance over
every pair. Its controller profile is intentionally faster and smaller-batch
than the parking example so the long routes remain practical to execute
interactively. Set `SCENARIO = 'symmetric_head_on_deadlock'` to run the second
variant, where exact reverse anchors create deadlock-prone head-on traffic at
the central bottleneck. The agents may stall or pass, depending on the sampled
controls. The included `rrt_connect_warehouse2_deadlock.gif` captures a run
that stalled without colliding.

The default `close_crossing` uses anchor-heading alignment, a steering-change
cost, and a 10 rad/s² turn-acceleration limit for steadier tracking. The
head-on variant retains the original controller profile for comparison.
Rebuild `mppi_all` and restart the Jupyter kernel after C++ changes.

The [larger-agent example](rrt_connect_warehouse2_large_agents.ipynb) compares
radius-0.30 and radius-0.70 agents at narrow square-shelf gaps. It shows the
inflated obstacles, plans detours for two large agents, and animates their
concurrent ORCA-MPPI execution.

This notebook also enables anchor heading alignment, a modest steering-change
cost, and a 10 rad/s² angular-acceleration limit to reduce the visible
zigzagging that unconstrained MPPI sampling produced along the shelves.

Regenerate plots and the report without rerunning the controller:

```bash
.venv/bin/python benchmarks/bench_mr.py report
```

```bash
.venv/bin/python benchmarks/bench_mr.py run --task testing_parking1 --seed 42 --repeats 5 --timeout 120 --max-steps 4000 --output benchmarks/results/parking1
```

The notebook exposes `ROBOT_MAX_SPEED` directly and currently sets it to 3.0
units/s for the large maps. Command-line runs retain the original 1.0 default;
override it with `--max-speed 3`. `ROBOT_MAX_ANGULAR_SPEED`/
`--max-angular-speed` controls turning speed independently.

The `set_random_seed` binding seeds xtensor's shared native sampler **after**
controller construction. Seeds are `--seed + repeat`. NumPy seeding alone does
not control MPPI sampling. Reproduction requires the same native build and run
limits; wall-time cutoffs may stop at different steps on different machines.

## Source data

`bench_mr_tasks.json` records upstream revision
`c10ef1c81afe80ad1f4f3a2cc4369dd6ef7bf844`, file hashes, polygons, original
robot-model names, and all original start/goal poses from:

- `python/Benchmark Polygon Worlds.ipynb`: 20 tasks across three parking maps and a warehouse.
- `python/Polygon Worlds Testing.ipynb`: five additional tasks, including warehouse2.

Regenerate from a checkout with:

```bash
.venv/bin/python benchmarks/bench_mr.py import /path/to/bench-mr
```

The importer reads literal task definitions with `ast.literal_eval`; it does not
execute upstream notebooks. SVG paths are parsed with svgpathtools, translations
are applied, Y coordinates are flipped, and each obstacle is replaced by its
convex hull, matching `SvgPolygonLoader.hpp` and `PolygonMaze.h`. Scaling is 1.0,
as in the source notebooks. Unsupported curves, compound paths, and transforms
are rejected. Imported data retains its MIT license in `BENCH_MR_LICENSE`.

## Evaluation contract

This section describes the original `bench_mr.py`/`bench_mr.ipynb` baseline.
The separate RRT-Connect multi-agent notebook intentionally adds a global
planner and concurrent neighbors, as described above.

- Robot: circular radius 0.3, differential drive, forward speed up to 1.0,
  reverse speed down to -0.01, angular speed up to 2.0 rad/s. This is the current
  example profile, not Bench-MR's polygonal car or warehouse robot.
- MPPI: 700 samples, 20 states, dt=0.1, existing notebook cost settings and
  a 0.05 obstacle margin. No global planner, waypoints, or parameter tuning.
- Each task has one robot and no neighbors. ORCA therefore has no agent-agent
  constraints; static obstacles are still handled only by MPPI.
- Scene bounds follow Bench-MR's obstacle-extents bounds. Artificial external
  wall polygons enforce reference-point bounds inside MPPI; their offset
  compensates for the robot radius and margin. Original obstacle geometry is
  unchanged. Starts/goals outside the bounds or colliding with the robot disk
  are reported as `invalid_start_or_goal`, with explicit reasons, before planning.
- Success means reaching within 0.3 units of the goal without collision or
  leaving bounds. Heading error is recorded separately. `pose_success` additionally
  requires heading error <=0.15 rad, a diagnostic tolerance chosen for this
  adapter, not a claim of Bench-MR's native success criterion.
- Default limits: 2000 simulation steps (200 simulated seconds) or 30 wall
  seconds per run, whichever comes first. Timeout is checked between controller
  calls. `time_limit` and `step_limit` are unsuccessful runs, not proofs that a
  task is unsolvable. Warehouse tasks may need much larger budgets.
- Shapely independently checks the swept disk along every executed segment,
  rather than relying on the controller's collision helper. Minimum clearance
  is center-path distance to obstacle geometry minus the robot radius.
- Planning time sums controller calls; wall time also includes simulation and
  collision checks. Path length is meaningful as a solution metric only on
  successful runs. Invalid tasks have no executed trajectory.

This is an adapted static-obstacle evaluation, not a directly comparable score
against Bench-MR's published vehicle-planning results. The full-benchmark notebook
and current SVG files have coordinate inconsistencies: keep invalid tasks visible
and do not interpret them as MPPI planning failures or silently repair them.

## Tests

```bash
.venv/bin/python -m unittest discover -s tests
```

Tests cover SVG translation/Y flipping/convexification, unsupported paths,
source task coordinates, native RNG reproducibility, and obstacle collision cases.
