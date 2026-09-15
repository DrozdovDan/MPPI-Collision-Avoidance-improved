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
