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

`rrt_connect_warehouse2_car_like_multiagent.ipynb` restores the original
three-car crossing scenario, including the published `testing_warehouse2`
start and goal. `rrt_connect_warehouse2_car_like_traffic_jam.ipynb` keeps the
ten-car opposing-queue stress case. Both use the same `warehouse2` map and
bicycle-model wheelbase, steering, acceleration, jerk, and braking limits.
Both allow reverse at 0.6 map units/s and pass smoothly through zero speed when changing direction.
The three-car notebook also includes a separate reverse-then-forward maneuver
on the same map. Car state is `[x, y, heading, v, steering_angle, a]`, and MPPI
samples `[jerk, steering_rate]`, integrating acceleration, velocity, and steering
within physical bounds throughout each rollout. Configure `a_min`, `a_max`,
`j_min`, `j_max`, and
`steering_rate_max` in the agent parameters. Animations mark each car's front
and rear, and raw plots show velocity, acceleration, jerk, steering angle,
and steering rate. Both profiles penalize the squared bounded controls inside
MPPI: the three-car profile uses `jerk_cost_weight=100`,
`steering_rate_cost_weight=300`, and sampling variances `[0.25, 0.04]`;
the jam uses magnitude weights `300/300` and variances `[0.0625, 0.0025]`.
Both add `jerk_change_cost_weight=500` and
`steering_rate_change_cost_weight=1000`, penalizing squared changes between
bounded commands inside rollouts. At the horizon boundary, the previous
command is the last executed jerk/steering rate, rather than the shifted
warm-start plan. The sampling standard deviations are `0.5/0.2` and
`0.25/0.05`, respectively. Steering noise stays at 0.2 for the three-car
case because that is only 10% of its hard bound and smaller values hurt
exploration of its tight starting turns. Hard jerk and steering-rate limits remain
unchanged. The three-car notebook includes a seeded two-bend free-space
comparison against the previous magnitude-only settings, saving raw comparison
plots to `car_control_regularization_comparison.png` and RMS control,
successive-command-change, saturation, and progress metrics to
`car_control_regularization_metrics.json`. Neither profile filters commands
or introduces another derivative into the state.
The three-car run permits 65 simulated seconds to finish its original routes
with the gentler command transitions; the jam retains its 60-second budget.
Each notebook saves a matching `*_dynamics.png` for review. The jam notebook
also reports queue occupancy, stopping, clearance, and unfinished agents. The
car body fits inside the circular collision radius used by RRT-Connect, ORCA,
and obstacle checks. These are concurrent inference examples, not Bench-MR scores.
The jerk-controlled jam uses a 2.5-second prediction horizon, a 5-second ORCA
warning time, 0.15 map units of controller clearance, full avoidance
responsibility, and a 1.2-unit/s forward cruise limit. Its earlier
acceleration-control profile reacted too late for the longer stopping response
introduced by jerk limits.
The run budget is 60 simulated seconds, so the reduced cruise speed does not
make the route lengths impossible solely because of the step limit.
ORCA additionally reserves each moving car's jerk-limited stopping distance
plus one step of travel as clearance; the reported gaps use the body envelope.

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
