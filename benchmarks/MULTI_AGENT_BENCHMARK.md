# Multi-agent Bench-MR suite

The saved suite contains **50 tasks: ten per original map**, with 760 robot
assignments. [SCENARIOS.md](scenarios/multi_agent/SCENARIOS.md) is the complete
50-row report, including dimensions, validity, limitations and actual pilots.
The [contact sheet](scenarios/multi_agent/visualizations/all_50.contact.png)
shows every scenario. Difficulty labels describe intentions, not measured failures.

## Repository inspection and coordinates

`bench_mr_tasks.json` contains `parking1`, `parking2`, `parking3`, `warehouse`,
and `warehouse2`, imported at revision
`c10ef1c81afe80ad1f4f3a2cc4369dd6ef7bf844`. Existing `bench_mr.load_svg`
applies group translations, flips SVG Y into a Cartesian frame, and constructs
polygon hulls. Bounds are `[xmin, ymin, xmax, ymax]`. Imported scaling is 1.0;
the data does not establish meters per map unit. Dimensions, clearances,
velocities and accelerations use native map units and seconds.

The existing corridor demo's `--scale` controls the integer geometry lattice,
not physical resizing. No reusable cropping/affine-map API was found. This
suite uses the original maps: identity transform, scale 1, no crop, explicitly
stored in every task. Source geometry is bundled in `maps.json`. Robot sizes
and geometry are never silently rescaled.

The implementation reuses `FreeSpace`, physical corridor/resource extraction,
`RRTConnectPlanner`, the native K-route CLI, `CarLikeDyn`, `DiffDriveDyn`, and
`utils.launch_mppi`. Neither planner nor MPPI/ORCA objectives are changed.
Simulator additions are optional observation callbacks, per-agent profiles for
different wheelbases, and bounded differential-drive goal parking. Existing
calls retain their previous behavior by default.

## Models and collision definitions

Parking uses the existing six-state bicycle model, with controls
`[jerk, steering_rate]`. Length/width is 2.2 and wheelbase/length is 0.65.
Three map-dependent planning radii are 0.40, 0.55, and 1.30 times the lower
quartile of interior widths in `(0.45, 2)` map units. The largest class's
actual width exceeds a selected narrow aperture; size differences are not
defined solely by circle inflation. Proportions and the original examples'
map-scale robots guide the choice. Absolute SI vehicle sizes cannot be claimed
without a map calibration.

Warehouse uses the implemented differential-drive/unicycle model with disk
footprints and valid in-place rotation. Radii are 0.18, 0.32, and 0.55 times
the median interior width in the same interval. No holonomic model is invented.
Tasks 09 and 10 contain all three sizes. Each robot stores its dimensions,
wheelbase, speed, acceleration, jerk, steering, steering-rate and safety limits.

Both global planners and ORCA use the same conservative circumscribed radius.
Evaluation also checks the actual oriented rectangular car body. Rectangle
sweeps use endpoint hulls plus a rotational sagitta bound; suspected contacts
are refined at synchronized intermediate poses. Disk pair clearance uses exact
relative-segment closest approach. Crossing the same location at different
times is not a collision. Unresolved conservative contacts are exported
separately, not presented as demonstrated collisions. Both sampled actual-body
clearance and a continuous conservative-envelope lower bound are reported.
The outer boundary contains the full footprint; RRT's center bounds shrink by
radius plus margin, matching K's boundary semantics.

## Generation, selection and validation

```bash
.venv/bin/python benchmarks/multi_agent_bench.py generate --seed 20261016
.venv/bin/python benchmarks/multi_agent_bench.py validate
.venv/bin/python benchmarks/multi_agent_bench.py visualize
```

Default data: `benchmarks/scenarios/multi_agent`. Place global `--suite PATH`
before the command to use another folder. Saved tasks load without regeneration.

The physical graph is loaded once per map. Exact compressed inputs are saved
in `geometry_cache/`, so reproduction does not depend on unrelated experiment
caches. Radius-specific analysis subdivides physical references and checks
continuous clearance. Deterministic free-space samples attach as single leaves,
adding two-dimensional staging capacity without inventing topological loops.
Geometry and shortest-path queries are cached; all rejection loops are bounded.

| Task | Category | Requested robots | Intended difficulty |
| --- | --- | ---: | --- |
| 01 | Low-density random | 4 | ordinary |
| 02 | Moderate random | 8 | ordinary |
| 03 | Dense random | 16 | ordinary |
| 04 | Opposing bottleneck flows | 8 | challenging |
| 05 | Dense opposing flows | 16 | challenging |
| 06 | Intersection crossing | 12 | challenging |
| 07 | Symmetric swaps | 12 | deadlock-prone |
| 08 | Competing bottlenecks | 20 | deadlock-prone |
| 09 | Congested heterogeneous fleet | 24 | deadlock-prone |
| 10 | Maximum target stress | 32 | deadlock-prone |

This is exactly 30% ordinary, 30% challenging and 40% deadlock-prone intentions.
An eight-robot core focuses conflicting flows in dense tasks. Additional
traffic may use other regions after narrow staging strips fill; per-robot
`focused_assignment` and actual resource traversals record this. Task 10
targets 32, not a proof of the map's absolute maximum packing.

Ordinary tasks keep the first valid candidate. Others select the maximum
geometric score among up to three valid candidates in eight attempts:
`0.15*load + 0.20*bottleneck + 0.25*opposition + 0.20*concentration +
0.10*junction + 0.10*alternative_fraction`.
Components are clipped to `[0,1]`. Load is
`3*sum(pi*r²)/sum(reference_length)`; bottleneck load is
`max(users*max_radius/(4*width))`; opposition is
`sum(min(forward_users,reverse_users))/(N/2)`; concentration is the largest
resource share; junction and alternative terms are route fractions. The load
index uses the fixed native coordinate scale. Compare scores within a
map/category, not as calibrated cross-map difficulty measurements. Seeds,
scores, rejected candidates, fixed constants and selection rules are saved.

Parking feasibility requires an actual bounded-control rollout using the
existing bicycle dynamics. A fixed set of preview distances is a bounded
feasibility check, not a new evaluated planner. Failed checks reject candidates.
Warehouse certificates execute bounded acceleration pulses and in-place turns
through the existing unicycle model. Full controls/states are in `witnesses/`.
Validation replays every command, checks actual swept bodies and margins,
verifies start/goal envelopes, rejects overlapping starts/goals and establishes
individual arrival at rest. Goal heading is displayed but not required.
Point connectivity alone is never accepted as car reachability. Joint
multi-agent success is not required or claimed.

**Task-08 parking limitation:** the smallest car's circle configuration spaces
have zero holes on all three parking maps. A distinct topological alternative
corridor route is unavailable. These three tasks retain competing resources
and explicitly record that limitation. Warehouse task-08 cases store validated
alternate geometry-graph paths obtained by removing a corridor edge. K-route
outputs never select or reject generated tasks; obstacles are never edited to
manufacture alternatives.

## Benchmark runner

Build the existing dependencies/controller and native route CLI as described
in [README.md](README.md) and [COMPLETE_ROUTES.md](COMPLETE_ROUTES.md).

```bash
# One fixed scenario, both global strategies.
.venv/bin/python benchmarks/multi_agent_bench.py run --scenario parking1_01 --planner both

# One map and a robot-count range.
.venv/bin/python benchmarks/multi_agent_bench.py run --map warehouse2 --min-robots 12 --max-robots 24

# All 50, separate selection/controller seeds, isolated process workers.
.venv/bin/python benchmarks/multi_agent_bench.py run --planner both \
  --selection-seeds 41 42 --controller-seeds 7 8 --workers 4

# Explicit short pilot profile and observation budget, with actual GIFs.
.venv/bin/python benchmarks/multi_agent_bench.py run --scenario warehouse2_07 \
  --planner rrt --pilot --max-steps 300 --animate --output benchmarks/results/pilot

# Recompute metrics from untouched saved trajectories and aggregate a directory.
.venv/bin/python benchmarks/multi_agent_bench.py report --output benchmarks/results/pilot --audit
```

RRT is the naive independent baseline: existing RRT-Connect and shortcut
smoothing, without demonstration-notebook traffic waypoints or parking
reservations. Queries allow 15,000 iterations, with step sizes 0.5 in parking
and 1.5 in warehouses. K requests five radius-aware routes and independently
samples one uniformly per robot from the returned set. No route-probability
optimization or performance-based selection is used. Empty sets are recorded
as global planning failures.

Both use the same simulator and identical per-robot controller profiles. Car
settings reuse the current correlated/guided MPPI profile; warehouse settings
start from `bench_mr.parameters()`. Heterogeneous wheelbases are physical
per-robot inputs. `--pilot` uses 96 samples and 31/21 car/warehouse states,
equally for both planners, and is reported separately from default settings.
Profiles, seeds, code/binary hashes and native diagnostics are saved. Process
workers isolate the native sampler; threads do not share its random state.

Default simulation duration is `2*max(saved_reference_length/v_max)+60`
seconds. It comes from saved references, so both global strategies get the
same budget. `--max-steps` and `--wall-seconds` override observation limits.
Parked robots remain neighbors. Parking integrates bounded braking rather
than resetting pose, velocity, acceleration or steering.

## Metrics and operational deadlock

Runs save `run.json`, `run.csv`, `robots.csv`, `trajectory.npz`, `paths.json`
and `route_choices.json`. K also saves queries and all native routes. Batch
outputs are `results.json`, `runs.csv` and `aggregate.json`. They include
completion, all-robot success, individual/sum arrival times, makespan, path
lengths, stopped time, no-progress events, operational deadlocks, collisions,
clearance, resource occupancy, global planning times, local controller time
and simulation wall time. Global planning failures are distinct from simulation
timeouts and have no fabricated collision observation.

Unfinished arrivals and failed makespans/summed travel times are null.
Restricted total time charges unfinished robots the actual observation horizon.
Successful-only averages include their sample count. Completion requires a
settled terminal goal visit, not an early touch followed by departure.

`DeadlockConfig` distinguishes temporary slowing, congestion, sustained
no-progress, operational deadlock, collision and timeout. Defaults use slow
speed 0.15, stationary speed 0.03, progress 0.10 map units and 2/4/8-second
congestion/no-progress/deadlock windows. Operational deadlock additionally
requires at least two unfinished robots, persistent proximity for 85% of the
window, and a directed mutual-obstruction cycle within a forward cone toward
their goals. Isolated stalls, completed robots, brief waits and queues without
such a cycle are excluded. This is evidence of persistent blocking, not a
proof that no joint solution exists. Timeouts remain separate even when both
occur. Override named thresholds with `--deadlock-config FILE.json`.

The executed-state audit also checks commanded bounds. The current native
differential-drive emergency fallback can set yaw rate to zero immediately,
exceeding a configured `w_accel_max`. Such runs retain their raw data and are
flagged `control_limit_violation`, cannot count as all-robot success, and are
counted separately in aggregates. The benchmark does not change that controller
or hide its limitation with output filtering. Car pilots retain the actual
bounded jerk/steering-rate model.

## Visuals, pilots and tests

`visualizations/` contains 50 overviews, 20 stress zooms, five map contact
sheets and the all-50 contact sheet. Filled bodies/arrows show starts/headings;
outlines/stars show goals; IDs and physical resources are labeled. Equal axes
and native units are retained. `animations/` contains recorded examples for
every category.

Pilots in `pilots/` run after scenario selection and never feed back into
generation. The report lists actual completion, operational deadlocks,
collisions and timeouts. Short pilots are examples and smoke checks, not
full-budget method rankings. Generation and evaluation do not require pilots.
The packaged examples use RRT on `warehouse2_01` through `_10` and paired
RRT/K runs on `parking1_01`, `parking1_09`, `warehouse2_01` and `warehouse2_09`
(where recorded). Use `catalog-pilots` to audit the saved data and recreate
the animation index; category 09's fixed example is the heterogeneous parking
fleet. Task JSON is never annotated or reselected using pilot results.

```bash
OPENBLAS_NUM_THREADS=1 .venv/bin/python -m unittest discover -s tests
ctest --test-dir build/routes --output-on-failure
```
