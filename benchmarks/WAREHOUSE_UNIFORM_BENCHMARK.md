# Uniform-agent warehouse benchmark

The revised suite is saved in `scenarios/warehouse_uniform/`: **250 tasks,
125 on each of `warehouse` and `warehouse2`**. Each map has 25 scenario designs;
each design has five independently seeded start–goal variants. Every task has
exactly **32 agents**, giving 8,000 individual assignments.

The [comparison review](scenarios/warehouse_uniform/evaluation/REVIEW.md)
distinguishes individual-agent completion from all-32 fleet success, reports
actual returned route counts, and records controlled checks of path tracking,
waiting behavior and route assignment.

All agents in a scenario have identical footprints and motion limits. The
25 designs are the Cartesian product of these choices:

| Agent diameter | Traffic patterns, each with five variants |
| --- | --- |
| 0.6 m | distributed, opposing bottleneck, intersection, symmetric swap, competing bottlenecks |
| 0.7 m | same five patterns |
| 0.8 m | same five patterns |
| 0.9 m | same five patterns |
| 1.0 m | same five patterns |

The footprints are differential-drive disks, using the existing unicycle
model. The safety margin is 0.04 m. The existing limits are retained: forward
speed 1.2 m/s, acceleration 0.8 m/s², braking 1.2 m/s², angular speed 1.8 rad/s,
and angular acceleration 2 rad/s². The simulation retains its original unicycle
integration; native command generation and route construction have been corrected
as described below.

Differential-drive anchor projection checks the swept footprint of the connection
to a candidate segment. A later segment across shelving cannot receive progress
credit merely because it is closer in Euclidean distance. A regression from a
recorded off-route stall checks forward, collision-free reacquisition around
the correct corner.

The source maps do not document meters per coordinate unit. This suite
**explicitly adopts 1 native map unit = 1 meter as its physical calibration**.
This is a benchmark convention, not a measured source scale. The original
coordinates, obstacles and bounds are unchanged. Under this convention,
the map extents are approximately 85.71 × 80.43 m and 124.41 × 81.24 m.
Sizes in the saved assignments are therefore 0.6–1.0 m, rather than the old
map-dependent tiny robots. The maximum diameter excludes the safety margin;
the largest required planning envelope is 1.08 m wide.

## Saved assignments and previews

- [Complete design and task table](scenarios/warehouse_uniform/SCENARIOS.md)
- [warehouse: all 25 designs](scenarios/warehouse_uniform/visualizations/warehouse.25_scenarios.png)
- [warehouse2: all 25 designs](scenarios/warehouse_uniform/visualizations/warehouse2.25_scenarios.png)

Task IDs have the form `warehouse_s01_v01`: scenario design 01, endpoint
variant 01. There are 50 designs, 250 task JSONs, 250 control-witness archives,
250 assignment plots, 50 five-variant preview sheets and two map overview sheets.
`manifest.json` lists executable tasks; `scenario_definitions.json` lists designs.
The manifest's `scenario_count` is the executable task count for compatibility
with the existing runner; `scenario_design_count` records the 50 parent designs.

For a fixed design, the size, traffic pattern and selected resource/junction
focus remain fixed across variants. Every agent has five distinct starts and
five distinct goals. Resource patterns retain an opposing/conflicting core;
additional agents can occupy other map regions when local staging space fills.
Per-agent `focused_assignment` records this explicitly. The count of 32 is
never reduced on a failed attempt. It is the suite's fixed stress count,
not a proof of the map's largest possible fleet.
Corridor focus selection first verifies space for 16 nonoverlapping disks on
each approach using deterministic geometric packing. Approaches extend into
connected staging rooms, within 22% of the map diagonal (at least 12 m),
instead of relying on tiny pockets immediately beside a neck.
Each candidate corridor also needs at least eight crossing reference paths
among 32 deterministic geometry-only probes for that diameter, excluding
locally wide resources whose approaches are disconnected elsewhere.

Generation uses only the existing radius-filtered physical geometry graph,
deterministic sampling, geometric packing and bounded-control individual
feasibility checks. It accepts the first valid candidate for a seed, with
at most 64 attempts. No K-route/RRT evaluation result, MPPI rollout, measured
completion rate, or deadlock detection selects a task. Exact original graph
inputs are bundled as compressed JSON for reproduction.
The individual certificate uses the existing visibility-shortcut utility
before its bounded unicycle replay, avoiding a stop at every sampled graph
vertex. This certificate is not a route supplied to either evaluated method.

Validation replays every individual unicycle control witness, checks swept
disk clearance and limits, and checks nonoverlapping starts and goals. An
individual witness alone is not a joint collision-free schedule. The final suite
also has constructive joint schedules for **all 250 tasks**, independently replayed
with bounded controls, swept footprint checks and every stationary robot present.
See `joint_validation_report.json` and `joint_feasibility/`. These schedules
validate the assignments; they never supply anchors or controller fallbacks.
The frozen task JSONs retain their original individual-validation metadata.
Their `jointly_solvable_claim: false` field describes that earlier validation
stage; the later `joint_validation_report.json` certifies the current manifest
jointly and is the authoritative joint-feasibility result.

Six endpoint variants needed replacement before the final experiment:
`warehouse_s12_v01` through `warehouse_s12_v05`, and `warehouse_s14_v04`.
Their originals and deterministic repair history are retained in `revisions/`.
The replacements were selected using geometry and joint certification, without
evaluated planner or MPPI outcomes. The targeted conflict core now has at most
eight agents, with the remaining agents distributed through the map; the total
remains 32. This avoids placing most of a fleet inside a single-file staging aisle.
The final saved tasks and their hashes, including these replacements, define
the reproducible comparison.

## Generate and validate

From the repository root:

```bash
.venv/bin/python benchmarks/multi_agent_bench.py --suite benchmarks/scenarios/warehouse_uniform validate
.venv/bin/python benchmarks/multi_agent_bench.py --suite benchmarks/scenarios/warehouse_uniform visualize
.venv/bin/python benchmarks/multi_agent_bench.py certify-warehouse --workers 12
```

Use the saved task files to reproduce the reported comparison; no regeneration
is needed. Each task retains its exact generation configuration and endpoint seed,
including the six repaired variants. To generate a new candidate suite with the
current defaults without overwriting this frozen revision, use a separate folder:

```bash
.venv/bin/python benchmarks/multi_agent_bench.py --suite benchmarks/results/warehouse_regenerated generate-warehouse --seed 20261026
.venv/bin/python benchmarks/multi_agent_bench.py --suite benchmarks/results/warehouse_regenerated certify-warehouse --workers 12
```

The current generator caps the targeted conflict core at eight agents. Therefore
a fresh run of current defaults is a new candidate suite, rather than a byte copy
of the saved revision. Certification must pass before evaluating that new suite.
The old 50-task benchmark remains at its original path. Other commands retain
its default suite, so select the warehouse suite explicitly when validating it.

## Run comparisons

The paired warehouse evaluator uses the saved assignments with the same
MPPI/ORCA profile and shared safety layer for both methods. K-route selection
is uniform over the returned routes for each agent. A selected-task example:

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 .venv/bin/python benchmarks/multi_agent_bench.py \
  evaluate-warehouse --task warehouse_s01_v01 warehouse2_s01_v01 \
  --workers 4 --output benchmarks/results/warehouse_example
```

Omit `--task` to run all 250 tasks with both methods. The fixed paired protocol
uses planner-selection seed 42 and controller seed 42. `evaluate-warehouse`
contains the termination monitor and shared braking certificate described below;
the general-purpose `run` command remains a separate legacy experiment path.
Actual recorded GIFs are generated when the complete comparison is published.

The runner's physical audit remains enabled. The earlier differential-drive
fallback violated its nominal angular acceleration limit. The paired
evaluation now corrects native fallback commands to respect both linear
braking and angular acceleration. The original unicycle integration remains
in use; measured control violations still disqualify success.

The original one-second smoke artifacts are historical diagnostics. The final
routing fixes remove short local reversals at K-route junctions while preserving
radius clearance and keep-right certificates. RRT-Connect uses triangulation to
bias its sampling while retaining uniform exploration; RRT itself constructs and
checks every output path. Each final run checks all 32 continuous anchor paths.
All frozen assignments must also have current joint-feasibility certificates.
All warehouse RRT queries share a 100,000-iteration ceiling: two repaired
assignments required 17,561 and 32,340 iterations, beyond the earlier 15,000
ceiling. The 1.5 m extension step, 800 smoothing attempts, sampling rule,
footprint clearance and uniform exploration are unchanged. This raises the
search budget; it does not substitute a mesh path or a feasibility witness.

## Paired evaluation at 1.2 m/s

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 .venv/bin/python benchmarks/multi_agent_bench.py evaluate-warehouse --workers 16
.venv/bin/python benchmarks/multi_agent_bench.py report-warehouse --publish
```

The experiment runs 500 planner–task combinations with the same 128-sample,
31-state MPPI profile (3 seconds of controls), dt=0.1 s, ORCA enabled, and a forward limit of 1.2 m/s.
The limit matches the [OMRON LD-250 specification](https://assets.omron.eu/downloads/manual/en/v3/i642_ld-250_user_s_guide_users_manual_en.pdf).
Both planner selection and controller seeds are 42. Each K-route anchor is
sampled uniformly from the available returned set, independently per agent.

Both methods use MPPI/ORCA with the same shared synchronous braking-safety
layer. A joint command is accepted only with a continuously collision-free,
bounded-braking continuation to rest. Rejected proposals use the previously
certified stop, retaining safe motion by other robots when possible. Native
controllers acknowledge the actual applied commands for their next acceleration
bounds. This safety certificate assumes exact synchronous states and the existing
dynamics; it is not a claim that decentralized ORCA alone guarantees safety.

Off-anchor projection requires a clear swept-footprint connection. Guided
proposals include turning in place toward that visible join before moving;
heading cost blends from the join direction back to the path tangent during
reacquisition. This avoids driving into a shelf corner while still turning.

The common native controller waits after 15 seconds without 0.15 m of route
or best-goal improvement, if a stationary robot has persistently blocked its
route reacquisition or forward horizon for three seconds. MPPI/ORCA is still
evaluated; bounded braking replaces repeated avoidance oscillations. The
controller resumes after two seconds with no stationary route blocker. This
optional mode defaults off in the general controller and is enabled equally
for both methods in this paired warehouse experiment. It changes commands,
not poses, anchors, physical limits or the deadlock detector.

Completion requires all 32 agents physically stopped inside their goal regions.
An observed controller deadlock requires 30 seconds stationary/no-progress and
persistent directed obstruction accounting for every unfinished robot: a cycle,
a queue into a cycle, or a queue into a robot parked at its goal. A blocker must
intersect the forward route inside the common 3.6 m planning horizon, including
route reacquisition. Evidence records blocking edges and robot IDs. This is a
controller outcome under fixed anchors and parking policy; joint feasibility is
independently demonstrated by the constructive certificates.
Isolated stalls and livelocks require 45 seconds without route or best-goal
improvement for all unfinished robots, and remain separate outcomes. Actual
collisions terminate a run as a failure. Planning failures and finite observation
limits are explicit; they are never presented as demonstrated mutual deadlock.

The common simulated-time budget is at least 600 s, or six times the longest
saved reference travel time plus 180 s when longer. It is independent of the
chosen method's path. Each simulation also has a two-hour wall limit. Raw
states, commands, selected paths, planning diagnostics and termination evidence
are retained. Identical completed runs can be resumed without rerunning them.

Tables average the five variants for each scenario, the five traffic patterns
for each diameter/map, and the five diameters for each traffic pattern/map.
Fleet success includes global-planning failures in its denominator; a separate
conditional view describes MPPI behavior once all anchors are available.
Makespan means time for the entire fleet to finish. Only successful-run
makespans enter their mean and sample standard deviation, with sample count
shown; failure-only cohorts have no measured makespan.
