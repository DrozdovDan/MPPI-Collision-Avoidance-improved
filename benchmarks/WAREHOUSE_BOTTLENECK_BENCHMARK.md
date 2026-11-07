# Width-proportional K-route selection

This experiment adds `k_routes_width` alongside the preserved uniform-K and
RRT results. It evaluates the same 250 frozen tasks (125 per warehouse map),
with the same 32 robots, starts, goals and normal MPPI/ORCA controller.

For candidate route i, let w_i be its bottleneck width. One route is drawn once
per robot with `p_i = w_i / sum(w)`. For example widths 1, 2 and 4 m give
probabilities 1/7, 2/7 and 4/7. Width is not squared, inverted, divided by route
length, or reduced by the robot diameter.

The candidate sets are the exact up-to-five keep-right sets from the existing
uniform benchmark, with candidate search budget 24 and the same detour and
diversity filters. Their actual available counts remain explicit. This isolates
the selection policy without regenerating or filtering tasks to favor a method.

## Width measurement

The bottleneck is the minimum full wall-to-wall free cross-section normal to
the **physical reference path**, before lane displacement. Both sides of that
reference contribute. Clearance of a displaced right-side lane would incorrectly
make a wide corridor look narrow merely because the lane runs near one wall.

Polygon-vertex projection events, endpoints and interval interiors cover
boundary changes along each reference chord. Shared chords are cached, including
reverse travel. Each reported neck includes its position, normal and distances
to both walls; independent GEOS line/boundary intersections verify the witness.
Vertex hits use a 1e-9 m snapping tolerance to handle floating-point roundoff.

The weighted inverse-CDF draw is coupled to the existing uniform draw: an
independent jitter inside its equal-probability bin produces a uniform variate
on [0,1). This preserves width-proportional marginal probabilities and reproduces
the original selected index when all candidate widths are equal. It does not
guarantee improved traffic performance on any particular assignment.

## Right side and center fallback

The native planner's lane-aware `route['path']` is passed to MPPI. The unshifted
`physical_path` is used only to measure width.

On certified **SPLIT** corridor sections, the anchor is strictly on the
travel-right side of the reference. Reverse traversal changes physical sides;
its canonical graph mode may read `LeftOneWay` while still being travel-right.
On **SHARED** corridor sections, the anchor equals the reference centerline.
Transitions and start/goal/junction connectors join these sections continuously.

The preferred offset is half the footprint-safe available displacement on that
side, adjusted for opposing-body separation, corners and connections. It is not
always exactly one quarter of the raw corridor width. Splitting requires a
certified pair of opposing clear lanes; certification can conservatively fall
back to center when a different construction might fit. The guarantee concerns
planned anchors. Collision avoidance can move an actual robot off its anchor.

The independent preflight checked all 25,381 candidates: 542,834 split sections
were travel-right and 55,301 shared corridor sections exactly matched their
center references. Native wide/reverse/narrow tests also pass.

## Controller and comparison

This uses **normal MPPI/ORCA**, not the uncertainty-aware implementation.
The existing native binary, 1.2 m/s limit, dt=0.1 s, 128 rollouts, 31 states,
bounded accelerations, shared braking safety filter and waiting policy are
retained. Observation noise is zero; ordinary control-sampling noise remains.
The calibration is 1 map unit = 1 m. Controller profiles are compared exactly
against both historical baselines for every task.

The shared evaluator now accepts an explicit, versioned anchor provider.
Historical harness source is retained under the baseline's `source_snapshots/`;
controller, simulator, routing binary and dynamics hashes must still agree.
New identities include the selection implementation and frozen candidate/width
input hashes. Historical baseline records and trajectories are preserved.

Fleet success means all 32 robots finish. Individual-agent completion counts
each finished robot, including those in a failed fleet. Makespan is simulated
time until the full fleet finishes and is undefined for failures. Every design
average contains five start–goal variants. Paired makespan comparisons use only
the same tasks where both methods finished.

The frozen online terminal monitor uses the nearest geometric anchor projection.
One weighted run exposed a projection behind shelving: its raw label was
`stalled`, even though the robot's closest footprint-clear rejoin was persistently
blocked by the existing traffic cycle. Raw results and motion remain immutable.
The separate auditor can certify an operational deadlock only after replaying
the original label, corroborating its blocking edges, checking the clear rejoin
and its first 3.6 m of travel, and proving that every unfinished stationary robot
feeds a blocking cycle or a parked-goal blocker. Uncertified stalls reject
publication. Comparison data preserve both raw and verified terminal labels;
the correction cannot change success rates, arrival counts or makespans.

```bash
.venv/bin/python benchmarks/multi_agent_bench.py evaluate-warehouse-width --prepare-only
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 .venv/bin/python benchmarks/multi_agent_bench.py evaluate-warehouse-width --workers 20
```

Raw outputs go to `benchmarks/results/warehouse_bottleneck_width/`. Completed
results are published to `scenarios/warehouse_uniform/width_evaluation/` and
`WAREHOUSE_BOTTLENECK_RESULTS.md`, including grouped CSVs, plots and recorded GIFs.
