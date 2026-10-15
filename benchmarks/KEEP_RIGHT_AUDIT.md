# Mandatory keep-right audit — 2026-10-14

For the subsequent radius experiments, additional near-threshold corrections and
current performance measurements, see [RADIUS_VALIDATION.md](RADIUS_VALIDATION.md).
Measurements below describe the October 14 implementation.

The planner constructs paired directional geometry in C++ and enforces the
selected right-hand path for every certified SPLIT interval. SHARED remains the
central reference where construction cannot certify a usable pair. The ECM,
routing/resource extraction, physical K-search and diversity rules are unchanged.

## Findings and corrections

The previous warehouse2 route 0 already had real lane displacement: about 0.30
for effective radius 0.25, with opposing separation above 0.50. The normal sign
was correct, reversal used the other physical side, and plots read the final
`CompleteRoute.path`/segment geometry. At full-map scale a 0.30 shift was comparable
to the line thickness and visually covered the reference. It was not a nominal
SPLIT on two coincident centerlines.

However, capacity search was capped by a robot-scaled bound and selected a compact
1.2R offset without measuring all available space. It now searches actual radial
clearance up to the configured bound and starts at half the usable displacement
on each side, with footprint-sized separation and asymmetric compensation.
The comfort reserve cannot erase a usable positive offset on a constrained side;
the near-wall asymmetric test verifies that behaviour explicitly.
Straight-wall capacities use the certified clearance ball and a contact probe;
other rays retain bounded binary search. No unsafe ray skips an obstacle.

Increasing displacement exposed a second issue: a robot-sized tangent window
gave sharp contractions at junctions. Two bounded passes now scale the unit-normal
window with intended lane displacement and recertify radial capacities. Curvature
bounds and slope envelopes prevent foldbacks. Actual lane joins are also rounded;
unsafe local rounding is suppressed while retaining the certified SPLIT geometry.
Every change is checked continuously against obstacles and opposing lanes.

The earlier transition distance used the largest offset on the entire route.
Distances now come from the local entry/exit neighbourhood. Endpoint failures
are trimmed/retried locally, preserving distant cores. If preferred offsets need
too much room, one local footprint-sized pair is tried before classifying a short
section SHARED. This recovers the `short_usable_wide` test without shrinking long
wide corridors. A genuinely short wide pocket remains SHARED with a reason.
One complete compact certificate also supports local recovery of fragmented
preferred-frame failures. Paired blends are accepted only after global checks and
must preserve every already-certified SPLIT core; otherwise rejection is reported.

Continuous selected sections span graph vertices. Junction geometry does not
reset offsets independently. Residual links try a checked direct segment, then
checked local curve/node alternatives. Original physical edge IDs, resource IDs,
directions and canonical resource cuts are retained when certificates are sliced
back into complete route segments. Lane geometry is never a new K alternative.

## Certification and diagnostics

SPLIT requires both complete circular sweeps to clear every polygon/boundary by
more than R plus contact tolerance. Continuous segment distance checks cover all
opposing pairs, including nonadjacent parts of bends and separate SPLIT intervals.
Centerline distance must exceed 2R within the numerical reserve. Forward and
reverse signed sides are checked at both endpoints of each reference chord;
linear interpolation preserves that sign throughout the chord. Centerline-as-lane
and wrong-side candidates are rejected. Forward progress and self-intersection
checks apply to the final reconstructed path; transitions are footprint-checked.

Cached `LaneInterval` records contain validation, both side flags, both minimum
clearances and opposing separation. Complete routes report:

- SPLIT/SHARED interval counts and lengths, plus SPLIT fraction;
- length-weighted signed lateral mean/min/max on reference-chord normals;
- selected/opposing SPLIT clearance and global minimum opposing separation;
- rejected candidate spans/reasons, transitions and self-intersections;
- final length, preserved physical-search length and unshifted lane-reference length.

`shared_length` counts only SHARED geometry. `shared_exposure` additionally includes
transitions and residual links whose simultaneous opposing use is uncertified.
Zero-SPLIT nonzero paths emit an explicit diagnostic and their rejection reasons.
Empty query output reports blocked endpoints, conversion failure or exhausted
attachment/search budget. C++ JSON reports all main, K=1 and repeated queries.

Rejection counts group contiguous failed capacity/geometry/core spans and failed
preferred-pair proposals; they do not count individual binary-search steps.
Per-query counts also include rejected physical candidates' lane attempts. Reasons
distinguish capacity, bounded bend refinement, wrong side, footprint clearance,
opposing overlap, self-intersection, transition failure and insufficient usable
core/entry/exit room. A reason identifies what this bounded constructor failed to
certify; it is not an exhaustive impossibility proof for arbitrary free-space curves.

`PlanningTimings` optionally separates lane placement from attachment/search,
physical reconstruction and selection. Lane time includes profile construction or
cache lookup, geometric validation, actual reconstruction, diagnostics and resource
mapping. Loading, JSON, plotting and the extra native verification audit are
outside the query timer. No topology-selection rule was changed for instrumentation.

## Reproduction and validation

```bash
cmake -S . -B build/routes -DBUILD_MPPI_CONTROLLER=OFF -DCMAKE_BUILD_TYPE=Release
cmake --build build/routes -j4
ctest --test-dir build/routes --output-on-failure
.venv/bin/python benchmarks/complete_routes_demo.py --synthetic
```

The native lane suite renders 17 cases: wide, near-threshold and narrow straight
corridors; narrowing/widening; curves; sharp corners; junctions; opposing/reversed
travel; short pockets; blocked lateral geometry; different radii; asymmetric
references (including a reference almost touching a footprint-clearance wall);
obstacle grids; a short usable pair; and a difficult local junction.
It verifies actual separated sides, continuous full sweeps, finite tapers, no loops,
cache/concurrency behaviour, no graph mutation, original resource identity and
physical-only diversity. The wide test requires offsets above 1.1 and opposing
separation above 2.4 for R=0.55; the old compact/centerline geometry cannot pass.
The difficult-junction test checks SPLIT at distant stations on both incident edges.

Release and AddressSanitizer/UndefinedBehaviorSanitizer CTest runs pass. The latter
also compiles with `-Wall -Wextra -Wpedantic` without warnings. Benchmark verification
checks complete geometry, both lanes, continuity, resources, scores, physical
baseline retention and deterministic repetitions in C++.

The constructor remains bounded and conservative at complex bends and ambiguous
joins. It does not solve arbitrary free-space lane optimization; unresolved regions
remain explicitly diagnosed, validated SHARED references or reject the route.
The footprint is circular and both opposing certificates use the query's R.
Mixed robot profiles, nonholonomic dynamics, crossing turn conflicts, reservations
and coordination through SHARED/transitions are outside this implementation.

## Measurements and inspected figures

Measured with the Release build on an Intel i7-12700H under WSL2. The four-map
demo verified 190 native queries: 38 pose/radius configurations, each with K=5,
K=1 and three deterministic repeats. It returned 256 complete routes across those
calls; every returned route passed the continuous native audit and had zero
self-intersections. Ten configurations correctly returned no route because an
endpoint footprint was blocked. All reported nonzero SPLIT pairs had positive
signed offsets, obstacle clearance above R and global opposing separation above
2R. The original warehouse2 long query's five physical edge sequences, physical
paths and previously reported physical metrics are preserved.

For `warehouse2.long_r0.2`, R=0.25. Lengths use map units; SHARED excludes finite
transitions. Interval counts join adjacent fragments across physical graph edges.

| Route | Physical length | Final length | Ratio | SPLIT count / length | SHARED count / length | SPLIT fraction |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| 0 | 160.08 | 157.37 | 0.9831 | 1 / 148.83 | 0 / 0.00 | 94.58% |
| 1 | 165.56 | 169.49 | 1.0237 | 1 / 160.45 | 0 / 0.00 | 94.66% |
| 2 | 196.13 | 198.31 | 1.0111 | 3 / 183.73 | 2 / 8.77 | 92.65% |
| 3 | 216.43 | 222.59 | 1.0285 | 4 / 183.89 | 3 / 15.13 | 82.61% |
| 4 | 175.13 | 177.95 | 1.0161 | 2 / 159.29 | 1 / 3.38 | 89.52% |

| Route | SPLIT offset min / mean / max | Selected / opposing SPLIT clearance | Minimum opposing distance | Transition length / changes | Rejected preferred candidates/spans |
| --- | ---: | ---: | ---: | ---: | ---: |
| 0 | 0.261 / 1.609 / 2.073 | 0.311 / 0.327 | 0.539 | 8.54 / 2 | 0 |
| 1 | 1.419 / 1.714 / 1.999 | 1.726 / 1.060 | 2.863 | 9.04 / 2 | 0 |
| 2 | 0.253 / 1.665 / 2.073 | 0.698 / 0.697 | 0.510 | 5.82 / 6 | 9 |
| 3 | 0.059 / 1.626 / 2.010 | 0.308 / 0.308 | 0.526 | 23.57 / 8 | 7 |
| 4 | 0.250 / 1.617 / 1.999 | 0.308 / 0.308 | 0.526 | 15.28 / 4 | 1 |

The smallest distance between opposing SPLIT sweeps' centerlines is 0.509701,
above the required 0.50. The smallest selected **complete-path** clearance is
0.284793, above R=0.25. Route 3's small minimum offset is an asymmetric constrained
side, not coincident lanes: the paired sweep still passes the global 2R check.
Route 0's average displacement increased from about 0.30 to 1.61. A right lane
can shorten inside turns, explaining its length below the physical reference.
The five main length ratios stay between 0.9831 and 1.0285.

The main query recorded 51 failed preferred candidates/spans across all attempted
physical candidates: 17 preferred-pair/core-transition failures, 11 insufficient
core/transition-room spans, 11 insufficient-capacity spans, four transition
self-intersection spans, four lane self-intersection spans, two wrong-side spans,
and two preferred-frame failures recovered with a compact pair. Two complete
candidate routes were rejected for self-intersection; they were never returned.
Accepted routes 0 and 1 have no rejected SPLIT candidates. Accepted route 2 uses
local compact recovery at two spans while preserving its wide lanes; its remaining
SHARED spans report capacity or usable-core/transition-room limits. Accepted
routes 3 and 4 likewise retain SHARED only at diagnosed local spans. Exact station
ranges and reasons are in the native JSON, including each repeat and K=1 query.

| Map, first demo query | Load ms | First total ms | Search/reconstruction ms | Lane placement ms | Median cached repeat ms |
| --- | ---: | ---: | ---: | ---: | ---: |
| warehouse2 | 268.68 | 1073.96 | 12.34 | 1061.62 | 82.56 |
| warehouse | 253.03 | 674.04 | 7.56 | 666.48 | 40.90 |
| parking1 | 28.11 | 3.41 | 0.53 | 2.88 | 0.62 |
| parking3 | 37.60 | 11.72 | 0.70 | 11.02 | 1.58 |

Cold warehouse2 geometry takes about 1.06 seconds; cached planning stays around
83 ms on this query. Geometry correctness is retained rather than dropping
certification to meet a cold-query time target.

The native synthetic suite passed all 17 cases in both Release and ASan/UBSan.
Representative **complete-route** measurements:

| Synthetic case (R=0.55 unless noted) | SPLIT length | SHARED length | Transition length | Mean SPLIT offset | Opposing distance |
| --- | ---: | ---: | ---: | ---: | ---: |
| Wide straight, W=6 | 21.07 | 0.00 | 5.59 | 1.220 | 2.440 |
| Narrow straight, W=1.8 | 0.00 | 26.00 | 0.00 | — | — |
| Wide → narrow neck → wide | 7.12 | 9.03 | 11.19 | 1.220 | 2.440 |
| Curved corridor | 5.39 | 0.00 | 10.25 | 1.985 | 3.211 |
| Repeated obstacle grid | 23.26 | 0.00 | 3.04 | 0.570 | 1.140 |
| Short usable wide corridor | 3.27 | 0.00 | 3.08 | 0.660 | 1.320 |
| Difficult local junction | 50.09 | 0.00 | 5.87 | 1.068 | 1.440 |
| Near-wall asymmetric reference | 19.98 | 0.00 | 6.02 | 0.0025 | 1.445 |

The near-wall case has a tiny positive selected shift because its reference is
only 0.005 beyond footprint clearance on that side; the opposing lane is over
1.44 away. It is a genuinely separated asymmetric pair. Reversal tests select
the opposite physical side, while the narrow case retains its central path.

Inflation was inspected beyond the requested main query. The synthetic curved
case has final length 15.64 versus physical-search length 11.31, but its retained
quarter-circle reference is 12.57: the physical graph's endpoint chord understates
that curved reference. The outside right-hand lane, with approximately 1.98 offset
on radius 8, explains a further approximately 24.5% against the actual arc. Its
finite entry/exit tapers are continuous, with no loop or spike. Across the four-map
main configurations the largest final/physical ratio is 1.1018, on
`warehouse.testing_warehouse_r0.2`, route 1 (94.93 versus 86.16). The inspected
path follows changing lateral clearance around repeated obstacles with finite
endpoint tapers and no backward excursions; it has 90.19% SPLIT and no SHARED core.

Zero-SPLIT cases are explicit. The 0.2–1.0 length nearby queries lack usable
core/transition room. One additional case remains conservative:
`parking1.testing_parking1_r0.5`, R=0.55, returns a valid central path of length
9.20. Its short sections around sharp reference bends fail core/transition-room,
right-side and local footprint checks, including the compact attempt. The native
diagnostics identify every range. This is an unsupported paired construction in
the bounded algorithm, not a proof that every possible split is infeasible there.
The warehouse2 long query has no unresolved wrong-side or self-intersection
failure spans in its returned routes.

The regenerated figures were inspected: the wide horizontal zoom visibly places
eastbound blue south of the gray center and westbound orange north; the vertical
zoom places southbound blue west and northbound orange east. The narrow zoom has
only the dashed central SHARED path in its neck, with finite transitions outside.
The wide junction connects both right-hand paths continuously. Axes use equal
scaling; arrows are local markers, and small circles show the effective footprint.
The overview, narrowing/widening, difficult junction, inflated warehouse route,
curved case and conservative parking case were also inspected.

- [Four warehouse2 lane zooms](results/lane_routes/warehouse2.long_r0.2.lane_zoom.png)
- [Regenerated warehouse2 overview](results/lane_routes/warehouse2.long_r0.2.overview.png)
- [Forward/reverse decomposition overview](results/lane_routes/warehouse2.long_r0.2.lanes.png)
- [Variable-width lane proof](results/lane_routes/narrowing_widening.lanes.png)
- [Difficult junction](results/lane_routes/difficult_junction_local.route0.png)
- [Curved route](results/lane_routes/curved_corridor.route0.png)
- [Warehouse route with 10.18% inflation](results/lane_routes/warehouse.testing_warehouse_r0.2.route1.png)
- [Explicit conservative parking case](results/lane_routes/parking1.testing_parking1_r0.5.route0.png)
- [All-query measurements and lane reports](results/lane_routes/statistics.json)
- [Warehouse2 native routes/certificates/rejection spans](results/lane_routes/warehouse2.routes.json)
- [Native synthetic certificates](results/lane_routes/synthetic.routes.json)

The primary implementation is in `src/corridor_lanes.cpp`. The header and JSON
writer expose certificates/diagnostics; `src/corridor_planner.cpp` and the demo
tool instrument stages without changing physical search; native tests enforce
the geometry; `benchmarks/complete_routes_demo.py` renders the native final paths.
Generated outputs are in the ignored benchmark-results directory.
