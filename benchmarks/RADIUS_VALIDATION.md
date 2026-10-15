# Lane correctness, radius sensitivity and performance — 2026-10-15

Mandatory keep-right is applied to genuinely separated, continuously validated
opposing lanes. The new experiments also identify cases this bounded constructor
or the unchanged physical-search budget does not support. A SHARED certificate is
not an exhaustive impossibility proof for every arbitrary free-space curve.

No ECM, routing extraction, traffic-resource extraction, physical K-search or
bottleneck-diversity rule was rewritten. Both benchmark maps reuse their existing
schema-v2 bundles. Bundle SHA256 values are recorded in the experiment JSON.
Planning, geometry and certification are C++17; Python prepares queries and
analyses/renders native results.

## Corrections and measured optimization

1. **Truncated direct-reference geometry.** A certified physical shortcut could
   extend well beyond its two parent-edge projection stations. Placement replaced
   that complete physical reference with the short parent slice, then left the
   remainder as a long SHARED connector. It now retains the actual shortcut in
   this case, preserving physical IDs and canonical resource coordinates. The
   new independent regression requires a meaningful SPLIT core across the full
   extended reference and rejects the former false large-deviation diagnostic.
2. **Near-threshold pocket and bend failures.** Increasing offsets into wider
   pockets and shrinking a failed join could consume the tiny separation reserve
   in a corridor just above 4R. A bounded compact retry now tries consistent
   paired offsets over capacity-feasible runs. For nearly straight references it
   also compensates small reference wiggles using a fixed frame. Eligibility for
   that frame is geometric (deviation at most one quarter of sample spacing),
   independent of robot radius. Original stations still enforce side and progress;
   all chords are checked against polygons and the opposing sweep. Recovery can
   replace unnecessary internal transition regions as well as SHARED gaps, while
   preserving every already-certified SPLIT core. Preferred wide lanes are not
   globally shrunk, disabled or replaced with centerlines.
3. **Diagnostics.** Each interval now records both path lengths, signed offset
   extrema, both clearances, opposing distance, and entry/exit transition presence
   and validity. Maximum lane deviation uses corresponding certificate stations;
   residual connectors no longer compare mismatched point-array indices.
4. **Actual latency source.** An exclusive native profile measured about 89% of
   the old cold-query time in collision checks: 1,251,394 clearance calls. Every
   call scanned all obstacle boxes before consulting the existing boundary tree.
   Polygon containment now uses an immutable AABB tree too. The nearest-query
   callback is templated, avoiding type-erasure overhead. A 900-case independent
   exhaustive polygon oracle verifies identical clearance results, including
   inside endpoints, crossings and outer boundaries. The isolated indexed change
   reduced a profiled 1292.28 ms query to 552.61 ms with identical route paths and
   IDs; subsequent geometric corrections add checked recovery work.

The primary policy still begins around half the usable displacement, bounded by
actual clearance, separation and curvature, rather than selecting the maximum
radial offset. Consistent footprint-sized candidates are used locally where a
large preferred profile cannot form usable cores/connections. Safety margin is
included in R on every sweep, not applied only to center samples.

## Scope of the certificates

SPLIT means both directional circular sweeps are collision-free, on their travel
right sides, and mutually disjoint. Continuous segment-to-segment tests include
nonadjacent pieces and distinct SPLIT runs in the same certificate. Symmetric
centers must be more than 2R apart within the contact reserve; asymmetric centers
may be different distances from an off-center reference, but still require both
positive signed sides and the full 2R separation.

The absolute contact tolerance is 1e-8, with additional separation reserve in
construction. At exact 4R in a straight corridor, there is no extra wall clearance
and SPLIT is rejected. At exact 2R, even the central footprint touches the walls
and traversal is rejected under the strict clearance rule.

Transitions and SHARED regions can have overlapping opposing footprints and
require coordination. Independent junction turns are certified **as individual
opposing pairs**; crossing different turn pairs is not certified conflict-free.
The figures explicitly mark those scopes. Complete queries use actual finite
entry/exit tapers. Independent physical-section sweeps have open endpoints:
`*_transition_present=false` / `*_transition_valid=false` there means no such
transition was constructed, not a claim that an external junction link is safe.
Internal transition intervals retain full footprint validation.

`shared_length` excludes transitions; `shared_exposure` includes them and residual
links. Rejection counts refer to failed preferred candidates/spans, including
later successful recoveries, and include rejected physical candidates at query
level. Candidate path-validation failures, rejected SPLIT footprint spans,
rejected complete routes, and returned-route audit failures are separate metrics.
Raw clearance probes into blocked space are capacity-search work, not returned
collisions. Zero-SPLIT nonzero routes emit diagnostics and rejection reasons.

`feasible_physical_edges` counts complete original reference edges under exactly
the query filter's continuous clearance/stored-width rules; partial attachments
can still use a safe part of an edge. Physical reachability means a walk was found
inside the existing attachment/search budget. Complete-route reachability is
reported separately after reconstruction. Neither failure asserts global
free-space impossibility.

## Radius experiments

The sweep uses radius 0.10, 0.20, 0.35, 0.50, 0.75, 1.00, 1.45 and 2.95 with fixed
margin 0.05. Warehouse2 has 248 physical edges and five fixed pose pairs (long,
central, major-obstacle, dense and narrow passage). Warehouse has 273 edges and
three pairs. Every configuration runs K=5, K=1 and two cached repeats: **64
configurations / 256 native calls / 441 returned routes**. All returned routes
pass native endpoint, continuity, footprint, side, opposing-sweep, self-intersection,
resource-order, score, physical-baseline and deterministic-repeat verification.

For the warehouse2 long pair, lengths and SPLIT fraction aggregate its selected
complete routes; runtime is that query's first radius-specific lane-cache miss,
with loading/JSON/plotting/audit excluded. Warm runtime is the median of two
repeats. Reachable-pair counts cover all five warehouse2 scenarios.

| Radius | R | Feasible edges | Reachable pairs | Long K | Shortest / mean length | SPLIT fraction | First / warm ms |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |
| 0.10 | 0.15 | 228 | 5/5 | 5 | 157.37 / 186.09 | 92.84% | 680.75 / 78.66 |
| 0.20 | 0.25 | 172 | 5/5 | 5 | 157.37 / 184.33 | 92.29% | 725.26 / 59.60 |
| 0.35 | 0.40 | 134 | 4/5 | 5 | 157.82 / 183.05 | 90.44% | 416.29 / 40.44 |
| 0.50 | 0.55 | 122 | 3/5 | 5 | 157.25 / 182.47 | 89.65% | 464.38 / 45.44 |
| 0.75 | 0.80 | 90 | 3/5 | 5 | 159.68 / 180.28 | 95.38% | 530.61 / 46.52 |
| 1.00 | 1.05 | 90 | 3/5 | 5 | 159.11 / 179.89 | 95.56% | 709.91 / 48.05 |
| 1.45 | 1.50 | 90 | 3/5 | 5 | 158.70 / 180.30 | 80.38% | 910.08 / 50.62 |
| 2.95 | 3.00 | 83 | 3/5 | 4 | 160.41 / 177.11 | 0.00% | 1439.86 / 61.18 |

| Radius | Long total SPLIT | Long total SHARED | Mode changes | Rejected SPLIT candidates/spans | Returned collision failures |
| --- | ---: | ---: | ---: | ---: | ---: |
| 0.10 | 863.85 | 30.14 | 26 | 60 | 0 |
| 0.20 | 850.57 | 26.42 | 20 | 61 | 0 |
| 0.35 | 827.71 | 57.89 | 16 | 60 | 0 |
| 0.50 | 817.92 | 60.73 | 16 | 38 | 0 |
| 0.75 | 859.75 | 0.00 | 10 | 22 | 0 |
| 1.00 | 859.53 | 0.00 | 10 | 22 | 0 |
| 1.45 | 724.67 | 130.19 | 12 | 43 | 0 |
| 2.95 | 0.00 | 708.43 | 0 | 368 | 0 |

The shrinking feasible edge sets were checked as sets, not only counts. Seven
fixed physical sections were independently swept at 13 radii (91 certificates).
No larger-radius certificate gained any SPLIT station interval absent at the
previous smaller radius. The constructor classifications are observed results,
not post-processed to force monotonicity. Known-width synthetic feasibility was
also checked independently at and around each analytic threshold.

Selected-route fractions are not monotonic: at radius 0.75 the long query drops
narrower physical alternatives and uses wide detours. Its baseline switches from
edges beginning `[11,23,46,61,59,58,...]` at radius 0.20 to
`[10,18,37,45,64,69,...]` at 0.75. That change comes from radius-specific physical
filtering and existing diversity/geometry selection, not lane variants counted
as new routes. The original five radius-0.20 physical sequences are preserved.

Additional map, same `testing_warehouse` pair (all three physical pairs remain
reachable in the bounded search at each size):

| Radius | Feasible edges | Complete query pairs | Testing-pair K | Testing first / warm ms |
| --- | ---: | ---: | ---: | ---: |
| 0.10 | 215 | 3/3 | 5 | 426.30 / 36.53 |
| 0.20 | 157 | 3/3 | 5 | 445.58 / 31.63 |
| 0.35 | 121 | 3/3 | 5 | 204.44 / 19.85 |
| 0.50 | 106 | 3/3 | 2 | 164.83 / 10.23 |
| 0.75 | 69 | 3/3 | 1 | 187.97 / 18.44 |
| 1.00 | 66 | 3/3 | 1 | 291.32 / 12.05 |
| 1.45 | 64 | 3/3 | 2 | 165.69 / 10.19 |
| 2.95 | 56 | 1/3 | 0 | 285.29 / 72.61 |

All per-query and per-radius totals—including shortest/mean length, SPLIT, SHARED,
transitions, rejected candidates, collision-related failures, reachability and
all exclusive timing stages—are in the CSV/JSON below. Unavailable route lengths
and fractions are null/empty, never fabricated zeros. Quantitative plots mark no
returned route with × and leave length/fraction gaps.

## Representative feasibility switches

These are **tested brackets**, not claimed exact polygonal thresholds:

| Fixed geometry | SPLIT demonstrated | SHARED demonstrated | Impassable demonstrated |
| --- | --- | --- | --- |
| Warehouse2 wide horizontal, edge 11 stations 10–25 | radius 1.40 (R=1.45), 15.00 length | radius 1.425 (R=1.475), 15.00 length | radius 2.90 (R=2.95) |
| Warehouse2 `narrow_bent_reference`, edges 140 reverse → 141 forward | radius 0.10 (R=0.15), 4.70 length | radius 0.20 (R=0.25), 4.70 length | radius 0.35 (R=0.40) |
| Synthetic straight W=1.2 | radius 0.249999 (R=0.299999) | radius 0.25 (R=0.30) | radius 0.55 (R=0.60) |
| Synthetic straight W=6 | radius 1.449999 (R=1.499999) | radius 1.45 (R=1.50) | radius 2.95 (R=3.00) |

The complete six-unit warehouse narrow query has SPLIT length 4.00 and finite
transition length 2.03 at radius 0.10, with both sweeps certified. At radius 0.20
it is a central SHARED path of length 6.00; at 0.35 its endpoint footprints are
blocked. The variable-width warehouse section contains SPLIT and central SHARED
runs at 0.10/0.20 and loses through-traversability at 0.35 because of its neck.
Larger profiles can still have valid local SPLIT pieces on either side of a
blocked neck; those pieces do not imply a complete through path exists.

## Requested long query, radius 0.20

R=0.25. SHARED excludes transitions; every returned path has zero self-intersections.

| Route | Physical / final length | SPLIT length / fraction | SHARED length | Transition length / changes | Min opposing distance |
| --- | ---: | ---: | ---: | ---: | ---: |
| 0 | 160.08 / 157.37 | 148.83 / 94.58% | 0.00 | 8.54 / 2 | 0.539433 |
| 1 | 165.56 / 169.49 | 160.45 / 94.66% | 0.00 | 9.04 / 2 | 2.863463 |
| 2 | 196.13 / 196.45 | 184.38 / 93.85% | 7.91 | 4.16 / 4 | 0.502858 |
| 3 | 216.43 / 220.42 | 197.62 / 89.66% | 15.13 | 7.66 / 8 | 0.507632 |
| 4 | 175.13 / 177.95 | 159.29 / 89.52% | 3.38 | 15.28 / 4 | 0.526171 |

The smallest opposing distance is 0.502858 > 0.50; the smallest selected complete
clearance is 0.269497 > R=0.25. SPLIT is actual displaced geometry, not a label:
route 0's mean signed offset remains 1.609 and its wide horizontal pair visibly
straddles the original gray center. The independent horizontal interval at this
radius has offsets 1.328–1.339, opposing separation 2.656, and both clearances
1.588. The asymmetric small-offset extrema at difficult junctions are allowed
only because the complete paired sweep still exceeds 2R.

The five final/physical ratios are 0.9831–1.0237. Route 2's recovered geometry
shortens the old 198.31 path to 196.45; route 3 shortens 222.59 to 220.42 and
increases its SPLIT fraction from 82.61% to 89.66%. No corresponding graph or
resource identity changes were made.

Across the radius experiments the largest final/physical ratio is 1.1036, on
warehouse `testing_warehouse_r0.35`, route 1 (95.09 versus 86.16). Its longer
outside/right path follows changing clearance through repeated obstacles; the
inspected corresponding 0.20 route has the same shape and no loop/backtracking.
The synthetic quarter-circle case remains 15.64 versus physical-search chord
11.31, but its actual arc reference is 12.57. Approximately 1.98 outward lane
displacement on radius 8 explains the additional arc length. The continuous
certificate and inspected figure show finite tapers and no offset spike.

## Exclusive performance profile

Native Release build, Intel i7-12700H under WSL2. Loading and reusable static
geometry caches are outside the query timer. The extra `--verify` native audit,
JSON, independent corridor experiments and plots are also outside it.
`lane_placement_seconds` remains an inclusive convenience metric. The new stage
buckets below are exclusive: nested clearance/pair/self checks are charged to
collision validation rather than charged again to lane construction.
Inline construction/refinement predicates remain part of their owning stage.

Final standard-demo `warehouse2.long_r0.2`:

| Stage | ms |
| --- | ---: |
| attachment | 0.820 |
| graph_filtering | 0.211 |
| topology_search | 2.607 |
| lane_decomposition | 156.373 |
| transitions | 16.508 |
| collision_validation | 590.550 |
| reconstruction | 8.674 |
| other | 0.001 |
| Total | 775.747 |
| Cached repeat median | 64.567 |

The radius sweep independently measured 725.26 ms cold for that radius and
59.60 ms cached; the standard demo measured 775.75 / 64.57 ms. Both include the
full detailed profiler and stronger geometry recovery, versus the preceding
1073.96 ms benchmark. This is a sub-second result on the requested query, not a
claim that every radius/scenario is sub-second.

Collision work remains the main cost (about 76% of the standard cold query).
There are 1,359,843 clearance calls, seven top-level section builds and two local
connection builds on the first call. Repeats reuse all seven section certificates
and all connections. Caches are keyed by actual continuous reference geometry,
effective radius and complete lane configuration; copies share immutable spatial
indices, with synchronized certificate caches and thread-local profiling. Internal
compact attempts are included in construction time, not counted as additional
top-level cache misses.

Dense warehouse2 radius 0.10 and long radius 2.95 still take approximately 1.62 s
and 1.44 s cold respectively. Different feasible graphs, more candidate/recovery
work and difficult large footprints explain why a universal timing target is not
claimed. Exact collision checks and the existing search budget are retained.

## Tests, inspected figures and remaining limits

All three CTest targets pass in Release and under AddressSanitizer plus
UndefinedBehaviorSanitizer. The sanitizer build has `-Wall -Wextra -Wpedantic`
and no compiler warnings. Coverage includes 20 rendered lane cases; 60 native
radius/known-width cases with strict 2R/4R thresholds; 900 exhaustive-clearance
oracle cases; full opposing polyline checks; reversed directions; necks, curves,
T-junctions, sharp corners and obstacle grids; config/radius cache isolation;
concurrent queries; resource identity and physical-only diversity. The new pocket,
small-reference-bend and truncated-shortcut regressions require actual SPLIT
coverage, not just collision freedom.

The original four-map demo was rerun too: 190 calls and 256 returned routes all
passed native verification. Combined with the new sweep that is **446 calls /
697 returned complete paths**, with zero returned self-intersections or
returned-route collision failures. Invalid/blocked queries are included as tested
outcomes, not counted as successful plans.

Inspected: both map route-radius comparisons; independent horizontal, vertical,
narrow, variable-width and multi-incident junction views; wide/narrow/variable
radius comparisons; synthetic width/neck comparisons; quantitative panels; the
regenerated warehouse2 long overview and lane zooms; and the curved synthetic
case. Layouts were corrected after inspection so titles/axes remain inside the
exported figures. Equal axis scaling is used throughout geometric plots.

Limits remain explicit:

- The independent short `narrow_passage` entry fragment (length 2.87) retains
  SHARED at radius 0.10 with local core/transition/bend rejection reasons. The
  longer `narrow_bent_reference` section and full six-unit query do split at that
  radius. This short-entry construction is conservative; no global impossibility
  proof is asserted. More complex bent/variable-width references also retain
  individual failed spans with their exact reasons.
- `parking1.testing_parking1_r0.5` in the regression suite still has a valid 9.20
  SHARED path: short sections and sharp reference bends defeat the bounded paired
  constructor. This previously documented case remains unsupported as SPLIT.
- At radius 2.95 (R=3.0), warehouse `testing_warehouse` and `dense` find physical
  walks but return no complete routes after 24 rejected self-intersecting
  reconstructions. A separate native check confirmed that intersections already
  exist in the physical reference: the first testing walk has 25 before and 25
  after placement, with no SPLIT or transition geometry. The graph/search were
  not changed to hide this. This is a geometry/search-budget limitation, not a
  claim of physical disconnection or of a lane-offset loop.
- Certificates cover identical circular profiles in the two directions.
  Coordination through shared/transition/crossing regions, mixed robot sizes,
  nonholonomic turn constraints and arbitrary global free-space lane optimization
  remain outside this implementation.

## Reproduction and artifacts

```bash
cmake -S . -B build/routes -DBUILD_MPPI_CONTROLLER=OFF -DCMAKE_BUILD_TYPE=Release
cmake --build build/routes -j4
ctest --test-dir build/routes --output-on-failure
.venv/bin/python benchmarks/radius_sweep.py
.venv/bin/python benchmarks/complete_routes_demo.py --synthetic
# Replot/reanalyse saved native data without rerunning planning:
.venv/bin/python benchmarks/radius_sweep.py --analyse-only
```

`corridor_plan` also accepts optional top-level `corridors` in its query document,
independent of `queries`. Each record has `name`, `traversals` (edge ID, direction,
optional first/last native arclength), `radii`, `safety_margin`, and
`taper_endpoints`. It exports both directional geometries, exact traversals,
interval metrics/rejections and complete-section validity in `corridor_sweeps`.
These diagnostic decompositions do not participate in topology selection.

- [All experiments and definitions (JSON)](results/radius_sweep/statistics.json)
- [Per-query results and all stage timings (CSV)](results/radius_sweep/query_metrics.csv)
- [Per-radius aggregate results (CSV)](results/radius_sweep/radius_summary.csv)
- [Independent corridor metrics and certificates (CSV)](results/radius_sweep/corridor_metrics.csv)
- [Warehouse2 native queries/routes/certificates](results/radius_sweep/warehouse2.routes.json)
- [Additional warehouse native results](results/radius_sweep/warehouse.routes.json)
- [Native 60-case threshold results](results/radius_sweep/synthetic_radius.routes.json)
- [Five independent lane close-ups](results/radius_sweep/warehouse2.independent_lane_debug.png)
- [Wide corridor radius comparison](results/radius_sweep/warehouse2.wide_horizontal.radius_comparison.png)
- [Narrow corridor radius comparison](results/radius_sweep/warehouse2.narrow_bent_reference.radius_comparison.png)
- [Variable-width radius comparison](results/radius_sweep/warehouse2.variable_width.radius_comparison.png)
- [Synthetic wide comparison](results/radius_sweep/synthetic.wide_W6.radius_comparison.png)
- [Synthetic neck comparison](results/radius_sweep/synthetic.variable_W6_neck1.2.radius_comparison.png)
- [Warehouse2 routes by radius](results/radius_sweep/warehouse2.route_radius_comparison.png)
- [Additional map routes by radius](results/radius_sweep/warehouse.route_radius_comparison.png)
- [Quantitative plots](results/radius_sweep/radius_metrics.png)
- [Regenerated long-query overview](results/lane_routes/warehouse2.long_r0.2.overview.png)
- [Regenerated long-query lane zooms](results/lane_routes/warehouse2.long_r0.2.lane_zoom.png)
- [Native large-radius rejection investigation](results/radius_sweep/large_radius_rejections.log)

Implementation: `src/corridor_lanes.cpp`, `src/corridor_planner.cpp` and the private
`src/corridor_profile.hpp`; APIs in `include/corridor_planner.hpp`; JSON/export
in `src/corridor_map_io.cpp` and `tools/corridor_plan.cpp`; focused native tests in
`tests/corridor_lanes_test.cpp` and `tests/corridor_radius_test.cpp`; experiment
orchestration/analysis in `benchmarks/radius_sweep.py`. Generated results are in
the ignored benchmark-results directory.
