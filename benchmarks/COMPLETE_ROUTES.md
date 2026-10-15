# Complete lane-aware K routes in C++17

The latest radius sweep and performance profile are in
[RADIUS_VALIDATION.md](RADIUS_VALIDATION.md).

For the current mandatory-lane audit, full statistics and equal-scale proofs, see
[KEEP_RIGHT_AUDIT.md](KEEP_RIGHT_AUDIT.md). Earlier measurements below document the
previous geometry correction.

The planner selects bottleneck-diverse **physical routes**, then applies mandatory
keep-right lane placement. Every certified SPLIT interval uses the directional
lane on the robot's right. There is no cost option to substitute its centerline.
SHARED corridor intervals use the original reference where a complete opposing
lane pair cannot be certified, or a wide pocket lacks traversal/transition room.
No ECM, routing graph, or traffic-resource extraction runs during a query.

## Build and run

Routing needs a C++17 compiler, CMake, and Boost headers for JSON I/O. It is
independent of the original controller's OR-Tools, BLAS, and Python bindings.

```bash
cmake -S . -B build/routes -DBUILD_MPPI_CONTROLLER=OFF -DCMAKE_BUILD_TYPE=Release
cmake --build build/routes -j4
ctest --test-dir build/routes --output-on-failure

.venv/bin/python benchmarks/complete_routes_demo.py --synthetic
```

The visualization script defaults to the previously exported, unchanged bundles
in `benchmarks/results/complete_routes/input`. To use another existing bundle
folder, pass `--traffic-dir`. It invokes the native executable with `--verify`;
all routing, lane geometry, collision/separation checks, baseline comparisons,
and repeated-query checks run in C++. Python prepares demo inputs, reads native
JSON measurements, and draws plots. It performs no geometric planning/validation.

If bundles are unavailable, the existing offline exporter can generate them once,
reusing saved ECM and routing graphs. This is outside query timings:

```bash
.venv/bin/python benchmarks/corridor_traffic_demo.py \
  --world warehouse2 warehouse parking1 parking3 \
  --routing-dir benchmarks/results/routing_corridors \
  --output benchmarks/results/complete_routes/input
```

New results are written to the existing ignored results tree:
`benchmarks/results/lane_routes/`. They include native query inputs/outputs,
`statistics.json`, all-route overview figures, individual route figures, and
opposing-lane decomposition debug figures. `--synthetic` also runs the native
synthetic tests and renders their saved geometry. Offline bundles are unchanged.

## API and route data

```cpp
#include "corridor_planner.hpp"

const auto map = corridor::loadCorridorMap("warehouse2.resources.json");
corridor::RobotProfile robot{0.2, 0.05};
corridor::PlannerConfig config;
config.lanes.min_split_length = 1.5;
config.lanes.transition_length = 1.0;
auto routes = corridor::planKRoutes(map, {7.5, -10}, {116, -70}, robot, 5, config);
```

Link the `corridor_planner` CMake target. Maps can also be constructed from
`MapData`. Copies share immutable map geometry and synchronized runtime caches.

`CompleteRoute.path` is the actual continuous lane-aware reference, starting and
ending at the exact requested positions. `segments` is authoritative for placement:

- `physical_edge_id`, travel direction, and all overlapping traffic resource IDs;
- exact SPLIT, SHARED, and TRANSITION boundaries and geometry;
- explicit start, goal, and junction connector segments;
- original reference coordinates/arclengths and the selected reference geometry;
- opposing geometry and minimum separation for each SPLIT section;
- segment minimum clearance.

`RightOneWay` means the canonical u-to-v right lane. `LeftOneWay` means the
canonical left lane, traversed v-to-u; it is also the robot's travel-relative right.
`CenterBidirectional` represents shared use. TRANSITION is an explicit state/kind;
its taper is collision-free but is not certified for simultaneous opposing use.
`Blocked` exists only in the cached decomposition and is never returned in a route.
The legacy per-edge `modes` field summarizes whether an edge has a directional
section. Use `segments`, not that summary, to read mixed configurations.

`physical_path`, `physical_metrics`, and `physical_resource_traversals` preserve the
route before placement. Physical edge IDs and canonical resource mappings remain
those of the input bundle. Resource cuts are projected by original reference
arclength onto the actual selected geometry, so actual usage entries/exits and
route distances lie on the returned path. Shared junction connectors additionally
use the existing region/chain intersection mappings. A lateral lane is never a
new traffic resource. Off-graph connector pieces have edge ID -1.

## Geometric construction and certification

The circular swept radius is `R = radius + safety_margin`. All selected path
segments require continuous obstacle/wall clearance greater than `R + 1e-8`.
Straight-corridor `W >= 4R` is an idealized threshold, not the curved-corridor test.
Contact with a wall remains a collision, so exact equality can remain SHARED.

1. Stitch adjoining pieces of the chosen physical walk into one continuous
   travel-oriented section, including meaningful start/goal attachment pieces.
   Its certificate is sliced back into the original physical edge IDs and
   canonical resource arclengths. Graph vertices do not reset normals or offsets.
2. Remove duplicate/near-zero chords before tangent calculation. Resampling is
   bounded, retaining reference arclength correspondence and all meaningful bends.
   Offline geometry is untouched. An oversized reference rejects conversion.
3. Compute a unit right normal from a finite longitudinal tangent window scaled
   with the intended lane displacement, with two bounded frame/capacity passes. This
   rounds the offset field across tiny edges and forms bounded beveled bend joins;
   there is no miter denominator or unbounded miter amplification. The legacy
   `miter_limit` setting is retained for API compatibility, not used by these joins.
4. Find separate left/right capacities with bounded binary searches of complete
   radial sweeps, preventing jumps across walls. Prefer half of the actual usable lateral displacement, with footprint separation
   and asymmetric reference compensation. Locally retry a footprint-sized pair
   if a preferred pair lacks usable entry/exit room. `max_offset` is a hard
   radial bound. A wall reserve is reduced for legal near-threshold pairs.
5. Limit lateral slope and validate every actual lane chord. Bounded refinement
   shrinks obstacle-unsafe joins and expands finite neighbourhoods when additional
   opposing separation is needed at a corner, within checked radial capacities.
   Every resulting chord must progress forward against its reference chord.
6. Certify both opposing sweeps across all nearby chord pairs, including
   nonadjacent bends, and reject same-lane crossings. Sorted bounding boxes prune
   distant pairs without approximating continuous segment clearance/separation.
7. Keep maximal meaningful SPLIT runs, with minimum lengths and entry hysteresis.
   Internal mode changes and complete-section endpoints use local finite smoothstep
   tapers; the taper distance accounts for lateral slope. Both tapers are swept
   collision-checked and must progress monotonically. Failed endpoint joins are
   trimmed/retried locally. Final paired join rounding is accepted only after
   continuous footprint, side, progression, crossing and global separation checks. Failed runs remain SHARED
   only where the center is valid. A short wide pocket still cannot split.

At ordinary graph junctions the shared continuous profile supplies the connection;
explicit junction slices preserve adjacent physical IDs. No independent endpoint
lanes are pulled back through each centerline vertex. When an endpoint/local link
is needed, try the shortest collision-checked direct link first. A bounded
quadratic or through-junction fallback is accepted only without self-intersection.
The complete path is checked for footprint clearance, continuity and crossings;
failed conversion rejects the physical candidate within the unchanged search
budget. There is no unchecked lateral jump or loop-removal shortcut after routing.

Direct shortcuts are handled without creating spurious topological alternatives.
When a shortcut and a nearby physical edge bound a free ruled ribbon, C++ checks
both triangles of every ribbon cell against the full swept radius and obstacle
interiors. The shortcut then retains the parent edge/resource identity and uses
that corridor's lane decomposition. A shortcut and its curved parent are not
counted as two K routes merely because their center references differ.

## Caching, search, and metrics

The map retains the original AABB indexes and exact reference-segment clearances.
Robot-independent graph/resource geometry is never modified. Lane decompositions
are cached by continuous selected section/reference or physical corridor, effective
radius, and every lane parameter, including endpoint taper policy. Validated junction/endpoint connections and
shortcut-equivalence checks are cached too. Caches are synchronized and bounded;
accepted routes retain their immutable decomposition certificate after eviction.

The first physical candidate minimizes unpenalized physical length in the attached
network, including valid shortcuts. Additional candidates use the existing
bounded Dijkstra/resource-penalty scheme. Diversity and detour limits use physical
references and resource identities before lane placement. Changing a lane offset,
mode boundary, or direction does not create another K route. Result 0 preserves
the shortest physical baseline whenever its conversion succeeds. Failed
conversions are rejected; the remaining search budget can find another candidate.
This is a bounded heuristic, not exhaustive path enumeration.

`metrics.length` and `min_clearance` describe the actual returned geometry.
`split_length + shared_exposure == length` within numerical tolerance.
`shared_exposure` includes transitions and local connectors, because their opposing
sweeps are not certified. `transition_length` and `connector_length` report those
subsets separately. Junction slices can be certified SPLIT; the segment state is
authoritative. Diagnostics also report `physical_length`, `length_inflation_ratio`,
`self_intersection_count`, `max_lateral_deviation` from the corresponding corridor
reference, and `mode_transition_count` (continuous taper intervals). `narrow_exposure` is the union length of actual traversals
mapped to narrow resources, plus conservative narrow virtual-connector exposure.
The bottleneck term uses actual mapped lengths and stored geometric widths,
with the same footprint floor as the baseline. Score remains
`length + lambda_narrow*narrow_exposure + lambda_bottleneck*bottleneck_penalty`.
No traffic densities or congestion probabilities are invented.

## Configuration and native JSON tool

```bash
build/routes/corridor_plan \
  --map benchmarks/results/complete_routes/input/warehouse2.resources.json \
  --queries queries.json --output routes.json --k 5 --verify --debug-lanes
```

Query input remains:

```json
{"queries": [{"name": "long", "start": [7.5, -10], "goal": [116, -70],
              "radius": 0.2, "safety_margin": 0.05, "k": 5}]}
```

The existing candidate, detour, diversity, attachment, and score controls remain.
Lane controls are `--lane-sample-spacing`, `--lane-max-offset`,
`--lane-max-samples`, `--lane-min-split-length`, `--lane-transition-length`,
`--lane-hysteresis`, `--lane-clearance-guard`, `--lane-separation-margin`, and
`--lane-offset-slope`. `LaneConfig` also bounds binary-search iterations
and join refinement. The separation margin is a preferred reserve;
it does not reject a fully certified pair near the physical two-lane threshold.

`--verify` runs additional native verification **after** the planning timer. It
checks full swept paths, segment continuity, selected/cached mode agreement,
right-hand placement, opposing separation, actual resource event positions,
physical detour/diversity, score, K=1 baseline retention, and repetition consistency.
`--debug-lanes` exports both directional lane geometries on selected continuous
corridor sections. Loading time includes JSON and the robot-independent collision cache;
profile-specific lane construction belongs to the first query using that profile.

## Tests and limitations

The native lane tests cover 17 rendered cases: wide and near-threshold straight roads,
narrow roads, narrowing/widening, curved roads, a sharp corner, a T-junction,
explicit opposing traversal, a short wide pocket, valid-center/blocked-offset
geometry, two robot radii on the same map, an asymmetric center reference, a repeated obstacle grid, a short usable split, and a difficult local junction.
They verify both complete opposing SPLIT sweeps, reversal, mandatory placement,
mode continuity, actual endpoints, resource identity, physical-only diversity,
cache reuse, and graph immutability. A forged SHARED section on a certified SPLIT
interval is rejected. Additional regressions construct a 13-edge grid walk directly, require one shared
section certificate, and check duplicate/near-zero cleanup, bounded inflation,
near-coincident junction endpoints, crossing/reversal detection and forged loops.
Existing native baseline tests remain in the test suite.

This is circular-footprint geometric feasibility, not car-like feasibility.
There are no turning-radius, heading, acceleration, reservation, or scheduling
constraints. The opposing-lane certificate uses the same query footprint for both
streams; mixed robot profiles are not jointly coordinated. Junction connectors,
mode transitions, and shared regions still require future coordination.

Offset search and refinement are bounded and conservative. Uncertified local
geometry remains SHARED if its center path is safe; otherwise conversion fails.
Attachment/search budgets can omit alternatives. Lane placement can increase
length because it obeys keep-right and keeps opposing sweeps separated;
the detour ratio constrains the **physical route**, not the final lane/connector
length. Resources retain canonical longitudinal incidence rather than replacing
schema-v2 regions with new lane-specific reservation areas. C++ validates the
bundle structurally but does not verify its existing SHA-256 identities.

## Files changed for the lane extension

- `include/corridor_planner.hpp`: lane configuration, decomposition and route-segment APIs.
- `src/corridor_lanes.cpp`: adaptive geometry, separation proof, connections and placement.
- `src/corridor_planner.cpp`: profile caches, shortcut identity, physical-only selection,
  reconstruction and actual resource remapping.
- `src/corridor_map_io.cpp`, `tools/corridor_plan.cpp`: richer native JSON and verification.
- `tests/corridor_lanes_test.cpp`, `tests/corridor_planner_test.cpp`: geometric and baseline tests.
- `benchmarks/complete_routes_demo.py`: native benchmark invocation and lane figures.
- `CMakeLists.txt`, README files and this report: build/test integration and usage.

## Geometry correction

The prior reconstruction was collision-free against obstacles but geometrically
broken. Native diagnosis of the saved warehouse2 route 0 found **25 centerline
self-intersections**. Its 20 nonzero junction links consumed 83.12 length units;
the worst link was 32.11 times its endpoint distance. Each edge independently
selected offsets up to four units, and changed endpoint tangent/normal at small
graph pieces. A quadratic via the original center was preferred even for almost
coincident endpoints. Bounded miter normals amplified local bend displacement.
There was no monotonic-progress or self-intersection rejection. These errors
explain the repeated spikes/loops and 254.23 reconstructed length for a 160.08
physical route. The direction convention itself was correct; discontinuous
edge-wise frames and overlarge offsets were the problem.

The correction uses one continuous selected-section certificate, compact bounded
unit-normal offsets, finite tapers, checked direct-first residual links, forward
progress checks and crossing rejection. When a corner pair is too close, bounded
refinement expands its neighbourhood instead of shrinking separation. Short
physical shortcuts with no room for a split retain their validated SHARED
reference at the actual poses, preventing projection-induced out-and-back detours.
Physical edge/resource identity and canonical resource cuts are retained. Plotting
uses local direction markers: former long arrows also drew misleading straight
chords across bends, independently of the actual C++ geometry.

The K-route search and offline graph are unchanged. The shortest physical
warehouse2 baseline has exactly the same coordinates and edge sequence as the
saved pre-fix output. Some other accepted alternatives change because their acceptance now depends on
the corrected complete-geometry certificate; the existing candidate-search budget and selection rules are unchanged.

## Measured results for the prior geometry correction

Measured with the Release build on an Intel Core i7-12700H under WSL2/Linux.
Times are elapsed wall-clock measurements from one run. Planning includes physical
search, lane construction/cache lookup, connection validation, reconstruction,
resource mapping and final footprint/crossing checks; it excludes loading, JSON,
plotting and the additional native `--verify` audit. Repeated values are medians
of three queries on the same loaded map.

| Map / first query, radius 0.2 + margin 0.05 | Routes | Loading (ms) | First query (ms) | Repeated median (ms) |
| --- | ---: | ---: | ---: | ---: |
| warehouse2 / long_r0.2 | 5 | 287.06 | 392.01 | 75.27 |
| warehouse / testing_warehouse_r0.2 | 5 | 320.96 | 191.79 | 39.10 |
| parking1 / testing_parking1_r0.2 | 1 | 36.00 | 1.34 | 0.68 |
| parking3 / testing_parking3_r0.2 | 1 | 46.86 | 3.38 | 1.42 |

The warehouse2 long query is `(7.5, -10)` to `(116, -70)`, K=5. Its physical
baseline remains 160.08, and its actual length falls from **254.23 to 159.43**.
All five resulting routes have zero intersections and length ratios within about
0.6% of unity. A right lane can shorten an inside bend, so a ratio slightly below
one is expected. No topology-search setting or offline edge was changed.

| Route | Physical length | Actual length | Ratio | Crossings | Max deviation | Tapers | Shared exposure | Narrow exposure |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 0 | 160.08 | 159.43 | 0.9959 | 0 | 0.300 | 2 | 2.28 | 10.22 |
| 1 | 165.56 | 166.11 | 1.0033 | 0 | 0.305 | 2 | 2.06 | 0.00 |
| 2 | 196.13 | 196.30 | 1.0009 | 0 | 0.305 | 6 | 15.24 | 10.94 |
| 3 | 216.43 | 217.73 | 1.0060 | 0 | 0.305 | 8 | 23.85 | 30.71 |
| 4 | 175.13 | 175.24 | 1.0006 | 0 | 0.300 | 4 | 7.61 | 14.72 |

Lengths/exposure are in map units. Max deviation measures displacement from the
corresponding reference station; taper count counts continuous transition
intervals, including entry/exit tapers. Shared exposure includes uncertified
opposing use at transitions/links. Exact minimum clearance, bottleneck penalty,
score, resource overlap, certificates and detailed metrics are in native JSON.

The repeated 0.20-unit nearby queries on warehouse/parking maps originally made
projection-induced detours with ratios up to 9.18. The short-SHARED-reference
correction returns the valid physical shortcut, and no current map-benchmark
route has a ratio above 1.20. Short wide pockets still remain shared because they
cannot accommodate a meaningful core and finite entry/exit tapers.

One synthetic curved example has ratio 1.223 and was investigated: the physical
shortcut is an 11.31-unit chord, while its parent corridor is a 12.57-unit circular
arc. Mandatory right placement follows the certified outside of that arc, with
finite endpoint tapers, yielding 13.84 units. The extra length has an explicit
geometric cause; it has zero crossings, bounded 0.66 displacement and continuous
forward progress. The sharp-corner/T-junction examples have ratios 1.090 and
bounded 0.76 displacement. Straight/narrowing examples have ratios 1.000–1.027.

Both CTest executables pass in Release and Debug with AddressSanitizer and
UndefinedBehaviorSanitizer. The warning-enabled build (`-Wall -Wextra -Wpedantic`)
has no warnings. Fourteen synthetic cases are rendered, with additional native
regressions for fragmented grid walks, duplicate/near-zero tangents, short shared
shortcuts, near-coincident junction endpoints, crossing/reversal rejection,
configuration certificates and concurrency.

The full benchmark runs 38 pose/profile combinations, each with K=5, K=1 and
three repetitions: 190 native queries. Every returned path passes native
footprint, continuity, mandatory-placement, separation, crossing, resource and
baseline/repetition checks. Ten combinations return no route because endpoints
or bounded conversion/search do not yield one; empty output is not a proof of
free-space unreachability. All overview/route figures are regenerated; the
warehouse2 overview/route 0, opposing lanes, sharp corner and narrowing transitions
were visually inspected after correction.

- [Measurements](results/lane_routes/statistics.json)
- [Warehouse2 native routes and diagnostics](results/lane_routes/warehouse2.routes.json)
- [Warehouse2 overview](results/lane_routes/warehouse2.long_r0.2.overview.png)
- [Warehouse2 route 0](results/lane_routes/warehouse2.long_r0.2.route0.png)
- [Warehouse2 opposing lanes](results/lane_routes/warehouse2.long_r0.2.lanes.png)
- [Narrowing/widening lane debug](results/lane_routes/narrowing_widening.lanes.png)
- [Sharp-corner complete route](results/lane_routes/sharp_corner.route0.png)
- [T-junction complete route](results/lane_routes/t_junction.route0.png)
- [Saved pre-correction overview](results/lane_routes/before_geometry_fix.warehouse2.long_r0.2.overview.png)

Generated artifacts are in the ignored results tree and available in this
workspace. Reproduce them with the command above.
