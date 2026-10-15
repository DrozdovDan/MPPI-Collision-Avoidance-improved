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

## Physical corridor graphs

`corridor_graph/` builds a reusable undirected embedded multigraph directly from
the existing `world['obstacles']` and `world['bounds']` dictionaries. It uses
[Pyvoronoi](https://pyvoronoi.readthedocs.io/en/latest/), a wrapper around
[Boost.Polygon's segment Voronoi implementation](https://www.boost.org/doc/libs/latest/libs/polygon/doc/voronoi_main.htm).
Whole exposed polygon sides, their endpoint sites, and the rectangular outer
boundary define the generalized Voronoi diagram. This is a segment-site medial
axis, with straight and parabolic parts. Shapely unions touching/overlapping
obstacles and clips them to the workspace before boundary construction.
Coordinates remain in the adapter's Cartesian frame (SVG Y has already been
flipped); bounds are `[min_x, min_y, max_x, max_y]` and all distances use map units.

Install the benchmark requirements in the existing environment (NumPy, SciPy
and Matplotlib are the project's existing Python prerequisites), then run:

```bash
.venv/bin/pip install -r benchmarks/requirements.txt
.venv/bin/python benchmarks/corridor_graph_demo.py --world all rooms --clearance-lines 3
```

The demo runs independently of the native controller. It validates each graph,
prints node/edge/component counts, minimum/mean/maximum physical width and the
narrowest corridors, and saves `<world>.png`, `<world>.graph.json` and
`statistics.json` to `benchmarks/results/corridors/`. Mean width is weighted by
arclength over all width profiles; minimum and maximum are across all samples.
The JSON is graph output, while the map stays in the bundled Bench-MR format.
`rooms` is a tiny synthetic environment in the same format: two rooms joined
by a passage whose width is exactly 2 map units.

Options include `--show`, `--plain-edges`, `--labels`, `--spacing`, `--scale`,
`--boundary-trim`, `--no-bottlenecks`, `--no-split`, `--bottleneck-window`,
`--bottleneck-ratio` and `--bottleneck-drop`. See `--help` for their defaults.
Overlays use a width colormap, cyan junctions, red bottleneck diamonds, optional
nearest-boundary clearance lines, and selected edge minimum-width labels.

From code, with `benchmarks` on the Python import path:

```python
import json
from corridor_graph import build_corridor_graph, GeometryConfig, BottleneckConfig
from corridor_graph.visualization import plot_corridor_graph

world = json.load(open('benchmarks/bench_mr_tasks.json'))['worlds']['warehouse2']
graph = build_corridor_graph(
    world,
    GeometryConfig(sample_spacing=0.15),
    BottleneckConfig(window_distance=5.0, max_width_ratio=0.75, min_width_drop=0.25),
)
neighbors = graph.adjacency()  # node -> [(neighbor, edge_id, length)]
figure, axes = plot_corridor_graph(world, graph)
```

The implementation is separated by responsibility:

- `geometry.py`: physical free space, boundary union, segment Voronoi construction,
  parabola discretization, and collision-checked skeleton.
- `extraction.py`: maximal degree-two chain collapse and physical edge profiles.
- `model.py`: nodes, edges, width samples, adjacency, components and serialization.
- `bottlenecks.py`: configurable width-minimum detection and optional splitting.
- `visualization.py`: Matplotlib map overlays.
- `__init__.py`: public builder and geometry/profile/connectivity validation.

Nodes normally have skeleton degree 1 or at least 3. Degree-two bends remain
inside edge geometry; bottlenecks add explicit degree-two nodes. Parallel
corridors are retained as distinct edges. An all-degree-two cycle gets one
deterministic `cycle_anchor` and a self-loop, with two adjacency entries. Every
edge has `u`, `v`, ordered geometry, length, minimum clearance, minimum width,
the minimum-width position, and an ordered `WidthSample` profile containing
arclength, position, clearance and width. Geometry and profiles run from `u`
to `v`. These lengths and adjacency entries support future A*/Dijkstra work.

Clearance is distance to the nearest **exposed obstacle or outer wall**;
physical width is twice that distance. There is no robot radius, inflation,
routing algorithm, congestion model or controller integration. Corner-ending
medial branches have widths approaching zero; the demo reports narrowest
internal corridors separately so those tips do not conceal passage widths.
Very thin peripheral corridors are retained when present in the source map.

Bottleneck detection resamples by arclength and smooths only for detection.
It requires an absolute width drop and a relative drop against representative
widths on both sides, with configurable shoulder quantile, window size,
minimum edge/side lengths and minimum separation. Plateaus get one midpoint;
closed cycles are rotated away from their minimum before detection. Splits keep
the original geometric samples and recompute each subedge's unsmoothed physical
profile. `--no-split` retains the detected positions as markers on original edges.

Remaining numerical approximations are explicit in `GeometryConfig`: boundary
coordinates are quantized to a `1 / coordinate_scale` lattice (default `1e-6`),
curves are represented as polylines with maximum sample spacing (default `0.15`),
and zero-clearance boundary tips are clipped using a tiny inset (default `1e-4`).
The inset affects skeleton endpoints, not width measurement or robot clearance.
The inset adapts downward if needed to preserve positive passages and free-space
topology; effective trim and merge settings are recorded. Extremely thin features
may still require higher coordinate scale or smaller curve spacing;
integer overflow, collapsed rings and invalid quantized boundaries raise errors.
Widths/minima are sample-based, not exact continuous extrema. Every retained
chord is checked against the original polygons. Default validation requires
exactly one graph component for each connected free-space polygon and preservation
of independent free-space holes. It fails explicitly if numerical parameters
disconnect a region or lose an alternative passage.

Default demo results, including bottleneck splits:

| World | Nodes | Edges | Components | Bottlenecks |
| --- | ---: | ---: | ---: | ---: |
| parking1 | 45 | 51 | 1 | 1 |
| parking2 | 55 | 62 | 1 | 1 |
| parking3 | 58 | 67 | 2 | 0 |
| warehouse | 249 | 321 | 2 | 37 |
| warehouse2 | 225 | 300 | 2 | 51 |
| rooms | 15 | 14 | 1 | 1 |

The two-component maps contain actual small disconnected free-space pockets
in the imported geometry. They are preserved rather than silently removed.
`tests/test_corridor_graph.py` covers all five maps, full-polyline collision
checks, width/profile consistency, the analytical rectangular skeleton and a
corner/wall parabola, room/passage width, cycle/parallel-edge preservation,
overlapping/concave polygons, disconnected space and bottleneck noise rejection.

```bash
.venv/bin/python -m unittest discover -s tests -p test_corridor_graph.py -v
```

## Routing-level corridor graphs

`DetailedCorridorGraph` (an alias for the existing `CorridorGraph`) remains the
physical ECM representation. `simplify_corridor_graph` derives a separate
`RoutingCorridorGraph` without editing the detailed nodes, edges or profiles.
No new dependencies are required for this stage.

```bash
.venv/bin/python benchmarks/corridor_routing_demo.py --world all rooms

# Reuse the detailed graphs already written by the first-stage demo:
.venv/bin/python benchmarks/corridor_routing_demo.py --world all rooms \
    --detailed-dir benchmarks/results/corridors
```

Outputs go to `benchmarks/results/routing_corridors/`: three-panel
`<world>.comparison.png` figures (map, detailed ECM, routing graph), prominent
`<world>.routing.png` overlays, detailed/debug graph JSON, compact
`<world>.runtime.npz` arrays, and `statistics.json`. `--hide-detailed` hides the
faint ECM underlay; `--show` opens the routing overlay interactively. All numeric
`SimplificationConfig` fields are available as CLI options with hyphenated names;
for example `--junction-max-extent 0.5 --geometry-tolerance 0.02`. `--no-prune`
retains terminal branches. `--protected-node-ids 10 20` preserves known semantic
access points. These IDs belong to the selected detailed graph, so use that
option with the relevant single world.

```python
from corridor_graph import (
    build_corridor_graph, simplify_corridor_graph, SimplificationConfig, load_runtime,
)

detailed = build_corridor_graph(world)
routing = simplify_corridor_graph(detailed, world, SimplificationConfig())
neighbors = routing.adjacency()
edge = next(iter(routing.edges.values()))
full_reference_polyline = edge.reference_geometry(detailed)
original_edges = edge.source_edge_ids
sizes = routing.save_runtime('robot_routes.npz')
runtime = load_runtime('robot_routes.npz')
onboard_neighbors = runtime.adjacency()
```

The modular pipeline implements these rules:

1. **Protect features.** Existing bottleneck nodes and marker-only minima become
   explicit protected routing nodes. Width profiles are also rescanned with the
   detailed graph's bottleneck criterion, even if detailed splitting/detection
   was disabled. Markers on edge interiors retain source edge/sample indices.
   Strong width transitions at degree-two vertices and caller-protected access
   nodes are retained. Protected features cannot be pruned, clustered or collapsed.
2. **Prune local boundary tips.** A terminal branch must satisfy length and
   length/attachment-clearance limits, fade to a small tip width, and have no
   clearance-growing room along it. Every branch point must be visible from its
   attachment; triangle fans between samples certify the continuous visibility
   region against obstacles and the map boundary. Protected branches are kept.
   Eligible disjoint tips are removed in batches so pruning order cannot leave
   an arbitrary sibling corner as a long composite routing spur.
   Every removed branch and its attachment remain recorded in debug provenance.
   Pruning a leaf tree preserves connectivity among retained route locations and
   creates no cycle loss. The geometric definition of an insignificant access
   region is a heuristic; callers can protect meaningful locations explicitly.
3. **Merge local junction trees.** Only adjacent, unprotected junctions can join
   a cluster. Its induced subgraph must be a tree, its complete spatial diameter
   must fit both absolute and local-clearance limits, all junction pairs must
   have direct free-space visibility, and its internal width range must be small.
   Parallel links, self-loops and cycles prevent whole-cluster merging. These
   checks prevent transitive proximity merging across obstacles or narrow necks.
   A real original junction with minimum total internal tree distance represents
   the cluster. Port-to-representative paths along the original tree are attached
   to incident edges, preserving valid embedded paths, lengths and widths.
4. **Collapse trivial chains.** Maximal degree-two chains without protected
   features become one edge. All source spans are concatenated in order, keeping
   bends and width minima. Parallel corridors stay distinct; a pure cycle retains
   a deterministic anchor and a self-loop. Isolated free-space regions stay present.
5. **Reduce geometry separately.** Bounded-error polyline simplification checks
   the complete shortcut against free space, the tolerance tube of its own original
   subchain, and the original walk's minimum continuous clearance. Minimum-width
   positions and protected endpoints are pinned. Zero tolerance retains every
   original sample. A shortcut that is collision-free but leaves its corridor
   is rejected. The routing metric remains the detailed reference-walk length;
   it is not replaced by the shorter display-polyline length.

Each `RoutingEdge` stores ordered `SourceSpan(edge_id, first, last)` references.
Indices are inclusive and may decrease, so reversed traversals, subedges,
parallel corridors, cluster access paths and repeated source edges are explicit.
These spans reconstruct the full reference geometry from the retained detailed
graph. `length`, `min_width` and `min_width_position` are computed from the relevant
unsmoothed sample ranges. A discarded tip's tiny width does not contaminate an
adjacent routing corridor. Cluster connector spans can appear on multiple routing
edges, representing their shared physical junction access paths. Dense width
profiles are never copied into routing edges.

Configuration defaults (distances are map units):

| Parameter | Default | Meaning |
| --- | ---: | --- |
| `branch_max_length` | 5.0 | Maximum removable terminal branch length |
| `branch_length_clearance_ratio` | 1.5 | Branch length / attachment clearance limit |
| `branch_region_growth_ratio` | 1.10 | Maximum branch clearance / attachment clearance |
| `branch_tip_width_ratio` | 0.15 | Maximum tip / attachment width |
| `junction_max_extent` | 1.0 | Complete cluster diameter limit |
| `junction_extent_clearance_ratio` | 1.0 | Cluster diameter / minimum node clearance limit |
| `junction_max_width_ratio` | 1.5 | Internal maximum / minimum width limit |
| `width_transition_ratio` | 2.0 | Representative widths required for a protected transition |
| `width_transition_window` | 1.0 | Arclength window on each side of a degree-two node |
| `geometry_tolerance` | 0.05 | Maximum deviation and original-corridor tube radius |
| `validation_pairs` / `validation_seed` | 200 / 42 | Reproducible routing-node pair sample |
| `distortion_relative_warning` / `distortion_absolute_warning` | 0.25 / 0.5 | Report distance increases exceeding both limits |

`prune_branches=False` disables pruning. `protected_node_ids=()` is an optional
tuple of detailed access nodes that must remain explicit. There is no target
node count and no map-specific parameter profile.

Default suite results using the previously generated detailed graphs:

| World | Nodes detailed → routing | Edges detailed → routing | Geometry samples detailed → routing | Detailed JSON bytes | Runtime NPZ bytes |
| --- | ---: | ---: | ---: | ---: | ---: |
| parking1 | 45 → 25 | 51 → 31 | 1,404 → 204 | 270,081 | 9,271 |
| parking2 | 55 → 28 | 62 → 35 | 1,444 → 230 | 281,114 | 9,982 |
| parking3 | 58 → 30 | 67 → 39 | 2,114 → 261 | 403,673 | 10,736 |
| warehouse | 249 → 201 | 321 → 273 | 12,557 → 1,592 | 2,342,654 | 49,455 |
| warehouse2 | 225 → 173 | 300 → 248 | 15,616 → 1,578 | 2,890,089 | 47,231 |
| rooms | 15 → 7 | 14 → 6 | 332 → 26 | 58,649 | 4,601 |

Serialized sizes compare UTF-8 compact detailed JSON (including dense profiles)
against the actual uncompressed numeric-array NPZ file, including container/header
overhead. They are disk/onboard storage sizes, not Python object memory estimates.
Warehouse2's raw array payload is 43,579 bytes; its NPZ is 47,231 bytes, about
61 times smaller than detailed JSON. The conservative routing topology keeps its
2 components, 77 independent cycles and all 51 bottlenecks. Forty-four local
junction clusters are merged. The right-hand repeated-shelf grid has no routing
edge shorter than one map unit after simplification. Its remaining junctions and
narrow shelf passages correspond to genuine alternative routes; collapsing the
whole grid would lose those alternatives.

Validation checks source-span coverage (including recorded pruned branches),
exact metric/width aggregation, full-polyline collision/corridor checks, protected
feature positions, component mappings, and connectivity after cutting each
protected node. Fundamental routing cycles are expanded into oriented detailed
edge coordinates; equal rank and equal cycle subspaces check which alternatives
survive, beyond merely comparing cycle counts. This detects lost or duplicated
obstacle-separated loops without choosing map-specific homotopy representatives.

Shortest-path comparisons sample 200 pairs per benchmark map, using original
representative/feature locations. The routing length always corresponds to an
actual detailed walk. Cluster access paths can add a detour through their common
representative, so exact shortest-path distance is not promised. For warehouse2,
the 95th percentile ratio is 1.0221 and the maximum sampled ratio is 1.0530.
Across the suite, the largest sampled ratio is 1.1958 on parking1; no pair exceeds
both default reporting limits. Full results, absolute increases and any reported
worst pairs are stored in `statistics.json`. Pair sampling does not exhaustively
bound every shortest-path distortion; topology and protected-cut checks are exhaustive.

The version-1 onboard schema uses only numeric arrays, with pickle disabled:

- Nodes: int32 IDs, float64 positions and local widths, uint8 kind flags.
- Edges: int32 IDs/endpoints, float64 reference lengths, minimum widths and minimum
  positions; flat float64 geometry points plus int32 offsets.
- Bottlenecks: flat int32 protected endpoint IDs plus offsets; node flags, positions
  and local widths describe the actual bottleneck. Access/transition kinds remain explicit.

Float64 avoids shifting thin peripheral corridors into obstacles during float32
quantization. IDs may be sparse; array rows are sorted by ID. Parallel edges and
self-loops survive serialization. Loading validates the schema version, numeric
types, finite values, offsets, endpoints and bottleneck references. The runtime
contains neither dense width profiles nor source spans, cluster/pruning records,
or validation reports. Those remain in the detailed graph and routing debug JSON
for offline reconstruction and later congestion/resource modeling.

Known limits are deliberate: the branch-region criterion cannot infer semantic
goals near wall corners, so protect such access nodes; thin peripheral passages
and small disconnected pockets remain when topologically real; cycles and
bottlenecks limit node-count reduction in dense warehouse grids; larger cluster
thresholds may add larger shared access-path detours; and geometry reduction may
remain conservative in narrow curved passages. First-stage coordinate quantization
and sampled width extrema also remain. Shared internal cluster access paths are
recorded offline and may need explicit resource treatment in a future congestion
model. No routing policy, controller integration or congestion probabilities are added.

New modules are `routing_model.py`, `simplification.py`, `routing_validation.py`
and `runtime.py`; `visualization.py` adds comparison/overlay functions. The public
API is exported by `corridor_graph/__init__.py`; `corridor_routing_demo.py` runs
the benchmark. `tests/test_corridor_routing.py` covers chains, protected minima,
tree/cyclic/obstacle-separated junctions, short real passages, access protection,
collision-aware geometry, route distortion, all benchmark maps, detailed-graph
immutability and binary serialization. This stage added 25 tests to the existing suite.

```bash
.venv/bin/python -m unittest discover -s tests
```

## Robot-independent geometric traffic preprocessing

The offline pipeline is polygon map → detailed ECM → routing graph →
geometric passage resources and junction candidates. It retains all graph
connectivity and alternatives. Robot-specific feasibility, turning, passing,
traffic capacities, scheduling, stochastic routing and MPPI integration are
outside this stage. The earlier traffic feasibility helper has been removed.

```bash
# Reuse the existing graph geometry and IDs; audit/extract the offline layer.
.venv/bin/python benchmarks/corridor_traffic_demo.py --world all --synthetic \
    --routing-dir benchmarks/results/routing_corridors

# Or build all stages from the map, with no robot input.
.venv/bin/python benchmarks/corridor_traffic_demo.py --world warehouse2

# Geometric class controls only.
.venv/bin/python benchmarks/corridor_traffic_demo.py --world warehouse2 \
    --narrow-width 2.5 --width-hysteresis 0.0001
```

Outputs remain in `benchmarks/results/traffic_resources/`. Each
`<world>.resources.json` is a **self-contained schema-v2 geometry bundle**;
`width_audit.json` and `statistics.json` are separate diagnostics. Warehouse2
also has an overview, right-side detail, original selected-width panels,
continuous-width panels, and `boundary_diagnostics.png` with two-scale insets.
All distances, widths, radii of geometric regions, and source arclengths are
**map units**, not assumed meters.

### Clearance, tiny gaps and construction endpoints

Clearance is exact distance to the exposed obstacle-union and outer-wall
**segments**, including nearest points in the interiors of polygon sides.
Stored detailed profiles, their scalar sample minima, routing source-span
minima and node local widths agree with the original polygons. Resource minima
and cached routing minima additionally measure continuous clearance over every
reference chord. Width here means **2 × reference-path clearance**; it is not
an operational passage width or a maneuver feasibility answer.

The previously questioned warehouse2 sample widths remain unchanged:

| Routing edge | Sample width | Nearest boundaries |
| --- | ---: | --- |
| 88 | 0.1510060035 | Outer bottom wall and an obstacle side |
| 31 | 0.2319270425 | Opposing rotated obstacle sides |
| 165 | 0.3273400000 | Opposing shelf sides, including an adjacent corner |

Their continuous resource minima are 0.1507216571, 0.2319270365 and
0.3269354595. The slight differences reflect the discretized reference path
moving off the exact medial bisector; no original width is overwritten.
The audit's maximum sample/chord difference in warehouse2 is 0.004438.

The values near **0.0002** are accurate clearance diameters at endpoints cut
where clearance reaches the configured numerical inset, 0.0001. They do not
establish a uniform through-passage width. Exact feature separation identifies
six affected warehouse2 resources as boundary tapers whose supporting sides
ultimately meet, and one as a construction endpoint near a **positive 0.00001**
feature gap at the upper-right corner. A local positive gap alone does not prove
an independent through route; the full polygon geometry is retained. Seven
resource minima and eight terminal ports are annotated (one resource has two
trim endpoints). These are valid free-space centerlines, so none are deleted.
The figures mark trim endpoints with triangles and their resource chains with
dashed lines, while preserving actual measured minima.

`WidthAuditor` now scales contact ties to local clearance and machine precision.
An absolute 1e-5 tolerance could overwhelm a small gap and classify angled
contacts as opposed. Distinct contacts must now be nearly opposite; duplicate
contacts from two sides meeting at the same corner are not an opposing gap.
True positive gaps below the old tolerance remain positive and are retained.
Each witness records actual segment IDs, endpoints, contact positions and
distances. Classifications are local geometric evidence, not separator or
traffic-conflict certificates.

Medial-axis construction now checks the **topology of the numerical inset**.
If a positive inset cuts a genuine passage, it reduces that inset and, when
necessary, endpoint merging tolerance, without changing the map. Zero-width
contacts between rings are handled as physical closures. For a 0.00002 gap,
the default 0.0001 inset adapts to 0.000001 and preserves the independent route.
Graph validation also compares cycle rank with independent free-space holes:
a positive gap below coordinate quantization resolution raises explicitly
rather than silently losing an alternative. A 0.0000004 fixture is preserved
with a finer coordinate scale. Effective numerical settings are recorded.

Rebuilding all five benchmark detailed graphs produces **exactly the same node
and edge data** as the saved graphs. The routing simplifier has not been changed.

### Narrow passages and junction candidates

`geometric_intervals.py` intersects reference chords analytically with
distance-to-segment capsules: a side strip and its endpoint circles. It finds
all low-clearance intervals, even several disjoint necks within one coarse
chord, independently of width-profile sampling density. It never inflates
obstacles by a robot footprint.

| Geometric parameter | Default |
| --- | ---: |
| Narrow width class | 3.0 |
| Width-class hysteresis envelope | 0.0001 |
| Maximum junction-region radius | 1.5 |
| Junction radius / local clearance | 0.75 |
| Minimum angular separation of incident arms | 15 degrees |
| Connectivity tolerance / maximum contact-tie tolerance | 1e-7 / 1e-5 |

A resource must include clearance diameter strictly below the narrow class.
Its connected supporting intervals may extend into the explicitly recorded
class envelope (3.0001 by default). This avoids threshold-noise fragmentation;
it does not smooth profiles, clamp minima or hide wider portions.
Set `--width-hysteresis 0` to disable that envelope. A wholly envelope-only
component does not become a resource.

Intervals join only through the same routing node with two incident arms.
Branching decisions, parallel alternatives, disconnected passages and pure
cycle seams are retained. Width minima and protected degree-two bottlenecks do
not create arbitrary independent resources: the several-minima fixture has four
markers in one ordered chain. Resources can cover only part of an edge.

Junctions are **geometric candidates** with kind `junction_region`, exported
separately under `junction_regions`. They use a clearance-limited 64-sided
polygon approximating a disk, strictly inside free space. Incident reference
arms clip to the **stored polygon**, including oblique arms. Each port identifies
the incident edge, u/v arm and whether it is on the polygon boundary or is an
internal routing connection. Short edges and self-loop seams can produce
internal connections; loop arms partition their coverage without duplicate length.

`possible_connections` enumerates ordered entry/exit **port indices**, including
same-port return candidates. These are topological geometric candidates only.
No turning maneuver, movement-pair collision, capacity or traffic-direction
assignment is calculated. Junction/narrow overlaps and shared centerline
coverage are recorded explicitly in `relationships`, without merging choices.

### Exact data available to later planning queries

The schema-v2 JSON has these top-level collections:

- `map_geometry`: bounds and polygon coordinates in the original map frame.
- `boundary_features`: exposed segments with stable IDs for this bundle,
  obstacle/outer classification, free-space component and boundary-ring IDs.
- `routing_geometry`: complete original routing topology, simplified display
  geometry, lengths, bottleneck IDs and detailed source spans, plus full
  reference centerlines, arclength arrays, exact sample clearances, continuous
  minimum clearance/position and boundary witnesses.
- `resources`: narrow centerline chains, contributing routing and detailed IDs,
  length, minimum/representative clearance and width, ports, minimum witnesses,
  protected local-minimum witnesses and construction-end annotations.
- `junction_regions`: region polygons, connected arms, routing anchor, ports,
  incident edges, source intervals and possible entry/exit connections.
- `edge_intervals`: precise routing arclength ranges and their orientation on
  resource chains. Each also stores precise oriented arclength ranges on the
  contributing original detailed edges.
- `relationships`: touching/overlapping centerline or candidate-region geometry,
  resource IDs, shared routing intervals and intersection geometry.
- `metadata`: schema version, units, all extraction settings and fingerprints
  of the map, source graphs and geometry cache.

Minima are continuous over reference chords; representative width is an
arclength-weighted median of chord-midpoint clearance samples. Neither is a
general passage width. The polygons and reference profiles remain available
for subsequent queries with any caller-provided geometric model.

```python
from benchmarks.corridor_graph import load_traffic_resources, DirectedEdge, path_resource_usage

traffic = load_traffic_resources("benchmarks/results/traffic_resources/warehouse2.resources.json")
routing = traffic.routing_graph()  # restored from cached geometry; no ECM rebuild
edge = next(iter(routing.edges.values()))
usage = path_resource_usage(traffic, routing, [DirectedEdge(edge.id, +1)])
```

Reload checks map, routing and geometry-cache identities. Route usage retains
ordered entry/exit ports, centerline direction and covered arclength. Directions
mean orientation relative to stored geometry, **not** permitted traffic direction.
Repeated visits, reversals and loop laps remain distinct; overlapping candidate
regions remain separate records, ordered by route entry arclength. Their lengths
must not be added as though the regions partition the path. Ambiguous bare-edge
walks require an explicit start node or oriented edge list.

### Benchmark evaluation

Default class 3.0, envelope 0.0001; existing detailed/routing files reused.
Times include width audit, geometric cache construction, resource extraction and
validation, excluding graph construction, statistics, serialization and plotting.
Before counts come from the previous run saved in `baseline_statistics.json`;
`--baseline-statistics` can select another report. Counts are not an optimization
target.

| Map | Routing nodes/edges | Narrow before → after | Junction regions | Preprocessing s | Compact / formatted JSON MB |
| --- | ---: | ---: | ---: | ---: | ---: |
| parking1 | 25 / 31 | 23 → 23 | 16 | 0.29 | 0.540 / 1.097 |
| parking2 | 28 / 35 | 24 → 24 | 19 | 0.31 | 0.596 / 1.214 |
| parking3 | 30 / 39 | 28 → 28 | 23 | 0.35 | 0.728 / 1.477 |
| warehouse | 201 / 273 | 172 → 172 | 134 | 2.59 | 4.445 / 9.058 |
| warehouse2 | 173 / 248 | 108 → 108 | 114 | 2.33 | 4.248 / 8.641 |

Sizes are decimal MB of the **complete self-contained geometry bundle**, not an
incremental resource payload estimate. Warehouse2 contains 317 explicit region
relationships; its approximate Python object size is 9.72 MB including all geometry
and diagnostic metadata. Runtime and allocator sizes vary by environment.

Width and length distributions, **min / median / max**, in map units; quartiles
are also saved:

| Map | Narrow clearance diameter | Narrow length | Junction-arm clearance diameter | Junction total arm length |
| --- | --- | --- | --- | --- |
| parking1 | .0002 / .0831 / 2.9638 | .6417 / 5.2142 / 9.2145 | .1706 / 1.3491 / 5.8235 | .3085 / 4.0479 / 7.6261 |
| parking2 | .0002 / .0665 / .7677 | .4729 / 4.9311 / 11.1845 | .2232 / 1.6903 / 4.0603 | .4304 / 4.0436 / 6.1079 |
| parking3 | .0002 / .2570 / 1.2843 | 1.4367 / 4.9532 / 43.9709 | .2827 / 1.8466 / 5.2867 | .5403 / 3.9770 / 6.1922 |
| warehouse | .0002 / .4245 / 2.2997 | .6853 / 4.4024 / 42.1784 | .3490 / 1.0509 / 10.1095 | .6062 / 2.8404 / 6.5486 |
| warehouse2 | .0002 / .5696 / 1.4006 | .5290 / 5.4630 / 44.8628 | .3165 / 4.8776 / 12.3575 | .6446 / 4.4976 / 6.4948 |

Suspicious-measurement counts overlap; they are diagnostics, not reasons to
delete geometry:

| Map | Width <0.5 edges | Clearance-only sample-min witnesses | Trim-min edges | Boundary-taper resources |
| --- | ---: | ---: | ---: | ---: |
| parking1 | 22 | 5 | 8 | 8 |
| parking2 | 21 | 5 | 8 | 8 |
| parking3 | 26 | 4 | 4 | 4 |
| warehouse | 115 | 34 | 29 | 29 |
| warehouse2 | 76 | 16 | 7 | 6 |

The warehouse2 overview, repeated-shelf detail and two-scale boundary diagnostic
figures were inspected. Independent shelf passages remain separate, actual
narrow chains have distinct colors, junction candidates use translucent polygons,
and numerical trim endpoints are visibly annotated.

### Validation and remaining approximations

The synthetic suite covers a real neck between obstacles, a nearly closed
obstacle-to-wall gap, parallel passages, several minima in one passage, a narrow
T, a wide crossing, and duplicate corner-feature contacts. A straight strip is
an additional geometric route-mapping fixture. None uses robot dimensions.
Additional tests exercise noise envelopes without width clamping, several necks
inside one coarse chord, oblique junction boundaries, explicit overlaps,
precise ECM coverage, repeated/directional usage, JSON roundtrips and corruption
checks, sub-resolution gaps, and all five benchmark maps. Run:

```bash
.venv/bin/python -m unittest discover -s tests
```

The full suite passes **95 tests**, including 31 traffic-geometry tests.

Remaining approximations are the integer boundary-coordinate lattice, discretized
medial curves, positive numerical endpoint trim, midpoint representative-width
statistics, the configurable geometric class envelope, and localized polygonal
junction disks. Topology checks reject silent passage loss, but do not prove
metric accuracy of a quantized bisector. A local boundary witness does not prove
a complete separating cross-section. Shared source paths beyond a local junction
disk may require broader candidate regions in later geometry work.

Modified modules are `geometry.py`, `__init__.py`, `width_audit.py`,
`traffic_model.py`, `traffic_extraction.py` and `traffic_visualization.py`.
New modules are `geometric_intervals.py`, `traffic_geometry.py` and
`traffic_serialization.py`; `traffic_feasibility.py` was removed.
The demo, geometric fixtures, traffic tests and this documentation were updated.
Routing simplification and the controller remain unchanged.

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

## C++ complete K-route planner

[`COMPLETE_ROUTES.md`](COMPLETE_ROUTES.md) documents the C++17 planner, native
build, JSON demo, measured four-map benchmarks, and generated route figures. It
loads the existing schema-v2 traffic bundle once, attaches arbitrary circular-robot
queries without changing the graph, and returns complete paths with directed edge
and resource traversals. Routing, adaptive lane decomposition, swept geometry
validation and native benchmark verification run in C++. Python only prepares
demo inputs and plots native output. A validated SPLIT interval always uses the
robot's right-hand directional lane; narrower or uncertified intervals use a
shared center reference, with explicit validated transitions and junction links.

[`KEEP_RIGHT_AUDIT.md`](KEEP_RIGHT_AUDIT.md) records the mandatory lane-placement
audit, opposing-footprint certificates, query diagnostics, separate geometry
timings, synthetic tests and four equal-scale warehouse2 lane zooms.

[`RADIUS_VALIDATION.md`](RADIUS_VALIDATION.md) contains the subsequent radius
sweep, independent corridor certificates, exclusive stage profile and measured
spatial-index optimization. Reproduce it with
`.venv/bin/python benchmarks/radius_sweep.py` after the native build. Use
`--analyse-only` to regenerate CSV/JSON tables and plots from saved native output.
