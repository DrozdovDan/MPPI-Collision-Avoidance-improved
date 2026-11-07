# Uniform-agent warehouse suite

**250 tasks: 2 maps × 25 scenario designs × 5 start–goal variants.** Every task contains exactly 32 identical differential-drive disk agents (8,000 individually validated assignments).

Each map has all combinations of five diameters (0.6, 0.7, 0.8, 0.9, 1.0 m) and five patterns: distributed traffic, opposing bottleneck traffic, intersection crossings, symmetric swaps, and competing bottlenecks. A design keeps its diameter, pattern and geometric focus across all five variants. Every robot receives five different starts and five different goals.

The imported maps have no measured SI calibration. This suite explicitly adopts **1 native map unit = 1 meter** as a benchmark convention; it does not claim the source measured that scale. Geometry and bounds retain the identity transform. The 32-agent count is fixed for every task, not a claim of a packing maximum.

All start and goal footprints are disjoint and clear of walls. Every assignment has a replayed bounded-speed, bounded-acceleration unicycle witness with swept-disk clearance. These are individual feasibility certificates, not joint schedules or MPPI trajectories. No controller or evaluated K-route/RRT outcome selected a task. The simulation uses its original unicycle equations.

Generation and individual feasibility checks: 839.49 s.

| Map | Width × height under calibration | Designs | Variants/design | Tasks | Agents/task |
| --- | --- | ---: | ---: | ---: | ---: |
| warehouse | 85.71 × 80.43 m | 25 | 5 | 125 | 32 |
| warehouse2 | 124.41 × 81.24 m | 25 | 5 | 125 | 32 |

## All 50 scenario designs

The preview for each design shows all five variants; the overview sheets show variant 1 of all 25 designs on each map.

- [warehouse: 25 designs](visualizations/warehouse.25_scenarios.png)
- [warehouse2: 25 designs](visualizations/warehouse2.25_scenarios.png)

| Design | Diameter | Pattern | Tasks | Five-variant preview |
| --- | ---: | --- | ---: | --- |
| warehouse_s01 | 0.6 m | distributed | 5 | [preview](visualizations/warehouse_s01.variants.png) |
| warehouse_s02 | 0.6 m | opposing_bottleneck | 5 | [preview](visualizations/warehouse_s02.variants.png) |
| warehouse_s03 | 0.6 m | intersection | 5 | [preview](visualizations/warehouse_s03.variants.png) |
| warehouse_s04 | 0.6 m | symmetric_swap | 5 | [preview](visualizations/warehouse_s04.variants.png) |
| warehouse_s05 | 0.6 m | competing_bottlenecks | 5 | [preview](visualizations/warehouse_s05.variants.png) |
| warehouse_s06 | 0.7 m | distributed | 5 | [preview](visualizations/warehouse_s06.variants.png) |
| warehouse_s07 | 0.7 m | opposing_bottleneck | 5 | [preview](visualizations/warehouse_s07.variants.png) |
| warehouse_s08 | 0.7 m | intersection | 5 | [preview](visualizations/warehouse_s08.variants.png) |
| warehouse_s09 | 0.7 m | symmetric_swap | 5 | [preview](visualizations/warehouse_s09.variants.png) |
| warehouse_s10 | 0.7 m | competing_bottlenecks | 5 | [preview](visualizations/warehouse_s10.variants.png) |
| warehouse_s11 | 0.8 m | distributed | 5 | [preview](visualizations/warehouse_s11.variants.png) |
| warehouse_s12 | 0.8 m | opposing_bottleneck | 5 | [preview](visualizations/warehouse_s12.variants.png) |
| warehouse_s13 | 0.8 m | intersection | 5 | [preview](visualizations/warehouse_s13.variants.png) |
| warehouse_s14 | 0.8 m | symmetric_swap | 5 | [preview](visualizations/warehouse_s14.variants.png) |
| warehouse_s15 | 0.8 m | competing_bottlenecks | 5 | [preview](visualizations/warehouse_s15.variants.png) |
| warehouse_s16 | 0.9 m | distributed | 5 | [preview](visualizations/warehouse_s16.variants.png) |
| warehouse_s17 | 0.9 m | opposing_bottleneck | 5 | [preview](visualizations/warehouse_s17.variants.png) |
| warehouse_s18 | 0.9 m | intersection | 5 | [preview](visualizations/warehouse_s18.variants.png) |
| warehouse_s19 | 0.9 m | symmetric_swap | 5 | [preview](visualizations/warehouse_s19.variants.png) |
| warehouse_s20 | 0.9 m | competing_bottlenecks | 5 | [preview](visualizations/warehouse_s20.variants.png) |
| warehouse_s21 | 1.0 m | distributed | 5 | [preview](visualizations/warehouse_s21.variants.png) |
| warehouse_s22 | 1.0 m | opposing_bottleneck | 5 | [preview](visualizations/warehouse_s22.variants.png) |
| warehouse_s23 | 1.0 m | intersection | 5 | [preview](visualizations/warehouse_s23.variants.png) |
| warehouse_s24 | 1.0 m | symmetric_swap | 5 | [preview](visualizations/warehouse_s24.variants.png) |
| warehouse_s25 | 1.0 m | competing_bottlenecks | 5 | [preview](visualizations/warehouse_s25.variants.png) |
| warehouse2_s01 | 0.6 m | distributed | 5 | [preview](visualizations/warehouse2_s01.variants.png) |
| warehouse2_s02 | 0.6 m | opposing_bottleneck | 5 | [preview](visualizations/warehouse2_s02.variants.png) |
| warehouse2_s03 | 0.6 m | intersection | 5 | [preview](visualizations/warehouse2_s03.variants.png) |
| warehouse2_s04 | 0.6 m | symmetric_swap | 5 | [preview](visualizations/warehouse2_s04.variants.png) |
| warehouse2_s05 | 0.6 m | competing_bottlenecks | 5 | [preview](visualizations/warehouse2_s05.variants.png) |
| warehouse2_s06 | 0.7 m | distributed | 5 | [preview](visualizations/warehouse2_s06.variants.png) |
| warehouse2_s07 | 0.7 m | opposing_bottleneck | 5 | [preview](visualizations/warehouse2_s07.variants.png) |
| warehouse2_s08 | 0.7 m | intersection | 5 | [preview](visualizations/warehouse2_s08.variants.png) |
| warehouse2_s09 | 0.7 m | symmetric_swap | 5 | [preview](visualizations/warehouse2_s09.variants.png) |
| warehouse2_s10 | 0.7 m | competing_bottlenecks | 5 | [preview](visualizations/warehouse2_s10.variants.png) |
| warehouse2_s11 | 0.8 m | distributed | 5 | [preview](visualizations/warehouse2_s11.variants.png) |
| warehouse2_s12 | 0.8 m | opposing_bottleneck | 5 | [preview](visualizations/warehouse2_s12.variants.png) |
| warehouse2_s13 | 0.8 m | intersection | 5 | [preview](visualizations/warehouse2_s13.variants.png) |
| warehouse2_s14 | 0.8 m | symmetric_swap | 5 | [preview](visualizations/warehouse2_s14.variants.png) |
| warehouse2_s15 | 0.8 m | competing_bottlenecks | 5 | [preview](visualizations/warehouse2_s15.variants.png) |
| warehouse2_s16 | 0.9 m | distributed | 5 | [preview](visualizations/warehouse2_s16.variants.png) |
| warehouse2_s17 | 0.9 m | opposing_bottleneck | 5 | [preview](visualizations/warehouse2_s17.variants.png) |
| warehouse2_s18 | 0.9 m | intersection | 5 | [preview](visualizations/warehouse2_s18.variants.png) |
| warehouse2_s19 | 0.9 m | symmetric_swap | 5 | [preview](visualizations/warehouse2_s19.variants.png) |
| warehouse2_s20 | 0.9 m | competing_bottlenecks | 5 | [preview](visualizations/warehouse2_s20.variants.png) |
| warehouse2_s21 | 1.0 m | distributed | 5 | [preview](visualizations/warehouse2_s21.variants.png) |
| warehouse2_s22 | 1.0 m | opposing_bottleneck | 5 | [preview](visualizations/warehouse2_s22.variants.png) |
| warehouse2_s23 | 1.0 m | intersection | 5 | [preview](visualizations/warehouse2_s23.variants.png) |
| warehouse2_s24 | 1.0 m | symmetric_swap | 5 | [preview](visualizations/warehouse2_s24.variants.png) |
| warehouse2_s25 | 1.0 m | competing_bottlenecks | 5 | [preview](visualizations/warehouse2_s25.variants.png) |

## All 250 tasks

| Task | Design | Variant | Agents | Diameter | Validation | Assignment |
| --- | --- | ---: | ---: | ---: | --- | --- |
| [warehouse_s01_v01](visualizations/warehouse_s01_v01.png) | warehouse_s01 | 1 | 32 | 0.6 m | passed | [JSON](tasks/warehouse_s01_v01.json) |
| [warehouse_s01_v02](visualizations/warehouse_s01_v02.png) | warehouse_s01 | 2 | 32 | 0.6 m | passed | [JSON](tasks/warehouse_s01_v02.json) |
| [warehouse_s01_v03](visualizations/warehouse_s01_v03.png) | warehouse_s01 | 3 | 32 | 0.6 m | passed | [JSON](tasks/warehouse_s01_v03.json) |
| [warehouse_s01_v04](visualizations/warehouse_s01_v04.png) | warehouse_s01 | 4 | 32 | 0.6 m | passed | [JSON](tasks/warehouse_s01_v04.json) |
| [warehouse_s01_v05](visualizations/warehouse_s01_v05.png) | warehouse_s01 | 5 | 32 | 0.6 m | passed | [JSON](tasks/warehouse_s01_v05.json) |
| [warehouse_s02_v01](visualizations/warehouse_s02_v01.png) | warehouse_s02 | 1 | 32 | 0.6 m | passed | [JSON](tasks/warehouse_s02_v01.json) |
| [warehouse_s02_v02](visualizations/warehouse_s02_v02.png) | warehouse_s02 | 2 | 32 | 0.6 m | passed | [JSON](tasks/warehouse_s02_v02.json) |
| [warehouse_s02_v03](visualizations/warehouse_s02_v03.png) | warehouse_s02 | 3 | 32 | 0.6 m | passed | [JSON](tasks/warehouse_s02_v03.json) |
| [warehouse_s02_v04](visualizations/warehouse_s02_v04.png) | warehouse_s02 | 4 | 32 | 0.6 m | passed | [JSON](tasks/warehouse_s02_v04.json) |
| [warehouse_s02_v05](visualizations/warehouse_s02_v05.png) | warehouse_s02 | 5 | 32 | 0.6 m | passed | [JSON](tasks/warehouse_s02_v05.json) |
| [warehouse_s03_v01](visualizations/warehouse_s03_v01.png) | warehouse_s03 | 1 | 32 | 0.6 m | passed | [JSON](tasks/warehouse_s03_v01.json) |
| [warehouse_s03_v02](visualizations/warehouse_s03_v02.png) | warehouse_s03 | 2 | 32 | 0.6 m | passed | [JSON](tasks/warehouse_s03_v02.json) |
| [warehouse_s03_v03](visualizations/warehouse_s03_v03.png) | warehouse_s03 | 3 | 32 | 0.6 m | passed | [JSON](tasks/warehouse_s03_v03.json) |
| [warehouse_s03_v04](visualizations/warehouse_s03_v04.png) | warehouse_s03 | 4 | 32 | 0.6 m | passed | [JSON](tasks/warehouse_s03_v04.json) |
| [warehouse_s03_v05](visualizations/warehouse_s03_v05.png) | warehouse_s03 | 5 | 32 | 0.6 m | passed | [JSON](tasks/warehouse_s03_v05.json) |
| [warehouse_s04_v01](visualizations/warehouse_s04_v01.png) | warehouse_s04 | 1 | 32 | 0.6 m | passed | [JSON](tasks/warehouse_s04_v01.json) |
| [warehouse_s04_v02](visualizations/warehouse_s04_v02.png) | warehouse_s04 | 2 | 32 | 0.6 m | passed | [JSON](tasks/warehouse_s04_v02.json) |
| [warehouse_s04_v03](visualizations/warehouse_s04_v03.png) | warehouse_s04 | 3 | 32 | 0.6 m | passed | [JSON](tasks/warehouse_s04_v03.json) |
| [warehouse_s04_v04](visualizations/warehouse_s04_v04.png) | warehouse_s04 | 4 | 32 | 0.6 m | passed | [JSON](tasks/warehouse_s04_v04.json) |
| [warehouse_s04_v05](visualizations/warehouse_s04_v05.png) | warehouse_s04 | 5 | 32 | 0.6 m | passed | [JSON](tasks/warehouse_s04_v05.json) |
| [warehouse_s05_v01](visualizations/warehouse_s05_v01.png) | warehouse_s05 | 1 | 32 | 0.6 m | passed | [JSON](tasks/warehouse_s05_v01.json) |
| [warehouse_s05_v02](visualizations/warehouse_s05_v02.png) | warehouse_s05 | 2 | 32 | 0.6 m | passed | [JSON](tasks/warehouse_s05_v02.json) |
| [warehouse_s05_v03](visualizations/warehouse_s05_v03.png) | warehouse_s05 | 3 | 32 | 0.6 m | passed | [JSON](tasks/warehouse_s05_v03.json) |
| [warehouse_s05_v04](visualizations/warehouse_s05_v04.png) | warehouse_s05 | 4 | 32 | 0.6 m | passed | [JSON](tasks/warehouse_s05_v04.json) |
| [warehouse_s05_v05](visualizations/warehouse_s05_v05.png) | warehouse_s05 | 5 | 32 | 0.6 m | passed | [JSON](tasks/warehouse_s05_v05.json) |
| [warehouse_s06_v01](visualizations/warehouse_s06_v01.png) | warehouse_s06 | 1 | 32 | 0.7 m | passed | [JSON](tasks/warehouse_s06_v01.json) |
| [warehouse_s06_v02](visualizations/warehouse_s06_v02.png) | warehouse_s06 | 2 | 32 | 0.7 m | passed | [JSON](tasks/warehouse_s06_v02.json) |
| [warehouse_s06_v03](visualizations/warehouse_s06_v03.png) | warehouse_s06 | 3 | 32 | 0.7 m | passed | [JSON](tasks/warehouse_s06_v03.json) |
| [warehouse_s06_v04](visualizations/warehouse_s06_v04.png) | warehouse_s06 | 4 | 32 | 0.7 m | passed | [JSON](tasks/warehouse_s06_v04.json) |
| [warehouse_s06_v05](visualizations/warehouse_s06_v05.png) | warehouse_s06 | 5 | 32 | 0.7 m | passed | [JSON](tasks/warehouse_s06_v05.json) |
| [warehouse_s07_v01](visualizations/warehouse_s07_v01.png) | warehouse_s07 | 1 | 32 | 0.7 m | passed | [JSON](tasks/warehouse_s07_v01.json) |
| [warehouse_s07_v02](visualizations/warehouse_s07_v02.png) | warehouse_s07 | 2 | 32 | 0.7 m | passed | [JSON](tasks/warehouse_s07_v02.json) |
| [warehouse_s07_v03](visualizations/warehouse_s07_v03.png) | warehouse_s07 | 3 | 32 | 0.7 m | passed | [JSON](tasks/warehouse_s07_v03.json) |
| [warehouse_s07_v04](visualizations/warehouse_s07_v04.png) | warehouse_s07 | 4 | 32 | 0.7 m | passed | [JSON](tasks/warehouse_s07_v04.json) |
| [warehouse_s07_v05](visualizations/warehouse_s07_v05.png) | warehouse_s07 | 5 | 32 | 0.7 m | passed | [JSON](tasks/warehouse_s07_v05.json) |
| [warehouse_s08_v01](visualizations/warehouse_s08_v01.png) | warehouse_s08 | 1 | 32 | 0.7 m | passed | [JSON](tasks/warehouse_s08_v01.json) |
| [warehouse_s08_v02](visualizations/warehouse_s08_v02.png) | warehouse_s08 | 2 | 32 | 0.7 m | passed | [JSON](tasks/warehouse_s08_v02.json) |
| [warehouse_s08_v03](visualizations/warehouse_s08_v03.png) | warehouse_s08 | 3 | 32 | 0.7 m | passed | [JSON](tasks/warehouse_s08_v03.json) |
| [warehouse_s08_v04](visualizations/warehouse_s08_v04.png) | warehouse_s08 | 4 | 32 | 0.7 m | passed | [JSON](tasks/warehouse_s08_v04.json) |
| [warehouse_s08_v05](visualizations/warehouse_s08_v05.png) | warehouse_s08 | 5 | 32 | 0.7 m | passed | [JSON](tasks/warehouse_s08_v05.json) |
| [warehouse_s09_v01](visualizations/warehouse_s09_v01.png) | warehouse_s09 | 1 | 32 | 0.7 m | passed | [JSON](tasks/warehouse_s09_v01.json) |
| [warehouse_s09_v02](visualizations/warehouse_s09_v02.png) | warehouse_s09 | 2 | 32 | 0.7 m | passed | [JSON](tasks/warehouse_s09_v02.json) |
| [warehouse_s09_v03](visualizations/warehouse_s09_v03.png) | warehouse_s09 | 3 | 32 | 0.7 m | passed | [JSON](tasks/warehouse_s09_v03.json) |
| [warehouse_s09_v04](visualizations/warehouse_s09_v04.png) | warehouse_s09 | 4 | 32 | 0.7 m | passed | [JSON](tasks/warehouse_s09_v04.json) |
| [warehouse_s09_v05](visualizations/warehouse_s09_v05.png) | warehouse_s09 | 5 | 32 | 0.7 m | passed | [JSON](tasks/warehouse_s09_v05.json) |
| [warehouse_s10_v01](visualizations/warehouse_s10_v01.png) | warehouse_s10 | 1 | 32 | 0.7 m | passed | [JSON](tasks/warehouse_s10_v01.json) |
| [warehouse_s10_v02](visualizations/warehouse_s10_v02.png) | warehouse_s10 | 2 | 32 | 0.7 m | passed | [JSON](tasks/warehouse_s10_v02.json) |
| [warehouse_s10_v03](visualizations/warehouse_s10_v03.png) | warehouse_s10 | 3 | 32 | 0.7 m | passed | [JSON](tasks/warehouse_s10_v03.json) |
| [warehouse_s10_v04](visualizations/warehouse_s10_v04.png) | warehouse_s10 | 4 | 32 | 0.7 m | passed | [JSON](tasks/warehouse_s10_v04.json) |
| [warehouse_s10_v05](visualizations/warehouse_s10_v05.png) | warehouse_s10 | 5 | 32 | 0.7 m | passed | [JSON](tasks/warehouse_s10_v05.json) |
| [warehouse_s11_v01](visualizations/warehouse_s11_v01.png) | warehouse_s11 | 1 | 32 | 0.8 m | passed | [JSON](tasks/warehouse_s11_v01.json) |
| [warehouse_s11_v02](visualizations/warehouse_s11_v02.png) | warehouse_s11 | 2 | 32 | 0.8 m | passed | [JSON](tasks/warehouse_s11_v02.json) |
| [warehouse_s11_v03](visualizations/warehouse_s11_v03.png) | warehouse_s11 | 3 | 32 | 0.8 m | passed | [JSON](tasks/warehouse_s11_v03.json) |
| [warehouse_s11_v04](visualizations/warehouse_s11_v04.png) | warehouse_s11 | 4 | 32 | 0.8 m | passed | [JSON](tasks/warehouse_s11_v04.json) |
| [warehouse_s11_v05](visualizations/warehouse_s11_v05.png) | warehouse_s11 | 5 | 32 | 0.8 m | passed | [JSON](tasks/warehouse_s11_v05.json) |
| [warehouse_s12_v01](visualizations/warehouse_s12_v01.png) | warehouse_s12 | 1 | 32 | 0.8 m | passed | [JSON](tasks/warehouse_s12_v01.json) |
| [warehouse_s12_v02](visualizations/warehouse_s12_v02.png) | warehouse_s12 | 2 | 32 | 0.8 m | passed | [JSON](tasks/warehouse_s12_v02.json) |
| [warehouse_s12_v03](visualizations/warehouse_s12_v03.png) | warehouse_s12 | 3 | 32 | 0.8 m | passed | [JSON](tasks/warehouse_s12_v03.json) |
| [warehouse_s12_v04](visualizations/warehouse_s12_v04.png) | warehouse_s12 | 4 | 32 | 0.8 m | passed | [JSON](tasks/warehouse_s12_v04.json) |
| [warehouse_s12_v05](visualizations/warehouse_s12_v05.png) | warehouse_s12 | 5 | 32 | 0.8 m | passed | [JSON](tasks/warehouse_s12_v05.json) |
| [warehouse_s13_v01](visualizations/warehouse_s13_v01.png) | warehouse_s13 | 1 | 32 | 0.8 m | passed | [JSON](tasks/warehouse_s13_v01.json) |
| [warehouse_s13_v02](visualizations/warehouse_s13_v02.png) | warehouse_s13 | 2 | 32 | 0.8 m | passed | [JSON](tasks/warehouse_s13_v02.json) |
| [warehouse_s13_v03](visualizations/warehouse_s13_v03.png) | warehouse_s13 | 3 | 32 | 0.8 m | passed | [JSON](tasks/warehouse_s13_v03.json) |
| [warehouse_s13_v04](visualizations/warehouse_s13_v04.png) | warehouse_s13 | 4 | 32 | 0.8 m | passed | [JSON](tasks/warehouse_s13_v04.json) |
| [warehouse_s13_v05](visualizations/warehouse_s13_v05.png) | warehouse_s13 | 5 | 32 | 0.8 m | passed | [JSON](tasks/warehouse_s13_v05.json) |
| [warehouse_s14_v01](visualizations/warehouse_s14_v01.png) | warehouse_s14 | 1 | 32 | 0.8 m | passed | [JSON](tasks/warehouse_s14_v01.json) |
| [warehouse_s14_v02](visualizations/warehouse_s14_v02.png) | warehouse_s14 | 2 | 32 | 0.8 m | passed | [JSON](tasks/warehouse_s14_v02.json) |
| [warehouse_s14_v03](visualizations/warehouse_s14_v03.png) | warehouse_s14 | 3 | 32 | 0.8 m | passed | [JSON](tasks/warehouse_s14_v03.json) |
| [warehouse_s14_v04](visualizations/warehouse_s14_v04.png) | warehouse_s14 | 4 | 32 | 0.8 m | passed | [JSON](tasks/warehouse_s14_v04.json) |
| [warehouse_s14_v05](visualizations/warehouse_s14_v05.png) | warehouse_s14 | 5 | 32 | 0.8 m | passed | [JSON](tasks/warehouse_s14_v05.json) |
| [warehouse_s15_v01](visualizations/warehouse_s15_v01.png) | warehouse_s15 | 1 | 32 | 0.8 m | passed | [JSON](tasks/warehouse_s15_v01.json) |
| [warehouse_s15_v02](visualizations/warehouse_s15_v02.png) | warehouse_s15 | 2 | 32 | 0.8 m | passed | [JSON](tasks/warehouse_s15_v02.json) |
| [warehouse_s15_v03](visualizations/warehouse_s15_v03.png) | warehouse_s15 | 3 | 32 | 0.8 m | passed | [JSON](tasks/warehouse_s15_v03.json) |
| [warehouse_s15_v04](visualizations/warehouse_s15_v04.png) | warehouse_s15 | 4 | 32 | 0.8 m | passed | [JSON](tasks/warehouse_s15_v04.json) |
| [warehouse_s15_v05](visualizations/warehouse_s15_v05.png) | warehouse_s15 | 5 | 32 | 0.8 m | passed | [JSON](tasks/warehouse_s15_v05.json) |
| [warehouse_s16_v01](visualizations/warehouse_s16_v01.png) | warehouse_s16 | 1 | 32 | 0.9 m | passed | [JSON](tasks/warehouse_s16_v01.json) |
| [warehouse_s16_v02](visualizations/warehouse_s16_v02.png) | warehouse_s16 | 2 | 32 | 0.9 m | passed | [JSON](tasks/warehouse_s16_v02.json) |
| [warehouse_s16_v03](visualizations/warehouse_s16_v03.png) | warehouse_s16 | 3 | 32 | 0.9 m | passed | [JSON](tasks/warehouse_s16_v03.json) |
| [warehouse_s16_v04](visualizations/warehouse_s16_v04.png) | warehouse_s16 | 4 | 32 | 0.9 m | passed | [JSON](tasks/warehouse_s16_v04.json) |
| [warehouse_s16_v05](visualizations/warehouse_s16_v05.png) | warehouse_s16 | 5 | 32 | 0.9 m | passed | [JSON](tasks/warehouse_s16_v05.json) |
| [warehouse_s17_v01](visualizations/warehouse_s17_v01.png) | warehouse_s17 | 1 | 32 | 0.9 m | passed | [JSON](tasks/warehouse_s17_v01.json) |
| [warehouse_s17_v02](visualizations/warehouse_s17_v02.png) | warehouse_s17 | 2 | 32 | 0.9 m | passed | [JSON](tasks/warehouse_s17_v02.json) |
| [warehouse_s17_v03](visualizations/warehouse_s17_v03.png) | warehouse_s17 | 3 | 32 | 0.9 m | passed | [JSON](tasks/warehouse_s17_v03.json) |
| [warehouse_s17_v04](visualizations/warehouse_s17_v04.png) | warehouse_s17 | 4 | 32 | 0.9 m | passed | [JSON](tasks/warehouse_s17_v04.json) |
| [warehouse_s17_v05](visualizations/warehouse_s17_v05.png) | warehouse_s17 | 5 | 32 | 0.9 m | passed | [JSON](tasks/warehouse_s17_v05.json) |
| [warehouse_s18_v01](visualizations/warehouse_s18_v01.png) | warehouse_s18 | 1 | 32 | 0.9 m | passed | [JSON](tasks/warehouse_s18_v01.json) |
| [warehouse_s18_v02](visualizations/warehouse_s18_v02.png) | warehouse_s18 | 2 | 32 | 0.9 m | passed | [JSON](tasks/warehouse_s18_v02.json) |
| [warehouse_s18_v03](visualizations/warehouse_s18_v03.png) | warehouse_s18 | 3 | 32 | 0.9 m | passed | [JSON](tasks/warehouse_s18_v03.json) |
| [warehouse_s18_v04](visualizations/warehouse_s18_v04.png) | warehouse_s18 | 4 | 32 | 0.9 m | passed | [JSON](tasks/warehouse_s18_v04.json) |
| [warehouse_s18_v05](visualizations/warehouse_s18_v05.png) | warehouse_s18 | 5 | 32 | 0.9 m | passed | [JSON](tasks/warehouse_s18_v05.json) |
| [warehouse_s19_v01](visualizations/warehouse_s19_v01.png) | warehouse_s19 | 1 | 32 | 0.9 m | passed | [JSON](tasks/warehouse_s19_v01.json) |
| [warehouse_s19_v02](visualizations/warehouse_s19_v02.png) | warehouse_s19 | 2 | 32 | 0.9 m | passed | [JSON](tasks/warehouse_s19_v02.json) |
| [warehouse_s19_v03](visualizations/warehouse_s19_v03.png) | warehouse_s19 | 3 | 32 | 0.9 m | passed | [JSON](tasks/warehouse_s19_v03.json) |
| [warehouse_s19_v04](visualizations/warehouse_s19_v04.png) | warehouse_s19 | 4 | 32 | 0.9 m | passed | [JSON](tasks/warehouse_s19_v04.json) |
| [warehouse_s19_v05](visualizations/warehouse_s19_v05.png) | warehouse_s19 | 5 | 32 | 0.9 m | passed | [JSON](tasks/warehouse_s19_v05.json) |
| [warehouse_s20_v01](visualizations/warehouse_s20_v01.png) | warehouse_s20 | 1 | 32 | 0.9 m | passed | [JSON](tasks/warehouse_s20_v01.json) |
| [warehouse_s20_v02](visualizations/warehouse_s20_v02.png) | warehouse_s20 | 2 | 32 | 0.9 m | passed | [JSON](tasks/warehouse_s20_v02.json) |
| [warehouse_s20_v03](visualizations/warehouse_s20_v03.png) | warehouse_s20 | 3 | 32 | 0.9 m | passed | [JSON](tasks/warehouse_s20_v03.json) |
| [warehouse_s20_v04](visualizations/warehouse_s20_v04.png) | warehouse_s20 | 4 | 32 | 0.9 m | passed | [JSON](tasks/warehouse_s20_v04.json) |
| [warehouse_s20_v05](visualizations/warehouse_s20_v05.png) | warehouse_s20 | 5 | 32 | 0.9 m | passed | [JSON](tasks/warehouse_s20_v05.json) |
| [warehouse_s21_v01](visualizations/warehouse_s21_v01.png) | warehouse_s21 | 1 | 32 | 1.0 m | passed | [JSON](tasks/warehouse_s21_v01.json) |
| [warehouse_s21_v02](visualizations/warehouse_s21_v02.png) | warehouse_s21 | 2 | 32 | 1.0 m | passed | [JSON](tasks/warehouse_s21_v02.json) |
| [warehouse_s21_v03](visualizations/warehouse_s21_v03.png) | warehouse_s21 | 3 | 32 | 1.0 m | passed | [JSON](tasks/warehouse_s21_v03.json) |
| [warehouse_s21_v04](visualizations/warehouse_s21_v04.png) | warehouse_s21 | 4 | 32 | 1.0 m | passed | [JSON](tasks/warehouse_s21_v04.json) |
| [warehouse_s21_v05](visualizations/warehouse_s21_v05.png) | warehouse_s21 | 5 | 32 | 1.0 m | passed | [JSON](tasks/warehouse_s21_v05.json) |
| [warehouse_s22_v01](visualizations/warehouse_s22_v01.png) | warehouse_s22 | 1 | 32 | 1.0 m | passed | [JSON](tasks/warehouse_s22_v01.json) |
| [warehouse_s22_v02](visualizations/warehouse_s22_v02.png) | warehouse_s22 | 2 | 32 | 1.0 m | passed | [JSON](tasks/warehouse_s22_v02.json) |
| [warehouse_s22_v03](visualizations/warehouse_s22_v03.png) | warehouse_s22 | 3 | 32 | 1.0 m | passed | [JSON](tasks/warehouse_s22_v03.json) |
| [warehouse_s22_v04](visualizations/warehouse_s22_v04.png) | warehouse_s22 | 4 | 32 | 1.0 m | passed | [JSON](tasks/warehouse_s22_v04.json) |
| [warehouse_s22_v05](visualizations/warehouse_s22_v05.png) | warehouse_s22 | 5 | 32 | 1.0 m | passed | [JSON](tasks/warehouse_s22_v05.json) |
| [warehouse_s23_v01](visualizations/warehouse_s23_v01.png) | warehouse_s23 | 1 | 32 | 1.0 m | passed | [JSON](tasks/warehouse_s23_v01.json) |
| [warehouse_s23_v02](visualizations/warehouse_s23_v02.png) | warehouse_s23 | 2 | 32 | 1.0 m | passed | [JSON](tasks/warehouse_s23_v02.json) |
| [warehouse_s23_v03](visualizations/warehouse_s23_v03.png) | warehouse_s23 | 3 | 32 | 1.0 m | passed | [JSON](tasks/warehouse_s23_v03.json) |
| [warehouse_s23_v04](visualizations/warehouse_s23_v04.png) | warehouse_s23 | 4 | 32 | 1.0 m | passed | [JSON](tasks/warehouse_s23_v04.json) |
| [warehouse_s23_v05](visualizations/warehouse_s23_v05.png) | warehouse_s23 | 5 | 32 | 1.0 m | passed | [JSON](tasks/warehouse_s23_v05.json) |
| [warehouse_s24_v01](visualizations/warehouse_s24_v01.png) | warehouse_s24 | 1 | 32 | 1.0 m | passed | [JSON](tasks/warehouse_s24_v01.json) |
| [warehouse_s24_v02](visualizations/warehouse_s24_v02.png) | warehouse_s24 | 2 | 32 | 1.0 m | passed | [JSON](tasks/warehouse_s24_v02.json) |
| [warehouse_s24_v03](visualizations/warehouse_s24_v03.png) | warehouse_s24 | 3 | 32 | 1.0 m | passed | [JSON](tasks/warehouse_s24_v03.json) |
| [warehouse_s24_v04](visualizations/warehouse_s24_v04.png) | warehouse_s24 | 4 | 32 | 1.0 m | passed | [JSON](tasks/warehouse_s24_v04.json) |
| [warehouse_s24_v05](visualizations/warehouse_s24_v05.png) | warehouse_s24 | 5 | 32 | 1.0 m | passed | [JSON](tasks/warehouse_s24_v05.json) |
| [warehouse_s25_v01](visualizations/warehouse_s25_v01.png) | warehouse_s25 | 1 | 32 | 1.0 m | passed | [JSON](tasks/warehouse_s25_v01.json) |
| [warehouse_s25_v02](visualizations/warehouse_s25_v02.png) | warehouse_s25 | 2 | 32 | 1.0 m | passed | [JSON](tasks/warehouse_s25_v02.json) |
| [warehouse_s25_v03](visualizations/warehouse_s25_v03.png) | warehouse_s25 | 3 | 32 | 1.0 m | passed | [JSON](tasks/warehouse_s25_v03.json) |
| [warehouse_s25_v04](visualizations/warehouse_s25_v04.png) | warehouse_s25 | 4 | 32 | 1.0 m | passed | [JSON](tasks/warehouse_s25_v04.json) |
| [warehouse_s25_v05](visualizations/warehouse_s25_v05.png) | warehouse_s25 | 5 | 32 | 1.0 m | passed | [JSON](tasks/warehouse_s25_v05.json) |
| [warehouse2_s01_v01](visualizations/warehouse2_s01_v01.png) | warehouse2_s01 | 1 | 32 | 0.6 m | passed | [JSON](tasks/warehouse2_s01_v01.json) |
| [warehouse2_s01_v02](visualizations/warehouse2_s01_v02.png) | warehouse2_s01 | 2 | 32 | 0.6 m | passed | [JSON](tasks/warehouse2_s01_v02.json) |
| [warehouse2_s01_v03](visualizations/warehouse2_s01_v03.png) | warehouse2_s01 | 3 | 32 | 0.6 m | passed | [JSON](tasks/warehouse2_s01_v03.json) |
| [warehouse2_s01_v04](visualizations/warehouse2_s01_v04.png) | warehouse2_s01 | 4 | 32 | 0.6 m | passed | [JSON](tasks/warehouse2_s01_v04.json) |
| [warehouse2_s01_v05](visualizations/warehouse2_s01_v05.png) | warehouse2_s01 | 5 | 32 | 0.6 m | passed | [JSON](tasks/warehouse2_s01_v05.json) |
| [warehouse2_s02_v01](visualizations/warehouse2_s02_v01.png) | warehouse2_s02 | 1 | 32 | 0.6 m | passed | [JSON](tasks/warehouse2_s02_v01.json) |
| [warehouse2_s02_v02](visualizations/warehouse2_s02_v02.png) | warehouse2_s02 | 2 | 32 | 0.6 m | passed | [JSON](tasks/warehouse2_s02_v02.json) |
| [warehouse2_s02_v03](visualizations/warehouse2_s02_v03.png) | warehouse2_s02 | 3 | 32 | 0.6 m | passed | [JSON](tasks/warehouse2_s02_v03.json) |
| [warehouse2_s02_v04](visualizations/warehouse2_s02_v04.png) | warehouse2_s02 | 4 | 32 | 0.6 m | passed | [JSON](tasks/warehouse2_s02_v04.json) |
| [warehouse2_s02_v05](visualizations/warehouse2_s02_v05.png) | warehouse2_s02 | 5 | 32 | 0.6 m | passed | [JSON](tasks/warehouse2_s02_v05.json) |
| [warehouse2_s03_v01](visualizations/warehouse2_s03_v01.png) | warehouse2_s03 | 1 | 32 | 0.6 m | passed | [JSON](tasks/warehouse2_s03_v01.json) |
| [warehouse2_s03_v02](visualizations/warehouse2_s03_v02.png) | warehouse2_s03 | 2 | 32 | 0.6 m | passed | [JSON](tasks/warehouse2_s03_v02.json) |
| [warehouse2_s03_v03](visualizations/warehouse2_s03_v03.png) | warehouse2_s03 | 3 | 32 | 0.6 m | passed | [JSON](tasks/warehouse2_s03_v03.json) |
| [warehouse2_s03_v04](visualizations/warehouse2_s03_v04.png) | warehouse2_s03 | 4 | 32 | 0.6 m | passed | [JSON](tasks/warehouse2_s03_v04.json) |
| [warehouse2_s03_v05](visualizations/warehouse2_s03_v05.png) | warehouse2_s03 | 5 | 32 | 0.6 m | passed | [JSON](tasks/warehouse2_s03_v05.json) |
| [warehouse2_s04_v01](visualizations/warehouse2_s04_v01.png) | warehouse2_s04 | 1 | 32 | 0.6 m | passed | [JSON](tasks/warehouse2_s04_v01.json) |
| [warehouse2_s04_v02](visualizations/warehouse2_s04_v02.png) | warehouse2_s04 | 2 | 32 | 0.6 m | passed | [JSON](tasks/warehouse2_s04_v02.json) |
| [warehouse2_s04_v03](visualizations/warehouse2_s04_v03.png) | warehouse2_s04 | 3 | 32 | 0.6 m | passed | [JSON](tasks/warehouse2_s04_v03.json) |
| [warehouse2_s04_v04](visualizations/warehouse2_s04_v04.png) | warehouse2_s04 | 4 | 32 | 0.6 m | passed | [JSON](tasks/warehouse2_s04_v04.json) |
| [warehouse2_s04_v05](visualizations/warehouse2_s04_v05.png) | warehouse2_s04 | 5 | 32 | 0.6 m | passed | [JSON](tasks/warehouse2_s04_v05.json) |
| [warehouse2_s05_v01](visualizations/warehouse2_s05_v01.png) | warehouse2_s05 | 1 | 32 | 0.6 m | passed | [JSON](tasks/warehouse2_s05_v01.json) |
| [warehouse2_s05_v02](visualizations/warehouse2_s05_v02.png) | warehouse2_s05 | 2 | 32 | 0.6 m | passed | [JSON](tasks/warehouse2_s05_v02.json) |
| [warehouse2_s05_v03](visualizations/warehouse2_s05_v03.png) | warehouse2_s05 | 3 | 32 | 0.6 m | passed | [JSON](tasks/warehouse2_s05_v03.json) |
| [warehouse2_s05_v04](visualizations/warehouse2_s05_v04.png) | warehouse2_s05 | 4 | 32 | 0.6 m | passed | [JSON](tasks/warehouse2_s05_v04.json) |
| [warehouse2_s05_v05](visualizations/warehouse2_s05_v05.png) | warehouse2_s05 | 5 | 32 | 0.6 m | passed | [JSON](tasks/warehouse2_s05_v05.json) |
| [warehouse2_s06_v01](visualizations/warehouse2_s06_v01.png) | warehouse2_s06 | 1 | 32 | 0.7 m | passed | [JSON](tasks/warehouse2_s06_v01.json) |
| [warehouse2_s06_v02](visualizations/warehouse2_s06_v02.png) | warehouse2_s06 | 2 | 32 | 0.7 m | passed | [JSON](tasks/warehouse2_s06_v02.json) |
| [warehouse2_s06_v03](visualizations/warehouse2_s06_v03.png) | warehouse2_s06 | 3 | 32 | 0.7 m | passed | [JSON](tasks/warehouse2_s06_v03.json) |
| [warehouse2_s06_v04](visualizations/warehouse2_s06_v04.png) | warehouse2_s06 | 4 | 32 | 0.7 m | passed | [JSON](tasks/warehouse2_s06_v04.json) |
| [warehouse2_s06_v05](visualizations/warehouse2_s06_v05.png) | warehouse2_s06 | 5 | 32 | 0.7 m | passed | [JSON](tasks/warehouse2_s06_v05.json) |
| [warehouse2_s07_v01](visualizations/warehouse2_s07_v01.png) | warehouse2_s07 | 1 | 32 | 0.7 m | passed | [JSON](tasks/warehouse2_s07_v01.json) |
| [warehouse2_s07_v02](visualizations/warehouse2_s07_v02.png) | warehouse2_s07 | 2 | 32 | 0.7 m | passed | [JSON](tasks/warehouse2_s07_v02.json) |
| [warehouse2_s07_v03](visualizations/warehouse2_s07_v03.png) | warehouse2_s07 | 3 | 32 | 0.7 m | passed | [JSON](tasks/warehouse2_s07_v03.json) |
| [warehouse2_s07_v04](visualizations/warehouse2_s07_v04.png) | warehouse2_s07 | 4 | 32 | 0.7 m | passed | [JSON](tasks/warehouse2_s07_v04.json) |
| [warehouse2_s07_v05](visualizations/warehouse2_s07_v05.png) | warehouse2_s07 | 5 | 32 | 0.7 m | passed | [JSON](tasks/warehouse2_s07_v05.json) |
| [warehouse2_s08_v01](visualizations/warehouse2_s08_v01.png) | warehouse2_s08 | 1 | 32 | 0.7 m | passed | [JSON](tasks/warehouse2_s08_v01.json) |
| [warehouse2_s08_v02](visualizations/warehouse2_s08_v02.png) | warehouse2_s08 | 2 | 32 | 0.7 m | passed | [JSON](tasks/warehouse2_s08_v02.json) |
| [warehouse2_s08_v03](visualizations/warehouse2_s08_v03.png) | warehouse2_s08 | 3 | 32 | 0.7 m | passed | [JSON](tasks/warehouse2_s08_v03.json) |
| [warehouse2_s08_v04](visualizations/warehouse2_s08_v04.png) | warehouse2_s08 | 4 | 32 | 0.7 m | passed | [JSON](tasks/warehouse2_s08_v04.json) |
| [warehouse2_s08_v05](visualizations/warehouse2_s08_v05.png) | warehouse2_s08 | 5 | 32 | 0.7 m | passed | [JSON](tasks/warehouse2_s08_v05.json) |
| [warehouse2_s09_v01](visualizations/warehouse2_s09_v01.png) | warehouse2_s09 | 1 | 32 | 0.7 m | passed | [JSON](tasks/warehouse2_s09_v01.json) |
| [warehouse2_s09_v02](visualizations/warehouse2_s09_v02.png) | warehouse2_s09 | 2 | 32 | 0.7 m | passed | [JSON](tasks/warehouse2_s09_v02.json) |
| [warehouse2_s09_v03](visualizations/warehouse2_s09_v03.png) | warehouse2_s09 | 3 | 32 | 0.7 m | passed | [JSON](tasks/warehouse2_s09_v03.json) |
| [warehouse2_s09_v04](visualizations/warehouse2_s09_v04.png) | warehouse2_s09 | 4 | 32 | 0.7 m | passed | [JSON](tasks/warehouse2_s09_v04.json) |
| [warehouse2_s09_v05](visualizations/warehouse2_s09_v05.png) | warehouse2_s09 | 5 | 32 | 0.7 m | passed | [JSON](tasks/warehouse2_s09_v05.json) |
| [warehouse2_s10_v01](visualizations/warehouse2_s10_v01.png) | warehouse2_s10 | 1 | 32 | 0.7 m | passed | [JSON](tasks/warehouse2_s10_v01.json) |
| [warehouse2_s10_v02](visualizations/warehouse2_s10_v02.png) | warehouse2_s10 | 2 | 32 | 0.7 m | passed | [JSON](tasks/warehouse2_s10_v02.json) |
| [warehouse2_s10_v03](visualizations/warehouse2_s10_v03.png) | warehouse2_s10 | 3 | 32 | 0.7 m | passed | [JSON](tasks/warehouse2_s10_v03.json) |
| [warehouse2_s10_v04](visualizations/warehouse2_s10_v04.png) | warehouse2_s10 | 4 | 32 | 0.7 m | passed | [JSON](tasks/warehouse2_s10_v04.json) |
| [warehouse2_s10_v05](visualizations/warehouse2_s10_v05.png) | warehouse2_s10 | 5 | 32 | 0.7 m | passed | [JSON](tasks/warehouse2_s10_v05.json) |
| [warehouse2_s11_v01](visualizations/warehouse2_s11_v01.png) | warehouse2_s11 | 1 | 32 | 0.8 m | passed | [JSON](tasks/warehouse2_s11_v01.json) |
| [warehouse2_s11_v02](visualizations/warehouse2_s11_v02.png) | warehouse2_s11 | 2 | 32 | 0.8 m | passed | [JSON](tasks/warehouse2_s11_v02.json) |
| [warehouse2_s11_v03](visualizations/warehouse2_s11_v03.png) | warehouse2_s11 | 3 | 32 | 0.8 m | passed | [JSON](tasks/warehouse2_s11_v03.json) |
| [warehouse2_s11_v04](visualizations/warehouse2_s11_v04.png) | warehouse2_s11 | 4 | 32 | 0.8 m | passed | [JSON](tasks/warehouse2_s11_v04.json) |
| [warehouse2_s11_v05](visualizations/warehouse2_s11_v05.png) | warehouse2_s11 | 5 | 32 | 0.8 m | passed | [JSON](tasks/warehouse2_s11_v05.json) |
| [warehouse2_s12_v01](visualizations/warehouse2_s12_v01.png) | warehouse2_s12 | 1 | 32 | 0.8 m | passed | [JSON](tasks/warehouse2_s12_v01.json) |
| [warehouse2_s12_v02](visualizations/warehouse2_s12_v02.png) | warehouse2_s12 | 2 | 32 | 0.8 m | passed | [JSON](tasks/warehouse2_s12_v02.json) |
| [warehouse2_s12_v03](visualizations/warehouse2_s12_v03.png) | warehouse2_s12 | 3 | 32 | 0.8 m | passed | [JSON](tasks/warehouse2_s12_v03.json) |
| [warehouse2_s12_v04](visualizations/warehouse2_s12_v04.png) | warehouse2_s12 | 4 | 32 | 0.8 m | passed | [JSON](tasks/warehouse2_s12_v04.json) |
| [warehouse2_s12_v05](visualizations/warehouse2_s12_v05.png) | warehouse2_s12 | 5 | 32 | 0.8 m | passed | [JSON](tasks/warehouse2_s12_v05.json) |
| [warehouse2_s13_v01](visualizations/warehouse2_s13_v01.png) | warehouse2_s13 | 1 | 32 | 0.8 m | passed | [JSON](tasks/warehouse2_s13_v01.json) |
| [warehouse2_s13_v02](visualizations/warehouse2_s13_v02.png) | warehouse2_s13 | 2 | 32 | 0.8 m | passed | [JSON](tasks/warehouse2_s13_v02.json) |
| [warehouse2_s13_v03](visualizations/warehouse2_s13_v03.png) | warehouse2_s13 | 3 | 32 | 0.8 m | passed | [JSON](tasks/warehouse2_s13_v03.json) |
| [warehouse2_s13_v04](visualizations/warehouse2_s13_v04.png) | warehouse2_s13 | 4 | 32 | 0.8 m | passed | [JSON](tasks/warehouse2_s13_v04.json) |
| [warehouse2_s13_v05](visualizations/warehouse2_s13_v05.png) | warehouse2_s13 | 5 | 32 | 0.8 m | passed | [JSON](tasks/warehouse2_s13_v05.json) |
| [warehouse2_s14_v01](visualizations/warehouse2_s14_v01.png) | warehouse2_s14 | 1 | 32 | 0.8 m | passed | [JSON](tasks/warehouse2_s14_v01.json) |
| [warehouse2_s14_v02](visualizations/warehouse2_s14_v02.png) | warehouse2_s14 | 2 | 32 | 0.8 m | passed | [JSON](tasks/warehouse2_s14_v02.json) |
| [warehouse2_s14_v03](visualizations/warehouse2_s14_v03.png) | warehouse2_s14 | 3 | 32 | 0.8 m | passed | [JSON](tasks/warehouse2_s14_v03.json) |
| [warehouse2_s14_v04](visualizations/warehouse2_s14_v04.png) | warehouse2_s14 | 4 | 32 | 0.8 m | passed | [JSON](tasks/warehouse2_s14_v04.json) |
| [warehouse2_s14_v05](visualizations/warehouse2_s14_v05.png) | warehouse2_s14 | 5 | 32 | 0.8 m | passed | [JSON](tasks/warehouse2_s14_v05.json) |
| [warehouse2_s15_v01](visualizations/warehouse2_s15_v01.png) | warehouse2_s15 | 1 | 32 | 0.8 m | passed | [JSON](tasks/warehouse2_s15_v01.json) |
| [warehouse2_s15_v02](visualizations/warehouse2_s15_v02.png) | warehouse2_s15 | 2 | 32 | 0.8 m | passed | [JSON](tasks/warehouse2_s15_v02.json) |
| [warehouse2_s15_v03](visualizations/warehouse2_s15_v03.png) | warehouse2_s15 | 3 | 32 | 0.8 m | passed | [JSON](tasks/warehouse2_s15_v03.json) |
| [warehouse2_s15_v04](visualizations/warehouse2_s15_v04.png) | warehouse2_s15 | 4 | 32 | 0.8 m | passed | [JSON](tasks/warehouse2_s15_v04.json) |
| [warehouse2_s15_v05](visualizations/warehouse2_s15_v05.png) | warehouse2_s15 | 5 | 32 | 0.8 m | passed | [JSON](tasks/warehouse2_s15_v05.json) |
| [warehouse2_s16_v01](visualizations/warehouse2_s16_v01.png) | warehouse2_s16 | 1 | 32 | 0.9 m | passed | [JSON](tasks/warehouse2_s16_v01.json) |
| [warehouse2_s16_v02](visualizations/warehouse2_s16_v02.png) | warehouse2_s16 | 2 | 32 | 0.9 m | passed | [JSON](tasks/warehouse2_s16_v02.json) |
| [warehouse2_s16_v03](visualizations/warehouse2_s16_v03.png) | warehouse2_s16 | 3 | 32 | 0.9 m | passed | [JSON](tasks/warehouse2_s16_v03.json) |
| [warehouse2_s16_v04](visualizations/warehouse2_s16_v04.png) | warehouse2_s16 | 4 | 32 | 0.9 m | passed | [JSON](tasks/warehouse2_s16_v04.json) |
| [warehouse2_s16_v05](visualizations/warehouse2_s16_v05.png) | warehouse2_s16 | 5 | 32 | 0.9 m | passed | [JSON](tasks/warehouse2_s16_v05.json) |
| [warehouse2_s17_v01](visualizations/warehouse2_s17_v01.png) | warehouse2_s17 | 1 | 32 | 0.9 m | passed | [JSON](tasks/warehouse2_s17_v01.json) |
| [warehouse2_s17_v02](visualizations/warehouse2_s17_v02.png) | warehouse2_s17 | 2 | 32 | 0.9 m | passed | [JSON](tasks/warehouse2_s17_v02.json) |
| [warehouse2_s17_v03](visualizations/warehouse2_s17_v03.png) | warehouse2_s17 | 3 | 32 | 0.9 m | passed | [JSON](tasks/warehouse2_s17_v03.json) |
| [warehouse2_s17_v04](visualizations/warehouse2_s17_v04.png) | warehouse2_s17 | 4 | 32 | 0.9 m | passed | [JSON](tasks/warehouse2_s17_v04.json) |
| [warehouse2_s17_v05](visualizations/warehouse2_s17_v05.png) | warehouse2_s17 | 5 | 32 | 0.9 m | passed | [JSON](tasks/warehouse2_s17_v05.json) |
| [warehouse2_s18_v01](visualizations/warehouse2_s18_v01.png) | warehouse2_s18 | 1 | 32 | 0.9 m | passed | [JSON](tasks/warehouse2_s18_v01.json) |
| [warehouse2_s18_v02](visualizations/warehouse2_s18_v02.png) | warehouse2_s18 | 2 | 32 | 0.9 m | passed | [JSON](tasks/warehouse2_s18_v02.json) |
| [warehouse2_s18_v03](visualizations/warehouse2_s18_v03.png) | warehouse2_s18 | 3 | 32 | 0.9 m | passed | [JSON](tasks/warehouse2_s18_v03.json) |
| [warehouse2_s18_v04](visualizations/warehouse2_s18_v04.png) | warehouse2_s18 | 4 | 32 | 0.9 m | passed | [JSON](tasks/warehouse2_s18_v04.json) |
| [warehouse2_s18_v05](visualizations/warehouse2_s18_v05.png) | warehouse2_s18 | 5 | 32 | 0.9 m | passed | [JSON](tasks/warehouse2_s18_v05.json) |
| [warehouse2_s19_v01](visualizations/warehouse2_s19_v01.png) | warehouse2_s19 | 1 | 32 | 0.9 m | passed | [JSON](tasks/warehouse2_s19_v01.json) |
| [warehouse2_s19_v02](visualizations/warehouse2_s19_v02.png) | warehouse2_s19 | 2 | 32 | 0.9 m | passed | [JSON](tasks/warehouse2_s19_v02.json) |
| [warehouse2_s19_v03](visualizations/warehouse2_s19_v03.png) | warehouse2_s19 | 3 | 32 | 0.9 m | passed | [JSON](tasks/warehouse2_s19_v03.json) |
| [warehouse2_s19_v04](visualizations/warehouse2_s19_v04.png) | warehouse2_s19 | 4 | 32 | 0.9 m | passed | [JSON](tasks/warehouse2_s19_v04.json) |
| [warehouse2_s19_v05](visualizations/warehouse2_s19_v05.png) | warehouse2_s19 | 5 | 32 | 0.9 m | passed | [JSON](tasks/warehouse2_s19_v05.json) |
| [warehouse2_s20_v01](visualizations/warehouse2_s20_v01.png) | warehouse2_s20 | 1 | 32 | 0.9 m | passed | [JSON](tasks/warehouse2_s20_v01.json) |
| [warehouse2_s20_v02](visualizations/warehouse2_s20_v02.png) | warehouse2_s20 | 2 | 32 | 0.9 m | passed | [JSON](tasks/warehouse2_s20_v02.json) |
| [warehouse2_s20_v03](visualizations/warehouse2_s20_v03.png) | warehouse2_s20 | 3 | 32 | 0.9 m | passed | [JSON](tasks/warehouse2_s20_v03.json) |
| [warehouse2_s20_v04](visualizations/warehouse2_s20_v04.png) | warehouse2_s20 | 4 | 32 | 0.9 m | passed | [JSON](tasks/warehouse2_s20_v04.json) |
| [warehouse2_s20_v05](visualizations/warehouse2_s20_v05.png) | warehouse2_s20 | 5 | 32 | 0.9 m | passed | [JSON](tasks/warehouse2_s20_v05.json) |
| [warehouse2_s21_v01](visualizations/warehouse2_s21_v01.png) | warehouse2_s21 | 1 | 32 | 1.0 m | passed | [JSON](tasks/warehouse2_s21_v01.json) |
| [warehouse2_s21_v02](visualizations/warehouse2_s21_v02.png) | warehouse2_s21 | 2 | 32 | 1.0 m | passed | [JSON](tasks/warehouse2_s21_v02.json) |
| [warehouse2_s21_v03](visualizations/warehouse2_s21_v03.png) | warehouse2_s21 | 3 | 32 | 1.0 m | passed | [JSON](tasks/warehouse2_s21_v03.json) |
| [warehouse2_s21_v04](visualizations/warehouse2_s21_v04.png) | warehouse2_s21 | 4 | 32 | 1.0 m | passed | [JSON](tasks/warehouse2_s21_v04.json) |
| [warehouse2_s21_v05](visualizations/warehouse2_s21_v05.png) | warehouse2_s21 | 5 | 32 | 1.0 m | passed | [JSON](tasks/warehouse2_s21_v05.json) |
| [warehouse2_s22_v01](visualizations/warehouse2_s22_v01.png) | warehouse2_s22 | 1 | 32 | 1.0 m | passed | [JSON](tasks/warehouse2_s22_v01.json) |
| [warehouse2_s22_v02](visualizations/warehouse2_s22_v02.png) | warehouse2_s22 | 2 | 32 | 1.0 m | passed | [JSON](tasks/warehouse2_s22_v02.json) |
| [warehouse2_s22_v03](visualizations/warehouse2_s22_v03.png) | warehouse2_s22 | 3 | 32 | 1.0 m | passed | [JSON](tasks/warehouse2_s22_v03.json) |
| [warehouse2_s22_v04](visualizations/warehouse2_s22_v04.png) | warehouse2_s22 | 4 | 32 | 1.0 m | passed | [JSON](tasks/warehouse2_s22_v04.json) |
| [warehouse2_s22_v05](visualizations/warehouse2_s22_v05.png) | warehouse2_s22 | 5 | 32 | 1.0 m | passed | [JSON](tasks/warehouse2_s22_v05.json) |
| [warehouse2_s23_v01](visualizations/warehouse2_s23_v01.png) | warehouse2_s23 | 1 | 32 | 1.0 m | passed | [JSON](tasks/warehouse2_s23_v01.json) |
| [warehouse2_s23_v02](visualizations/warehouse2_s23_v02.png) | warehouse2_s23 | 2 | 32 | 1.0 m | passed | [JSON](tasks/warehouse2_s23_v02.json) |
| [warehouse2_s23_v03](visualizations/warehouse2_s23_v03.png) | warehouse2_s23 | 3 | 32 | 1.0 m | passed | [JSON](tasks/warehouse2_s23_v03.json) |
| [warehouse2_s23_v04](visualizations/warehouse2_s23_v04.png) | warehouse2_s23 | 4 | 32 | 1.0 m | passed | [JSON](tasks/warehouse2_s23_v04.json) |
| [warehouse2_s23_v05](visualizations/warehouse2_s23_v05.png) | warehouse2_s23 | 5 | 32 | 1.0 m | passed | [JSON](tasks/warehouse2_s23_v05.json) |
| [warehouse2_s24_v01](visualizations/warehouse2_s24_v01.png) | warehouse2_s24 | 1 | 32 | 1.0 m | passed | [JSON](tasks/warehouse2_s24_v01.json) |
| [warehouse2_s24_v02](visualizations/warehouse2_s24_v02.png) | warehouse2_s24 | 2 | 32 | 1.0 m | passed | [JSON](tasks/warehouse2_s24_v02.json) |
| [warehouse2_s24_v03](visualizations/warehouse2_s24_v03.png) | warehouse2_s24 | 3 | 32 | 1.0 m | passed | [JSON](tasks/warehouse2_s24_v03.json) |
| [warehouse2_s24_v04](visualizations/warehouse2_s24_v04.png) | warehouse2_s24 | 4 | 32 | 1.0 m | passed | [JSON](tasks/warehouse2_s24_v04.json) |
| [warehouse2_s24_v05](visualizations/warehouse2_s24_v05.png) | warehouse2_s24 | 5 | 32 | 1.0 m | passed | [JSON](tasks/warehouse2_s24_v05.json) |
| [warehouse2_s25_v01](visualizations/warehouse2_s25_v01.png) | warehouse2_s25 | 1 | 32 | 1.0 m | passed | [JSON](tasks/warehouse2_s25_v01.json) |
| [warehouse2_s25_v02](visualizations/warehouse2_s25_v02.png) | warehouse2_s25 | 2 | 32 | 1.0 m | passed | [JSON](tasks/warehouse2_s25_v02.json) |
| [warehouse2_s25_v03](visualizations/warehouse2_s25_v03.png) | warehouse2_s25 | 3 | 32 | 1.0 m | passed | [JSON](tasks/warehouse2_s25_v03.json) |
| [warehouse2_s25_v04](visualizations/warehouse2_s25_v04.png) | warehouse2_s25 | 4 | 32 | 1.0 m | passed | [JSON](tasks/warehouse2_s25_v04.json) |
| [warehouse2_s25_v05](visualizations/warehouse2_s25_v05.png) | warehouse2_s25 | 5 | 32 | 1.0 m | passed | [JSON](tasks/warehouse2_s25_v05.json) |

All 250 tasks additionally have independently replayed constructive joint schedules: [joint validation](joint_validation_report.json). These certificates check bounded controls and every stationary robot, and never supply evaluated anchors or controller fallbacks. Six endpoint variants were repaired using geometry and joint certification before the final comparison; [repair history](revisions/joint_feasibility_repairs.json) preserves their originals and replacement hashes.


No rollout success or deadlock rate is claimed by generation. The paired `evaluate-warehouse` command compares routing methods with a shared MPPI/ORCA profile and joint braking-safety layer. Native fallback acceleration bounds are corrected, and the measured execution audit remains active.
