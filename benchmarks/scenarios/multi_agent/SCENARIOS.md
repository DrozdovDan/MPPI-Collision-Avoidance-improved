# Saved multi-agent benchmark scenarios

50 tasks, 760 robot assignments. Generation: 308.68 s (includes feasibility checks; cached graph reuse recorded separately).

All maps retain their original coordinates and bounds. Transform: identity, scale 1, no crop. Distances are native map units; a meters-per-unit calibration is not provided by the imported data.

Difficulty is a design intention. No scenario is called an observed deadlock based on its label or timeout.

| Scenario | Source map | Robots (target) | Model | Size classes | Category | Intended difficulty | Transform | Validation | Alternative-route witnesses |
| --- | --- | ---: | --- | --- | --- | --- | --- | --- | ---: |
| [parking1_01](tasks/parking1_01.json) | parking1 | 4 (4) | car_like | small | random_low | ordinary | identity | PASS | 0 |
| [parking1_02](tasks/parking1_02.json) | parking1 | 8 (8) | car_like | medium | random_moderate | ordinary | identity | PASS | 0 |
| [parking1_03](tasks/parking1_03.json) | parking1 | 16 (16) | car_like | large | random_dense | ordinary | identity | PASS | 0 |
| [parking1_04](tasks/parking1_04.json) | parking1 | 8 (8) | car_like | small | opposing_bottleneck | challenging | identity | PASS | 0 |
| [parking1_05](tasks/parking1_05.json) | parking1 | 16 (16) | car_like | small | opposing_dense | challenging | identity | PASS | 0 |
| [parking1_06](tasks/parking1_06.json) | parking1 | 12 (12) | car_like | small | intersection | challenging | identity | PASS | 0 |
| [parking1_07](tasks/parking1_07.json) | parking1 | 12 (12) | car_like | small | symmetric_swap | deadlock_prone | identity | PASS | 0 |
| [parking1_08](tasks/parking1_08.json) | parking1 | 20 (20) | car_like | small | competing_bottlenecks | deadlock_prone | identity | PASS | 0 |
| [parking1_09](tasks/parking1_09.json) | parking1 | 24 (24) | car_like | small/medium/large | heterogeneous_congestion | deadlock_prone | identity | PASS | 0 |
| [parking1_10](tasks/parking1_10.json) | parking1 | 32 (32) | car_like | small/medium/large | maximum_stress | deadlock_prone | identity | PASS | 0 |
| [parking2_01](tasks/parking2_01.json) | parking2 | 4 (4) | car_like | small | random_low | ordinary | identity | PASS | 0 |
| [parking2_02](tasks/parking2_02.json) | parking2 | 8 (8) | car_like | medium | random_moderate | ordinary | identity | PASS | 0 |
| [parking2_03](tasks/parking2_03.json) | parking2 | 16 (16) | car_like | large | random_dense | ordinary | identity | PASS | 0 |
| [parking2_04](tasks/parking2_04.json) | parking2 | 8 (8) | car_like | small | opposing_bottleneck | challenging | identity | PASS | 0 |
| [parking2_05](tasks/parking2_05.json) | parking2 | 16 (16) | car_like | small | opposing_dense | challenging | identity | PASS | 0 |
| [parking2_06](tasks/parking2_06.json) | parking2 | 12 (12) | car_like | small | intersection | challenging | identity | PASS | 0 |
| [parking2_07](tasks/parking2_07.json) | parking2 | 12 (12) | car_like | small | symmetric_swap | deadlock_prone | identity | PASS | 0 |
| [parking2_08](tasks/parking2_08.json) | parking2 | 20 (20) | car_like | small | competing_bottlenecks | deadlock_prone | identity | PASS | 0 |
| [parking2_09](tasks/parking2_09.json) | parking2 | 24 (24) | car_like | small/medium/large | heterogeneous_congestion | deadlock_prone | identity | PASS | 0 |
| [parking2_10](tasks/parking2_10.json) | parking2 | 32 (32) | car_like | small/medium/large | maximum_stress | deadlock_prone | identity | PASS | 0 |
| [parking3_01](tasks/parking3_01.json) | parking3 | 4 (4) | car_like | small | random_low | ordinary | identity | PASS | 0 |
| [parking3_02](tasks/parking3_02.json) | parking3 | 8 (8) | car_like | medium | random_moderate | ordinary | identity | PASS | 0 |
| [parking3_03](tasks/parking3_03.json) | parking3 | 16 (16) | car_like | large | random_dense | ordinary | identity | PASS | 0 |
| [parking3_04](tasks/parking3_04.json) | parking3 | 8 (8) | car_like | small | opposing_bottleneck | challenging | identity | PASS | 0 |
| [parking3_05](tasks/parking3_05.json) | parking3 | 16 (16) | car_like | small | opposing_dense | challenging | identity | PASS | 0 |
| [parking3_06](tasks/parking3_06.json) | parking3 | 12 (12) | car_like | small | intersection | challenging | identity | PASS | 0 |
| [parking3_07](tasks/parking3_07.json) | parking3 | 12 (12) | car_like | small | symmetric_swap | deadlock_prone | identity | PASS | 0 |
| [parking3_08](tasks/parking3_08.json) | parking3 | 20 (20) | car_like | small | competing_bottlenecks | deadlock_prone | identity | PASS | 0 |
| [parking3_09](tasks/parking3_09.json) | parking3 | 24 (24) | car_like | small/medium/large | heterogeneous_congestion | deadlock_prone | identity | PASS | 0 |
| [parking3_10](tasks/parking3_10.json) | parking3 | 32 (32) | car_like | small/medium/large | maximum_stress | deadlock_prone | identity | PASS | 0 |
| [warehouse_01](tasks/warehouse_01.json) | warehouse | 4 (4) | diff_drive | small | random_low | ordinary | identity | PASS | 0 |
| [warehouse_02](tasks/warehouse_02.json) | warehouse | 8 (8) | diff_drive | medium | random_moderate | ordinary | identity | PASS | 0 |
| [warehouse_03](tasks/warehouse_03.json) | warehouse | 16 (16) | diff_drive | large | random_dense | ordinary | identity | PASS | 0 |
| [warehouse_04](tasks/warehouse_04.json) | warehouse | 8 (8) | diff_drive | small | opposing_bottleneck | challenging | identity | PASS | 0 |
| [warehouse_05](tasks/warehouse_05.json) | warehouse | 16 (16) | diff_drive | small | opposing_dense | challenging | identity | PASS | 0 |
| [warehouse_06](tasks/warehouse_06.json) | warehouse | 12 (12) | diff_drive | small | intersection | challenging | identity | PASS | 0 |
| [warehouse_07](tasks/warehouse_07.json) | warehouse | 12 (12) | diff_drive | small | symmetric_swap | deadlock_prone | identity | PASS | 0 |
| [warehouse_08](tasks/warehouse_08.json) | warehouse | 20 (20) | diff_drive | small | competing_bottlenecks | deadlock_prone | identity | PASS | 20 |
| [warehouse_09](tasks/warehouse_09.json) | warehouse | 24 (24) | diff_drive | small/medium/large | heterogeneous_congestion | deadlock_prone | identity | PASS | 0 |
| [warehouse_10](tasks/warehouse_10.json) | warehouse | 32 (32) | diff_drive | small/medium/large | maximum_stress | deadlock_prone | identity | PASS | 0 |
| [warehouse2_01](tasks/warehouse2_01.json) | warehouse2 | 4 (4) | diff_drive | small | random_low | ordinary | identity | PASS | 0 |
| [warehouse2_02](tasks/warehouse2_02.json) | warehouse2 | 8 (8) | diff_drive | medium | random_moderate | ordinary | identity | PASS | 0 |
| [warehouse2_03](tasks/warehouse2_03.json) | warehouse2 | 16 (16) | diff_drive | large | random_dense | ordinary | identity | PASS | 0 |
| [warehouse2_04](tasks/warehouse2_04.json) | warehouse2 | 8 (8) | diff_drive | small | opposing_bottleneck | challenging | identity | PASS | 0 |
| [warehouse2_05](tasks/warehouse2_05.json) | warehouse2 | 16 (16) | diff_drive | small | opposing_dense | challenging | identity | PASS | 0 |
| [warehouse2_06](tasks/warehouse2_06.json) | warehouse2 | 12 (12) | diff_drive | small | intersection | challenging | identity | PASS | 0 |
| [warehouse2_07](tasks/warehouse2_07.json) | warehouse2 | 12 (12) | diff_drive | small | symmetric_swap | deadlock_prone | identity | PASS | 0 |
| [warehouse2_08](tasks/warehouse2_08.json) | warehouse2 | 20 (20) | diff_drive | small | competing_bottlenecks | deadlock_prone | identity | PASS | 20 |
| [warehouse2_09](tasks/warehouse2_09.json) | warehouse2 | 24 (24) | diff_drive | small/medium/large | heterogeneous_congestion | deadlock_prone | identity | PASS | 0 |
| [warehouse2_10](tasks/warehouse2_10.json) | warehouse2 | 32 (32) | diff_drive | small/medium/large | maximum_stress | deadlock_prone | identity | PASS | 0 |

## Size classes

Car dimensions retain length/width = 2.2 and wheelbase/length = 0.65. Radii derive from usable interior corridor widths, excluding tiny terminal tapers. These are map-scale dimensions, not claimed SI vehicle measurements.

| Map | Class | Footprint | Length × width | Planning radius | Wheelbase |
| --- | --- | --- | --- | ---: | ---: |
| parking1 | small | rectangle | 0.490 × 0.223 | 0.269 | 0.3186371151568939 |
| parking1 | medium | rectangle | 0.674 × 0.306 | 0.370 | 0.4381260333407292 |
| parking1 | large | rectangle | 1.593 × 0.724 | 0.875 | 1.0355706242599052 |
| parking2 | small | rectangle | 0.437 × 0.198 | 0.240 | 0.28382289285369366 |
| parking2 | medium | rectangle | 0.600 × 0.273 | 0.330 | 0.3902564776738287 |
| parking2 | large | rectangle | 1.419 × 0.645 | 0.779 | 0.9224244017745042 |
| parking3 | small | rectangle | 0.371 × 0.169 | 0.204 | 0.24101616031618806 |
| parking3 | medium | rectangle | 0.510 × 0.232 | 0.280 | 0.33139722043475855 |
| parking3 | large | rectangle | 1.205 × 0.548 | 0.662 | 0.7833025210276112 |
| warehouse | small | disk | 0.302 × 0.302 | 0.151 | — |
| warehouse | medium | disk | 0.536 × 0.536 | 0.268 | — |
| warehouse | large | disk | 0.921 × 0.921 | 0.461 | — |
| warehouse2 | small | disk | 0.245 × 0.245 | 0.123 | — |
| warehouse2 | medium | disk | 0.436 × 0.436 | 0.218 | — |
| warehouse2 | large | disk | 0.749 × 0.749 | 0.375 | — |

## Geometry limitations

- **parking1_08**: No alternative topological corridor route exists in the smallest planning-circle configuration space (zero holes); task 08 uses multiple competing resources without claiming route alternatives.
- **parking2_08**: No alternative topological corridor route exists in the smallest planning-circle configuration space (zero holes); task 08 uses multiple competing resources without claiming route alternatives.
- **parking3_08**: No alternative topological corridor route exists in the smallest planning-circle configuration space (zero holes); task 08 uses multiple competing resources without claiming route alternatives.

The category-08 parking adaptations are explicit: the car-sized configuration spaces have zero holes, so the suite does not invent alternative topological routes or modify obstacles to create them. Warehouse category-08 tasks store independently checked geometry-graph alternatives. These checks never call the proposed K-route planner.

## Actual pilot observations

Short pilots are separate from task selection; they are not full-budget benchmark rankings.

| Scenario | Planner | Completion | Operational deadlocks | Collisions | Timeout | Control limits valid | Observation (s) |
| --- | --- | ---: | ---: | ---: | --- | --- | ---: |
| parking1_01 | k_routes | 0.25 | 0 | 0 | True | True | 30.0 |
| parking1_01 | rrt | 0.25 | 0 | 0 | True | True | 30.0 |
| parking1_09 | k_routes | 0.00 | 3 | 0 | True | True | 30.0 |
| parking1_09 | rrt | 0.08 | 2 | 0 | True | True | 30.0 |
| warehouse2_01 | k_routes | 0.00 | 0 | 0 | True | False | 30.0 |
| warehouse2_01 | rrt | 0.00 | 0 | 0 | True | True | 30.0 |
| warehouse2_02 | rrt | 0.12 | 0 | 0 | True | True | 30.0 |
| warehouse2_03 | rrt | 0.19 | 0 | 0 | True | False | 30.0 |
| warehouse2_04 | rrt | 0.00 | 0 | 0 | True | False | 30.0 |
| warehouse2_05 | rrt | 0.25 | 0 | 0 | True | False | 30.0 |
| warehouse2_06 | rrt | 0.17 | 0 | 0 | True | False | 30.0 |
| warehouse2_07 | rrt | 0.00 | 0 | 0 | True | False | 30.0 |
| warehouse2_08 | rrt | 0.30 | 0 | 0 | True | False | 30.0 |
| warehouse2_09 | k_routes | 0.12 | 1 | 0 | True | False | 30.0 |
| warehouse2_09 | rrt | 0.00 | 0 | not observed | False | not simulated | 0.0 |
| warehouse2_10 | rrt | 0.12 | 0 | 0 | True | False | 30.0 |

Operational deadlock detected in valid-control runs: parking1_09.

Additional detections in control-limit-violating traces (excluded from valid-control outcomes): warehouse2_09.

Runs marked with invalid control limits retain their original traces and are flagged separately. The current differential-drive controller can reset yaw rate immediately in its emergency fallback, exceeding the requested angular-acceleration cap. The benchmark detects this and does not modify the controller or smooth the evidence.
