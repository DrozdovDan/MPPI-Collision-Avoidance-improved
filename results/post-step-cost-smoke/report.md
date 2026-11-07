# Measured numeric reproduction

Completed 42 / 42 requested runs.

These are local measurements, not the authors' original results. Unpublished controller parameters and generated instances are recorded in manifest.json.

| Scenario | Method | Agents | Runs | Success | 95% Wilson interval | Collisions | Makespan (successful runs) |
|---|---|---:|---:|---:|---:|---:|---:|
| circle | deterministic-mppi | 2 | 2 | 100.0% | 34.2–100.0% | 0.0% | 20.05 s |
| circle | deterministic-mppi | 5 | 2 | 100.0% | 34.2–100.0% | 0.0% | 29.15 s |
| circle | deterministic-mppi | 10 | 2 | 100.0% | 34.2–100.0% | 0.0% | 40.60 s |
| circle | legacy-mppi-orca | 2 | 2 | 100.0% | 34.2–100.0% | 0.0% | 13.30 s |
| circle | legacy-mppi-orca | 5 | 2 | 100.0% | 34.2–100.0% | 0.0% | 15.35 s |
| circle | legacy-mppi-orca | 10 | 2 | 100.0% | 34.2–100.0% | 0.0% | 20.95 s |
| circle | uncertainty-mppi | 2 | 2 | 100.0% | 34.2–100.0% | 0.0% | 19.45 s |
| circle | uncertainty-mppi | 5 | 2 | 50.0% | 9.5–90.5% | 0.0% | 56.90 s |
| circle | uncertainty-mppi | 10 | 2 | 50.0% | 9.5–90.5% | 0.0% | 99.90 s |
| random | deterministic-mppi | 5 | 4 | 100.0% | 51.0–100.0% | 0.0% | 24.35 s |
| random | deterministic-mppi | 10 | 4 | 100.0% | 51.0–100.0% | 0.0% | 40.53 s |
| random | legacy-mppi-orca | 5 | 4 | 100.0% | 51.0–100.0% | 0.0% | 17.88 s |
| random | legacy-mppi-orca | 10 | 4 | 100.0% | 51.0–100.0% | 0.0% | 25.85 s |
| random | uncertainty-mppi | 5 | 4 | 100.0% | 51.0–100.0% | 0.0% | 24.82 s |
| random | uncertainty-mppi | 10 | 4 | 75.0% | 30.1–95.4% | 25.0% | 45.70 s |

Wilson intervals are descriptive per-run intervals; Random repeats share instances and are not independent samples of the environment distribution.

## Published reference (Table I; not measured here)

| Algorithm | Circle success / collision | Random success / collision |
|---|---:|---:|
| ORCA-DD | 74.3% / 0% | 98% / 0% |
| B-UAVC | 99.3% / 0.7% | 85.4% / 0% |
| MPPI-ORCA | 85% / 15% | 96.8% / 3.2% |
| Ours (paper) | 100% / 0% | 100% / 0% |

Source: https://arxiv.org/html/2507.20293v2#S6.T1

The deterministic-mppi method is an ablation of this reconstruction, not the original MPPI-ORCA implementation. legacy-mppi-orca runs the existing implementation.

ORCA-DD, B-UAVC, and ROS2/Gazebo have not been reproduced by this numeric runner.

Total infeasible sampling decisions: 175. A zero-translation fallback is not a probabilistic safety certificate.
