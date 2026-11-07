# Warehouse benchmark with disappearance at target

The experiment repeats all five existing anchor methods on all 250 frozen tasks: RRT-Connect; K routes with uniform sampling; K routes with bottleneck-width sampling; graph-edge cost sampling with λ=1, τ=1; and graph-edge cost sampling with λ=2, τ=10. There are 1,250 new executions, 125 tasks per map per method, 32 same-size agents per task, and five endpoint variants per map/scenario. The diameter range remains 0.6–1.0 m.

A disk disappears at the first completed timestep at which its center is inside its assigned goal tolerance and both physically applied speed and yaw-rate commands have magnitude at most 1e-8. The original goal-capture rule and bounded deceleration are retained. The disk is present throughout the incoming integration interval, which is collision-checked. Starting at the next interval it is excluded from neighboring-state observations, ORCA, the joint braking certificate, collision checks and persistent-obstruction evidence. No other robot's state is changed. No goal snapping, teleportation or instantaneous speed reset is used.

Traces retain finite terminal snapshots to preserve agent IDs and array shapes. An explicit boolean `active[time, agent]` mask distinguishes snapshots from physical disks. A snapshot is not an obstacle. Arrival and makespan include physical deceleration and are independently recovered from the first mask transition.

Every method reuses its exact audited original `paths.json` and `route_choices.json`, including the original random choices. Routes are neither regenerated nor resampled. This preserves the requested K, returned candidate sets, certified right-side/shared-center geometry, all edge-cost definitions, temperatures and seeds. The baseline's global planning timings are carried over as reference timings; they are not newly timed planner executions.

Normal MPPI/ORCA, the 1.2 m/s speed bound, 0.1 s timestep, 128 rollouts, 31-state horizon, 15 s anchor-yield policy, original differential-drive equations and acceleration bounds remain fixed. The synchronous joint braking safety filter retains its previous certificate and rotating priorities; only presence affects which disks it checks. The original parked experiment and its source files are preserved.

The terminal monitor retains its original 30 s deadlock and 45 s stall windows. Removed robots cannot form blocking edges or count as parked-goal blockers. Success requires all 32 disks to arrive and disappear. Operational deadlocks require persistent blocking cycles covering all unfinished agents. Unclassified stalls and other terminal failures remain distinct; a stall is certified as deadlock only by the existing visible-join certificate applied to the still-present subset. No absent robot may serve as a blocker in that certificate.

All new trajectories are audited for monotone presence, removal exactly at the first settled goal visit, physically bounded commands and differential-drive equations, continuous swept-disk collisions while present, settled arrivals, makespans and terminal evidence. The old and new motion must match bit for bit up to and including the first disappearance. This verifies that the changed target policy is the first behavioral difference.

Metrics report all-32 fleet success, individual-agent completion and mean ± sample standard deviation of successful makespan with its sample count. Failed makespans stay null. Each scenario is averaged over its five endpoint variants; size and traffic-pattern averages are provided separately for each map. Park/disappear makespan differences use only the same tasks completed under both policies; unconditional successful means can cover different cohorts.

Run or resume with:

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 PYTHONPATH=benchmarks \
.venv/bin/python -m multi_agent.warehouse_disappear_benchmark --workers 20
```

Outputs are checkpointed under `benchmarks/results/warehouse_disappear/`; the persistent process log is `benchmarks/results/warehouse_disappear.log`. Completed runs are reused only when their full evaluation identity matches. The final audit publishes `benchmarks/scenarios/warehouse_uniform/disappear_evaluation/REPORT.md` and adds a separate target-policy comparison to the existing Review and results document.

The memory-efficient orchestration wrapper accepts explicit simulation and
audit worker counts, for example:

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 PYTHONPATH=benchmarks MALLOC_ARENA_MAX=2 \
.venv/bin/python -m multi_agent.warehouse_disappear_low_memory --workers 20 --audit-workers 8
```

The wrapper reads intermediate result files one at a time, retains only fields
needed for aggregation, and returns unused allocator pages after preparation and
aggregation. It calls the original simulation functions and final auditor. On
the 644-run restart checkpoint, all 11 intermediate output files were byte
identical to the original aggregation. Simulation sources, binary hashes,
experiment identity, seeds, controller settings and numerical precision remain
unchanged. Verification and restart receipts are stored in the raw result root.

All 1,250 executions are complete. [Standalone disappearance results and
figures, with stay-at-target comparisons](WAREHOUSE_DISAPPEAR_RESULTS.md) are
available. Rebuild the audit and report from saved traces without rerunning
simulations with `PYTHONPATH=benchmarks .venv/bin/python -m
multi_agent.warehouse_disappear_report --workers 20`.
