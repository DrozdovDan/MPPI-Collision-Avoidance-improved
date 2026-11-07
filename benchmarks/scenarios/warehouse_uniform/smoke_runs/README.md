# Runner compatibility smoke check

These are actual saved executions of `warehouse_s20_v01`, containing 32
identical agents of diameter 0.9 m. They use the same controller profile,
planner-selection seed 42 and controller seed 42. The simulation budget was
only **10 steps / 1 second** with the existing pilot controller profile.
This checks the saved-task interface; it is not a completion-rate comparison.

| Method | Observed result |
| --- | --- |
| RRT | Planned all 32 routes and executed all 10 steps; zero measured body collisions; all measured equations and control limits passed. The short budget ends as `timeout`, with no completed agents. |
| K routes | `global_planning_failure`: no keep-right route returned for `r15`. No simulation ran; collision and control-limit metrics are inapplicable. |

For `r15`, the native report marks `physical_route_reachable=true`. It explored
24 physical candidates and rejected 14 complete lane routes with
`complete_lane_path_self_intersection`. This is an observed limitation of the
existing bounded keep-right search, not an infeasible start–goal assignment:
the saved task has a swept-disk, bounded-control individual witness, and RRT
also found its route. No task was removed or regenerated because of this result.

- [Summary](summary.json)
- [RRT measured run](warehouse_s20_v01__rrt__p42__c42__pilot/run.json)
- [K-route failure](warehouse_s20_v01__k_routes__p42__c42__pilot/run.json)
- [Native K-route diagnostics](warehouse_s20_v01__k_routes__p42__c42__pilot/agents.routes.json)

Raw states, controls, paths, query diagnostics and aggregate exports are retained
in this directory. The native controller and routing algorithms were unchanged.
