# Warehouse edge-cost sampling benchmark

Run the same 250 frozen tasks as the RRT-Connect, uniform K-route and bottleneck-width K-route baselines: 125 tasks per warehouse map, 32 equal-sized agents per task, 25 scenarios per map and five start–goal variants per scenario. Only the distribution used to select one fixed anchor path per agent changes.

The parameters are **λ = 1 and temperature τ = 1**. For each agent independently, use its original returned K-route candidate set and the stored graph edges:

```text
W_max = max(w_e over all edges in this agent's candidate routes)
q_e = clamp(1 - (w_e - 2*r)/(2*W_max), 0, 1)
L_k = sum(l_e over edges in route k)
C_k = L_k + lambda * sum(l_e*q_e over edges in route k)
p_k = exp(-(C_k - min(C))/tau) / sum_j exp(-(C_j - min(C))/tau)
```

`l_e` is the stored graph `edge.length`; `w_e` is `edge.min_width`; `r` is physical robot radius without an added safety margin. Repeated edges count repeatedly. As explicitly requested, connectors are excluded and a partial-edge entry is charged the full stored edge length. The 680 candidates with no graph edges have L = C = 0; they are retained without an additional penalty. When all candidates have no graph edges, probabilities are uniform over zero costs and W_max is recorded as null because no q needs evaluating. These costs affect selection only: every executed anchor still includes the full start-to-goal geometry and its certified keep-right/shared-center placement.

Use the same uniform variate as the bottleneck policy for each agent, followed by inverse-CDF sampling with the new probabilities. Those variates come from the seed-42 uniform candidate index and an independent within-bin jitter, so the marginal distribution remains the specified softmax. Each agent samples once; its path remains fixed throughout the simulation. Both selection and controller seeds remain 42. Requested K is 5; use all actually returned candidates, without generating a new set or selecting tasks based on outcomes.

The controller remains normal MPPI/ORCA with a 1.2 m/s forward speed bound, dt = 0.1 s, 128 rollouts, 31 horizon states, unchanged acceleration/yaw limits and original dynamics. Preserve the shared joint braking safety filter, 15-second anchor waiting policy, goal parking and online termination monitor. Check controller profiles and source/binary hashes against the historical baselines. No uncertainty-aware controller is used.

Fleet success requires all 32 robots settled at their goals, valid measured dynamics and motion limits, and zero continuous footprint collisions. Agent completion counts every finished agent. Makespan is simulated time until the complete fleet finishes, with failed makespans left undefined. Average the five endpoint variants within each scenario, and provide map, size and traffic-pattern aggregates. Also compare makespans on the common tasks where both methods succeed. Preserve raw terminal labels; a stationary stall can be additionally certified as operational deadlock only with the existing independent visible-join certificate. Uncertified stalls remain failures with their original label.

Validate every selected anchor before simulation, then independently replay every saved trajectory to check sampling, unchanged path geometry, dynamics, motion limits, continuous collisions, arrivals and terminal evidence. Recheck the 750 historical execution checksums. All four-method tables are published only after all 250 new tasks complete and pass the audit.

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 PYTHONPATH=benchmarks \
  .venv/bin/python -m multi_agent.warehouse_cost_benchmark --workers 20
```

Raw new outputs are stored under `benchmarks/results/warehouse_edge_cost/`. The combined comparison is published under `benchmarks/scenarios/warehouse_uniform/edge_cost_evaluation/`, with the new row added to the existing results table and Review.
