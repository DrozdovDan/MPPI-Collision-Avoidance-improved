These 42 runs used the standard-deviation objective before the literal variance
objective was added. Their source and binary hashes identify that earlier
implementation. Its behavior corresponds to `variance_objective=false` and
`post_step_running_cost=true` in the current code (see
`reproduction/configs/std-objective.json`). The native sampler
is replayable within the same compiler/standard-library build; the legacy
controller uses its original global PID/time-seeded RNG.

The uncertainty-aware reconstruction succeeded in 11/14 smoke runs, with two
timeouts and one collision. These failures are retained. See `report.md` and
`runs.jsonl` for the complete per-count results and configurations. The sample
does not constitute the full paper protocol.
