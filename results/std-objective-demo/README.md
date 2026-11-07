This five-agent Circle demo used the earlier standard-deviation LP objective,
equivalent to `variance_objective=false, post_step_running_cost=true` in the
current implementation. The run
seed was 2025. It succeeded in 59.7 simulation seconds, with minimum center
separation 1.4973 m. `run.json` records the original source hash, configuration,
and measurements; `trajectory.npz` contains states and controls. This single
successful demo does not establish the paper's statistical results.
