This five-agent Circle demo used the literal covariance objective and
`post_step_running_cost=true`. It succeeded in 59.7 simulation seconds with
minimum center separation 1.4973 m. Configuration, seed, measurements and source
hash are in `run.json`; states and controls are in `trajectory.npz`. Replay this
variant with `reproduction/configs/post-step-cost.json`. The current default
uses running costs at `x_t` before integration, as written in Eq. (4).
