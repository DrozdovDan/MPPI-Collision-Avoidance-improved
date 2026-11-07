These 42 runs used the literal covariance objective, but evaluated running costs
after integration instead of at `x_t` as written in Eq. (4). Replay that variant
with `--config reproduction/configs/post-step-cost.json`. Source/binary hashes
identify the earlier build. With the default bounds and confidence level, the
literal covariance objective selected the same distributions as the preceding
standard-deviation LP variant, giving identical primary/ablation trajectories.
The original controller has its own non-replayable global RNG.

The primary reconstruction succeeded in 11/14 runs, with two timeouts and one
collision. All failures and individual outcomes are retained in `runs.jsonl`.
This is a smoke suite, not the complete paper protocol. The current default
uses the literal pre-integration cost convention.
