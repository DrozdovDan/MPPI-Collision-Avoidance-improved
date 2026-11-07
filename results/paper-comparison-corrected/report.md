# Table I and Figure 3 — local measurements

Completed **1280/1280 runs**, with 140 Circle and 500 Random runs per method.

| Algorithm | Circle SR | Circle % Col. | Random SR | Random % Col. |
|---|---:|---:|---:|---:|
| MPPI-ORCA | 98.6% | 1.4% | 98% | 2% |
| UA-MPPI (reconstruction) | 59.3% | 0% | 48% | 6.2% |

SR is the fraction of runs where every robot reached its goal without a collision. % Col. is the fraction of all runs that terminated due to collision. Timeouts count as failures.

![Figure 3: makespan by number of agents](figure_3.png)

Makespan is the mean number of simulation time steps among successful runs at each agent count. One step is 0.1 s. A point is included only when strictly more than 50% of that method's runs at that count succeeded; missing points do not mean zero makespan.

Circle: diameter 12 m, every integer agent count from 2 through 15, ten repetitions. Random: 20 × 20 m, counts 5/10/15/20/25, ten instances per count, ten repetitions each. Both methods use the same saved instance geometry, environment noise seeds, 1000-step limit, 0.4 m goal tolerance, physical limits and noise model.

**Noise is enabled for both methods**, including standard MPPI-ORCA: position noise has standard deviation 0.1 m per coordinate; observed velocity noise has standard deviation 0.1 m/s per coordinate; execution standard deviations [v, w] are [0.1, 0.2] [m/s, rad/s], followed by actuator clipping. Standard MPPI-ORCA uses its original deterministic safety model on these noisy inputs.

The [native-input noise audit](NOISE.md) verifies these channels on 200 calls
to the actual baseline C++ controller. Its module/library hashes match the
full experiment. The [completed-data validation](validation.json) verifies all
1,280 measurements, unchanged source/binaries and a zero-work resume.

The scenario grid, table columns, makespan units, line colors/markers and success inclusion rule follow [Table I and Figure 3 in the paper](https://arxiv.org/html/2507.20293v2#S6.F3). The original instances and complete controller settings were not published; these measurements use this repository's documented reconstruction and newly generated saved instances. MPPI-ORCA uses the original PID/time-seeded global controller RNG; its internal samples cannot be replayed from the environment seed.

Exports: `table_i.{md,csv,tex,png,pdf,svg}`, `figure_3.{png,pdf,svg}`, `makespan_circle.{png,pdf,svg}`, `makespan_random.{png,pdf,svg}`, `per_count_metrics.csv`, `runs.jsonl`, `manifest.json`, and `instances.npz`.
