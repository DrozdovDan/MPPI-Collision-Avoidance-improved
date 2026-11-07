# Noise verification for standard MPPI-ORCA

Both methods run in the same noisy simulator. Standard MPPI-ORCA receives noisy
neighbor positions and velocities, then its command receives execution noise
before actuator clipping. Its controller keeps the original deterministic ORCA
safety model. The uncertainty-aware reconstruction additionally models noise
in its constraints and predictions.

| Quantity | Configured standard deviation | Measured in the baseline probe |
|---|---:|---:|
| Neighbor position x / y | 0.1 / 0.1 m | 0.1000 / 0.0970 m |
| Neighbor velocity x / y | 0.1 / 0.1 m/s | 0.0966 / 0.0980 m/s |
| Linear execution noise | 0.1 m/s | 0.1017 m/s |
| Angular execution noise | 0.2 rad/s | 0.1990 rad/s |

The independent probe runs five real legacy C++ controllers for 40 steps,
recording 200 native controller calls. It delegates every call to the original
engine, without replacing its commands. The assertions verify that:

* The actual C++ inputs equal true neighbor states plus the recorded Gaussian
  position and velocity errors.
* Each controller receives its exact ego state.
* Executed controls equal `clip(command + execution_noise, lower, upper)`.
* The audited baseline module and controller-library hashes match this full
  comparison's manifest.

Measured execution standard deviations refer to the Gaussian draws before
clipping. Actuator saturation changes the distribution of the applied error.
The 40-step probe is excluded from the 1,280 benchmark outcome counts.

The full comparison uses fresh runs of both methods with the same geometry,
environment seeds and configured noise. The manifest's `environment_noise`
field explicitly lists both methods and all standard deviations. A simulator
regression also checks that the two methods receive identical observation and
execution disturbances when given identical commands.

Noise was also enabled in the previous full comparison. The current simulator
and baseline adapter match the archived sources from before the sampler fix
byte for byte. The old and new baseline binary hashes and physical noise
settings match as well. The sampler correction changed the new controller;
it did not enable previously absent baseline noise. The
[historical verification](previous-baseline-verification.json) records these checks.

Artifacts: [verification summary](baseline-noise-audit.json),
[captured inputs, draws and trajectories](baseline-noise-audit.npz),
[probe source](verify_baseline_noise.py), and
[full experiment manifest](manifest.json).

The noise values and physical clipping follow
[Section VI-A of the paper](https://arxiv.org/html/2507.20293v2#S6.SS1).
The simulator implementation is in
[simulation.py](../../reproduction/simulation.py), with the unmodified baseline
adapter in [controller.py](../../reproduction/controller.py).
