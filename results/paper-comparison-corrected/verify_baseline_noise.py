"""Audit actual legacy C++ inputs and actuator noise without changing its policy."""
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import sys
from unittest.mock import patch

import numpy as np

output = Path(__file__).resolve().parent
repo = output.parents[1]
sys.path.insert(0, str(repo))
from reproduction.config import Config
from reproduction.controller import LegacyController
from reproduction.scenarios import circle
from reproduction.simulation import simulate

observations, own_states = [], []


class NativeInputSpy:
    def __init__(self, engine):
        self.engine = engine

    def next_step(self, state, neighbors, radii, goal):
        own_states.append(np.array(state, copy=True))
        observations.append(np.array(neighbors, copy=True))
        return self.engine.next_step(state, neighbors, radii, goal)


class AuditedLegacyController(LegacyController):
    def __init__(self, *args):
        super().__init__(*args)
        self.engine = NativeInputSpy(self.engine)


class DrawSpy:
    def __init__(self, generator):
        self.generator = generator
        self.draws = []

    def normal(self, *args, **kwargs):
        value = self.generator.normal(*args, **kwargs)
        self.draws.append(np.array(value, copy=True))
        return value

    def __getattr__(self, name):
        return getattr(self.generator, name)


original_rng = np.random.default_rng
streams = []


def audited_rng(seed):
    stream = DrawSpy(original_rng(seed))
    streams.append(stream)
    return stream


config = Config(max_steps=40)
states, goals = circle(5, config.circle_radius)
with patch("reproduction.simulation.LegacyController", AuditedLegacyController), \
     patch("reproduction.simulation.np.random.default_rng", audited_rng):
    result, trace = simulate(config, states, goals, "legacy-mppi-orca", 2025, True)

assert len(streams) == 3
assert len(observations) == result["steps"] * len(states)
noise = []
for call, (own, observed) in enumerate(zip(own_states, observations)):
    step, agent = divmod(call, len(states))
    ids = np.delete(np.arange(len(states)), agent)
    np.testing.assert_array_equal(own, trace["states"][step, agent])
    truth = trace["states"][step, ids][:, [0, 1, 3, 4]]
    generated = np.hstack(streams[0].draws[2*call:2*call+2])
    np.testing.assert_allclose(observed-truth, generated, rtol=1e-12, atol=1e-12)
    noise.append(generated)
execution_noise = np.array(streams[1].draws)
np.testing.assert_array_equal(trace["executed"], np.clip(
    trace["commands"]+execution_noise, config.lower, config.upper))
noise = np.concatenate(noise)
assert np.all(np.std(noise, axis=0) > 0)
assert np.all(np.std(execution_noise.reshape(-1,2), axis=0) > 0)

arrays = output/"baseline-noise-audit.npz"
np.savez_compressed(arrays, **trace, native_observations=np.array(observations),
                    observation_noise=noise, execution_noise=execution_noise)
sha = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
summary = dict(
    method="legacy-mppi-orca", probe_environment_seed=2025,
    probe_steps=result["steps"], native_controller_calls=len(observations),
    configured_position_std_m=config.position_std,
    configured_velocity_std_mps=config.observed_velocity_std,
    configured_execution_std=config.execution_std,
    measured_observation_std_per_coordinate=np.std(noise, axis=0, ddof=1).tolist(),
    measured_execution_draw_std=np.std(execution_noise.reshape(-1,2),axis=0,ddof=1).tolist(),
    noisy_values_reach_actual_native_controller=True,
    ego_state_is_exact=True,
    executed_control_equals_clipped_command_plus_noise=True,
    observations_and_execution_share_simulator_with_uncertainty_method=True,
    probe_config=asdict(config),
    legacy_native_sha256=sha(repo/"build/release/mppi_ca_cpp.so"),
    legacy_controller_sha256=sha(repo/"build/release/libcontroller_library.so"),
    simulator_sha256=sha(repo/"reproduction/simulation.py"),
    adapter_sha256=sha(repo/"reproduction/controller.py"),
    audit_script_sha256=sha(Path(__file__)), data_sha256=sha(arrays),
    note="Independent 40-step input/noise probe; excluded from benchmark outcome counts.")
(output/"baseline-noise-audit.json").write_text(json.dumps(summary,indent=2)+"\n")
print(json.dumps({k:summary[k] for k in ["native_controller_calls",
    "measured_observation_std_per_coordinate", "measured_execution_draw_std",
    "noisy_values_reach_actual_native_controller",
    "executed_control_equals_clipped_command_plus_noise"]},indent=2))
