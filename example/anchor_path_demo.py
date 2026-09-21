"""Single-agent example using a handmade polygonal anchor path."""
import matplotlib.pyplot as plt
import numpy as np

import dyn_models
from params_names import *
import utils


# This route is deliberately authored by hand; no global path planner is used.
anchor_path = [
    (-4.0, 0.0),
    (-2.0, -2.3),
    (2.0, -2.3),
    (4.0, 0.0),
]
obstacles = [[(-1.5, -1.5), (1.5, -1.5), (1.5, 1.5), (-1.5, 1.5)]]
start_states = np.array([[-4.0, 0.0, -0.7, 0.0, 0.0]])
goal_states = np.array([[4.0, 0.0, 0.7, 0.0, 0.0]])

agent_params = [{
    AGENT_SIZE: 0.3,
    VISIBILITY_RADIUS: 30.0,
    DIFF_DRIVE_V_MAX: 1.0,
    DIFF_DRIVE_V_MIN: -0.01,
    DIFF_DRIVE_W_MAX: 2.0,
    DIFF_DRIVE_W_MIN: -2.0,
}]
alg_params = {
    SAMPLE_BATCH_SIZE: 700,
    SAMPLE_TIME_STEPS: 20,
    TIME_STEP: 0.1,
    INIT_VALUE: np.zeros(2),
    CONTROL_MEAN: np.zeros(2),
    CONTROL_COV: np.diag([1.0, 6.0]),
    RUNNING_COST_WEIGH: 395.0,
    TERMINAL_WEIGHT: 1600.0,
    SOFTMAX_TEMPERATURE: 0.25,
    MA_COST_EN: False,
    MA_DIST_WEIGHT: 750.0,
    MA_COLLISION_WEIGHT: 1e7,
    MA_DIST_THREASHOLD: 3.5,
    USE_CAR_LIKE_DYN: False,
    K_ALPHA: -2.96,
    RAD_EPS: 0.05,
    VELOCITY_COST_WEIGHT: 0.5,
    TERMINAL_LOOKAHEAD_DISTANCE: 2.0,
    MA_EN_CLOSE_TO_GOAL_DIST: 1.0,
    MA_EN_CLOSE_TO_GOAL_TS: 3,
    ORCA_TIME_BOUNDARY: 1.0,
    ORCA_RESPONSIBILITY_FACTOR: 0.5,
    MA_EXCLUDE_COLLISION_TRAJ: True,
    VELOCITY_COST_CLOSE_TO_GOAL_DIST: 0.4,
    ANCHOR_PATH_WEIGHT: 1600.0,
}
experiment_params = {
    EXP_TIME_STEP: 0.1,
    XY_GOAL_TOLERANCE: 0.3,
    MAX_EXP_STEPS: 200,
}

success, collisions, steps, duration, states = utils.launch_mppi(
    start_states,
    goal_states,
    1,
    agent_params,
    alg_params,
    experiment_params,
    [dyn_models.DiffDriveDyn()],
    seed=7,
    obstacles=obstacles,
    anchor_paths=[anchor_path],
)
print(f"success={success}, collisions={collisions}, steps={steps}, time={duration:.2f}s")

states = np.asarray(states)[:, 0, :2]
path = np.asarray(anchor_path)
fig, ax = plt.subplots()
ax.add_patch(plt.Polygon(obstacles[0], color="0.7"))
ax.plot(path[:, 0], path[:, 1], "--", label="anchor path")
ax.plot(states[:, 0], states[:, 1], label="executed trajectory")
ax.scatter(*start_states[0, :2], label="start")
ax.scatter(*goal_states[0, :2], marker="*", s=120, label="goal")
ax.set_aspect("equal")
ax.grid()
ax.legend()
ax.set_title(f"success={success}, collisions={collisions}, steps={steps}, time={duration:.2f}s")
plt.show()
