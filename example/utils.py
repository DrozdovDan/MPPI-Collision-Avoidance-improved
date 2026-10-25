import numpy as np
import os
import time
import sys
from pathlib import Path
from scipy.spatial import KDTree

from params_names import *

sys.path.append(str(Path(__file__).resolve().parents[1] / "build/release"))
import mppi_ca_cpp



def circle_instance(circ_r, agents_num):
    start_states = np.zeros((agents_num, 5), dtype=np.float64)
    goal_states = np.zeros((agents_num, 5), dtype=np.float64)
    ang_steps = np.linspace(0.0, 2*np.pi, agents_num+1)
    
    for i, ang in enumerate(ang_steps):
        if i == agents_num:
            break

        st_x = circ_r * np.cos(ang)
        st_y = circ_r * np.sin(ang)

        g_x = circ_r * np.cos(ang + np.pi)
        g_y = circ_r * np.sin(ang + np.pi)

        dx = g_x - st_x
        dy = g_y - st_y

        st_theta = np.arctan2(dy, dx)
        g_theta = st_theta

        start_states[i] = np.array([st_x, st_y, st_theta, 0.0, 0.0])
        goal_states[i] = np.array([g_x, g_y, g_theta, 0.0, 0.0])
        
    return start_states, goal_states


def get_neighbors(a_pos_id, all_pos, r):
    tree = KDTree(all_pos)
    neighbors_ids = tree.query_ball_point(all_pos[a_pos_id], r)
    if a_pos_id in neighbors_ids:
        neighbors_ids.remove(a_pos_id)
    return neighbors_ids


def check_collisions(agents_states, agent_sizes):
    collisions = 0
    for a1 in range(len(agents_states)):
        for a2 in range(len(agents_states)):
            if a1 == a2:
                continue
            if np.linalg.norm(agents_states[a1, 0:2] - agents_states[a2, 0:2]) < (agent_sizes[a1] + agent_sizes[a2]):
                collisions += 1
    return collisions / 2 

def launch_mppi(
    start_states,
    goal_states,
    agents_num,
    agents_params,
    alg_params,
    exp_params,
    dyn_models,
    save_logs=True,
    save_traj_lines=False,
    seed=None,
    positions_std=0.0,
    direction_std=0.0,
    velocities_std=0.0,
    obstacles=None,
    anchor_paths=None,
    park_at_goal=False,
    on_step=None,
):
    """Run existing decentralized controllers.

    ``alg_params`` may be one shared dictionary or one dictionary per robot
    (e.g. heterogeneous wheelbases). Optional ``on_step(step, states, controls,
    controller_seconds)`` observes completed steps; returning False ends an
    unfinished run. Defaults retain the existing simulation behavior.
    """

    if seed is None:
        seed = int.from_bytes(os.urandom(4), byteorder="little")
    np.random.seed(seed)

    if anchor_paths is not None and len(anchor_paths) != agents_num:
        raise ValueError("anchor_paths must contain one path per agent")

    # if type(exp_params) is not dict:
    #     exp_params = exp_params.__dict__
    max_sim_steps = exp_params[MAX_EXP_STEPS]
    dt = exp_params[EXP_TIME_STEP]
    goal_xy_offset = exp_params[XY_GOAL_TOLERANCE]

    agents_sizes = []
    agents_r_vis = []
    profiles = [alg_params] * agents_num if isinstance(alg_params, dict) else list(alg_params)
    if len(profiles) != agents_num:
        raise ValueError("Provide one algorithm profile per agent")
    car_like = profiles[0].get(USE_CAR_LIKE_DYN, False)
    if any(p.get(USE_CAR_LIKE_DYN, False) != car_like for p in profiles):
        raise ValueError("One simulation must use a consistent state representation")
    controllers = [mppi_ca_cpp.Controller() for a_id in range(agents_num)]
    # Controller construction seeds the shared native sampler. Apply the
    # requested experiment seed afterwards so notebook runs are reproducible.
    mppi_ca_cpp.set_random_seed(seed)

    for a_id in range(agents_num):
        # if type(agents_params[a_id]) is not dict:
        #     agents_params[a_id] = agents_params[a_id].__dict__

        agp = mppi_ca_cpp.convert_agent_params(agents_params[a_id])
        alp = mppi_ca_cpp.convert_alg_params(profiles[a_id])
        controllers[a_id].set_agent_params(agp)
        controllers[a_id].set_alg_params(alp)
        if car_like:
            dyn_models[a_id].configure_limits(agents_params[a_id])
        controllers[a_id].set_obstacles([] if obstacles is None else obstacles)
        controllers[a_id].set_anchor_path(
            [] if anchor_paths is None else anchor_paths[a_id]
        )
        agents_sizes.append(agents_params[a_id][AGENT_SIZE])
        agents_r_vis.append(agents_params[a_id][VISIBILITY_RADIUS])

    agents_states = start_states.copy()
    parked = np.zeros(agents_num, dtype=bool)
    last_controls = np.zeros((agents_num, 2))
    new_states = agents_states.copy()
    agents_sizes = np.array(agents_sizes)

    log_state = [start_states]
    log_traj = []
    collisions = 0
    steps = 0

    t_start = time.time()
    for iter in range(max_sim_steps):
        previous_states = agents_states.copy()
        step_traj = []
        step_controls, step_controller_seconds = [], []
        steps += 1

        for a_id in range(agents_num):

            curr_state = new_states[a_id].copy() if car_like else new_states[a_id, 0:5].copy()
            goal_state = goal_states[a_id, 0:3].copy()
            neighbours_id = get_neighbors(
                a_id, agents_states[:, 0:2], agents_r_vis[a_id]
            )
            neighbours_states = agents_states[neighbours_id]
            if car_like:
                # ORCA still consumes Cartesian velocity, not speed/steering.
                neighbours_states = np.c_[neighbours_states[:, :2],
                    neighbours_states[:, 3] * np.cos(neighbours_states[:, 2]),
                    neighbours_states[:, 3] * np.sin(neighbours_states[:, 2])]
            else:
                neighbours_states = neighbours_states[:, [0, 1, 3, 4]]
            neighbours_sizes = agents_sizes[neighbours_id]

            noized_curr_state = curr_state.copy()
            noized_neighbours_states = neighbours_states.copy()


            if positions_std != 0.0:
                noized_curr_state[0:2] = np.random.normal(
                    curr_state[0:2], positions_std, 2
                )
                for i in range(len(neighbours_states)):
                    noized_neighbours_states[i, 0:2] = np.random.normal(
                        neighbours_states[i, 0:2], positions_std, 2
                    )

            if direction_std != 0.0:
                noized_curr_state[2] = np.random.normal(curr_state[2], direction_std, 1)

            if velocities_std != 0.0:
                if car_like:
                    noized_curr_state[3] = np.clip(np.random.normal(curr_state[3], velocities_std),
                        agents_params[a_id][DIFF_DRIVE_V_MIN], agents_params[a_id][DIFF_DRIVE_V_MAX])
                else:
                    noized_curr_state[3:] = np.random.normal(curr_state[3:], velocities_std, 2)
                for i in range(len(neighbours_states)):
                    noized_neighbours_states[i, 2:] = np.random.normal(
                        neighbours_states[i, 2:], velocities_std, 2
                    )

            control_start = time.perf_counter() if on_step is not None else 0.
            if park_at_goal:
                # Stopping arc length bounds displacement even with steering.
                # Capture only when bounded braking stays inside the goal disk.
                if car_like:
                    stopping_distance = controllers[a_id].stopping_distance(curr_state)
                else:
                    brake = agents_params[a_id].get(SPEED_BRAKE_MAX, 0.)
                    if brake <= 0:
                        raise ValueError("Differential-drive parking needs a positive v_brake_max")
                    speed = abs(last_controls[a_id, 0])
                    stopping_steps = int(np.ceil(speed / (brake * dt)))
                    stopping_distance = dt * np.maximum(speed - brake * dt * np.arange(1, stopping_steps+1), 0.).sum()
                if np.linalg.norm(curr_state[:2] - goal_state[:2]) + stopping_distance <= goal_xy_offset:
                    parked[a_id] = True
            if parked[a_id]:
                # Continue integrating physical jerk/acceleration/velocity until
                # rest. Parked cars remain visible to every other agent's ORCA.
                if car_like:
                    step_res = controllers[a_id].braking_control(curr_state)
                else:
                    # The supported differential-drive model controls speed
                    # and yaw rate directly. Honor their configured change
                    # limits during parking, without editing a pose or velocity.
                    decrement = np.array([brake, agents_params[a_id].get(DIFF_DRIVE_W_ACCEL_MAX, np.inf)]) * dt
                    step_res = np.clip(np.zeros(2), last_controls[a_id]-decrement, last_controls[a_id]+decrement)
                traj = np.empty((0, 0, len(curr_state)))
            else:
                step_res, traj = controllers[a_id].next_step(
                    noized_curr_state,
                    noized_neighbours_states,
                    neighbours_sizes,
                    goal_state,
                )
            u = step_res
            last_controls[a_id] = u
            if on_step is not None:
                step_controls.append(np.asarray(u).copy())
                step_controller_seconds.append(time.perf_counter()-control_start)
            if save_logs and save_traj_lines:
                step_traj.append(traj)

            if car_like:
                new_states[a_id] = (dyn_models[a_id].state_shift(curr_state, dt)
                                   + dyn_models[a_id].dynamic_model(curr_state, u, dt))
            else:
                new_states[a_id, 0:3] = dyn_models[a_id].state_shift(
                    curr_state[0:3], dt
                ) + dyn_models[a_id].dynamic_model(curr_state[0:3], u, dt)
                new_states[a_id, 3:5] = (new_states[a_id, 0:2] - curr_state[0:2]) / dt

        agents_states = new_states.copy()

        if save_logs:
            log_state.append(agents_states.copy())
            if save_traj_lines:
                log_traj.append(step_traj.copy())

        collisions += check_collisions(agents_states, agents_sizes)
        if obstacles is not None:
            for a_id in range(agents_num):
                collisions += any(
                    mppi_ca_cpp.polygon_collision(
                        previous_states[a_id, :2], agents_states[a_id, :2],
                        polygon, agents_sizes[a_id]
                    ) for polygon in obstacles
                )

        done = True
        for a_id in range(agents_num):
            if car_like and park_at_goal and (
                not parked[a_id] or abs(agents_states[a_id, 3]) > 1e-8 or abs(agents_states[a_id, 5]) > 1e-8
            ):
                done = False
                break
            if not car_like and park_at_goal and (not parked[a_id] or np.max(np.abs(last_controls[a_id])) > 1e-8):
                done = False
                break
            if (
                np.linalg.norm(agents_states[a_id, 0:2] - goal_states[a_id, 0:2])
                > goal_xy_offset
            ):
                done = False
                break

        continue_run = True
        if on_step is not None:
            continue_run = on_step(steps, agents_states.copy(), np.asarray(step_controls),
                                   np.asarray(step_controller_seconds)) is not False

        if done:
            t_end = time.time()
            t_dur = t_end - t_start

            if save_logs:
                if save_traj_lines:
                    return (
                        True,
                        collisions,
                        steps,
                        t_dur,
                        log_state,
                        log_traj
                    )
                else:
                    return True, collisions, steps, t_dur, log_state
            else:
                return True, collisions, steps, t_dur
        if not continue_run:
            break

    t_end = time.time()
    t_dur = t_end - t_start

    if save_logs:
        if save_traj_lines:
            return False, collisions, steps, t_dur, log_state, log_traj
        else:
            return False, collisions, steps, t_dur, log_state
    else:
        return False, collisions, steps, t_dur
