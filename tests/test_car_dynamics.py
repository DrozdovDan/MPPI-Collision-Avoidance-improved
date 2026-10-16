import sys
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'build/release'))
sys.path.insert(0, str(ROOT / 'example'))
import mppi_ca_cpp as mppi
from dyn_models import CarLikeDyn


class CarDynamicsTests(unittest.TestCase):
    dt = .1
    agent = {'size': .7, 'r_vis': 10., 'v_min': -.6, 'v_max': 3.,
             'a_min': -3., 'a_max': 1.5, 'w_min': -.6, 'w_max': .6,
             'steering_rate_max': 2., 'j_min': -2., 'j_max': 2.}

    def controller(self, **settings):
        controller = mppi.Controller()
        controller.set_agent_params(mppi.convert_agent_params(self.agent))
        alg = {
            'car_like_dyn': True, 'dist_between_axles': 1.,
            'batch_size': 96, 'time_steps': 15, 'dt': self.dt,
            'init_value': np.zeros(2), 'control_mean': np.zeros(2),
            'control_cov': np.diag([25., 4.]),
            'dist_to_goal_weight': 300., 'terminal_weight': 2000.,
            'mppi_lambda': .25, 'ma_cost_en': True,
            'ma_dist_weight': 1200., 'ma_collision_weight': 1e7,
            'ma_dist_threashhold': 4., 'k_alpha': -2.96, 'rad_eps': .05,
            'velocity_cost_weight': .5, 'terminal_lookahead_distance': 2.,
            'ma_en_close_to_goal_dist': 1., 'ma_en_close_to_goal_ts': 3,
            'orca_time_boundary': 1., 'orca_responsibility_factor': .5,
            'ma_exclude_collision_traj': True, 'velocity_cost_close_to_goal_dist': .7,
            'anchor_path_weight': 2000., 'anchor_heading_weight': 500.,
        }
        alg.update(settings)
        self.alg_settings = alg.copy()
        controller.set_alg_params(mppi.convert_alg_params(alg))
        mppi.set_random_seed(23)
        return controller

    def model(self):
        model = CarLikeDyn(1.)
        model.configure_limits(self.agent)
        return model

    def assert_feasible(self, control, rollouts, state):
        self.assertEqual(rollouts.shape[2], 6)
        np.testing.assert_allclose(rollouts[:, 0], np.tile(state, (len(rollouts), 1)))
        accelerations = rollouts[:, :, 5]
        jerks = np.diff(accelerations, axis=1) / self.dt
        np.testing.assert_allclose(np.diff(rollouts[:, :, 3], axis=1) / self.dt,
                                   rollouts[:, :-1, 5], atol=1e-9)
        rates = np.diff(rollouts[:, :, 4], axis=1) / self.dt
        for values, lower, upper in ((accelerations, -3., 1.5),
                                     (jerks, -2., 2.),
                                     (rates, -2., 2.),
                                     (rollouts[:, :, 3], -.6, 3.),
                                     (rollouts[:, :, 4], -.6, .6)):
            self.assertGreaterEqual(values.min(), lower - 1e-9)
            self.assertLessEqual(values.max(), upper + 1e-9)
        following = state + self.model().dynamic_model(state, control, self.dt)
        # The applied command itself must be feasible, without simulator clipping.
        np.testing.assert_allclose((following[[5, 4]] - state[[5, 4]]) / self.dt, control, atol=1e-9)
        self.assertAlmostEqual((following[3] - state[3]) / self.dt, state[5])
        self.assertGreaterEqual(following[3], -.600000001)
        self.assertLessEqual(following[3], 3.000000001)
        self.assertLessEqual(abs(following[4]), .600000001)
        # Both native and Python pose propagation use updated v and steering.
        heading = rollouts[:, :-1, 2]
        v, steering = rollouts[:, 1:, 3], rollouts[:, 1:, 4]
        expected = np.stack([v * np.cos(heading), v * np.sin(heading),
                             v * np.tan(steering)], axis=2) * self.dt
        np.testing.assert_allclose(np.diff(rollouts[:, :, :3], axis=1), expected, atol=1e-9)
        return following

    def test_rollouts_and_first_command_use_measured_state(self):
        controller = self.controller()
        controller.set_anchor_path([(0., 0.), (20., 0.)])
        # Includes state corrections between horizons, unrelated to the last command.
        for state in (np.array([0., 0., .2, 2.98, .59, .2]),
                      np.array([0., 0., -.2, -.58, -.59, -.2]),
                      np.array([0., 0., .4, 1.2, .3, -.7]),
                      np.zeros(6)):
            control, rollouts = controller.next_step(
                state, np.empty((0, 4)), np.empty(0), np.array([20., 0., 0.]))
            self.assert_feasible(control, rollouts, state)

    def test_reverse_then_forward(self):
        controller = self.controller(ma_cost_en=False)
        controller.set_anchor_path([(0., 0.), (-2., 0.)])
        state = np.zeros(6)
        speeds = []
        for step in range(100):
            if step == 35:
                controller.set_anchor_path([state[:2].tolist(), (3., 0.)])
            goal = np.array([-2., 0., 0.]) if step < 35 else np.array([3., 0., 0.])
            control, rollouts = controller.next_step(state, np.empty((0, 4)), np.empty(0), goal)
            state = self.assert_feasible(control, rollouts, state)
            speeds.append(state[3])
        self.assertLess(min(speeds[:35]), -.05)
        self.assertGreater(max(speeds[35:]), .05)

    def test_obstacle_fallback_brakes_physically(self):
        controller = self.controller()
        controller.set_obstacles([[(-1., -1.), (1., -1.), (1., 1.), (-1., 1.)]])
        state = np.array([0., 0., 0., 2., .4, 0.])
        control, rollouts = controller.next_step(state, np.empty((0, 4)), np.empty(0), np.array([10., 0., 0.]))
        following = self.assert_feasible(control, rollouts, state)
        np.testing.assert_allclose(control, [-2., 0.])
        self.assertAlmostEqual(following[3], 2.)
        self.assertAlmostEqual(following[5], -.2)
        self.assertAlmostEqual(following[4], .4)

    def test_orca_resampling_keeps_jerk_units(self):
        controller = self.controller()
        state = np.array([0., 0., .3, 1., .2, -.1])
        control, rollouts = controller.next_step(
            state, np.array([[6., 0., -.3, 0.]]), np.array([.7]), np.array([10., 0., 0.]))
        self.assert_feasible(control, rollouts, state)

    def test_anchor_objective_does_not_pull_towards_remote_goal(self):
        # Both runs have the same local route, neighbor and native seed. A
        # remote goal across a wall must not change local anchor tracking.
        outputs = []
        for goal in ([12., 0., 0.], [100., 100., 0.]):
            controller = self.controller(
                batch_size=160, time_steps=31, terminal_lookahead_distance=6.,
                control_cov=np.diag([1., .25]), jerk_cost_weight=300.,
                steering_rate_cost_weight=300., jerk_change_cost_weight=500.,
                steering_rate_change_cost_weight=1000.)
            controller.set_anchor_path([(0., 0.), (0., -6.), (12., -6.), (12., 0.)])
            # Keep a real ORCA neighbor in both calls.
            outputs.append(controller.next_step(
                np.zeros(6), np.array([[-4., 0., 0., 0.]]), np.array([.7]), np.array(goal)))
        np.testing.assert_allclose(outputs[0][0], outputs[1][0], atol=1e-12)
        np.testing.assert_allclose(outputs[0][1], outputs[1][1], atol=1e-12)

    def test_acquires_perpendicular_route_with_orca_enabled(self):
        controller = self.controller(
            batch_size=160, time_steps=31, terminal_lookahead_distance=6.,
            control_cov=np.diag([1., .25]), anchor_path_weight=300.,
            jerk_cost_weight=300., steering_rate_cost_weight=300.,
            jerk_change_cost_weight=500., steering_rate_change_cost_weight=1000.)
        controller.set_anchor_path([(0., 0.), (0., -6.), (12., -6.), (12., 0.)])
        state = np.zeros(6)  # Nose east; first route leg runs south.
        neighbor = np.array([[-4., 0., 0., 0.]])
        lowest_y = 0.
        for _ in range(150):
            control, rollouts = controller.next_step(
                state, neighbor, np.array([.7]), np.array([12., 0., 0.]))
            state = self.assert_feasible(control, rollouts, state)
            lowest_y = min(lowest_y, state[1])
            self.assertGreater(np.linalg.norm(state[:2] - neighbor[0, :2]), 1.4)
        self.assertLess(lowest_y, -5.)
        self.assertLess(np.linalg.norm(state[:2] - [12., 0.]), 1.)

    def test_forward_preference_turns_instead_of_reversing_down_route(self):
        controller = self.controller(
            batch_size=256, time_steps=61, terminal_lookahead_distance=6.,
            control_cov=np.diag([1., .25]), anchor_path_weight=300.,
            anchor_forward_weight=80000., reverse_cost_weight=1000.,
            jerk_cost_weight=300., steering_rate_cost_weight=300.,
            jerk_change_cost_weight=500., steering_rate_change_cost_weight=1000.)
        controller.set_anchor_path([(0., 0.), (80., 0.)])
        state = np.array([0., 0., np.pi, 0., 0., 0.])
        reverse_distance = 0.
        recent_speeds = []
        for _ in range(150):
            control, rollouts = controller.next_step(
                state, np.array([[0., 10., 0., 0.]]), np.array([.7]), np.array([80., 0., 0.]))
            state = self.assert_feasible(control, rollouts, state)
            reverse_distance += max(0., -state[3]) * self.dt
            recent_speeds.append(state[3])
        self.assertLess(reverse_distance, 1.)
        self.assertGreater(state[0], 10.)
        self.assertGreater(min(recent_speeds[-30:]), 0.)

    def test_goal_braking_distance_bounds_actual_displacement(self):
        controller = self.controller()
        state = np.array([.2, 0., .3, .4, .4, .1])
        start = state.copy()
        bound = controller.stopping_distance(state)
        self.assertLess(np.linalg.norm(state[:2]) + bound, 1.)
        distance = 0.
        for _ in range(100):
            control = controller.braking_control(state)
            following = state + self.model().dynamic_model(state, control, self.dt)
            np.testing.assert_allclose((following[[5, 4]] - state[[5, 4]]) / self.dt, control, atol=1e-9)
            self.assertGreaterEqual(control[0], self.agent['j_min'] - 1e-9)
            self.assertLessEqual(control[0], self.agent['j_max'] + 1e-9)
            distance += np.linalg.norm(following[:2] - state[:2])
            state = following
        self.assertLessEqual(distance, bound + 1e-8)
        self.assertLess(np.linalg.norm(state[:2]), 1.)
        np.testing.assert_allclose(state[[3, 5]], 0., atol=1e-8)
        self.assertGreater(np.linalg.norm(state[:2] - start[:2]), 0.)

    def test_arrived_car_brakes_and_parks_while_other_car_finishes(self):
        import utils
        self.controller(batch_size=160, time_steps=31, control_cov=np.diag([1., .25]),
                        anchor_path_weight=300., terminal_lookahead_distance=6.)
        starts = np.array([[0., 0., 0., .35, 0., .1], [-5., 4., 0., 0., 0., 0.]])
        goals = np.array([[.2, 0., 0., 0., 0., 0.], [5., 4., 0., 0., 0., 0.]])
        success, collisions, _, _, trace = utils.launch_mppi(
            starts, goals, 2, [self.agent.copy(), self.agent.copy()], self.alg_settings,
            {'timestep': .1, 'xy_goal_tolerance': 1., 'max_steps': 400},
            [self.model(), self.model()], seed=42, obstacles=[], park_at_goal=True,
            anchor_paths=[[(0., 0.), (.2, 0.)], [(-5., 4.), (5., 4.)]])
        trace = np.asarray(trace)
        self.assertTrue(success)
        self.assertEqual(collisions, 0)
        self.assertLess(np.linalg.norm(trace[-1, :, :2] - goals[:, :2], axis=1).max(), 1.)
        np.testing.assert_allclose(trace[-1, :, [3, 5]], 0., atol=1e-8)
        np.testing.assert_allclose(trace[-20:, 0, :2], np.tile(trace[-1, 0, :2], (20, 1)), atol=1e-8)
        self.assertGreater(np.linalg.norm(trace[-1, 0, :2] - trace[0, 0, :2]), .01)
        np.testing.assert_allclose(np.diff(trace[:, :, 3], axis=0) / .1, trace[:-1, :, 5], atol=1e-9)

    def test_speed_saturation_keeps_acceleration_continuous(self):
        model = self.model()
        for sign in (1., -1.):
            state = np.zeros(6)
            trace = [state.copy()]
            for _ in range(150):
                state = state + model.dynamic_model(state, [sign * 2., 0.], self.dt)
                trace.append(state.copy())
            trace = np.asarray(trace)
            np.testing.assert_allclose(np.diff(trace[:, 3]) / self.dt, trace[:-1, 5], atol=1e-9)
            self.assertLessEqual(np.abs(np.diff(trace[:, 5]) / self.dt).max(), 2.000000001)
            self.assertGreaterEqual(trace[:, 3].min(), -.600000001)
            self.assertLessEqual(trace[:, 3].max(), 3.000000001)
            self.assertAlmostEqual(state[3], 3. if sign > 0 else -.6)
            self.assertAlmostEqual(state[5], 0.)

    def test_jerk_limited_braking_keeps_room_for_a_stopped_neighbor(self):
        controller = self.controller()
        controller.set_anchor_path([(0., 0.), (20., 0.)])
        state = np.array([0., 0., 0., 2., 0., 0.])
        neighbor = np.array([[4., 0., 0., 0.]])
        for _ in range(40):
            control, rollouts = controller.next_step(state, neighbor, np.array([.7]), np.array([20., 0., 0.]))
            state = self.assert_feasible(control, rollouts, state)
            self.assertGreaterEqual(np.linalg.norm(state[:2] - neighbor[0, :2]), 1.4)

    def test_control_regularization_reduces_chatter_without_losing_progress(self):
        def run(jerk_weight, rate_weight, covariance):
            controller = self.controller(
                ma_cost_en=False, batch_size=160, terminal_lookahead_distance=6.,
                control_smoothness_weight=20., control_cov=np.diag(covariance),
                jerk_cost_weight=jerk_weight, steering_rate_cost_weight=rate_weight)
            controller.set_anchor_path([(0., 0.), (15., 0.), (25., 10.), (40., 10.)])
            state = np.zeros(6)
            commands = []
            for _ in range(260):
                control, rollouts = controller.next_step(
                    state, np.empty((0, 4)), np.empty(0), np.array([40., 10., 0.]))
                state = self.assert_feasible(control, rollouts, state)
                commands.append(control)
            self.assertLess(np.linalg.norm(state[:2] - [40., 10.]), 3.)
            commands = np.asarray(commands)
            return (np.sqrt(np.mean(commands**2, axis=0)),
                    np.sqrt(np.mean(np.diff(commands, axis=0)**2, axis=0)))

        original_effort, original_changes = run(0., 0., [25., .16])
        # Compare identical noise with/without penalties as well, to check the cost itself.
        unpenalized_effort, _ = run(0., 0., [.5625, .04])
        effort, changes = run(100., 300., [.5625, .04])
        self.assertTrue(np.all(effort < .6 * original_effort))
        self.assertTrue(np.all(changes < .5 * original_changes))
        self.assertTrue(np.all(effort < .9 * unpenalized_effort))

    def test_rejects_invalid_control_effort_weights(self):
        for key in ('jerk_cost_weight', 'steering_rate_cost_weight',
                    'jerk_change_cost_weight', 'steering_rate_change_cost_weight',
                    'anchor_forward_weight', 'reverse_cost_weight'):
            for value in (-1., float('nan'), float('inf')):
                with self.assertRaises(ValueError):
                    self.controller(**{key: value})

    def test_correlated_exploration_with_orca_preserves_variance_and_reduces_chatter(self):
        # Inspect physical rollout derivatives, including the first jerk that
        # ORCA resamples. Small noise avoids saturation obscuring the covariance.
        changes, magnitudes = [], []
        for correlation in (0., .92):
            controller = self.controller(
                batch_size=2048, time_steps=21, control_cov=np.diag([.0025, .0004]),
                control_noise_correlation=correlation)
            state = np.array([0., 0., 0., 1., 0., 0.])
            _, rollouts = controller.next_step(
                state, np.array([[-8., 0., 0., 0.]]), np.array([.7]), np.array([20., 0., 0.]))
            commands = np.diff(rollouts[:, :, [5, 4]], axis=1) / self.dt
            self.assert_feasible(commands[0, 0], rollouts, state)
            magnitudes.append(np.sqrt(np.mean(commands**2, axis=(0, 1))))
            changes.append(np.sqrt(np.mean(np.diff(commands, axis=1)**2, axis=(0, 1))))
        np.testing.assert_allclose(magnitudes[1], magnitudes[0], rtol=.08)
        self.assertTrue(np.all(changes[1] < .35 * changes[0]))

    def test_rejects_invalid_sampling_and_acquisition_parameters(self):
        for key, values in {
            'control_noise_correlation': (-.01, 1., float('nan'), float('inf')),
            'anchor_acquisition_scale': (-.01, 1.01, float('nan'), float('inf')),
            'anchor_guided_fraction': (-.01, 1.01, float('nan'), float('inf')),
        }.items():
            for value in values:
                with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                    self.controller(**{key: value})

    def test_smooth_forward_tracking_acquires_turn_and_reaches_goal_with_orca(self):
        results = []
        for smooth in (False, True):
            settings = dict(
                batch_size=256, time_steps=61, terminal_lookahead_distance=6.,
                control_cov=np.diag([1., .25]), anchor_path_weight=300.,
                anchor_heading_weight=500., anchor_forward_weight=80000., reverse_cost_weight=1000.,
                jerk_cost_weight=100., steering_rate_cost_weight=300.,
                jerk_change_cost_weight=500., steering_rate_change_cost_weight=1000.)
            if smooth:
                self.agent = self.agent | dict(v_min=0., j_min=-1., j_max=1., steering_rate_max=.6)
                settings.update(
                    control_cov=np.diag([.25, .0225]), control_noise_correlation=.92,
                    anchor_guided_fraction=.25, anchor_acquisition_scale=.02,
                    anchor_path_weight=3000., anchor_heading_weight=1000.,
                    jerk_cost_weight=1000., steering_rate_cost_weight=2000.,
                    jerk_change_cost_weight=4000., steering_rate_change_cost_weight=10000.)
            controller = self.controller(**settings)
            controller.set_anchor_path([(0., 0.), (0., -6.), (40., -6.)])
            state = np.zeros(6)  # Nose east; entry needs a physical right turn.
            trace = [state.copy()]
            for _ in range(250):
                command, rollouts = controller.next_step(
                    state, np.array([[-8., 0., 0., 0.]]), np.array([.7]), np.array([40., -6., 0.]))
                state = self.assert_feasible(command, rollouts, state)
                trace.append(state.copy())
            trace = np.asarray(trace)
            self.assertLess(np.linalg.norm(state[:2] - [40., -6.]), 1.)
            commands = np.diff(trace[:, [5, 4]], axis=0) / self.dt
            if smooth:
                self.assertGreaterEqual(trace[:, 3].min(), -1e-9)
                self.assertLessEqual(np.abs(commands[:, 0]).max(), 1.000000001)
                self.assertLessEqual(np.abs(commands[:, 1]).max(), .600000001)
            results.append((np.sqrt(np.mean(commands**2, axis=0)),
                            np.sqrt(np.mean(np.diff(commands, axis=0)**2, axis=0)),
                            np.sqrt(np.mean((trace[100:, 1] + 6.)**2))))
        self.assertTrue(np.all(results[1][0] < .6 * results[0][0]))
        self.assertTrue(np.all(results[1][1] < .4 * results[0][1]))
        self.assertLess(results[1][2], .5 * results[0][2])

    def test_neighbor_proximity_cutoff_does_not_create_an_artificial_stop_line(self):
        self.agent = self.agent | dict(v_min=0., v_max=1.2, j_min=-1., j_max=1., steering_rate_max=.6)
        controller = self.controller(
            batch_size=256, time_steps=61, terminal_lookahead_distance=6.,
            control_cov=np.diag([.25, .0225]), control_noise_correlation=.92,
            anchor_acquisition_scale=.1, anchor_path_weight=3000., anchor_heading_weight=1000.,
            anchor_forward_weight=80000., reverse_cost_weight=1000.,
            jerk_cost_weight=1000., steering_rate_cost_weight=2000.,
            jerk_change_cost_weight=4000., steering_rate_change_cost_weight=10000.,
            orca_time_boundary=5., orca_responsibility_factor=1., rad_eps=.15)
        controller.set_anchor_path([(0., 0.), (25., 0.)])
        state = np.zeros(6)
        for _ in range(160):
            command, rollouts = controller.next_step(
                state, np.array([[10., 0., 0., 0.]]), np.array([.7]), np.array([25., 0., 0.]))
            state = self.assert_feasible(command, rollouts, state)
            self.assertGreaterEqual(np.linalg.norm(state[:2] - [10., 0.]), 1.7 - 1e-9)
        # The old weight/d cutoff step trapped this same seeded car near x=6.
        # Crossing the cost boundary still requires physical, collision-free motion.
        self.assertGreater(state[0], 6.5)

    def test_guided_passing_preserves_a_safe_mode_and_finishes_forward(self):
        self.agent = self.agent | dict(v_min=0., v_max=1.2, j_min=-1., j_max=1., steering_rate_max=.6)

        def build():
            controller = self.controller(
                batch_size=256, time_steps=61, terminal_lookahead_distance=6.,
                control_cov=np.diag([.25, .0225]), control_noise_correlation=.92,
                anchor_guided_fraction=.25, anchor_acquisition_scale=.02,
                anchor_path_weight=3000., anchor_heading_weight=1000.,
                anchor_forward_weight=80000., reverse_cost_weight=1000.,
                jerk_cost_weight=1000., steering_rate_cost_weight=2000.,
                jerk_change_cost_weight=4000., steering_rate_change_cost_weight=10000.,
                orca_time_boundary=5., orca_responsibility_factor=1., rad_eps=.15)
            controller.set_anchor_path([(0., 0.), (25., 0.)])
            return controller

        neighbor = np.array([[10., 0., 0., 0.]])
        goal = np.array([25., 0., 0.])
        controller = build()
        # Exactly symmetric left/right proposals must keep a passing mode;
        # their mean would drive towards the neighbor. Ties prefer the right.
        command, rollouts = controller.next_step(
            np.array([4., 0., 0., .8, 0., 0.]), neighbor, np.array([.7]), goal)
        self.assertLess(command[1], -.1)

        controller = build()
        state = np.zeros(6)
        trace = [state.copy()]
        for _ in range(400):
            command, rollouts = controller.next_step(state, neighbor, np.array([.7]), goal)
            state = self.assert_feasible(command, rollouts, state)
            self.assertLessEqual(abs(command[0]), 1.000000001)
            self.assertLessEqual(abs(command[1]), .600000001)
            self.assertGreaterEqual(state[3], -1e-9)
            trace.append(state.copy())
        self.assertLess(np.linalg.norm(state[:2] - goal[:2]), 1.)
        relative = np.asarray(trace)[:, :2] - neighbor[0, :2]
        delta = np.diff(relative, axis=0)
        squared = np.sum(delta**2, axis=1)
        fraction = np.divide(-np.sum(relative[:-1] * delta, axis=1), squared,
                             out=np.zeros_like(squared), where=squared > 0)
        closest = relative[:-1] + np.clip(fraction, 0, 1)[:, None] * delta
        self.assertGreaterEqual(np.linalg.norm(closest, axis=1).min(), 1.7 - 1e-9)

    def test_existing_clearance_overlap_allows_only_separating_motion(self):
        self.agent = self.agent | dict(v_min=0., j_min=-1., j_max=1., steering_rate_max=.6)
        controller = self.controller(
            time_steps=61, batch_size=256, terminal_lookahead_distance=6.,
            control_cov=np.diag([.25, .0225]), control_noise_correlation=.92,
            anchor_guided_fraction=.25, anchor_acquisition_scale=.02,
            anchor_forward_weight=80000., rad_eps=.15,
            jerk_cost_weight=1000., steering_rate_cost_weight=2000.,
            jerk_change_cost_weight=4000., steering_rate_change_cost_weight=10000.)
        controller.set_anchor_path([(0., 0.), (-12., 0.)])
        state = np.array([0., 0., np.pi, 0., 0., 0.])
        neighbor = np.array([[1.695, 0., 0., 0.]])
        previous_distance = 1.695
        for _ in range(100):
            command, rollouts = controller.next_step(state, neighbor, np.array([.7]), np.array([-12., 0., 0.]))
            state = self.assert_feasible(command, rollouts, state)
            distance = np.linalg.norm(state[:2] - neighbor[0, :2])
            self.assertGreaterEqual(distance, min(previous_distance, 1.7) - 1e-9)
            self.assertGreaterEqual(distance, 1.4)
            previous_distance = distance
        self.assertLess(state[0], -5.)

    def test_overshot_endpoint_is_reacquired_without_reversing(self):
        self.agent = self.agent | dict(v_min=0., v_max=1.2, j_min=-1., j_max=1., steering_rate_max=.6)
        controller = self.controller(
            time_steps=61, batch_size=256, terminal_lookahead_distance=6.,
            control_cov=np.diag([.25, .0225]), control_noise_correlation=.92,
            anchor_guided_fraction=.25, anchor_acquisition_scale=.02,
            anchor_path_weight=3000., anchor_heading_weight=1000., anchor_forward_weight=80000.,
            jerk_cost_weight=1000., steering_rate_cost_weight=2000.,
            jerk_change_cost_weight=4000., steering_rate_change_cost_weight=10000.,
            orca_time_boundary=5., orca_responsibility_factor=1., rad_eps=.15)
        controller.set_anchor_path([(0., 0.), (5., 0.)])
        # The car has passed the route end and faces away from its goal. The
        # old final-tangent objective rewarded staying here with this heading.
        state = np.array([6.5, 0., 0., 0., 0., 0.])
        for _ in range(200):
            command, rollouts = controller.next_step(
                state, np.array([[-8., 4., 0., 0.]]), np.array([.7]), np.array([5., 0., 0.]))
            state = self.assert_feasible(command, rollouts, state)
            self.assertGreaterEqual(state[3], -1e-9)
        self.assertLess(np.linalg.norm(state[:2] - [5., 0.]), .5)

    def test_same_direction_traffic_follows_the_route_without_parallel_overtaking(self):
        self.agent = self.agent | dict(v_min=0., v_max=1.2, j_min=-1., j_max=1., steering_rate_max=.6)
        controller = self.controller(
            time_steps=61, batch_size=256, terminal_lookahead_distance=6.,
            control_cov=np.diag([.25, .0225]), control_noise_correlation=.92,
            anchor_guided_fraction=.25, anchor_acquisition_scale=.02,
            anchor_path_weight=3000., anchor_heading_weight=1000., anchor_forward_weight=80000.,
            jerk_cost_weight=1000., steering_rate_cost_weight=2000.,
            jerk_change_cost_weight=4000., steering_rate_change_cost_weight=10000.,
            orca_time_boundary=5., orca_responsibility_factor=1., rad_eps=.15)
        controller.set_anchor_path([(0., 0.), (60., 0.)])
        state = np.array([0., 0., 0., 1.2, 0., 0.])
        for step in range(120):
            neighbor = np.array([[2.5 + 1.2 * step * self.dt, 0., 1.2, 0.]])
            command, rollouts = controller.next_step(state, neighbor, np.array([.7]), np.array([60., 0., 0.]))
            state = self.assert_feasible(command, rollouts, state)
            self.assertLess(abs(state[1]), .2)
            self.assertGreaterEqual(state[3], -1e-9)
            self.assertGreaterEqual(neighbor[0, 0] + 1.2 * self.dt - state[0], 1.4)
        self.assertGreater(state[0], 5.)

    def test_temporal_cost_reduces_raw_command_changes(self):
        def run(covariance, change_weights):
            controller = self.controller(
                ma_cost_en=False, batch_size=160, terminal_lookahead_distance=6.,
                control_smoothness_weight=20., control_cov=np.diag(covariance),
                jerk_cost_weight=100., steering_rate_cost_weight=300.,
                jerk_change_cost_weight=change_weights[0],
                steering_rate_change_cost_weight=change_weights[1])
            controller.set_anchor_path([(0., 0.), (15., 0.), (25., 10.), (40., 10.)])
            state = np.zeros(6)
            commands = []
            for _ in range(260):
                control, rollouts = controller.next_step(
                    state, np.empty((0, 4)), np.empty(0), np.array([40., 10., 0.]))
                state = self.assert_feasible(control, rollouts, state)
                commands.append(control.copy())
            self.assertLess(np.linalg.norm(state[:2] - [40., 10.]), 3.)
            return np.sqrt(np.mean(np.diff(commands, axis=0)**2, axis=0))

        previous = run([.5625, .04], [0., 0.])
        same_noise_without_change_cost = run([.25, .04], [0., 0.])
        temporal = run([.25, .04], [500., 1000.])
        self.assertTrue(np.all(temporal < .65 * previous))
        self.assertTrue(np.all(temporal < .85 * same_noise_without_change_cost))

    def test_change_cost_uses_last_executed_command_across_horizons(self):
        controller = self.controller(
            ma_cost_en=False, batch_size=1024, time_steps=2,
            init_value=np.array([.8, .4]), control_cov=np.diag([.64, .16]),
            dist_to_goal_weight=0., terminal_weight=0., velocity_cost_weight=0.,
            anchor_path_weight=0., anchor_heading_weight=0., mppi_lambda=.0001,
            jerk_change_cost_weight=1e6, steering_rate_change_cost_weight=1e6)
        commands = []
        # Saturated measured a/steering force the second command to zero.
        # A subsequent state correction allows either sign again. The third
        # command should stay near the executed zero, despite warm-start .8/.4.
        for acceleration, steering in ((0., 0.), (1.5, .6), (0., 0.)):
            state = np.array([0., 0., 0., 1., steering, acceleration])
            control, rollouts = controller.next_step(
                state, np.empty((0, 4)), np.empty(0), np.zeros(3))
            self.assert_feasible(control, rollouts, state)
            commands.append(control.copy())
        np.testing.assert_allclose(commands[0], [.8, .4], atol=.05)
        np.testing.assert_allclose(commands[1], [0., 0.], atol=1e-8)
        np.testing.assert_allclose(commands[2], [0., 0.], atol=.05)

    def test_rejects_invalid_physical_state_and_limits(self):
        controller = self.controller()
        for state in (np.zeros(5), np.array([0., 0., 0., 4., 0., 0.]),
                      np.array([0., 0., 0., 0., .7, 0.]), np.full(6, np.nan),
                      np.array([0., 0., 0., 3., 0., .1]),
                      np.array([0., 0., 0., 0., 0., 4.])):
            with self.assertRaises(ValueError):
                controller.next_step(state, np.empty((0, 4)), np.empty(0), np.zeros(3))
            with self.assertRaises(ValueError):
                controller.braking_control(state)
            with self.assertRaises(ValueError):
                controller.stopping_distance(state)
        for update in ({'a_min': 1.}, {'a_max': -1.}, {'steering_rate_max': 0.},
                       {'a_min': float('nan')}, {'j_min': 0.}, {'j_max': 0.}, {'j_max': float('inf')}):
            with self.assertRaises(ValueError):
                controller.set_agent_params(mppi.convert_agent_params(self.agent | update))


if __name__ == '__main__':
    unittest.main()
