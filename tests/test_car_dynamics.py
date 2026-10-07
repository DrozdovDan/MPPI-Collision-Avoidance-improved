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
                    'jerk_change_cost_weight', 'steering_rate_change_cost_weight'):
            for value in (-1., float('nan'), float('inf')):
                with self.assertRaises(ValueError):
                    self.controller(**{key: value})

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
        for update in ({'a_min': 1.}, {'a_max': -1.}, {'steering_rate_max': 0.},
                       {'a_min': float('nan')}, {'j_min': 0.}, {'j_max': 0.}, {'j_max': float('inf')}):
            with self.assertRaises(ValueError):
                controller.set_agent_params(mppi.convert_agent_params(self.agent | update))


if __name__ == '__main__':
    unittest.main()
