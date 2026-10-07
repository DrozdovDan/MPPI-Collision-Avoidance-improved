



import numpy as np


class BaseDyn:
    def dynamic_model(self, state, control, dt):
        pass

    def state_shift(self, state, dt):
        pass


class DiffDriveDyn(BaseDyn):

    def dynamic_model(self, state, control, dt):
        dt = np.float128(dt)
        theta = np.float128(state[2])
        v = np.float128(control[0])
        w = np.float128(control[1])
        dx = v * np.cos(theta, dtype=np.float128) * dt
        dy = v * np.sin(theta, dtype=np.float128) * dt
        dtheta = w * dt
        return np.array([dx, dy, dtheta])

    def state_shift(self, state, dt):
        return state


class CarLikeDyn (BaseDyn):

    """State [x, y, heading, v, steering, a]; control [jerk, steering_rate]."""

    def __init__(self, dist_l, *, v_min=-1.0, v_max=1.0, a_min=-3.0, a_max=1.5,
                 steering_min=-0.6, steering_max=0.6, steering_rate_max=2.0, j_min=-2.0, j_max=2.0):
        self.dist_between_axles = dist_l
        self.v_min, self.v_max = v_min, v_max
        self.a_min, self.a_max = a_min, a_max
        self.j_min, self.j_max = j_min, j_max
        self.steering_min, self.steering_max = steering_min, steering_max
        self.steering_rate_max = steering_rate_max

    def configure_limits(self, params):
        """Use the same physical limits as the native controller."""
        self.v_min, self.v_max = params['v_min'], params['v_max']
        self.a_min = params.get('a_min', -params.get('v_brake_max', 0.0) or -3.0)
        self.a_max = params.get('a_max', params.get('v_accel_max', 0.0) or 1.5)
        self.j_min, self.j_max = params.get('j_min', -2.0), params.get('j_max', 2.0)
        self.steering_min, self.steering_max = params['w_min'], params['w_max']
        self.steering_rate_max = params.get('steering_rate_max', params.get('w_accel_max', 0.0) or 2.0)

    @staticmethod
    def acceleration_headroom(headroom, jerk_magnitude, dt):
        if headroom <= 0.0:
            return 0.0
        h = jerk_magnitude * dt
        n = max(1.0, np.ceil((np.sqrt(1.0 + 8.0 * headroom / (dt * h)) - 1.0) / 2.0))
        return (headroom / dt + h * n * (n - 1.0) / 2.0) / n

    def jerk_bounds(self, state, dt):
        v, acceleration = state[3], state[5]
        next_v = v + acceleration * dt
        lower_a = max(self.a_min, -self.acceleration_headroom(next_v - self.v_min, self.j_max, dt))
        upper_a = min(self.a_max, self.acceleration_headroom(self.v_max - next_v, -self.j_min, dt))
        lower = max(self.j_min, (lower_a - acceleration) / dt)
        upper = min(self.j_max, (upper_a - acceleration) / dt)
        if next_v < self.v_min - 1e-9 or next_v > self.v_max + 1e-9 or lower > upper + 1e-9:
            raise ValueError('Car velocity/acceleration cannot satisfy speed and jerk limits; insufficient headroom')
        return min(lower, upper), upper

    def limit_control(self, state, control, dt):
        lower, upper = self.jerk_bounds(state, dt)
        steering = state[4]
        return np.array([
            np.clip(control[0], lower, upper),
            np.clip(control[1], max(-self.steering_rate_max, (self.steering_min - steering) / dt),
                    min(self.steering_rate_max, (self.steering_max - steering) / dt)),
        ])

    def dynamic_model(self, state, control, dt):
        if np.shape(state) != (6,):
            raise ValueError('Car state must be [x, y, heading, velocity, steering angle, acceleration]')
        jerk, steering_rate = self.limit_control(state, control, dt)
        v = state[3] + state[5] * dt
        steering = state[4] + steering_rate * dt
        return np.array([v * np.cos(state[2]) * dt, v * np.sin(state[2]) * dt,
                         v * np.tan(steering) / self.dist_between_axles * dt,
                         state[5] * dt, steering_rate * dt, jerk * dt])

    def state_shift(self, state, dt):
        return state
