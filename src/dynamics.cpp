#include "dynamics.hpp"
#include <stdexcept>


auto DiffDrive::shift(const xt::xtensor<double, 1> &state, double dt) -> xt::xtensor<double, 1> {
	return xt::zeros<double>({DD_STATE_DIM});
}

auto DiffDrive::dynamic(const xt::xtensor<double, 1> &state, const xt::xtensor<double, 1> &control,
						double dt) -> xt::xtensor<double, 1> {
	xt::xtensor<double, 1> result = xt::empty<double>({DD_STATE_DIM});
	double theta = state[2];
	double v = control[0];
	double w = control[1];
	result[0] = v * std::cos(theta) * dt;
	result[1] = v * std::sin(theta) * dt;
	result[2] = w * dt;
	return result;
}


auto CarLike::shift(const xt::xtensor<double, 1> &state, double dt) -> xt::xtensor<double, 1> {
	return xt::zeros<double>({CAR_STATE_DIM});
}

double CarLike::accelerationHeadroom(double headroom, double jerk_magnitude, double dt) {
	// Invert S(a) = dt * sum_{n>=0} max(a - n*J*dt, 0).
	// S is the unavoidable velocity change while returning acceleration to zero
	// with maximum opposing jerk under explicit Euler integration.
	if (headroom <= 0.0) return 0.0;
	double h = jerk_magnitude * dt;
	double n = std::max(1.0, std::ceil((std::sqrt(1.0 + 8.0 * headroom / (dt * h)) - 1.0) / 2.0));
	return (headroom / dt + h * n * (n - 1.0) / 2.0) / n;
}

auto CarLike::jerkBounds(const xt::xtensor<double, 1> &state, double dt) const -> std::pair<double, double> {
	double next_v = state[3] + state[5] * dt;
	double lower_a = std::max(a_min, -accelerationHeadroom(next_v - v_min, j_max, dt));
	double upper_a = std::min(a_max, accelerationHeadroom(v_max - next_v, -j_min, dt));
	double lower = std::max(j_min, (lower_a - state[5]) / dt);
	double upper = std::min(j_max, (upper_a - state[5]) / dt);
	if (next_v < v_min - 1e-9 || next_v > v_max + 1e-9 || lower > upper + 1e-9)
		throw std::invalid_argument("Car velocity/acceleration cannot satisfy speed and jerk limits; insufficient headroom");
	// Collapse floating-point disagreement at the boundary to a single control.
	if (lower > upper) lower = upper;
	return {lower, upper};
}

double CarLike::brakingJerk(const xt::xtensor<double, 1> &state, double dt) const {
	// Select an acceleration that can be removed with bounded jerk before v=0.
	double next_v = state[3] + state[5] * dt;
	double target_a = next_v > 0.0 ? -std::min(-a_min, accelerationHeadroom(next_v, j_max, dt)) :
		(next_v < 0.0 ? std::min(a_max, accelerationHeadroom(-next_v, -j_min, dt)) : 0.0);
	return limitControl(state, {(target_a - state[5]) / dt, 0.0}, dt)[0];
}

double CarLike::stoppingDistance(const xt::xtensor<double, 1> &state, double dt) const {
	if (std::abs(state[3]) < 1e-8 && std::abs(state[5]) < 1e-8) return 0.0;
	double braking_cap = std::min(-a_min, a_max);
	if (braking_cap <= 0.0)
		throw std::invalid_argument("Moving cars require acceleration limits that allow braking in both directions");
	double time_bound = 4.0 * (std::max(std::abs(v_min), std::abs(v_max)) / braking_cap +
		std::max(std::abs(a_min), std::abs(a_max)) / std::min(-j_min, j_max) + dt);
	size_t max_steps = static_cast<size_t>(std::ceil(time_bound / dt)) + 1;
	xt::xtensor<double, 1> braking_state = state;
	double distance = 0.0;
	for (size_t step = 0; step < max_steps; ++step) {
		if (std::abs(braking_state[3]) < 1e-8 && std::abs(braking_state[5]) < 1e-8) return distance;
		double jerk = brakingJerk(braking_state, dt);
		braking_state[3] += braking_state[5] * dt;
		braking_state[5] += jerk * dt;
		distance += std::abs(braking_state[3]) * dt;
	}
	throw std::invalid_argument("Car cannot stop within the supported braking prediction window");
}

auto CarLike::limitControl(const xt::xtensor<double, 1> &state,
						  const xt::xtensor<double, 1> &control, double dt) const -> xt::xtensor<double, 1> {
	auto [lower, upper] = jerkBounds(state, dt);
	double rate_lower = std::max(-steering_rate_max, (steering_min - state[4]) / dt);
	double rate_upper = std::min(steering_rate_max, (steering_max - state[4]) / dt);
	return {std::clamp(control[0], lower, upper), std::clamp(control[1], rate_lower, rate_upper)};
}

auto CarLike::dynamic(const xt::xtensor<double, 1> &state, const xt::xtensor<double, 1> &control,
					  double dt) -> xt::xtensor<double, 1> {

	double in_L = 1.0/dist_between_axles;
	xt::xtensor<double, 1> result = xt::empty<double>({CAR_STATE_DIM});
	double theta = state[2];
	auto feasible = limitControl(state, control, dt);
	double v = state[3] + state[5] * dt;
	double phi = state[4] + feasible[1] * dt;
	// v_next = v + a*dt; a_next = a + j*dt. Pose uses updated v/steering.
	result[0] = v * std::cos(theta) * dt;
	result[1] = v * std::sin(theta) * dt;
	result[2] = (v * in_L *  std::tan(phi)) * dt;
	result[3] = state[5] * dt;
	result[4] = feasible[1] * dt;
	result[5] = feasible[0] * dt;
	return result;
}
