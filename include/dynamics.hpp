#include <xtensor/xtensor.hpp>
#include <xtensor/xarray.hpp>
#include <xtensor/xview.hpp>
#include <xtensor/xfixed.hpp>
#include <xtensor/xio.hpp>
#include <xtensor/xrandom.hpp>
#include <iostream>
#include <cmath>
#include <algorithm>
#include <utility>

#include "consts.hpp"

#ifndef MPPI_CA_CPP_DIFF_DRIVE_HPP
#define MPPI_CA_CPP_DIFF_DRIVE_HPP


class DiffDrive {
	public:
		auto shift(const xt::xtensor<double, 1> &state, double dt) -> xt::xtensor<double, 1>;

		auto dynamic(const xt::xtensor<double, 1> &state, const xt::xtensor<double, 1> &control,
					 double dt) -> xt::xtensor<double, 1>;


};


class CarLike {
	public:
		// Controls are longitudinal jerk and steering angular rate.
		// Acceleration headroom that permits returning a to zero before a speed limit.
		static double accelerationHeadroom(double velocity_headroom, double jerk_magnitude, double dt);
		auto jerkBounds(const xt::xtensor<double, 1> &state, double dt) const -> std::pair<double, double>;
		double brakingJerk(const xt::xtensor<double, 1> &state, double dt) const;
		double stoppingDistance(const xt::xtensor<double, 1> &state, double dt) const;
		auto limitControl(const xt::xtensor<double, 1> &state,
						  const xt::xtensor<double, 1> &control, double dt) const -> xt::xtensor<double, 1>;
		auto shift(const xt::xtensor<double, 1> &state, double dt) -> xt::xtensor<double, 1>;

		auto dynamic(const xt::xtensor<double, 1> &state, const xt::xtensor<double, 1> &control,
					 double dt) -> xt::xtensor<double, 1>;

		double dist_between_axles = 0.2;
		double v_min = -1.0, v_max = 1.0;
		double a_min = -3.0, a_max = 1.5;
		double j_min = -2.0, j_max = 2.0;
		double steering_min = -0.6, steering_max = 0.6;
		double steering_rate_max = 2.0;
};


#endif //MPPI_CA_CPP_DIFF_DRIVE_HPP
