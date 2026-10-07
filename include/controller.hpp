#ifndef MPPI_CA_CPP_CONTROLLER_HPP
#define MPPI_CA_CPP_CONTROLLER_HPP

#include <xtensor/xtensor.hpp>
#include <xtensor/xarray.hpp>
#include <xtensor/xview.hpp>
#include <xtensor/xfixed.hpp>
#include <xtensor/xio.hpp>
#include <xtensor/xrandom.hpp>
#include <xtensor-blas/xlinalg.hpp>
#include <iostream>
#include <ctime>
#include <limits>
#include <cmath>
#include <memory>
#include <random>

#include "ortools/linear_solver/linear_solver.h"

#include "consts.hpp"
#include "dynamics.hpp"
#include "geometry.hpp"
#include "orca_lines.hpp"

namespace mppica {
	// Seed in the same shared library that owns the rollout generator.
	void seedSampler(unsigned int seed);
	using std::cout;
	using std::endl;
	using namespace xt::placeholders;


	struct MPPIParams {
		size_t batch_size = 100;
		size_t time_steps = 10;
		double dt = 0.1;
		xt::xtensor<double, 1> initial_values = xt::zeros<double>({2});
		xt::xtensor<double, 1> mean = xt::zeros<double>({2});;
		xt::xtensor<double, 2> cov = xt::eye<double>(2);
		double run_cost_weight = 1.0;
		double terminal_cost_weight = 1.0;
		double lambda = 0.1;
		bool ma_cost_en = true;

		double ma_dist_weight = 1000.0;
		double ma_collision_value = 10000.0;
		double ma_dist_threshold = 2.0;

		bool car_like_dyn = false;
		double dist_between_axles = 0.2;
		double k_alpha = -2.96;
		double rad_eps = 0.05;
		double velocity_cost_weight = 0.5;
		double terminal_lookahead_distance = 2.0;
		double ma_en_close_to_goal_dist = 1.0;
		size_t ma_en_close_to_goal_ts = 3;
		double orca_time_boundary = 1.0;
		double orca_responsibility_factor = 0.5;
		bool ma_exclude_collision_traj = true;
		double velocity_cost_close_to_goal_dist = 0.3;
		double anchor_path_weight = 1.0;
		double control_smoothness_weight = 0.0;
		// Car control effort, summed over rollout steps (independent of lambda).
		double jerk_cost_weight = 0.0;
		double steering_rate_cost_weight = 0.0;
		// Car command changes, including the last executed command at horizon start.
		double jerk_change_cost_weight = 0.0;
		double steering_rate_change_cost_weight = 0.0;
		double anchor_heading_weight = 0.0;

		int max_safe_distr_step = 0; // TODO: Add constraints for several steps
		size_t max_neighbors_num = 100; // TODO: Sort neighbors by distance
        double alpha = 0.3; // TODO: Sampling with initial mean value in alpha% of batches
	};

	struct AgentParams {

		double size = 1.0;
		double vis_radius = 1.0;
		double dd_v_max = 1.0;
		double dd_v_min = -1.0;
		double dd_v_accel_max = 0.0; // Zero disables the speed-up limit.
		double dd_v_brake_max = 0.0; // Zero disables the braking limit.
		double dd_w_max = 2.0;
		double dd_w_min = -2.0;
		double dd_w_accel_max = 0.0; // Zero disables the angular acceleration limit.
		// Physical car controls: signed jerk and steering angular rate.
		double a_min = -3.0;
		double a_max = 1.5;
		double j_min = -2.0;
		double j_max = 2.0;
		double steering_rate_max = 2.0;
	};


	class Controller {
		public:
			Controller();

			auto nextStep(const xt::xtensor<double, 1> &curr_state, const xt::xtensor<double, 2> &neighbors_states, const xt::xtensor<double, 1> &neighbors_sizes,
						  const xt::xtensor<double, 1> &goal_state) -> std::tuple<xt::xtensor<double, 1>, xt::xtensor<double, 3>>;

			void setAgentParams(const AgentParams &ag_params);

			void setAlgParams(const MPPIParams &alg_params);

			void reset();

            void setObstacles(const std::vector<Polygon> &polygons);

			// An empty path disables path following. Otherwise the points define an
			// ordered polyline; they are not treated as a closed polygon.
			void setAnchorPath(const std::vector<Point2> &path);

		private:
			struct PathProjection {
				double distance;
				double progress;
				Point2 point;
			};

            std::vector<Polygon> obstacles;
			std::vector<Point2> anchor_path;
			std::vector<double> anchor_path_lengths;
			double anchor_path_length = 0.0;
			double anchor_progress = 0.0;
			bool anchor_progress_initialized = false;
            bool obstacleCollision(const Point2 &a, const Point2 &b) const;
            bool controlSequenceHitsObstacle();
			void setBrakingControlSequence();
			PathProjection projectToAnchorPath(const Point2 &point, double min_progress) const;
			Point2 anchorPointAt(double progress) const;
			void resetControlSeq();

			void resetTrajectories(const xt::xtensor<double, 1> &initial_state);

			void resetSamples();

			void resetNeighborsTrajectories();

			void generateSamples();

			double limitSpeed(double desired, double previous) const;
			void limitControl(size_t batch, size_t time_step);

			void updateControl(size_t batch, size_t time_step);

			void updateTrajectory(size_t batch, size_t time_step);

			void updateControlSequence();

			auto getCurrentControl() -> xt::xtensor<double, 1>;

			void computeRunningCosts(size_t batch, size_t time_step, const xt::xtensor<double, 1> &initial_state,
									 const xt::xtensor<double, 1> &goal_state, const xt::xtensor<double, 1> &neighbors_sizes);

			void commonStateCost(size_t batch, size_t time_step, const xt::xtensor<double, 1> &initial_state,
								 const xt::xtensor<double, 1> &goal_state);

			void anchorPathCost(size_t batch, size_t time_step);

			void controlCost(size_t batch, size_t time_step);

			void computeVelocityCost(size_t batch, size_t time_step, const xt::xtensor<double, 1> &goal_state);


			void computeTerminalCosts(size_t batch, const xt::xtensor<double, 1> &goal_state);

			void commonTerminalCost(size_t batch, const xt::xtensor<double, 1> &goal_state);

			void shiftControlSequence();

			bool checkCollisionFree();

			bool generateStopControl();

			void propagateNeighboursTrajectories(const xt::xtensor<double, 2> &neighbors_states);


			void multiAgentCost(size_t batch, size_t time_step, const xt::xtensor<double, 1> &goal_state, const xt::xtensor<double, 1> &neighbors_sizes);

			void computeLinearConstraintsForState(const xt::xtensor<double, 1> &state,
												  const xt::xtensor<double, 2> &neighbors_states,
												  const xt::xtensor<double, 1> &neighbors_sizes);

			void resampleInitialWithConstraints();


			std::tuple<bool, double, double> computeNewDistr(const xt::xtensor<double, 1> &state, const xt::xtensor<double, 1> &control);

			std::tuple<xt::xtensor<double, 1>, xt::xtensor<double, 1>> translateConstraintsToControlBounds(const xt::xtensor<double, 1> &state);

			bool computationShouldStop(); // TODO
			void updateStopComputationConditions(); // TODO
			void computeMainTrajectory(); // TODO


			size_t step; // For debug purpuses
			size_t controller_id; // For debug purpuses



			size_t batch_size;
			size_t time_steps;
			xt::xtensor<double, 1> mean;
			xt::xtensor<double, 2> sample_std;
			xt::xtensor<double, 2> control_limits;
			double linear_accel_limit = 0.0;
			double linear_brake_limit = 0.0;
			double angular_accel_limit = 0.0;
			xt::xtensor<double, 1> initial_values;

			double dt;
			double lambda;

			bool safe_control_not_found;
			size_t neighbors_num;

			xt::xtensor<double, 3> sampled_controls;
			xt::xtensor<double, 3> samples;
			xt::xtensor<double, 2> control_sequence;
			double last_linear_control = 0.0;
			double last_angular_control = 0.0;
			xt::xtensor<double, 3> generated_trajectories;
			xt::xtensor<double, 1> traj_costs;
			xt::xtensor<double, 3> neighbors_traj;
			xt::xtensor<double, 2> linear_constraints;
			std::vector<ORCALine> orca_lines;
			bool orca_complete = true;


			bool ma_cost_en;
			double ma_dist_weight;
			double ma_collision_value;
			double ma_dist_threshold;
			bool ma_exclude_collision_traj = true;

			double ma_en_close_to_goal_dist;
			size_t ma_en_close_to_goal_ts;


			size_t max_neighbors_num;
			int max_safe_distr_step;
			double k_alpha;

			double common_run_cost_weight;
			double common_terminal_cost_weight;
			double velocity_cost_weight;

			double velocity_cost_close_to_goal_dist;
			double anchor_path_weight;
			double control_smoothness_weight;
			double jerk_cost_weight;
			double steering_rate_cost_weight;
			double jerk_change_cost_weight;
			double steering_rate_change_cost_weight;
			double anchor_heading_weight;

			double terminal_lookahead_distance;

			double agent_size;
			double vis_radius;
			double rad_eps;

			double orca_time_boundary;
			double orca_responsibility_factor;


			std::vector<bool> excluded_traj;


			bool use_car_like_dyn;
			DiffDrive dyn_model; //TODO Generalize for other models
			CarLike car_like_dyn;


	};

}


#endif //MPPI_CA_CPP_CONTROLLER_HPP
