#include "controller.hpp"
#include <chrono>
#include <vector>
#include <unistd.h>

namespace mppica {

    void Controller::setObstacles(const std::vector<Polygon> &polygons) {
        for (const auto &polygon : polygons) {
            if (polygon.size() < 3) throw std::invalid_argument("An obstacle needs at least three vertices");
            double area = 0;
            for (size_t i = 0; i < polygon.size(); ++i) {
                const auto &p = polygon[i];
                const auto &q = polygon[(i+1)%polygon.size()];
                if (!std::isfinite(p[0]) || !std::isfinite(p[1]))
                    throw std::invalid_argument("Obstacle vertices must be finite");
                area += p[0]*q[1]-p[1]*q[0];
            }
            if (std::abs(area) < 1e-12) throw std::invalid_argument("Obstacle must have nonzero area");
        }
        obstacles = polygons;
    }

    bool Controller::obstacleCollision(const Point2 &a, const Point2 &b) const {
        for (const auto &polygon : obstacles)
            if (polygonCollision(a, b, polygon, agent_size + rad_eps)) return true;
        return false;
    }

    bool Controller::controlSequenceHitsObstacle() {
        xt::xtensor<double, 1> state = xt::view(generated_trajectories, 0, 0);
        for (size_t t = 0; t < time_steps-1; ++t) {
            xt::xtensor<double, 1> u = xt::view(control_sequence, t);
            xt::xtensor<double, 1> next;
            if (use_car_like_dyn) next = state + car_like_dyn.shift(state, dt) + car_like_dyn.dynamic(state, u, dt);
            else next = state + dyn_model.shift(state, dt) + dyn_model.dynamic(state, u, dt);
            if (obstacleCollision({state[0], state[1]}, {next[0], next[1]})) return true;
            state = next;
        }
        return false;
    }

	Controller::Controller() {
		step = 0;
		controller_id = 0;
		xt::random::seed(getpid() * time(NULL));
		setAlgParams(MPPIParams());
		setAgentParams(AgentParams());
		dyn_model = DiffDrive();
		car_like_dyn = CarLike();
		reset();
	}

	auto Controller::nextStep(const xt::xtensor<double, 1> &curr_state, const xt::xtensor<double, 2> &neighbors_states, const xt::xtensor<double, 1> &neighbors_sizes,
							  const xt::xtensor<double, 1> &goal_state) -> std::tuple<xt::xtensor<double, 1>, xt::xtensor<double, 3>> {
		do {
			resetTrajectories(curr_state);
			if(ma_cost_en) {
				propagateNeighboursTrajectories(neighbors_states);
			}

			generateSamples();

			if (max_safe_distr_step >= 0) {
				computeLinearConstraintsForState(curr_state, neighbors_states, neighbors_sizes);
				resampleInitialWithConstraints();
			}

			for (size_t batch = 0; batch < batch_size; ++batch) {
				for (size_t t_step = 0; t_step < time_steps - 1; ++t_step) {
					updateControl(batch, t_step);
					limitControl(batch, t_step);
					updateTrajectory(batch, t_step);
					computeRunningCosts(batch, t_step, curr_state, goal_state, neighbors_sizes);
				}
				computeTerminalCosts(batch, goal_state);
			}
			updateControlSequence();
			safe_control_not_found = checkCollisionFree();
		} while (safe_control_not_found and not computationShouldStop());

		if (safe_control_not_found) {
			generateStopControl();
			std::cerr << "No safe trajectory";
		}

		auto result = getCurrentControl();
		shiftControlSequence();
		step += 1;

		return {result, generated_trajectories};
	}

	void Controller::reset() {
        resetTrajectories(xt::zeros<double>({DD_STATE_DIM})); // TODO: Generalize to common dynamics
		resetControlSeq();
		resetSamples();
		resetNeighborsTrajectories();
	}

	void Controller::resetTrajectories(const xt::xtensor<double, 1> &initial_state) {
        generated_trajectories = xt::zeros<double>({batch_size, time_steps, DD_STATE_DIM}); // TODO: Generalize to common dynamics
		traj_costs = xt::zeros<double>({batch_size});
		excluded_traj = std::vector<bool>(batch_size, false);
		xt::view(generated_trajectories, xt::all(), 0) = xt::view(initial_state, xt::range(0, 3));
	}

	void Controller::resetControlSeq() {
		// TODO: Generalize to common dynamics
        control_sequence = xt::empty<double>({time_steps - 1, DD_CONTROL_DIM});
		xt::view(control_sequence, xt::all(), xt::all()) = initial_values;
	}

	void Controller::resetSamples() {
		// TODO: Generalize to common dynamics
        samples = xt::zeros<double>({batch_size, time_steps - 1, DD_CONTROL_DIM});
        sampled_controls = xt::zeros<double>({batch_size, time_steps - 1, DD_CONTROL_DIM});
	}

	void Controller::generateSamples() {
		// TODO: Generalize to common dynamics
		xt::xtensor<double, 2>::shape_type control_row_shape = {batch_size, time_steps - 1};
        for (size_t i = 0; i < DD_CONTROL_DIM; ++i) {
			xt::view(samples, xt::all(), xt::all(), i) = xt::random::randn<double>(control_row_shape, mean[i],
																				   sample_std[{i, i}]);
		}
	}

	void Controller::updateControl(size_t batch, size_t time_step) {
		xt::view(sampled_controls, batch, time_step, xt::all()) = xt::view(control_sequence, time_step, xt::all()) +
																  xt::view(samples, batch, time_step, xt::all());
	}

	void Controller::limitControl(size_t batch, size_t time_step) {
		// TODO: Generalize to common dynamics
        for (size_t i = 0; i < DD_CONTROL_DIM; ++i) {
			sampled_controls[{batch, time_step, i}] = xt::clip(xt::view(sampled_controls, batch, time_step, i),
															   control_limits[{i, static_cast<size_t>(0)}],
															   control_limits[{i, static_cast<size_t>(1)}]);
		}

	}

	void Controller::updateTrajectory(size_t batch, size_t time_step) {

        // TODO: Generalize to common dynamics
		if (use_car_like_dyn) {
			xt::view(generated_trajectories, batch, time_step + 1) = xt::view(generated_trajectories, batch, time_step) +
																	 car_like_dyn.shift(xt::view(generated_trajectories, batch,
																							  time_step), dt) +
																							  car_like_dyn.dynamic(
																			 xt::view(generated_trajectories, batch,
																					  time_step),
																			 xt::view(sampled_controls, batch, time_step),
																			 dt);
			return;
		}


		xt::view(generated_trajectories, batch, time_step + 1) = xt::view(generated_trajectories, batch, time_step) +
																 dyn_model.shift(xt::view(generated_trajectories, batch,
																						  time_step), dt) +
																 dyn_model.dynamic(
																		 xt::view(generated_trajectories, batch,
																				  time_step),
																		 xt::view(sampled_controls, batch, time_step),
																		 dt);
	}

	void Controller::computeRunningCosts(size_t batch, size_t time_step, const xt::xtensor<double, 1> &initial_state,
										 const xt::xtensor<double, 1> &goal_state, const xt::xtensor<double, 1> &neighbors_sizes) {
		if (excluded_traj[batch]) return;
        if (obstacleCollision({generated_trajectories(batch,time_step,0), generated_trajectories(batch,time_step,1)},
                              {generated_trajectories(batch,time_step+1,0), generated_trajectories(batch,time_step+1,1)})) {
            traj_costs[batch] += 1e10;
            excluded_traj[batch] = true;
            return;
        }
		commonStateCost(batch, time_step, initial_state, goal_state);
		if (ma_cost_en) multiAgentCost(batch, time_step, goal_state, neighbors_sizes);
		controlCost(batch, time_step);
		computeVelocityCost(batch, time_step, goal_state);
	}

	void Controller::computeVelocityCost(size_t batch, size_t time_step, const xt::xtensor<double, 1> &goal_state) {

		auto goal_pos = xt::view(goal_state, xt::range(0, 2));
		auto current_pos = xt::view(generated_trajectories, batch, static_cast<size_t>(0), xt::range(0, 2));
		auto dist = xt::linalg::norm(current_pos - goal_pos, 2);
		if (dist < velocity_cost_close_to_goal_dist) {
			return;
		}
		if (time_step == 0) {
			return;
		}

		auto dx = generated_trajectories[{batch, time_step, static_cast<size_t>(0)}] -
			 generated_trajectories[{batch, time_step - 1, static_cast<size_t>(0)}];
		auto dy = generated_trajectories[{batch, time_step, static_cast<size_t>(1)}] -
			 generated_trajectories[{batch, time_step - 1, static_cast<size_t>(1)}];

		auto step_movement = sqrt(dx * dx + dy * dy);
		if (abs(step_movement) < 0.001) {
			step_movement = 0.001;
		}
		traj_costs[batch] += (1 / step_movement) * velocity_cost_weight;


	}


	void Controller::controlCost(size_t batch, size_t time_step) {
		traj_costs[batch] += lambda/2 * sampled_controls[{batch, time_step, static_cast<size_t>(0)}] *
			sampled_controls[{batch, time_step, static_cast<size_t>(0)}] / (sample_std[{0, 0}] * sample_std[{0, 0}]);
		traj_costs[batch] += lambda/2 * sampled_controls[{batch, time_step, static_cast<size_t>(1)}] *
			sampled_controls[{batch, time_step, static_cast<size_t>(1)}] / (sample_std[{1, 1}] * sample_std[{1, 1}]);
		traj_costs[batch] += lambda * sampled_controls[{batch, time_step, static_cast<size_t>(0)}] *
			samples[{batch, time_step, static_cast<size_t>(0)}] / (sample_std[{0, 0}] * sample_std[{0, 0}]);
		traj_costs[batch] += lambda * sampled_controls[{batch, time_step, static_cast<size_t>(1)}] *
			samples[{batch, time_step, static_cast<size_t>(1)}] / (sample_std[{1, 1}] * sample_std[{1, 1}]);
	}


	// void Controller::excludeOutsideORCAConstraints(size_t batch) {
	// 	// If in a current state the agent cannot find a solution that satisfies the constraints (and we assume that the agent stops),
	// 	// then we check that the next state in the trajectory satisfies the constraints.
	// 	// If this does not happen, then we exclude the trajectories from the sample.
	//
	// 	if (not orca_complete) {
	// 		auto curr_x = generated_trajectories[{batch, static_cast<size_t>(1), static_cast<size_t>(0)}];
	// 		auto curr_y = generated_trajectories[{batch, static_cast<size_t>(1), static_cast<size_t>(1)}];
	// 		auto next_x = generated_trajectories[{batch, static_cast<size_t>(2), static_cast<size_t>(0)}];
	// 		auto next_y = generated_trajectories[{batch, static_cast<size_t>(2), static_cast<size_t>(1)}];
	//
	// 		xt::xtensor<double, 1> curr_v = xt::xtensor<double, 1>({next_x - curr_x, next_y - curr_y}) / dt;
	//
	// 		size_t line_num = linear_constraints.shape(0);
	// 		for (size_t line_id = 0; line_id < line_num; ++line_id) {
	// 			xt::xtensor<double, 1> line = xt::view(linear_constraints, line_id, xt::all());
	// 			if (line[0] * curr_v[0] + line[1] * curr_v[1] + line[2] > 0) {
	// 				excluded_traj[batch] = true;
	// 				break;
	// 			}
	// 		}
	// 	}
	// }



	void Controller::commonStateCost(size_t batch, size_t time_step, const xt::xtensor<double, 1> &initial_state,
									 const xt::xtensor<double, 1> &goal_state) {

		auto dist = xt::linalg::norm(
				xt::view(generated_trajectories, batch, time_step, xt::range(0, 2)) -
                    xt::view(goal_state, xt::range(0, 2))); // TODO: Add lookahead distance based cost

		traj_costs[batch] += common_run_cost_weight * dist;
	}

	void Controller::computeTerminalCosts(size_t batch, const xt::xtensor<double, 1> &goal_state) {
		if (excluded_traj[batch] or not orca_complete) return;
		commonTerminalCost(batch, goal_state);
	}

	void Controller::commonTerminalCost(size_t batch, const xt::xtensor<double, 1> &goal_state) {

		auto last_pos = xt::view(generated_trajectories, batch, -1, xt::range(0, 2));
		auto goal_pos = xt::view(goal_state, xt::range(0, 2));
		auto goal_vector = goal_pos - xt::view(generated_trajectories, batch, 0, xt::range(0, 2));
		double goal_vector_len = xt::linalg::norm(goal_vector, 2);

		double dist = 0.0;
		if (goal_vector_len > terminal_lookahead_distance) {
			auto curr_goal_pos = xt::view(generated_trajectories, batch, 0, xt::range(0, 2)) + terminal_lookahead_distance *
					(goal_vector / goal_vector_len);

			dist = xt::linalg::norm(last_pos - curr_goal_pos, 2);
		}
		else {
			dist = xt::linalg::norm(last_pos - goal_pos, 2);
		}


		traj_costs[batch] += common_terminal_cost_weight * dist;
	}

	bool Controller::generateStopControl() {
		// TODO
		return false;
	}

	bool Controller::computationShouldStop() {
		// TODO
		return false;
	}

	void Controller::updateStopComputationConditions() {
		// TODO
	}

	void Controller::updateControlSequence() {
        if (!obstacles.empty()) {
            double best_cost = std::numeric_limits<double>::infinity();
            size_t best = batch_size;
            for (size_t b = 0; b < batch_size; ++b)
                if (!excluded_traj[b] && traj_costs[b] < best_cost) { best_cost = traj_costs[b]; best = b; }
            if (best == batch_size) { control_sequence.fill(0.0); return; }
            xt::xtensor<double, 1> weights = xt::zeros<double>({batch_size});
            double total = 0;
            for (size_t b = 0; b < batch_size; ++b) {
                if (!excluded_traj[b]) weights[b] = std::exp(-(traj_costs[b]-best_cost)/lambda);
                total += weights[b];
            }
            weights /= total;
            control_sequence = xt::sum(xt::view(weights, xt::all(), xt::newaxis(), xt::newaxis()) * sampled_controls, {0},
                                      xt::evaluation_strategy::immediate);
            // Safe samples can average to a path through an obstacle.
            if (controlSequenceHitsObstacle()) control_sequence = xt::view(sampled_controls, best);
            return;
        }

		auto beta = xt::amin(traj_costs, xt::evaluation_strategy::immediate);
		auto eta =
				xt::sum(xt::exp(-1. / lambda * (traj_costs - beta)), {0}, xt::evaluation_strategy::immediate) + 1e-10;
		auto weights = xt::eval(xt::exp(-1. / lambda * (traj_costs - beta)) / eta);


		for (size_t batch = 0; batch < batch_size; ++batch) {
			if (excluded_traj[batch]) {
				weights[batch] = 0.0;
			}
		}


		control_sequence = xt::sum(xt::view(weights, xt::all(), xt::newaxis(), xt::newaxis()) * sampled_controls, {0},
								   xt::evaluation_strategy::immediate);
	}

	auto Controller::getCurrentControl() -> xt::xtensor<double, 1> {
		return xt::view(control_sequence, 0);
	}

	void Controller::shiftControlSequence() {
		using namespace xt::placeholders;
		xt::view(control_sequence, xt::range(_, -1), xt::all()) = xt::view(control_sequence, xt::range(1, _),
																		   xt::all());
		xt::view(control_sequence, -1, xt::all()) = initial_values;
	}

	void Controller::computeMainTrajectory() {
		// TODO
	}

	bool Controller::checkCollisionFree() {
		// TODO
		return false;
	}

	void Controller::setAlgParams(const MPPIParams &alg_params) {
		time_steps = alg_params.time_steps;
		batch_size = alg_params.batch_size;
		mean = alg_params.mean;
		sample_std = xt::sqrt(alg_params.cov);
		initial_values = alg_params.initial_values;
		dt = alg_params.dt;
		lambda = alg_params.lambda;
		common_run_cost_weight = alg_params.run_cost_weight;
		common_terminal_cost_weight = alg_params.terminal_cost_weight;
		ma_cost_en = alg_params.ma_cost_en;
		max_neighbors_num = alg_params.max_neighbors_num;
        max_safe_distr_step = alg_params.max_safe_distr_step;
		ma_dist_weight = alg_params.ma_dist_weight;
		ma_collision_value = alg_params.ma_collision_value;
		ma_dist_threshold = alg_params.ma_dist_threshold;
		k_alpha = alg_params.k_alpha;
		rad_eps = alg_params.rad_eps;
		use_car_like_dyn = alg_params.car_like_dyn;
		car_like_dyn.dist_between_axles = alg_params.dist_between_axles;
		velocity_cost_weight = alg_params.velocity_cost_weight;
		terminal_lookahead_distance = alg_params.terminal_lookahead_distance;
		ma_en_close_to_goal_dist = alg_params.ma_en_close_to_goal_dist;
		ma_en_close_to_goal_ts = alg_params.ma_en_close_to_goal_ts;
		orca_time_boundary = alg_params.orca_time_boundary;
		orca_responsibility_factor = alg_params.orca_responsibility_factor;
		ma_exclude_collision_traj = alg_params.ma_exclude_collision_traj;
		velocity_cost_close_to_goal_dist = alg_params.velocity_cost_close_to_goal_dist;
		reset();
	}

	void Controller::setAgentParams(const AgentParams &ag_params) {
        // TODO: Generalize to common dynamics
        agent_size = ag_params.size;
		vis_radius = ag_params.vis_radius;
		control_limits = {{ag_params.dd_v_min, ag_params.dd_v_max},
						  {ag_params.dd_w_min, ag_params.dd_w_max}};
		reset();
	}

	void Controller::propagateNeighboursTrajectories(const xt::xtensor<double, 2> &neighbors_states) {

		if (neighbors_states.shape(0) == 0) {
			neighbors_num = 0;
			return;
		}

		neighbors_num = (max_neighbors_num > neighbors_states.shape(0)) ? neighbors_states.shape(0) : max_neighbors_num;
		xt::view(neighbors_traj, xt::range(0, neighbors_num), 0, xt::all()) =
				xt::view(neighbors_states, xt::range(0, neighbors_num), xt::range(0, POSITION_DIM));

		for (size_t t = 1; t < time_steps; ++t)
			xt::view(neighbors_traj, xt::range(0, neighbors_num), t, xt::all()) = xt::view(neighbors_traj, xt::range(0, neighbors_num), 0, xt::all())
					+ xt::view(neighbors_states, xt::range(0, neighbors_num), xt::range(POSITION_DIM, _)) * t * dt ;
	}

	void Controller::resetNeighborsTrajectories() {
		neighbors_traj = xt::zeros<double>({max_neighbors_num, time_steps, POSITION_DIM});
	}

	void Controller::multiAgentCost(size_t batch, size_t time_step, const xt::xtensor<double, 1> &goal_state, const xt::xtensor<double, 1> &neighbors_sizes) {
		if (neighbors_num == 0)
			return;

		auto curr_pos = xt::view(generated_trajectories, batch, 0, xt::range(0, POSITION_DIM));
		auto goal_pos = xt::view(goal_state, xt::range(0, 2));
		auto goal_dist = xt::linalg::norm(curr_pos - goal_pos, 2);
        if (goal_dist < ma_en_close_to_goal_dist and time_step > ma_en_close_to_goal_ts)
			return;

		double min_dist = std::numeric_limits<double>::max();

		for (int agent = 0; agent < neighbors_num; ++agent) {
			double dist = xt::linalg::norm(
					xt::view(neighbors_traj, agent, time_step, xt::all()) -
					xt::view(generated_trajectories, batch, time_step, xt::range(0, POSITION_DIM))
					, 2);
			if (dist < min_dist)
				min_dist = dist;
			auto neighbor_size = neighbors_sizes[agent];
            if (dist < ( (neighbor_size + agent_size) + 2 * rad_eps)) {
            	if (ma_exclude_collision_traj)
            		excluded_traj[batch] = true;
				traj_costs[batch] = ma_collision_value;
				return;
			}
		}
		if (min_dist > ma_dist_threshold)
			return;
        traj_costs[batch] += ma_dist_weight * (1 / min_dist);
	}

	void Controller::computeLinearConstraintsForState(const xt::xtensor<double, 1> &state,
													  const xt::xtensor<double, 2> &neighbors_states,
													  const xt::xtensor<double, 1> &neighbors_sizes) {

		ORCAParams orca_params;
        orca_params.agents_max_num = max_neighbors_num;


        orca_params.agent_size =  (agent_size + rad_eps);
        orca_params.time_boundary = orca_time_boundary;
        orca_params.time_step = dt;
        orca_params.responsibility_factor = orca_responsibility_factor;
		orca_params.opt_zero_vel = false;

		size_t neighbors_num = (orca_params.agents_max_num < neighbors_states.shape(0)) ? orca_params.agents_max_num
																				   : neighbors_states.shape(0);


		orca_lines = computeORCALines(orca_params, state, neighbors_states, neighbors_sizes);

		linear_constraints = xt::zeros<double>({orca_lines.size(), static_cast<size_t>(3)});

		for (size_t i = 0; i < orca_lines.size(); ++i) {
			auto orca_line = orca_lines[i];
			double a = orca_line.dir[1];
			double b = -orca_line.dir[0];
			double c = -orca_line.dir[1]*orca_line.point_lies_on[0] + orca_line.dir[0]*orca_line.point_lies_on[1];

			if (orca_params.opt_zero_vel and c > 0) {
				c = 0;
			}

			linear_constraints[{i, static_cast<size_t>(0)}] = a;
			linear_constraints[{i, static_cast<size_t>(1)}] = b;
			linear_constraints[{i, static_cast<size_t>(2)}] = c;
		}
	}

	void Controller::resampleInitialWithConstraints() {
        // TODO: Generalize to common dynamics
		if (linear_constraints.shape(0) == 0) {
			return;
		}

		xt::xtensor<double, 1> first_state = xt::view(generated_trajectories, 0, 0);
		xt::xtensor<double, 1> first_control = xt::view(control_sequence, 0);

		bool feasible = false;
		double new_mean = 0.0;
		double new_std = 0.0;

		std::tie(feasible, new_mean, new_std) = computeNewDistr(first_state, first_control);
		orca_complete = feasible;

		xt::xtensor<double, 1>::shape_type control_row_shape = {batch_size};
		xt::view(samples, xt::all(), 0, 0) = xt::random::randn<double>(control_row_shape, new_mean, new_std);
	}

	std::tuple<bool, double, double>
	Controller::computeNewDistr(const xt::xtensor<double, 1> &state, const xt::xtensor<double, 1> &control) {
        // TODO: Generalize to common dynamics using SOCP
		using operations_research::MPSolver;
		using operations_research::MPVariable;
		using operations_research::MPConstraint;
		using operations_research::MPObjective;

		xt::xtensor<double, 1> coefs;
		xt::xtensor<double, 1> bounds;
		std::tie(coefs, bounds) = translateConstraintsToControlBounds(state);

		double mu0 = mean[0] + control[0];
		double sigma0 = sample_std[{0, 0}];
		double v_min = control_limits[{0, 0}];
		double v_max = control_limits[{0, 1}];

		std::unique_ptr<MPSolver> solver(MPSolver::CreateSolver("SCIP"));

		if (not solver) {
			std::cout << "SCIP solver unavailable.";
			exit(-1);
		}

		const double infinity = solver->infinity();

		auto variables = std::vector<MPVariable*>(4);
		variables[0] = solver->MakeNumVar(-infinity, infinity, "x0");
		variables[1] = solver->MakeNumVar(-0.00001, infinity, "x1");
		variables[2] = solver->MakeNumVar(-infinity, infinity, "x2");
		variables[3] = solver->MakeNumVar(-infinity, infinity, "x3");

		auto n_constr = coefs.shape(0);
		for (size_t i = 0; i < n_constr; ++i) {
			MPConstraint* const c0 = solver->MakeRowConstraint(-infinity, bounds[i]);
			c0->SetCoefficient(variables[0], coefs[i]);
			c0->SetCoefficient(variables[1], coefs[i] * -k_alpha);
		}
		MPConstraint* const c1 = solver->MakeRowConstraint(-infinity, v_max);
		c1->SetCoefficient(variables[0], 1.0);
		c1->SetCoefficient(variables[1], -k_alpha);
		MPConstraint* const c2 = solver->MakeRowConstraint(-infinity, -v_min);
		c2->SetCoefficient(variables[0], -1.0);
		c2->SetCoefficient(variables[1], -k_alpha);
		MPConstraint* const c3 = solver->MakeRowConstraint(-infinity, mu0);
		c3->SetCoefficient(variables[0], 1.0);
		c3->SetCoefficient(variables[1], 0.0);
		c3->SetCoefficient(variables[2], -1.0);
		c3->SetCoefficient(variables[3], 0.0);
		MPConstraint* const c4 = solver->MakeRowConstraint(-infinity, sigma0);
		c4->SetCoefficient(variables[0], 0.0);
		c4->SetCoefficient(variables[1], 1.0);
		c4->SetCoefficient(variables[2], 0.0);
		c4->SetCoefficient(variables[3], -1.0);
		MPConstraint* const c5 = solver->MakeRowConstraint(-infinity, -mu0);
		c5->SetCoefficient(variables[0], -1.0);
		c5->SetCoefficient(variables[1], 0.0);
		c5->SetCoefficient(variables[2], -1.0);
		c5->SetCoefficient(variables[3], 0.0);
		MPConstraint* const c6 = solver->MakeRowConstraint(-infinity, -sigma0);
		c6->SetCoefficient(variables[0], 0.0);
		c6->SetCoefficient(variables[1], -1.0);
		c6->SetCoefficient(variables[2], 0.0);
		c6->SetCoefficient(variables[3], -1.0);
		MPObjective* const objective = solver->MutableObjective();
		objective->SetCoefficient(variables[0], 0.0);
		objective->SetCoefficient(variables[1], 0.0);
		objective->SetCoefficient(variables[2], 1.0);
		objective->SetCoefficient(variables[3], 1.0);
		objective->SetMinimization();

		const MPSolver::ResultStatus result_status = solver->Solve();

		if (result_status != MPSolver::OPTIMAL and result_status != MPSolver::FEASIBLE) {
			return {false, -control[0], 0.0};
		}
		double result_std = variables[1]->solution_value();
		if (result_std < 0) result_std = 0.0;
		return {true, variables[0]->solution_value() - control[0], result_std};

	}

	std::tuple<xt::xtensor<double, 1>, xt::xtensor<double, 1>>
	Controller::translateConstraintsToControlBounds(const xt::xtensor<double, 1> &state) {

		size_t neighbors_num = linear_constraints.shape(0);
		xt::xtensor<double, 1> coefs = xt::zeros<double>({neighbors_num});
		xt::xtensor<double, 1> bounds = xt::zeros<double>({neighbors_num});
		double a, b, c, theta = state[2];

		for (size_t line_num = 0; line_num < neighbors_num; ++line_num) {
			a = linear_constraints[{line_num, static_cast<size_t>(0)}];
			b = linear_constraints[{line_num, static_cast<size_t>(1)}];
			c = linear_constraints[{line_num, static_cast<size_t>(2)}];
			coefs[line_num] = (a * cos(theta) + b * sin(theta));
			bounds[line_num] = -c;
		}
		return {coefs, bounds};
	}
}
