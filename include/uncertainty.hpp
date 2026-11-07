#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <random>
#include <vector>

namespace mppica::uncertainty {

using Vec2 = std::array<double, 2>;
using State = std::array<double, 5>; // x, y, heading, world vx, world vy
using Neighbor = std::array<double, 4>; // x, y, world vx, world vy
using Plane = std::array<double, 3>; // ax * vx + ay * vy <= b

struct Params {
    std::size_t samples = 700;
    std::size_t horizon = 20; // number of controls, not states
    double dt = 0.1;
    double radius = 0.3;
    Vec2 lower = {-1.0, -2.0};
    Vec2 upper = {1.0, 2.0};
    Vec2 sampling_std = {1.0, 2.449489742783178};
    Vec2 execution_std = {0.1, 0.2};
    double position_std = 0.1;
    double observation_confidence = 0.9975;
    double execution_confidence = 0.999;
    double sampling_confidence = 0.999;
    double time_horizon = 1.0;
    double temperature = 0.25;
    double goal_weight = 395.0;
    double terminal_weight = 1600.0;
    double distance_weight = 750.0;
    double collision_weight = 1e7;
    double velocity_weight = 0.5;
    double distance_threshold = 3.5;
    double goal_projection_radius = 2.0;
    double near_goal_distance = 0.4;
    double extra_buffer = 0.0;
    bool uncertainty_aware = true;
    bool reject_first_step_tails = false;
    bool truncate_rollouts = false; // diagnostic replay of the former rejection sampler
    bool variance_objective = true; // literal covariance objective in paper Eq. (15)
    bool post_step_running_cost = false; // optional conventional MPPI rollout variant
};

struct Distribution {
    bool feasible;
    double mean;
    double std;
};

// Exact solution of min |mu-mu0|+|s-s0| (or |s*s-s0*s0|) subject to
// lower <= mu-z*s <= mu+z*s <= upper, s >= 0.
Distribution fitDistribution(double mean, double std, double lower, double upper, double z,
                             bool variance_objective = false);
double normalQuantile(double probability);
std::vector<Plane> orcaHalfplanes(const State& state, const std::vector<Neighbor>& neighbors,
                                 const std::vector<double>& radii, double self_radius,
                                 double time_horizon, double dt, bool zero_velocity);

struct Result {
    Vec2 control;
    bool feasible;
    Vec2 safe_interval;
    Vec2 first_mean;
    Vec2 first_std;
    double effective_samples;
    double best_cost;
    std::vector<double> trajectory; // best sampled trajectory, (horizon+1, 3)
    std::vector<Vec2> sampled_controls; // physical controls of the best rollout
    std::vector<Vec2> proposal_controls; // Gaussian controls before actuator clipping
};

class Controller {
public:
    explicit Controller(const Params& params, std::uint64_t seed = 0);
    std::size_t horizon() const { return p.horizon; }
    void reset(std::uint64_t seed);
    Result step(const State& state, const std::vector<Neighbor>& observations,
                const std::vector<double>& radii, const Vec2& goal,
                const std::vector<double>& predictions, // (neighbors, horizon+1, 2)
                const std::vector<double>& buffers);    // (neighbors, horizon+1)
private:
    Params p;
    std::mt19937_64 rng;
    std::vector<Vec2> sequence;
    double z_u, z_execution, observation_buffer;
    double draw(double mean, double std, double lower, double upper);
};

} // namespace mppica::uncertainty
