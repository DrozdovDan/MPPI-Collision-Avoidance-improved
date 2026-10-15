#include "corridor_planner.hpp"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <stdexcept>

using namespace corridor;
namespace {
void require(bool ok, const std::string &why) {
    if (!ok)
        throw std::runtime_error(why);
}
MapData straightCorridor(double width, bool neck = false) {
    MapData m;
    m.bounds = {-20, -10, 60, 10};
    const double half = width / 2;
    m.obstacles = {{{-20, half}, {60, half}, {60, 10}, {-20, 10}},
                   {{-20, -10}, {60, -10}, {60, -half}, {-20, -half}}};
    if (neck) {
        m.obstacles.push_back({{16, .6}, {24, .6}, {24, half}, {16, half}});
        m.obstacles.push_back({{16, -half}, {24, -half}, {24, -.6}, {16, -.6}});
    }
    m.nodes = {{0, {0, 0}}, {1, {40, 0}}};
    const double clearance = neck ? .6 : half;
    m.edges = {{7, 0, 1, {{0, 0}, {40, 0}}, {0, 40}, clearance, 2 * clearance, {{42, 0, 40, 1}}}};
    m.resources = {{42, true, false, 2 * clearance, {{{0, 0}, {40, 0}}}, {}}};
    return m;
}
void point(std::ostream &out, const Vec2 &p) {
    out << '[' << p[0] << ',' << p[1] << ']';
}
void world(std::ostream &out, const MapData &m) {
    out << "{\"bounds\":[" << m.bounds[0] << ',' << m.bounds[1] << ',' << m.bounds[2] << ','
        << m.bounds[3] << "],\"obstacles\":[";
    for (size_t i = 0; i < m.obstacles.size(); ++i) {
        if (i)
            out << ',';
        out << '[';
        for (size_t j = 0; j < m.obstacles[i].size(); ++j) {
            if (j)
                out << ',';
            point(out, m.obstacles[i][j]);
        }
        out << ']';
    }
    out << "]}";
}
void checkPair(const CorridorMap &map, const LaneDecomposition &d, RobotProfile robot) {
    const double R = robot.radius + robot.safety_margin;
    for (const auto &interval : d.intervals)
        if (interval.state == LaneState::Split)
            require(interval.validated && interval.sweeps_disjoint && interval.forward_right &&
                        interval.reverse_right && interval.min_separation > 2 * R &&
                        interval.forward_offset_min > 0 && interval.reverse_offset_min > 0 &&
                        interval.forward_clearance > R && interval.reverse_clearance > R,
                    "radius sweep returned an uncertified pair");
    // Independent exhaustive continuous chord check, including different runs.
    for (size_t i = 0; i < d.cells.size(); ++i)
        if (d.cells[i] == LaneState::Split)
            for (size_t j = 0; j < d.cells.size(); ++j)
                if (d.cells[j] == LaneState::Split)
                    require(segmentDistanceSquared(d.forward_lane[i], d.forward_lane[i + 1],
                                                   d.reverse_lane[j],
                                                   d.reverse_lane[j + 1]) > 4 * R * R,
                            "opposing radius-sweep sweeps overlap");
    if (map.validatePath(d.center, robot))
        require(
            map.validatePath(d.forward_lane, robot) && map.validatePath(d.reverse_lane, robot) &&
                !pathSelfIntersections(d.forward_lane) && !pathSelfIntersections(d.reverse_lane),
            "traversable corridor lost continuity or transition clearance");
}
// Exhaustive polygon oracle checks the spatial-index optimization, including
// inside endpoints and segments crossing obstacles with both endpoints outside.
void clearanceOracle() {
    auto data = straightCorridor(6, true);
    for (int i = 0; i < 32; ++i) {
        double x = -17 + 2.2 * i;
        data.obstacles.push_back({{x, 4}, {x + .7, 4}, {x + .7, 7}, {x, 7}});
    }
    CorridorMap map(data);
    for (int i = 0; i < 900; ++i) {
        Vec2 a{-22.0 + (i * 37 % 850) * .1, -11.0 + (i * 53 % 230) * .1};
        Vec2 b{-22.0 + (i * 19 % 850) * .1, -11.0 + (i * 31 % 230) * .1};
        if (i % 3 == 0)
            b = a;
        double expected = std::max(0.0, std::min({a[0] + 20, a[1] + 10, 60 - a[0], 10 - a[1],
                                                  b[0] + 20, b[1] + 10, 60 - b[0], 10 - b[1]}));
        for (const auto &p : data.obstacles) {
            if (pointInsidePolygon(a, p) || pointInsidePolygon(b, p))
                expected = 0;
            for (size_t j = 0; j < p.size(); ++j)
                expected = std::min(
                    expected, std::sqrt(segmentDistanceSquared(a, b, p[j], p[(j + 1) % p.size()])));
        }
        require(std::abs(map.segmentClearance(a, b) - expected) < 1e-10,
                "indexed clearance differs from exhaustive polygon oracle");
    }
}
} // namespace
int main(int argc, char **argv) {
    try {
        clearanceOracle();
        std::ofstream out;
        if (argc == 2) {
            out.open(argv[1]);
            require(static_cast<bool>(out), "cannot open radius report");
            out << std::setprecision(17) << "{\"cases\":[";
        }
        bool first = true;
        const std::vector<double> radii{.1,       .2,  .249999,  .25,  .250001,  .35,      .5,
                                        .549999,  .55, .550001,  .75,  1,        1.449999, 1.45,
                                        1.450001, 2.9, 2.949999, 2.95, 2.950001, 3.1};
        for (const std::string name : {"wide_W6", "threshold_W1.2", "variable_W6_neck1.2"}) {
            const bool neck = name.find("neck") != std::string::npos;
            const double W = name == "threshold_W1.2" ? 1.2 : 6;
            const auto data = straightCorridor(W, neck);
            const CorridorMap map(data);
            const auto reference = map.data().edges[0].reference_geometry;
            bool wasPassable = true;
            for (double radius : radii) {
                RobotProfile robot{radius, .05};
                const double R = radius + .05;
                const auto d = map.laneDecomposition(7, robot);
                checkPair(map, *d, robot);
                const bool passable = map.validatePath(reference, robot);
                require(!passable || wasPassable,
                        "fixed-corridor footprint feasibility is nonmonotonic");
                wasPassable = passable;
                const double throat = neck ? 1.2 : W;
                require(passable == (throat > 2 * R + 2e-8), "wrong 2R traversal threshold");
                double split = 0, shared = 0, blocked = 0;
                for (const auto &i : d->intervals) {
                    const double length = i.last_distance - i.first_distance;
                    if (i.state == LaneState::Split)
                        split += length;
                    if (i.state == LaneState::Shared)
                        shared += length;
                    if (i.state == LaneState::Blocked)
                        blocked += length;
                }
                if (!neck) {
                    require((split > 39) == (W > 4 * R + 8e-8), "wrong 4R SPLIT threshold");
                    if (passable && W <= 4 * R)
                        require(shared > 39, "marginal/narrow path is not SHARED");
                    if (!passable)
                        require(blocked > 39 && split == 0, "impassable corridor not blocked");
                }
                PlannerConfig config;
                config.max_attachments = 1;
                PlanningTimings timings;
                const auto routes = planKRoutes(map, {0, 0}, {40, 0}, robot, 5, config, &timings);
                require(!routes.empty() == passable,
                        "physical query lost known straight-corridor feasibility");
                for (const auto &r : routes)
                    require(r.path.front() == Vec2{0, 0} && r.path.back() == Vec2{40, 0} &&
                                validateCompleteRoute(map, r, robot, config),
                            "invalid radius-sweep complete route");
                const double stageSum = std::accumulate(timings.stage_seconds.begin(),
                                                        timings.stage_seconds.end(), 0.0);
                require(stageSum <= timings.total_seconds + 1e-6 &&
                            stageSum >= timings.total_seconds - .002,
                        "exclusive profiler loses or double-counts time");
                require(map.feasiblePhysicalEdges(robot).size() == (passable ? 1u : 0u),
                        "edge feasibility disagrees with query filter");
                require(map.data().edges[0].reference_geometry == reference,
                        "radius sweep modified graph");
                if (out) {
                    if (!first)
                        out << ',';
                    first = false;
                    out << "{\"name\":" << std::quoted(name) << ",\"radius\":" << radius
                        << ",\"safety_margin\":0.05,\"effective_radius\":" << R
                        << ",\"width\":" << W
                        << ",\"center_traversable\":" << (passable ? "true" : "false")
                        << ",\"split_reference_length\":" << split
                        << ",\"shared_reference_length\":" << shared
                        << ",\"blocked_reference_length\":" << blocked
                        << ",\"planning_seconds\":" << timings.total_seconds
                        << ",\"routes\":" << routesJson(routes)
                        << ",\"decomposition\":" << laneDecompositionJson(*d)
                        << ",\"map_geometry\":";
                    world(out, data);
                    out << '}';
                }
            }
        }
        if (out)
            out << "]}\n";
        std::cout << "60 native radius/width cases, 900 clearance oracle cases and threshold "
                     "certificates passed\n";
    } catch (const std::exception &e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}
