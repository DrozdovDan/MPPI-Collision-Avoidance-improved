#include "corridor_planner.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <future>
#include <iomanip>
#include <iostream>
#include <stdexcept>

using namespace corridor;
namespace {
void check(bool ok, const std::string &why) {
    if (!ok)
        throw std::runtime_error(why);
}
double dist(const Vec2 &a, const Vec2 &b) {
    return std::hypot(a[0] - b[0], a[1] - b[1]);
}
RoutingEdge edge(int id, int u, int v, std::vector<Vec2> p, double clearance, int resource) {
    RoutingEdge e;
    e.id = id;
    e.u = u;
    e.v = v;
    e.reference_geometry = std::move(p);
    e.reference_distances = {0};
    for (size_t i = 1; i < e.reference_geometry.size(); ++i)
        e.reference_distances.push_back(e.reference_distances.back() +
                                        dist(e.reference_geometry[i - 1], e.reference_geometry[i]));
    e.stored_min_clearance = clearance;
    e.min_width = 2 * clearance;
    e.resources = {{resource, 0, e.reference_distances.back(), 1}};
    return e;
}
MapData straight(double halfWidth = 3) {
    MapData m;
    m.bounds = {0, -halfWidth, 30, halfWidth};
    m.nodes = {{0, {2, 0}}, {1, {28, 0}}};
    m.edges = {edge(7, 0, 1, {{2, 0}, {28, 0}}, halfWidth, 42)};
    m.resources = {{42, true, false, 2 * halfWidth, {m.edges[0].reference_geometry}, {}}};
    return m;
}
MapData obstacleGrid() {
    MapData m;
    m.bounds = {0, -4, 30, 4};
    for (int i = 0; i <= 13; ++i)
        m.nodes.push_back({i, {2.0 + 2 * i, 0}});
    for (int i = 0; i < 13; ++i) {
        double x = 2.0 + 2 * i;
        const double noise = (i % 2 ? 1 : -1) * 2e-8;
        auto e = edge(7 + i, i, i + 1,
                      {{x, 0}, {x + 1e-7, noise}, {x + .8, 0}, {x + 2 - 1e-7, -noise}, {x + 2, 0}},
                      1.5, 42 + i);
        m.resources.push_back({42 + i, true, false, 3, {e.reference_geometry}, {}});
        m.edges.push_back(std::move(e));
        m.obstacles.push_back({{x + .2, 1.5}, {x + 1.8, 1.5}, {x + 1.8, 4}, {x + .2, 4}});
        m.obstacles.push_back({{x + .2, -4}, {x + 1.8, -4}, {x + 1.8, -1.5}, {x + .2, -1.5}});
    }
    return m;
}
void placementRegressions() {
    const auto data = obstacleGrid();
    CorridorMap map(data);
    CompleteRoute route;
    for (const auto &e : data.edges) {
        RouteSegment source;
        source.physical_edge_id = e.id;
        source.geometry = source.reference_geometry = e.reference_geometry;
        source.reference_distances = e.reference_distances;
        route.segments.push_back(source);
        route.edge_ids.push_back(e.id);
        route.directions.push_back(1);
        route.modes.push_back(CorridorMode::CenterBidirectional);
        route.edge_traversals.push_back(
            {e.id, 1, CorridorMode::CenterBidirectional, 0, e.reference_distances.back()});
        route.metrics.length += e.reference_distances.back();
        for (const auto &p : e.reference_geometry)
            if (route.path.empty() || dist(route.path.back(), p) > 1e-8)
                route.path.push_back(p);
    }
    const auto physical = route.path;
    const auto ids = route.edge_ids;
    RobotProfile robot{.5, .05};
    PlannerConfig config;
    check(applyLanePlacement(map, robot, config, route), "fragmented grid conversion failed");
    check(validateCompleteRoute(map, route, robot, config), "fragmented grid validation failed");
    check(route.physical_path == physical && route.edge_ids == ids,
          "lane placement changed grid topology");
    check(route.metrics.length_inflation_ratio < 1.04 &&
              route.metrics.self_intersection_count == 0 &&
              route.metrics.max_lateral_deviation < .7 && route.metrics.split_length > 23,
          "tiny grid edges produced spikes, loops, or excess shared use");
    std::shared_ptr<const LaneDecomposition> certificate;
    for (const auto &s : route.segments)
        if (s.decomposition) {
            if (certificate)
                check(certificate == s.decomposition, "grid edges were offset independently");
            certificate = s.decomposition;
        }
    check(map.data().edges[0].reference_geometry == data.edges[0].reference_geometry,
          "lane cleaning mutated offline graph");
    auto duplicate = edge(100, 0, 1, {{2, 0}, {2, 0}, {2 + 1e-9, 1e-10}, {28, 0}}, 1.5, 42);
    const auto cleaned = buildLaneDecomposition(map, duplicate, robot, config.lanes);
    check(pathSelfIntersections(cleaned.forward_lane) == 0 && cleaned.center.size() > 2 &&
              cleaned.center[1][0] > 2.01,
          "duplicate/near-zero chords contaminated tangents");
    // The former via-center quadratic doubled back for nearly identical endpoints.
    const auto connection =
        buildLaneConnection(map, {10, -.66}, {10.01, -.66}, {10, 0}, robot, config.lanes);
    check(connection.size() == 2 && dist(connection[0], connection[1]) < .011,
          "near-coincident junction endpoints created a detour");
    check(pathSelfIntersections({{0, 0}, {2, 2}, {0, 2}, {2, 0}}) == 1 &&
              pathSelfIntersections({{0, 0}, {2, 0}, {1, 0}}) == 1,
          "intersection detector missed a crossing or reversal");
    auto forged = route;
    forged.path.insert(forged.path.begin() + 2, forged.path.front());
    check(!validateCompleteRoute(map, forged, robot, config), "validator accepted a loop");
    const CorridorMap open(straight());
    auto nearby = planKRoutes(open, {10, 1}, {10.2, 1}, {.2, .05}, 1);
    check(nearby.size() == 1 && validateCompleteRoute(open, nearby[0], {.2, .05}, {}) &&
              nearby[0].metrics.length_inflation_ratio < 1.01,
          "short shared shortcut detoured to a distant corridor projection");
    auto truncated = straight(.30251);
    truncated.nodes = {{0, {4, 0}}, {1, {6, 0}}};
    truncated.edges = {edge(7, 0, 1, {{4, 0}, {6, 0}}, .30251, 42)};
    truncated.resources[0].geometry = {truncated.edges[0].reference_geometry};
    const CorridorMap shortReference(truncated);
    const auto extended = planKRoutes(shortReference, {2, 0}, {10, 0}, {.1, .05}, 1);
    check(extended.size() == 1 &&
              validateCompleteRoute(shortReference, extended[0], {.1, .05}, {}) &&
              extended[0].metrics.split_length > 4 &&
              extended[0].metrics.lane_reference_length > 7.9 &&
              extended[0].metrics.max_lateral_deviation < .2 &&
              extended[0].edge_ids == std::vector<int>{7},
          "extended physical shortcut lost mandatory lanes to a truncated parent reference");
}
struct Case {
    std::string name;
    MapData map;
    Vec2 start, goal;
    RobotProfile robot{.5, .05};
};
std::vector<Case> cases() {
    std::vector<Case> c;
    c.push_back({"wide_straight", straight(), {2, 0}, {28, 0}});
    c.push_back({"near_threshold_wide", straight(1.105), {2, 0}, {28, 0}});
    c.push_back({"narrow_straight", straight(.9), {2, 0}, {28, 0}});
    auto neck = straight();
    neck.obstacles = {{{11, .9}, {19, .9}, {19, 3}, {11, 3}},
                      {{11, -3}, {19, -3}, {19, -.9}, {11, -.9}}};
    neck.edges[0].stored_min_clearance = .9;
    neck.edges[0].min_width = 1.8;
    c.push_back({"narrowing_widening", neck, {2, 0}, {28, 0}});
    MapData curve;
    curve.bounds = {-4, -4, 12, 12};
    Polygon inner{{0, 0}};
    std::vector<Vec2> arc;
    for (int i = 0; i <= 64; ++i) {
        double angle = 1.5707963267948966 * i / 64;
        inner.push_back({5 * std::cos(angle), 5 * std::sin(angle)});
        arc.push_back({8 * std::cos(angle), 8 * std::sin(angle)});
    }
    curve.obstacles = {inner};
    curve.nodes = {{0, arc.front()}, {1, arc.back()}};
    curve.edges = {edge(7, 0, 1, arc, 2, 42)};
    curve.resources = {{42, true, false, 4, {arc}, {}}};
    c.push_back({"curved_corridor", curve, arc.front(), arc.back()});
    MapData corner;
    corner.bounds = {0, 0, 14, 14};
    corner.obstacles = {{{0, 5}, {8, 5}, {8, 14}, {0, 14}},
                        {{0, 0}, {14, 0}, {14, 1}, {0, 1}},
                        {{12, 0}, {14, 0}, {14, 14}, {12, 14}}};
    corner.nodes = {{0, {2, 3}}, {1, {10, 11}}};
    corner.edges = {edge(7, 0, 1, {{2, 3}, {10, 3}, {10, 11}}, 2, 42)};
    corner.resources = {{42, true, false, 4, {corner.edges[0].reference_geometry}, {}}};
    c.push_back({"sharp_corner", corner, {2, 3}, {10, 11}});
    MapData tee;
    tee.bounds = {0, 0, 20, 20};
    tee.obstacles = {{{0, 0}, {20, 0}, {20, 8}, {0, 8}},
                     {{0, 12}, {8, 12}, {8, 20}, {0, 20}},
                     {{12, 12}, {20, 12}, {20, 20}, {12, 20}}};
    tee.nodes = {{0, {2, 10}}, {1, {10, 10}}, {2, {18, 10}}, {3, {10, 18}}};
    tee.edges = {edge(7, 0, 1, {{2, 10}, {10, 10}}, 2, 42),
                 edge(8, 1, 2, {{10, 10}, {18, 10}}, 2, 43),
                 edge(9, 1, 3, {{10, 10}, {10, 18}}, 2, 44)};
    for (const auto &e : tee.edges)
        tee.resources.push_back({e.id == 7   ? 42
                                 : e.id == 8 ? 43
                                             : 44,
                                 true,
                                 false,
                                 4,
                                 {e.reference_geometry},
                                 {}});
    c.push_back({"t_junction", tee, {2, 10}, {10, 18}});
    c.push_back({"opposing_robots", straight(), {28, 0}, {2, 0}});
    auto pocket = straight();
    pocket.obstacles = {{{0, .9}, {13, .9}, {13, 3}, {0, 3}},
                        {{15, .9}, {30, .9}, {30, 3}, {15, 3}},
                        {{0, -3}, {13, -3}, {13, -.9}, {0, -.9}},
                        {{15, -3}, {30, -3}, {30, -.9}, {15, -.9}}};
    pocket.edges[0].stored_min_clearance = .9;
    pocket.edges[0].min_width = 1.8;
    c.push_back({"short_wide_pocket", pocket, {2, 0}, {28, 0}});
    auto unsafe = straight();
    unsafe.obstacles = {{{10, .6}, {20, .6}, {20, 3}, {10, 3}},
                        {{10, -3}, {20, -3}, {20, -.6}, {10, -.6}}};
    unsafe.edges[0].stored_min_clearance = .6;
    unsafe.edges[0].min_width = 1.2;
    c.push_back({"center_valid_offset_blocked", unsafe, {2, 0}, {28, 0}});
    c.push_back({"smaller_radius", straight(.9), {2, 0}, {28, 0}, {.2, .05}});
    c.push_back({"larger_radius", straight(.9), {2, 0}, {28, 0}, {.6, .05}});
    // Off-center references require asymmetric offsets, not a fixed +/-R shift.
    auto biased = straight(2);
    biased.nodes = {{0, {2, -1.4}}, {1, {28, -1.4}}};
    biased.edges = {edge(7, 0, 1, {{2, -1.4}, {28, -1.4}}, .6, 42)};
    biased.resources[0].geometry = {biased.edges[0].reference_geometry};
    c.push_back({"asymmetric_clearance", biased, {2, -1.4}, {28, -1.4}});
    auto nearWall = straight(2);
    nearWall.nodes = {{0, {2, -1.445}}, {1, {28, -1.445}}};
    nearWall.edges = {edge(7, 0, 1, {{2, -1.445}, {28, -1.445}}, .555, 42)};
    nearWall.resources[0].geometry = {nearWall.edges[0].reference_geometry};
    c.push_back({"near_wall_reference", nearWall, {2, -1.445}, {28, -1.445}});
    c.push_back({"repeated_obstacle_grid", obstacleGrid(), {2, 0}, {28, 0}});
    auto marginal = straight();
    for (int i = 0; i < 5; ++i) {
        const double x = 6.0 * i;
        marginal.obstacles.push_back({{x, .30251}, {x + 5, .30251}, {x + 5, 3}, {x, 3}});
        marginal.obstacles.push_back({{x, -3}, {x + 5, -3}, {x + 5, -.30251}, {x, -.30251}});
    }
    marginal.edges[0].stored_min_clearance = .30251;
    marginal.edges[0].min_width = .60502;
    c.push_back({"marginal_obstacle_grid", marginal, {2, 0}, {28, 0}, {.1, .05}});
    auto bentReference = straight(.30251);
    bentReference.edges = {
        edge(7, 0, 1, {{2, 0}, {10, 0}, {10.15, .05}, {10.3, 0}, {28, 0}}, .25251, 42)};
    bentReference.resources[0].geometry = {bentReference.edges[0].reference_geometry};
    c.push_back({"near_threshold_bent_reference", bentReference, {2, 0}, {28, 0}, {.1, .05}});
    c.push_back(
        {"near_threshold_bent_reference_small_robot", bentReference, {2, 0}, {28, 0}, {.01, .05}});
    auto shortWide = straight();
    shortWide.bounds[2] = 10;
    shortWide.nodes[1].position = {8, 0};
    shortWide.edges = {edge(7, 0, 1, {{2, 0}, {8, 0}}, 3, 42)};
    shortWide.resources[0].geometry = {shortWide.edges[0].reference_geometry};
    c.push_back({"short_usable_wide", shortWide, {2, 0}, {8, 0}});
    MapData difficult;
    difficult.bounds = {0, 0, 40, 40};
    difficult.obstacles = {{{0, 0}, {40, 0}, {40, 8}, {0, 8}},
                           {{0, 12}, {26, 12}, {26, 40}, {0, 40}},
                           {{34, 12}, {40, 12}, {40, 40}, {34, 40}},
                           {{30.7, 10.8}, {32, 10.8}, {32, 11.8}, {30.7, 11.8}}};
    difficult.nodes = {{0, {2, 10}}, {1, {30, 10}}, {2, {30, 36}}};
    difficult.edges = {edge(7, 0, 1, {{2, 10}, {30, 10}}, 2, 42),
                       edge(9, 1, 2, {{30, 10}, {30, 36}}, .7, 44)};
    difficult.resources = {{42, true, false, 4, {difficult.edges[0].reference_geometry}, {}},
                           {44, true, false, 1.4, {difficult.edges[1].reference_geometry}, {}}};
    c.push_back({"difficult_junction_local", difficult, {2, 10}, {30, 36}});
    return c;
}
void point(std::ostream &o, const Vec2 &p) {
    o << '[' << p[0] << ',' << p[1] << ']';
}
void world(std::ostream &o, const MapData &m) {
    o << "{\"bounds\":[";
    for (size_t i = 0; i < 4; ++i) {
        if (i)
            o << ',';
        o << m.bounds[i];
    }
    o << "],\"obstacles\":[";
    for (size_t i = 0; i < m.obstacles.size(); ++i) {
        if (i)
            o << ',';
        o << '[';
        for (size_t j = 0; j < m.obstacles[i].size(); ++j) {
            if (j)
                o << ',';
            point(o, m.obstacles[i][j]);
        }
        o << ']';
    }
    o << "]}";
}
} // namespace
int main(int argc, char **argv) {
    try {
        std::ofstream output;
        if (argc == 2) {
            output.open(argv[1]);
            check(static_cast<bool>(output), "cannot open synthetic report");
            output << std::setprecision(17) << "{\"cases\":[";
        }
        PlannerConfig cfg;
        cfg.max_attachments = 1;
        placementRegressions();
        bool first = true;
        for (const auto &scenario : cases()) {
            CorridorMap map(scenario.map);
            const auto before = map.data().edges[0].reference_geometry;
            auto began = std::chrono::steady_clock::now();
            PlanningTimings timings;
            auto routes =
                planKRoutes(map, scenario.start, scenario.goal, scenario.robot, 3, cfg, &timings);
            double planning =
                std::chrono::duration<double>(std::chrono::steady_clock::now() - began).count();
            check(!routes.empty(), scenario.name + ": no complete lane-aware path");
            const auto &r = routes.front();
            check(r.path.front() == scenario.start && r.path.back() == scenario.goal,
                  scenario.name + ": endpoints changed");
            check(validateCompleteRoute(map, r, scenario.robot, cfg),
                  scenario.name + ": native full geometry validation failed");
            check(r.metrics.self_intersection_count == 0 &&
                      (scenario.name == "curved_corridor"
                           ? r.metrics.length < 1.30 * r.metrics.lane_reference_length
                           : r.metrics.length_inflation_ratio < 1.2) &&
                      std::abs(r.metrics.physical_length - r.physical_metrics.length) < 1e-8,
                  scenario.name + ": geometry inflation or diagnostic failure, ratio " +
                      std::to_string(r.metrics.length_inflation_ratio));
            auto d = map.laneDecomposition(7, scenario.robot, cfg.lanes);
            for (const auto &interval : d->intervals)
                if (interval.state == LaneState::Split)
                    check(interval.validated && interval.forward_right && interval.reverse_right &&
                              interval.sweeps_disjoint &&
                              interval.forward_clearance >
                                  scenario.robot.radius + scenario.robot.safety_margin &&
                              interval.reverse_clearance >
                                  scenario.robot.radius + scenario.robot.safety_margin,
                          scenario.name + ": missing directional sweep certificate");
            check(d.get() == map.laneDecomposition(7, scenario.robot, cfg.lanes).get(),
                  "lane cache not reused");
            double split = 0, shared = 0, transition = 0;
            for (const auto &section : d->intervals) {
                double ds = section.last_distance - section.first_distance;
                if (section.state == LaneState::Split)
                    split += ds;
                if (section.state == LaneState::Shared)
                    shared += ds;
                if (section.state == LaneState::Transition)
                    transition += ds;
            }
            if (scenario.name == "wide_straight" || scenario.name == "opposing_robots" ||
                scenario.name == "smaller_radius" || scenario.name == "asymmetric_clearance" ||
                scenario.name == "near_threshold_wide")
                check(split > 20 && shared == 0, scenario.name + ": split is mandatory");
            if (scenario.name == "marginal_obstacle_grid")
                check(split > 24 && r.metrics.split_length > 20 &&
                          r.metrics.min_opposing_separation > .3 && r.metrics.min_clearance > .15,
                      "near-threshold pockets erased a feasible consistent pair: independent=" +
                          std::to_string(split) +
                          ", route=" + std::to_string(r.metrics.split_length));
            if (scenario.name.find("near_threshold_bent_reference") == 0)
                check(split > 25 && r.metrics.split_length > 23 &&
                          r.metrics.min_opposing_separation >
                              2 * (scenario.robot.radius + scenario.robot.safety_margin),
                      "small reference bends erased a valid straight-frame pair");
            if (scenario.name == "wide_straight") {
                check(r.metrics.split_fraction > .75 && r.metrics.split_offset_min > 1.1 &&
                          r.metrics.min_opposing_separation > 2.4 &&
                          r.metrics.rejected_split_candidates == 0,
                      "wide road did not use half-capacity opposing lanes");
                auto reverse =
                    planKRoutes(map, scenario.goal, scenario.start, scenario.robot, 1, cfg);
                check(!reverse.empty() &&
                          validateCompleteRoute(map, reverse[0], scenario.robot, cfg),
                      "reverse keep-right path invalid");
                for (const auto &segment : r.segments)
                    if (segment.state == LaneState::Split)
                        for (const auto &p : segment.geometry)
                            check(p[1] < 0, "forward robot is on left side");
                for (const auto &segment : reverse[0].segments)
                    if (segment.state == LaneState::Split)
                        for (const auto &p : segment.geometry)
                            check(p[1] > 0, "reverse robot did not swap sides");
                auto forged = r;
                for (auto &segment : forged.segments)
                    if (segment.state == LaneState::Split) {
                        segment.state = LaneState::Shared;
                        segment.mode = CorridorMode::CenterBidirectional;
                        break;
                    }
                check(!validateCompleteRoute(map, forged, scenario.robot, cfg),
                      "validator accepted SHARED where cached SPLIT exists");
                CorridorMap concurrent(scenario.map);
                std::vector<std::future<std::vector<CompleteRoute>>> workers;
                for (int job = 0; job < 4; ++job)
                    workers.push_back(std::async(std::launch::async, [&] {
                        return planKRoutes(concurrent, scenario.start, scenario.goal,
                                           scenario.robot, 1, cfg);
                    }));
                for (auto &worker : workers) {
                    auto paths = worker.get();
                    check(paths.size() == 1 && paths[0].path == r.path,
                          "concurrent cache changed lane placement");
                }
                check(concurrent.laneCacheEntries() == 1,
                      "concurrent cache duplicated a profile entry");
                PlannerConfig other = cfg;
                other.lanes.min_split_length = 100;
                auto shared =
                    planKRoutes(map, scenario.start, scenario.goal, scenario.robot, 1, other);
                check(!shared.empty() && shared[0].metrics.split_length == 0,
                      "minimum traversal length ignored");
                check(!validateCompleteRoute(map, shared[0], scenario.robot, cfg),
                      "validator reused a SHARED certificate for different lane parameters");
            }
            if (scenario.name == "narrow_straight" || scenario.name == "larger_radius" ||
                scenario.name == "short_wide_pocket")
                check(split == 0 && transition == 0, scenario.name + ": shared is mandatory");
            if (scenario.name == "narrow_straight")
                check(!r.lane_diagnostics.empty() &&
                          std::any_of(r.lane_rejections.begin(), r.lane_rejections.end(),
                                      [](const auto &reason) {
                                          return reason.reason == "insufficient_lateral_capacity";
                                      }),
                      "narrow road failed to explain why SPLIT is impossible");
            if (scenario.name == "near_wall_reference")
                check(r.metrics.split_length > 18 && r.metrics.split_offset_min > 1e-5 &&
                          r.metrics.min_opposing_separation > 1.4 &&
                          r.metrics.min_split_clearance > .55,
                      "wall guard erased a feasible asymmetric right-hand lane");
            if (scenario.name == "short_usable_wide")
                check(r.metrics.split_length > 2 && r.metrics.rejected_split_candidates > 0 &&
                          r.metrics.min_opposing_separation > 1.1,
                      "short wide road abandoned a feasible compact pair");
            if (scenario.name == "difficult_junction_local") {
                auto splitAt = [&](int edgeId, double station) {
                    return std::any_of(r.segments.begin(), r.segments.end(), [&](const auto &s) {
                        return s.physical_edge_id == edgeId && s.state == LaneState::Split &&
                               station >= std::min(s.reference_distances.front(),
                                                   s.reference_distances.back()) &&
                               station <= std::max(s.reference_distances.front(),
                                                   s.reference_distances.back());
                    });
                };
                check(splitAt(7, 10) && splitAt(9, 20) && r.metrics.split_fraction > .5,
                      "difficult junction disabled splitting in distant wide corridors");
            }
            if (scenario.name == "narrowing_widening" ||
                scenario.name == "center_valid_offset_blocked")
                check(split > 5 && shared > 5 && transition > 1,
                      scenario.name + ": adaptive intervals missing");
            if (scenario.name == "curved_corridor")
                check(routes.size() == 1 && r.edge_ids == std::vector<int>{7},
                      "a free shortcut and its parent corridor counted as different topologies");
            if (scenario.name == "curved_corridor" || scenario.name == "sharp_corner")
                check(split > 3, scenario.name + ": no valid curved split");
            if (scenario.name == "sharp_corner")
                check(split > 15 && shared == 0 && r.metrics.max_lateral_deviation < 1.1,
                      "bounded corner refinement: split=" + std::to_string(split) +
                          " shared=" + std::to_string(shared) +
                          " max=" + std::to_string(r.metrics.max_lateral_deviation));
            if (scenario.name == "t_junction")
                check(std::any_of(r.segments.begin(), r.segments.end(),
                                  [](const auto &s) {
                                      return s.kind == SegmentKind::JunctionConnector &&
                                             s.geometry.size() > 1;
                                  }),
                      "T junction has no explicit lane connection");
            // Validate both *full* opposing SPLIT sweeps, including cross-interval
            // interactions. Transitions/shared endpoints are intentionally excluded.
            for (size_t i = 0; i < d->cells.size(); ++i)
                if (d->cells[i] == LaneState::Split)
                    for (size_t j = 0; j < d->cells.size(); ++j)
                        if (d->cells[j] == LaneState::Split)
                            check(segmentDistanceSquared(d->forward_lane[i], d->forward_lane[i + 1],
                                                         d->reverse_lane[j],
                                                         d->reverse_lane[j + 1]) >=
                                      std::pow(2 * (scenario.robot.radius +
                                                    scenario.robot.safety_margin) +
                                                   1e-8,
                                               2),
                                  scenario.name + ": opposing sweeps overlap");
            for (const auto &usage : r.physical_resource_traversals)
                check(std::find(r.resource_ids.begin(), r.resource_ids.end(), usage.resource_id) !=
                          r.resource_ids.end(),
                      "physical resource ID lost after placement");
            check(map.data().edges[0].reference_geometry == before, "preprocessed graph mutated");
            auto baseline = planKRoutes(map, scenario.start, scenario.goal, scenario.robot, 1, cfg);
            check(std::abs(baseline.front().physical_metrics.length - r.physical_metrics.length) <
                      1e-8,
                  "physical baseline changed with K");
            size_t entries = map.laneCacheEntries();
            planKRoutes(map, scenario.start, scenario.goal, scenario.robot, 3, cfg);
            check(entries == map.laneCacheEntries(), "repeat query rebuilt lane decomposition");
            if (output) {
                if (!first)
                    output << ',';
                first = false;
                output << "{\"name\":\"" << scenario.name << "\",\"map_geometry\":";
                world(output, scenario.map);
                output << ",\"start\":";
                point(output, scenario.start);
                output << ",\"goal\":";
                point(output, scenario.goal);
                output << ",\"radius\":" << scenario.robot.radius
                       << ",\"safety_margin\":" << scenario.robot.safety_margin
                       << ",\"planning_seconds\":" << planning
                       << ",\"lane_placement_seconds\":" << timings.lane_placement_seconds
                       << ",\"route_search_seconds\":" << timings.route_search_seconds
                       << ",\"routes\":" << routesJson(routes)
                       << ",\"decomposition\":" << laneDecompositionJson(*d) << '}';
            }
            std::cout << scenario.name << ": SPLIT " << split << ", SHARED " << shared
                      << ", transitions " << transition << ", complete length " << r.metrics.length
                      << ", physical " << r.physical_metrics.length << ", ratio "
                      << r.metrics.length_inflation_ratio << '\n';
        }
        if (output)
            output << "]}\n";
        std::cout << "All synthetic lane invariants passed\n";
    } catch (const std::exception &e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}
