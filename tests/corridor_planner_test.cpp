#include "corridor_planner.hpp"
#include <cmath>
#include <iostream>
#include <numeric>
#include <stdexcept>

using namespace corridor;
void require(bool condition, const char *message) {
    if (!condition)
        throw std::runtime_error(message);
}
RoutingEdge edge(int id, int u, int v, std::vector<Vec2> p, double clearance, int resource = -1) {
    RoutingEdge e;
    e.id = id;
    e.u = u;
    e.v = v;
    e.reference_geometry = std::move(p);
    e.reference_distances = {0};
    for (size_t i = 1; i < e.reference_geometry.size(); ++i)
        e.reference_distances.push_back(
            e.reference_distances.back() +
            std::hypot(e.reference_geometry[i][0] - e.reference_geometry[i - 1][0],
                       e.reference_geometry[i][1] - e.reference_geometry[i - 1][1]));
    e.stored_min_clearance = clearance;
    e.min_width = 2 * clearance;
    if (resource >= 0)
        e.resources.push_back({resource, 0, e.reference_distances.back(), 1});
    return e;
}
MapData diamond() {
    MapData d;
    d.bounds = {0, -10, 10, 10};
    d.obstacles = {{{4, -2}, {6, -2}, {6, 2}, {4, 2}}};
    d.nodes = {{0, {2, 0}}, {1, {8, 0}}};
    d.resources = {{10, true, false, 1.6, {}, {}}, {20, true, false, 1.6, {}, {}}};
    d.edges = {edge(7, 0, 1, {{2, 0}, {3, 4}, {7, 4}, {8, 0}}, .8, 10),
               edge(9, 0, 1, {{2, 0}, {3, -4}, {7, -4}, {8, 0}}, .8, 20)};
    for (size_t i = 0; i < d.edges.size(); ++i)
        d.resources[i].geometry = {d.edges[i].reference_geometry};
    return d;
}
int main() {
    try {
        CorridorMap map(diamond());
        const Vec2 start{1, 0}, goal{9, 0};
        RobotProfile robot{.3, .05};
        auto before = map.data().edges;
        PlanningTimings timings;
        auto routes = planKRoutes(map, start, goal, robot, 5, {}, &timings);
        require(timings.lane_placement_seconds > 0 && timings.route_search_seconds > 0 &&
                    std::abs(timings.total_seconds - timings.lane_placement_seconds -
                             timings.route_search_seconds) < 1e-9,
                "query timing stages do not account for search and lane construction");
        require(routes.size() == 2, "expected exactly two physical corridor alternatives");
        const double exclusive =
            std::accumulate(timings.stage_seconds.begin(), timings.stage_seconds.end(), 0.0);
        require(exclusive > 0 && exclusive <= timings.total_seconds &&
                    timings.total_seconds - exclusive < .001 && timings.clearance_queries > 0 &&
                    timings.lane_builds > 0 && timings.physical_route_reachable,
                "exclusive native profile is missing or double-counts stages");
        require(std::abs(routes[0].physical_metrics.length - (6 + 2 * std::sqrt(17.0))) < 1e-8,
                "Dijkstra did not recover the independently known shortest fixture length");
        auto shortest = planKRoutes(map, start, goal, robot, 1);
        require(std::abs(shortest[0].metrics.length - routes[0].metrics.length) < 1e-8,
                "baseline changed");
        for (const auto &r : routes) {
            require(r.path.front() == start && r.path.back() == goal, "wrong endpoints");
            require(map.validatePath(r.path, robot), "collision");
            double length = 0;
            for (size_t i = 1; i < r.path.size(); ++i)
                length +=
                    std::hypot(r.path[i][0] - r.path[i - 1][0], r.path[i][1] - r.path[i - 1][1]);
            require(std::abs(length - r.metrics.length) < 1e-7, "wrong geometric length");
            require(r.edge_ids.size() == r.modes.size() && r.edge_ids.size() == r.directions.size(),
                    "unaligned traversals");
            require(r.physical_resource_traversals.size() == 1,
                    "connector endpoint contaminated resource signature");
            require(r.metrics.min_clearance > robot.radius + robot.safety_margin,
                    "clearance too small");
        }
        require(routeOverlap(routes[0], routes[1]) == 0, "resource alternatives overlap");
        auto reverse = planKRoutes(map, goal, start, robot, 2);
        require(reverse.size() == 2, "reverse alternatives lost");
        require(reverse[0].directions[0] == -1 &&
                    reverse[0].physical_resource_traversals[0].direction == -1,
                "wrong reverse direction");
        require(map.data().edges.size() == before.size() &&
                    map.data().edges[0].reference_geometry == before[0].reference_geometry &&
                    map.data().edges[0].resources.size() == before[0].resources.size(),
                "shared graph mutated");
        PlannerConfig tight;
        tight.max_detour_ratio = 1;
        tight.diversity_threshold = .8;
        require(planKRoutes(map, start, goal, robot, 5, tight).size() == 2,
                "equal length alternatives rejected");
        require(planKRoutes(map, {5, 0}, goal, robot, 5).empty(), "start in obstacle accepted");
        require(planKRoutes(map, {.1, 0}, goal, robot, 5).empty(), "wall clearance ignored");
        require(planKRoutes(map, start, goal, {3, 0}, 5).empty(), "oversized robot accepted");
        require(planKRoutes(map, start, start, robot, 5).size() == 1, "coincident query");
        require(planKRoutes(map, start, goal, robot, 0).empty(), "K=0");
        auto direct = planKRoutes(map, {1, -7}, {2, -7}, robot, 5);
        require(direct.size() == 1 && std::abs(direct[0].metrics.length - 1) < 1e-8,
                "direct baseline or geometric deduplication failed");
        bool threw = false;
        try {
            planKRoutes(map, start, goal, {-1, 0}, 1);
        } catch (const std::invalid_argument &) {
            threw = true;
        }
        require(threw, "invalid radius accepted");
        MapData d;
        d.bounds = {0, 0, 12, 8};
        d.nodes = {{0, {.01, 4}}, {1, {10, 4}}};
        d.resources = {{1, true, false, .02, {}, {}}};
        d.edges = {edge(1, 0, 1, {{.01, 4}, {4, 4}, {10, 4}}, .01, 1)};
        CorridorMap taper(d);
        PlannerConfig local;
        local.max_attachments = 1;
        auto partial = planKRoutes(taper, {3, 3}, {9, 5}, {.2, .1}, 2, local);
        require(!partial.empty(), "safe partial edge discarded because remote endpoint is narrow");
        // Force graph use with a blocked direct chord; also verifies split clipping.
        d.obstacles = {{{5, 2}, {6, 2}, {6, 3.5}, {5, 3.5}}};
        d.nodes = {{0, {.01, 4}}, {1, {10, 4}}};
        CorridorMap partialMap(d);
        partial = planKRoutes(partialMap, {3, 3}, {9, 3}, {.2, .1}, 2, local);
        require(!partial.empty() && !partial[0].edge_traversals.empty(),
                "partial corridor attachment failed");
        require(partial[0].edge_traversals[0].first_distance > 2,
                "partial traversal starts at unsafe original endpoint");
        require(partial[0].physical_resource_traversals[0].traversal_length <
                    d.edges[0].reference_distances.back(),
                "resource interval was not clipped");
        // Continuous segment feasibility, not endpoint sample clearance.
        d.obstacles = {{{5, 3.8}, {6, 3.8}, {6, 4.2}, {5, 4.2}}};
        CorridorMap blocked(d);
        require(planKRoutes(blocked, {3, 4}, {9, 4}, {.2, 0}, 1, local).empty(),
                "edge passing through obstacle accepted");
        // Junction usage is clipped to the actual off-graph connector geometry.
        MapData empty;
        empty.bounds = {0, 0, 20, 20};
        empty.resources = {{4, false, false, 4, {}, {{8, 8}, {12, 8}, {12, 12}, {8, 12}}}};
        CorridorMap open(empty);
        auto straight = planKRoutes(open, {5, 10}, {15, 10}, {.2, 0}, 3);
        require(straight.size() == 1 && straight[0].resource_ids == std::vector<int>{4},
                "direct junction clipping failed");
        require(std::abs(straight[0].physical_resource_traversals[0].traversal_length - 4) < 1e-8,
                "direct resource exposure wrong");
        require(straight[0].physical_resource_traversals[0].entry == Vec2{8, 10},
                "direct resource entry wrong");
        MapData narrowDirect;
        narrowDirect.bounds = {0, 0, 20, 2};
        CorridorMap strip(narrowDirect);
        auto stripRoute = planKRoutes(strip, {5, 1}, {15, 1}, {.2, 0}, 1);
        require(stripRoute.size() == 1 && stripRoute[0].metrics.narrow_exposure >= 10,
                "narrow direct connector exposure omitted");
        require(stripRoute[0].resource_ids.empty(), "connector invented a resource region");
        // Preserve resource event order and merge a shared resource across edges,
        // including overlapping constraints and reverse traversal.
        MapData chain;
        chain.bounds = {0, 0, 20, 20};
        chain.nodes = {{0, {2, 10}}, {1, {10, 10}}, {2, {18, 10}}};
        chain.resources = {{4, true, false, 4, {}, {}}, {5, false, false, 4, {}, {}}};
        chain.edges = {edge(4, 0, 1, {{2, 10}, {10, 10}}, 2, 4),
                       edge(5, 1, 2, {{10, 10}, {18, 10}}, 2, 4)};
        chain.edges[0].resources.push_back({5, 2, 6, 0});
        CorridorMap linked(chain);
        PlannerConfig nearest;
        nearest.max_attachments = 1;
        auto linkedRoute = planKRoutes(linked, {2, 9}, {18, 9}, {.2, 0}, 1, nearest);
        // The unobstructed direct path should still win globally.
        require(std::abs(linkedRoute[0].physical_metrics.length - 16) < 1e-8,
                "direct shortest path lost");
        chain.obstacles = {{{9, 8}, {11, 8}, {11, 9.5}, {9, 9.5}}};
        CorridorMap linkedBlocked(chain);
        linkedRoute = planKRoutes(linkedBlocked, {2, 9}, {18, 9}, {.2, 0}, 1, nearest);
        require(linkedRoute.size() == 1 && linkedRoute[0].physical_resource_traversals.size() == 2,
                "overlapping resource ordering/merging wrong");
        require(std::abs(linkedRoute[0].physical_resource_traversals[0].traversal_length - 16) <
                    1e-8,
                "cross-edge resource not merged");
        auto linkedReverse = planKRoutes(linkedBlocked, {18, 9}, {2, 9}, {.2, 0}, 1, nearest);
        require(linkedReverse[0].physical_resource_traversals[0].direction == -1,
                "merged reverse traversal direction wrong");
        d = diamond();
        d.edges[0].reference_distances.back() += 1;
        threw = false;
        try {
            CorridorMap malformed(d);
        } catch (const std::invalid_argument &) {
            threw = true;
        }
        require(threw, "corrupt arclength accepted");
        d = diamond();
        d.edges[0].reference_distances.front() = 1e-9;
        threw = false;
        try {
            CorridorMap malformed(d);
        } catch (const std::invalid_argument &) {
            threw = true;
        }
        require(threw, "nonzero arclength origin accepted");
        std::cout << "All native planner invariants passed\n";
    } catch (const std::exception &e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}
