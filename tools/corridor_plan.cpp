#include "corridor_planner.hpp"
#include <algorithm>
#include <boost/property_tree/json_parser.hpp>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>

using namespace corridor;
namespace {
using Clock = std::chrono::steady_clock;
Vec2 point(const boost::property_tree::ptree &p) {
    if (p.size() != 2)
        throw std::invalid_argument("query coordinates must be 2D");
    auto i = p.begin();
    double x = (i++)->second.get_value<double>();
    return {x, i->second.get_value<double>()};
}
double seconds(Clock::time_point start) {
    return std::chrono::duration<double>(Clock::now() - start).count();
}
} // namespace
int main(int argc, char **argv) {
    try {
        std::string mapFile, queryFile, outputFile;
        PlannerConfig config;
        int k = 5;
        bool verify = false, debugLanes = false;
        for (int i = 1; i < argc; ++i) {
            std::string flag = argv[i];
            if (flag == "--verify") {
                verify = true;
                continue;
            }
            if (flag == "--debug-lanes") {
                debugLanes = true;
                continue;
            }
            if (flag == "--help") {
                std::cout << "corridor_plan --map SCHEMA_V2.json --queries QUERIES.json --output "
                             "ROUTES.json [--k 5] [--max-candidates 24] [--max-detour-ratio 1.8] "
                             "[--diversity-threshold .2] [--verify] [--debug-lanes]\n";
                return 0;
            }
            if (i + 1 >= argc)
                throw std::invalid_argument("missing option value");
            std::string value = argv[++i];
            if (flag == "--map")
                mapFile = value;
            else if (flag == "--queries")
                queryFile = value;
            else if (flag == "--output")
                outputFile = value;
            else if (flag == "--k")
                k = std::stoi(value);
            else if (flag == "--max-candidates")
                config.max_candidates = std::stoi(value);
            else if (flag == "--lane-max-samples")
                config.lanes.max_samples_per_edge = std::stoi(value);
            else if (flag == "--lane-sample-spacing")
                config.lanes.sample_spacing = std::stod(value);
            else if (flag == "--lane-max-offset")
                config.lanes.max_offset = std::stod(value);
            else if (flag == "--lane-min-split-length")
                config.lanes.min_split_length = std::stod(value);
            else if (flag == "--lane-transition-length")
                config.lanes.transition_length = std::stod(value);
            else if (flag == "--lane-hysteresis")
                config.lanes.hysteresis_length = std::stod(value);
            else if (flag == "--lane-separation-margin")
                config.lanes.separation_margin = std::stod(value);
            else if (flag == "--lane-clearance-guard")
                config.lanes.clearance_guard = std::stod(value);
            else if (flag == "--lane-offset-slope")
                config.lanes.max_offset_slope = std::stod(value);
            else if (flag == "--max-attachments")
                config.max_attachments = std::stoi(value);
            else if (flag == "--max-attachment-segments")
                config.max_attachment_segments = std::stoi(value);
            else if (flag == "--max-attachment-distance")
                config.max_attachment_distance = std::stod(value);
            else if (flag == "--min-route-separation")
                config.min_route_separation = std::stod(value);
            else if (flag == "--lambda-narrow")
                config.lambda_narrow = std::stod(value);
            else if (flag == "--lambda-bottleneck")
                config.lambda_bottleneck = std::stod(value);
            else if (flag == "--penalty-factor")
                config.penalty_factor = std::stod(value);
            else if (flag == "--max-detour-ratio")
                config.max_detour_ratio = std::stod(value);
            else if (flag == "--diversity-threshold")
                config.diversity_threshold = std::stod(value);
            else
                throw std::invalid_argument("unknown option: " + flag);
        }
        if (mapFile.empty() || queryFile.empty() || outputFile.empty())
            throw std::invalid_argument("--map, --queries and --output are required");
        auto start = Clock::now();
        const auto map = loadCorridorMap(mapFile);
        const double loadTime = seconds(start);
        boost::property_tree::ptree queries;
        boost::property_tree::read_json(queryFile, queries);
        std::ofstream out(outputFile);
        if (!out)
            throw std::runtime_error("cannot open output");
        out << std::setprecision(17) << "{\"map_load_seconds\":" << loadTime
            << ",\"config\":{\"max_candidates\":" << config.max_candidates
            << ",\"max_detour_ratio\":" << config.max_detour_ratio
            << ",\"diversity_threshold\":" << config.diversity_threshold
            << ",\"min_route_separation\":" << config.min_route_separation
            << ",\"lambda_narrow\":" << config.lambda_narrow
            << ",\"lambda_bottleneck\":" << config.lambda_bottleneck
            << ",\"penalty_factor\":" << config.penalty_factor
            << ",\"max_attachments\":" << config.max_attachments
            << ",\"max_attachment_segments\":" << config.max_attachment_segments
            << ",\"max_attachment_distance\":" << config.max_attachment_distance
            << ",\"lanes\":{\"sample_spacing\":" << config.lanes.sample_spacing
            << ",\"max_offset\":" << config.lanes.max_offset
            << ",\"min_split_length\":" << config.lanes.min_split_length
            << ",\"transition_length\":" << config.lanes.transition_length
            << ",\"hysteresis_length\":" << config.lanes.hysteresis_length
            << ",\"separation_margin\":" << config.lanes.separation_margin
            << ",\"clearance_guard\":" << config.lanes.clearance_guard
            << ",\"max_offset_slope\":" << config.lanes.max_offset_slope
            << ",\"miter_limit\":" << config.lanes.miter_limit
            << ",\"max_samples_per_edge\":" << config.lanes.max_samples_per_edge
            << ",\"offset_iterations\":" << config.lanes.offset_iterations
            << ",\"refinement_iterations\":" << config.lanes.refinement_iterations
            << "}},\"supported_modes\":[\"CenterBidirectional\",\"LeftOneWay\",\"RightOneWay\"],"
               "\"queries\":[";
        struct RouteSnapshot {
            double physical_length;
            std::vector<int> edges;
            uint64_t path_hash;
        };
        std::map<std::pair<std::string, int>, std::vector<RouteSnapshot>> observed;
        auto snapshot = [](const CompleteRoute &route) {
            uint64_t hash = 1469598103934665603ULL;
            for (const auto &point : route.path)
                for (double value : point) {
                    uint64_t bits;
                    std::memcpy(&bits, &value, sizeof(bits));
                    hash = (hash ^ bits) * 1099511628211ULL;
                }
            return RouteSnapshot{route.physical_metrics.length, route.edge_ids, hash};
        };
        bool first = true;
        for (const auto &record : queries.get_child("queries")) {
            const auto &q = record.second;
            const auto a = point(q.get_child("start")), b = point(q.get_child("goal"));
            RobotProfile robot{q.get<double>("radius", .3), q.get<double>("safety_margin", .05)};
            start = Clock::now();
            PlanningTimings timings;
            auto routes = planKRoutes(map, a, b, robot, q.get<int>("k", k), config, &timings);
            const double planning = seconds(start);
            start = Clock::now();
            if (verify) {
                for (const auto &r : routes) {
                    if (!validateCompleteRoute(map, r, robot, config) || r.path.front() != a ||
                        r.path.back() != b)
                        throw std::runtime_error("lane-aware complete route validation failed: " +
                                                 q.get<std::string>("name", "query"));
                    if (r.physical_metrics.length >
                        routes.front().physical_metrics.length * config.max_detour_ratio + 1e-7)
                        throw std::runtime_error("physical detour constraint violated");
                    double score = r.metrics.length +
                                   config.lambda_narrow * r.metrics.narrow_exposure +
                                   config.lambda_bottleneck * r.metrics.bottleneck_penalty;
                    if (std::abs(score - r.metrics.score) > 1e-6)
                        throw std::runtime_error("invalid route score");
                }
                for (size_t i = 0; i < routes.size(); ++i)
                    for (size_t j = i + 1; j < routes.size(); ++j)
                        if (1 - routeOverlap(routes[i], routes[j]) + 1e-8 <
                            config.diversity_threshold)
                            throw std::runtime_error("physical resource diversity violated");
            }
            if (verify && q.get<int>("k", k) > 0) {
                std::ostringstream queryKey;
                queryKey << std::setprecision(17) << a[0] << ':' << a[1] << ':' << b[0] << ':'
                         << b[1] << ':' << robot.radius << ':' << robot.safety_margin;
                std::vector<RouteSnapshot> snapshots;
                for (const auto &route : routes)
                    snapshots.push_back(snapshot(route));
                for (const auto &entry : observed)
                    if (entry.first.first == queryKey.str()) {
                        if (entry.second.empty() != snapshots.empty())
                            throw std::runtime_error("baseline/repetition feasibility changed");
                        if (!snapshots.empty() &&
                            (std::abs(entry.second.front().physical_length -
                                      snapshots.front().physical_length) > 1e-8 ||
                             entry.second.front().edges != snapshots.front().edges ||
                             entry.second.front().path_hash != snapshots.front().path_hash))
                            throw std::runtime_error("shortest physical baseline changed with K");
                        if (entry.first.second == q.get<int>("k", k)) {
                            if (entry.second.size() != snapshots.size())
                                throw std::runtime_error("repeat route count changed");
                            for (size_t i = 0; i < snapshots.size(); ++i)
                                if (entry.second[i].edges != snapshots[i].edges ||
                                    entry.second[i].path_hash != snapshots[i].path_hash)
                                    throw std::runtime_error("repeated lane geometry changed");
                        }
                    }
                observed.emplace(std::make_pair(queryKey.str(), q.get<int>("k", k)),
                                 std::move(snapshots));
            }
            const double validation = seconds(start);
            std::ostringstream quoted;
            boost::property_tree::ptree name;
            name.put("name", q.get<std::string>("name", "query"));
            boost::property_tree::write_json(quoted, name, false);
            std::string prefix = quoted.str();
            prefix.resize(prefix.find_last_of('}'));
            const auto feasibleEdges = map.feasiblePhysicalEdges(robot);
            if (!first)
                out << ',';
            first = false;
            out << prefix << ",\"start\":[" << a[0] << ',' << a[1] << "],\"goal\":[" << b[0] << ','
                << b[1] << "],\"radius\":" << robot.radius
                << ",\"safety_margin\":" << robot.safety_margin
                << ",\"planning_seconds\":" << planning
                << ",\"lane_placement_seconds\":" << timings.lane_placement_seconds
                << ",\"route_search_seconds\":" << timings.route_search_seconds
                << ",\"physical_route_reachable\":"
                << (timings.physical_route_reachable ? "true" : "false")
                << ",\"feasible_physical_edges\":" << feasibleEdges.size()
                << ",\"feasible_edge_ids\":[";
            for (size_t i = 0; i < feasibleEdges.size(); ++i) {
                if (i)
                    out << ',';
                out << feasibleEdges[i];
            }
            out << ']' << ",\"profile\":{\"stage_seconds\":{";
            const char *stageNames[] = {
                "attachment",  "graph_filtering",      "topology_search", "lane_decomposition",
                "transitions", "collision_validation", "reconstruction",  "other"};
            for (size_t i = 0; i < timings.stage_seconds.size(); ++i) {
                if (i)
                    out << ',';
                out << std::quoted(stageNames[i]) << ':' << timings.stage_seconds[i];
            }
            out << "},\"clearance_queries\":" << timings.clearance_queries
                << ",\"lane_builds\":" << timings.lane_builds
                << ",\"lane_cache_hits\":" << timings.lane_cache_hits
                << ",\"transition_builds\":" << timings.transition_builds
                << ",\"transition_cache_hits\":" << timings.transition_cache_hits
                << ",\"path_validation_calls\":" << timings.path_validation_calls
                << ",\"path_validation_failures\":" << timings.path_validation_failures
                << ",\"physical_candidates\":" << timings.physical_candidates << '}'
                << ",\"rejected_lane_routes\":" << timings.rejected_lane_routes
                << ",\"lane_failures\":[";
            for (size_t i = 0; i < timings.lane_failures.size(); ++i) {
                if (i)
                    out << ',';
                out << std::quoted(timings.lane_failures[i]);
            }
            out << "],\"rejected_split_candidates\":" << timings.rejected_split_candidates
                << ",\"split_failure_reasons\":[";
            for (size_t i = 0; i < timings.split_failure_reasons.size(); ++i) {
                if (i)
                    out << ',';
                out << std::quoted(timings.split_failure_reasons[i]);
            }
            out << "],\"routes\":" << routesJson(routes) << ",\"resource_overlap\":[";
            for (size_t i = 0; i < routes.size(); ++i) {
                if (i)
                    out << ',';
                out << '[';
                for (size_t j = 0; j < routes.size(); ++j) {
                    if (j)
                        out << ',';
                    out << routeOverlap(routes[i], routes[j]);
                }
                out << ']';
            }
            out << "],\"validation_passed\":" << (verify ? "true" : "false")
                << ",\"validation_seconds\":" << validation
                << ",\"lane_cache_entries\":" << map.laneCacheEntries()
                << ",\"transition_cache_entries\":" << map.transitionCacheEntries()
                << ",\"lane_decompositions\":[";
            if (debugLanes && q.get<int>("k", k) > 1 &&
                q.get<std::string>("name", "query").find("repeat") == std::string::npos) {
                std::set<const LaneDecomposition *> sections;
                int emitted = 0;
                for (const auto &r : routes)
                    for (const auto &segment : r.segments)
                        if (segment.decomposition && emitted < 6 &&
                            sections.insert(segment.decomposition.get()).second) {
                            if (emitted++)
                                out << ',';
                            out << laneDecompositionJson(*segment.decomposition);
                        }
            }
            out << "]}";
            std::cout << q.get<std::string>("name", "query") << ": " << routes.size() << " routes, "
                      << planning * 1000 << " ms (search " << timings.route_search_seconds * 1000
                      << ", lanes " << timings.lane_placement_seconds * 1000 << ")\n";
            for (const auto &r : routes)
                if (r.metrics.split_length == 0 && r.metrics.length > 1e-8) {
                    std::cout << "  route " << r.id << ": ZERO SPLIT;";
                    for (const auto &rejection : r.lane_rejections)
                        std::cout << ' ' << rejection.reason;
                    std::cout << '\n';
                }
        }
        out << "],\"corridor_sweeps\":[";
        bool firstCorridor = true;
        if (auto corridors = queries.get_child_optional("corridors"))
            for (const auto &record : *corridors) {
                const auto &c = record.second;
                std::vector<EdgeTraversal> traversals;
                for (const auto &item : c.get_child("traversals")) {
                    const auto &t = item.second;
                    const int id = t.get<int>("edge_id"), direction = t.get<int>("direction", 1);
                    const auto &edges = map.data().edges;
                    auto edge = std::find_if(edges.begin(), edges.end(),
                                             [&](const auto &e) { return e.id == id; });
                    if (edge == edges.end() || (direction != 1 && direction != -1))
                        throw std::invalid_argument("invalid diagnostic edge/direction");
                    const double length = edge->reference_distances.back();
                    traversals.push_back(
                        {id, direction, CorridorMode::CenterBidirectional,
                         t.get<double>("first_distance", direction > 0 ? 0 : length),
                         t.get<double>("last_distance", direction > 0 ? length : 0)});
                }
                auto laneConfig = config.lanes;
                laneConfig.taper_endpoints = c.get<bool>("taper_endpoints", false);
                for (const auto &size : c.get_child("radii")) {
                    RobotProfile robot{size.second.get_value<double>(),
                                       c.get<double>("safety_margin", .05)};
                    start = Clock::now();
                    const auto d = map.physicalSectionDecomposition(traversals, robot, laneConfig);
                    const double elapsed = seconds(start);
                    const bool centralValid = map.validatePath(d->source_reference, robot);
                    const bool forwardValid = map.validatePath(d->forward_lane, robot) &&
                                              !pathSelfIntersections(d->forward_lane);
                    const bool reverseValid = map.validatePath(d->reverse_lane, robot) &&
                                              !pathSelfIntersections(d->reverse_lane);
                    double split = 0, shared = 0, transition = 0, blocked = 0;
                    bool certified = true;
                    for (const auto &interval : d->intervals) {
                        const double length = interval.last_distance - interval.first_distance;
                        if (interval.state == LaneState::Split) {
                            split += length;
                            certified = certified && interval.validated && interval.forward_right &&
                                        interval.reverse_right && interval.sweeps_disjoint;
                        } else if (interval.state == LaneState::Shared)
                            shared += length;
                        else if (interval.state == LaneState::Transition)
                            transition += length;
                        else
                            blocked += length;
                    }
                    if (verify && !certified)
                        throw std::runtime_error("independent corridor pair certification failed");
                    if (!firstCorridor)
                        out << ',';
                    firstCorridor = false;
                    out << "{\"name\":" << std::quoted(c.get<std::string>("name"))
                        << ",\"traversals\":[";
                    for (size_t i = 0; i < traversals.size(); ++i) {
                        if (i)
                            out << ',';
                        const auto &t = traversals[i];
                        out << "{\"edge_id\":" << t.edge_id << ",\"direction\":" << t.direction
                            << ",\"first_distance\":" << t.first_distance
                            << ",\"last_distance\":" << t.last_distance << '}';
                    }
                    out << ']' << ",\"radius\":" << robot.radius
                        << ",\"safety_margin\":" << robot.safety_margin
                        << ",\"effective_radius\":" << robot.radius + robot.safety_margin
                        << ",\"taper_endpoints\":"
                        << (laneConfig.taper_endpoints ? "true" : "false")
                        << ",\"decomposition_seconds\":" << elapsed
                        << ",\"center_traversable\":" << (centralValid ? "true" : "false")
                        << ",\"forward_path_valid\":" << (forwardValid ? "true" : "false")
                        << ",\"reverse_path_valid\":" << (reverseValid ? "true" : "false")
                        << ",\"split_certified\":" << (certified ? "true" : "false")
                        << ",\"split_reference_length\":" << split
                        << ",\"shared_reference_length\":" << shared
                        << ",\"transition_reference_length\":" << transition
                        << ",\"blocked_reference_length\":" << blocked
                        << ",\"decomposition\":" << laneDecompositionJson(*d) << '}';
                }
            }
        out << "]}\n";
        std::cout << "Map loading and reusable collision cache: " << loadTime * 1000 << " ms\n";
        if (!out)
            throw std::runtime_error("failed writing output");
    } catch (const std::exception &error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
