#pragma once
#include "polygon_geometry.hpp"
#include <memory>
#include <string>
#include <vector>

namespace corridor {
using Vec2 = Point2;
enum class CorridorMode { CenterBidirectional, LeftOneWay, RightOneWay };
struct RobotProfile {
    double radius = 0.3, safety_margin = 0.05;
};
enum class LaneState { Shared, Split, Transition, Blocked };
enum class SegmentKind {
    Corridor,
    ModeTransition,
    StartConnector,
    JunctionConnector,
    GoalConnector,
    DirectConnector
};
struct LaneConfig {
    double sample_spacing = 0.4;
    double max_offset = 4.0;
    double clearance_guard = 0.01;
    double separation_margin = 0.02;
    double hysteresis_length = 0.03;
    double min_split_length = 1.5;
    double transition_length = 1.0;
    double max_offset_slope = 0.75;
    double miter_limit = 2.5;
    int max_samples_per_edge = 4096;
    int offset_iterations = 20;
    int refinement_iterations = 3;
    bool taper_endpoints = false; // finite entry/exit tapers for complete continuous sections
};
struct LaneInterval {
    double first_distance, last_distance;
    LaneState state;
    bool validated = false, forward_right = false, reverse_right = false, sweeps_disjoint = false;
    double forward_clearance = 0, reverse_clearance = 0, min_separation = 0;
    double forward_offset_min = 0, forward_offset_max = 0;
    double reverse_offset_min = 0, reverse_offset_max = 0;
    double forward_length = 0, reverse_length = 0;
    bool entry_transition_present = false, exit_transition_present = false;
    bool entry_transition_valid = false, exit_transition_valid = false;
};
struct LaneRejection {
    double first_distance = 0, last_distance = 0;
    std::string reason;
};
// Arrays use canonical u->v arclength. The reverse lane is traversed in reverse
// array order, and lies on the canonical left / reverse-travel right side.
struct LaneDecomposition {
    int physical_edge_id = -1;
    double effective_radius = 0;
    std::vector<double> distances;
    std::vector<Vec2> center, forward_lane, reverse_lane, source_reference;
    std::vector<LaneState> cells;
    std::vector<LaneInterval> intervals;
    std::vector<LaneRejection> rejections;
    double min_split_separation = 0;
};
struct PlannerConfig {
    LaneConfig lanes;
    int max_candidates = 24;
    double max_detour_ratio = 1.8;
    double diversity_threshold = 0.2; // minimum 1 - weighted resource overlap
    double lambda_narrow = 0.2, lambda_bottleneck = 0.5;
    double penalty_factor = 2.0;
    double min_route_separation = 0.5; // suppress tiny geometric variations (map units)
    int max_attachments = 12;
    int max_attachment_segments = 512;
    double max_attachment_distance = 15.0;
};
struct RouteMetrics {
    double length = 0, min_clearance = 0, narrow_exposure = 0, bottleneck_penalty = 0, score = 0;
    double shared_exposure = 0, split_length = 0, transition_length = 0, connector_length = 0;
    double physical_length = 0, length_inflation_ratio = 1, max_lateral_deviation = 0;
    int self_intersection_count = 0, mode_transition_count = 0;
    int split_interval_count = 0, shared_interval_count = 0, rejected_split_candidates = 0;
    double shared_length = 0, split_fraction = 0;
    double lane_reference_length = 0;
    double split_offset_mean = 0, split_offset_min = 0, split_offset_max = 0;
    double min_opposing_clearance = 0, min_opposing_separation = 0;
    double min_split_clearance = 0;
};
// Exclusive wall-clock buckets. Collision checks inside another stage are
// charged to CollisionValidation, rather than counted twice.
enum class PlanningStage {
    Attachment,
    GraphFiltering,
    TopologySearch,
    LaneDecomposition,
    Transitions,
    CollisionValidation,
    Reconstruction,
    Other,
    Count
};
struct PlanningTimings {
    double total_seconds = 0, lane_placement_seconds = 0, route_search_seconds = 0;
    std::array<double, static_cast<size_t>(PlanningStage::Count)> stage_seconds{};
    size_t clearance_queries = 0;
    size_t lane_builds = 0, lane_cache_hits = 0, transition_builds = 0, transition_cache_hits = 0;
    size_t path_validation_calls = 0, path_validation_failures = 0;
    int physical_candidates = 0;
    bool physical_route_reachable = false;
    int rejected_lane_routes = 0;
    std::vector<std::string> lane_failures;
    int rejected_split_candidates = 0;
    std::vector<std::string> split_failure_reasons;
};
struct RoutingNode {
    int id;
    Vec2 position;
};
struct ResourceInterval {
    int resource_id;
    double start, end;
    int direction;
};
struct RoutingEdge {
    int id, u, v;
    std::vector<Vec2> reference_geometry;
    std::vector<double> reference_distances;
    double stored_min_clearance = 0, min_width = 0;
    std::vector<ResourceInterval> resources;
};
struct TrafficResource {
    int id;
    bool narrow = true, closed = false;
    double min_width = 0;
    std::vector<std::vector<Vec2>> geometry;
    Polygon region;
};
struct MapData {
    std::array<double, 4> bounds{};
    double narrow_width = 3.0; // existing offline geometric width class, in map units
    std::vector<Polygon> obstacles;
    std::vector<RoutingNode> nodes;
    std::vector<RoutingEdge> edges;
    std::vector<TrafficResource> resources;
};
struct EdgeTraversal {
    int edge_id, direction;
    CorridorMode mode = CorridorMode::CenterBidirectional;
    double first_distance, last_distance; // distances on the original dense reference
};
struct ResourceTraversal {
    int resource_id, direction; // 0 = junction / transverse connector crossing
    Vec2 entry, exit;
    double route_start, route_end, traversal_length;
    int entry_edge_id = -1, exit_edge_id = -1; // -1 = off-graph connector
};
struct RouteSegment {
    int physical_edge_id = -1, traffic_resource_id = -1;
    std::vector<int> traffic_resource_ids;
    bool forward = true;
    CorridorMode mode = CorridorMode::CenterBidirectional;
    LaneState state = LaneState::Shared;
    SegmentKind kind = SegmentKind::Corridor;
    std::vector<Vec2> geometry, opposing_geometry, reference_geometry;
    std::vector<double> reference_distances;
    std::vector<double> certificate_distances; // stations on a continuous multi-edge section
    double min_clearance = 0, min_separation = 0;
    bool narrow_reference = false;
    double reference_origin = 0;
    std::shared_ptr<const LaneDecomposition> decomposition;
};
struct CompleteRoute {
    int id = 0;
    std::vector<Vec2> path, physical_path;
    std::vector<RouteSegment> segments;
    std::vector<int> edge_ids, directions, resource_ids;
    std::vector<CorridorMode> modes;
    std::vector<EdgeTraversal> edge_traversals;
    std::vector<ResourceTraversal> resource_traversals;
    RouteMetrics metrics, physical_metrics;
    std::vector<ResourceTraversal> physical_resource_traversals;
    std::vector<LaneRejection> lane_rejections;
    std::vector<std::string> lane_diagnostics;
};
// Immutable, reusable map. Copies share read-only spatial indexes and exact
// segment clearances computed at construction; concurrent queries are safe.
class CorridorMap {
  public:
    explicit CorridorMap(MapData data);
    const MapData &data() const;
    double segmentClearance(const Vec2 &a, const Vec2 &b) const;
    bool validatePath(const std::vector<Vec2> &path, const RobotProfile &robot) const;
    // Whole reference-edge feasibility under exactly the same clearance rules
    // as robot-specific graph filtering. Uses immutable precomputed geometry.
    std::vector<int> feasiblePhysicalEdges(const RobotProfile &robot) const;
    std::shared_ptr<const LaneDecomposition>
    laneDecomposition(int edge_id, const RobotProfile &robot, const LaneConfig &config = {}) const;
    std::shared_ptr<const LaneDecomposition> laneDecomposition(const std::vector<Vec2> &reference,
                                                               const RobotProfile &robot,
                                                               const LaneConfig &config = {}) const;
    std::shared_ptr<const LaneDecomposition>
    physicalSectionDecomposition(const std::vector<EdgeTraversal> &traversals,
                                 const RobotProfile &robot, const LaneConfig &config = {}) const;
    std::vector<Vec2> laneConnection(const Vec2 &from, const Vec2 &to, const Vec2 &via,
                                     const RobotProfile &robot,
                                     const LaneConfig &config = {}) const;
    size_t laneCacheEntries() const;
    size_t transitionCacheEntries() const;
    struct Impl;

  private:
    std::shared_ptr<const Impl> impl_;
    friend std::vector<CompleteRoute> planKRoutes(const CorridorMap &, const Vec2 &, const Vec2 &,
                                                  const RobotProfile &, int, const PlannerConfig &,
                                                  PlanningTimings *);
};
// No graph/ECM/resource extraction here: reads existing schema-v2 traffic JSON.
CorridorMap loadCorridorMap(const std::string &filename);
std::vector<CompleteRoute> planKRoutes(const CorridorMap &map, const Vec2 &start, const Vec2 &goal,
                                       const RobotProfile &robot, int k,
                                       const PlannerConfig &config = {},
                                       PlanningTimings *timings = nullptr);
// Weighted resource-length overlap. Falls back to physical edge-length overlap
// only when both routes have no resources; exact resource signatures also dedupe.
bool validateLaneConfig(const LaneConfig &config);
double pathSeparation(const std::vector<Vec2> &a, const std::vector<Vec2> &b);
int pathSelfIntersections(const std::vector<Vec2> &path);
// These helpers are implemented in C++; topology is chosen before placement.
LaneDecomposition buildLaneDecomposition(const CorridorMap &map, const RoutingEdge &edge,
                                         const RobotProfile &robot, const LaneConfig &config);
std::vector<Vec2> buildLaneConnection(const CorridorMap &map, const Vec2 &from, const Vec2 &to,
                                      const Vec2 &via, const RobotProfile &robot,
                                      const LaneConfig &config);
bool applyLanePlacement(const CorridorMap &map, const RobotProfile &robot,
                        const PlannerConfig &config, CompleteRoute &route);
bool validateCompleteRoute(const CorridorMap &map, const CompleteRoute &route,
                           const RobotProfile &robot, const PlannerConfig &config);
double routeOverlap(const CompleteRoute &a, const CompleteRoute &b);
std::string laneDecompositionJson(const LaneDecomposition &decomposition);
std::string routesJson(const std::vector<CompleteRoute> &routes);
} // namespace corridor
