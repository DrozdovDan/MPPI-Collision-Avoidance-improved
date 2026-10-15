#include "corridor_planner.hpp"
#include <boost/property_tree/json_parser.hpp>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace corridor {
namespace {
using boost::property_tree::ptree;
Vec2 point(const ptree &value) {
    if (value.size() != 2)
        throw std::invalid_argument("expected 2D coordinate");
    auto it = value.begin();
    double x = (it++)->second.get_value<double>();
    return {x, it->second.get_value<double>()};
}
std::vector<Vec2> points(const ptree &value) {
    std::vector<Vec2> result;
    for (const auto &p : value)
        result.push_back(point(p.second));
    return result;
}
void writePoint(std::ostream &out, const Vec2 &p) {
    out << '[' << p[0] << ',' << p[1] << ']';
}
const char *modeName(CorridorMode m) {
    switch (m) {
    case CorridorMode::CenterBidirectional:
        return "CenterBidirectional";
    case CorridorMode::LeftOneWay:
        return "LeftOneWay";
    case CorridorMode::RightOneWay:
        return "RightOneWay";
    }
    return "CenterBidirectional";
}
} // namespace
CorridorMap loadCorridorMap(const std::string &filename) {
    ptree root;
    boost::property_tree::read_json(filename, root);
    if (root.get<int>("metadata.schema_version") != 2)
        throw std::invalid_argument("expected traffic geometry schema v2");
    MapData data;
    data.narrow_width = root.get<double>("metadata.config.narrow_width");
    const auto &world = root.get_child("map_geometry");
    const auto &bounds = world.get_child("bounds");
    if (bounds.size() != 4)
        throw std::invalid_argument("expected four map bounds");
    size_t i = 0;
    for (const auto &v : bounds)
        data.bounds[i++] = v.second.get_value<double>();
    for (const auto &polygon : world.get_child("obstacles"))
        data.obstacles.push_back(points(polygon.second));
    for (const auto &item : root.get_child("routing_geometry.nodes")) {
        const auto &n = item.second;
        data.nodes.push_back({n.get<int>("id"), point(n.get_child("position"))});
    }
    for (const char *key : {"resources", "junction_regions"})
        for (const auto &item : root.get_child(key)) {
            const auto &r = item.second;
            TrafficResource resource;
            resource.id = r.get<int>("id");
            const auto kind = r.get<std::string>("kind");
            if (kind != "narrow_corridor" && kind != "junction_region")
                throw std::invalid_argument("unknown resource kind");
            resource.narrow = kind == "narrow_corridor";
            resource.closed = r.get<bool>("closed");
            resource.min_width = r.get<double>("min_width");
            resource.region = points(r.get_child("region"));
            for (const auto &line : r.get_child("geometry"))
                resource.geometry.push_back(points(line.second));
            data.resources.push_back(std::move(resource));
        }
    const auto &mappings = root.get_child("edge_intervals");
    for (const auto &item : root.get_child("routing_geometry.edges")) {
        const auto &e = item.second;
        RoutingEdge edge;
        edge.id = e.get<int>("id");
        edge.u = e.get<int>("u");
        edge.v = e.get<int>("v");
        edge.reference_geometry = points(e.get_child("reference_geometry"));
        for (const auto &d : e.get_child("reference_distances"))
            edge.reference_distances.push_back(d.second.get_value<double>());
        edge.stored_min_clearance = e.get<double>("continuous_min_clearance");
        edge.min_width = e.get<double>("min_width");
        if (edge.reference_distances.empty() ||
            std::abs(edge.reference_distances.back() - e.get<double>("length")) > 1e-6)
            throw std::invalid_argument("edge length differs from reference arclength");
        for (const auto &piece : mappings.get_child(std::to_string(edge.id))) {
            const auto &r = piece.second;
            if (r.get<int>("edge_id") != edge.id)
                throw std::invalid_argument("resource mapping edge mismatch");
            edge.resources.push_back({r.get<int>("resource_id"), r.get<double>("start"),
                                      r.get<double>("end"), r.get<int>("direction")});
        }
        data.edges.push_back(std::move(edge));
    }
    if (mappings.size() != data.edges.size())
        throw std::invalid_argument("unexpected edge mapping count");
    return CorridorMap(std::move(data));
}
namespace {
const char *stateName(LaneState s) {
    switch (s) {
    case LaneState::Shared:
        return "SHARED";
    case LaneState::Split:
        return "SPLIT";
    case LaneState::Transition:
        return "TRANSITION";
    case LaneState::Blocked:
        return "BLOCKED";
    }
    return "BLOCKED";
}
const char *kindName(SegmentKind k) {
    switch (k) {
    case SegmentKind::Corridor:
        return "corridor";
    case SegmentKind::ModeTransition:
        return "mode_transition";
    case SegmentKind::StartConnector:
        return "start_connector";
    case SegmentKind::JunctionConnector:
        return "junction_connector";
    case SegmentKind::GoalConnector:
        return "goal_connector";
    case SegmentKind::DirectConnector:
        return "direct_connector";
    }
    return "corridor";
}
void writePath(std::ostream &out, const std::vector<Vec2> &p) {
    out << '[';
    for (size_t i = 0; i < p.size(); ++i) {
        if (i)
            out << ',';
        writePoint(out, p[i]);
    }
    out << ']';
}
void writeMetrics(std::ostream &out, const RouteMetrics &m) {
    out << "{\"length\":" << m.length << ",\"min_clearance\":" << m.min_clearance
        << ",\"narrow_exposure\":" << m.narrow_exposure
        << ",\"bottleneck_penalty\":" << m.bottleneck_penalty << ",\"score\":" << m.score
        << ",\"shared_exposure\":" << m.shared_exposure << ",\"split_length\":" << m.split_length
        << ",\"transition_length\":" << m.transition_length
        << ",\"connector_length\":" << m.connector_length
        << ",\"physical_length\":" << m.physical_length
        << ",\"length_inflation_ratio\":" << m.length_inflation_ratio
        << ",\"self_intersection_count\":" << m.self_intersection_count
        << ",\"max_lateral_deviation\":" << m.max_lateral_deviation
        << ",\"mode_transition_count\":" << m.mode_transition_count
        << ",\"split_interval_count\":" << m.split_interval_count
        << ",\"shared_interval_count\":" << m.shared_interval_count
        << ",\"shared_length\":" << m.shared_length << ",\"split_fraction\":" << m.split_fraction
        << ",\"lane_reference_length\":" << m.lane_reference_length
        << ",\"split_offset_mean\":" << m.split_offset_mean
        << ",\"split_offset_min\":" << m.split_offset_min
        << ",\"split_offset_max\":" << m.split_offset_max
        << ",\"min_opposing_clearance\":" << m.min_opposing_clearance
        << ",\"min_split_clearance\":" << m.min_split_clearance
        << ",\"min_opposing_separation\":" << m.min_opposing_separation
        << ",\"rejected_split_candidates\":" << m.rejected_split_candidates << '}';
}
void writeUsage(std::ostream &out, const std::vector<ResourceTraversal> &usages) {
    out << '[';
    for (size_t i = 0; i < usages.size(); ++i) {
        if (i)
            out << ',';
        const auto &u = usages[i];
        out << "{\"resource_id\":" << u.resource_id << ",\"direction\":" << u.direction
            << ",\"entry\":";
        writePoint(out, u.entry);
        out << ",\"exit\":";
        writePoint(out, u.exit);
        out << ",\"route_start\":" << u.route_start << ",\"route_end\":" << u.route_end
            << ",\"traversal_length\":" << u.traversal_length
            << ",\"entry_edge_id\":" << u.entry_edge_id << ",\"exit_edge_id\":" << u.exit_edge_id
            << '}';
    }
    out << ']';
}
} // namespace
std::string laneDecompositionJson(const LaneDecomposition &d) {
    std::ostringstream out;
    out << std::setprecision(17) << "{\"physical_edge_id\":" << d.physical_edge_id
        << ",\"effective_radius\":" << d.effective_radius << ",\"reference_distances\":[";
    for (size_t i = 0; i < d.distances.size(); ++i) {
        if (i)
            out << ',';
        out << d.distances[i];
    }
    out << "],\"center\":";
    writePath(out, d.center);
    out << ",\"forward_lane\":";
    writePath(out, d.forward_lane);
    out << ",\"reverse_lane\":";
    writePath(out, d.reverse_lane);
    out << ",\"intervals\":[";
    for (size_t i = 0; i < d.intervals.size(); ++i) {
        if (i)
            out << ',';
        const auto &section = d.intervals[i];
        out << "{\"first_distance\":" << section.first_distance
            << ",\"last_distance\":" << section.last_distance << ",\"state\":\""
            << stateName(section.state)
            << "\",\"validated\":" << (section.validated ? "true" : "false")
            << ",\"forward_right\":" << (section.forward_right ? "true" : "false")
            << ",\"reverse_right\":" << (section.reverse_right ? "true" : "false")
            << ",\"sweeps_disjoint\":" << (section.sweeps_disjoint ? "true" : "false")
            << ",\"forward_clearance\":" << section.forward_clearance
            << ",\"reverse_clearance\":" << section.reverse_clearance
            << ",\"min_separation\":" << section.min_separation
            << ",\"reference_length\":" << section.last_distance - section.first_distance
            << ",\"forward_length\":" << section.forward_length
            << ",\"reverse_length\":" << section.reverse_length
            << ",\"forward_offset_min\":" << section.forward_offset_min
            << ",\"forward_offset_max\":" << section.forward_offset_max
            << ",\"reverse_offset_min\":" << section.reverse_offset_min
            << ",\"reverse_offset_max\":" << section.reverse_offset_max
            << ",\"entry_transition_present\":"
            << (section.entry_transition_present ? "true" : "false")
            << ",\"exit_transition_present\":"
            << (section.exit_transition_present ? "true" : "false")
            << ",\"entry_transition_valid\":" << (section.entry_transition_valid ? "true" : "false")
            << ",\"exit_transition_valid\":" << (section.exit_transition_valid ? "true" : "false")
            << '}';
    }
    out << "],\"min_split_separation\":" << d.min_split_separation << ",\"rejections\":[";
    for (size_t i = 0; i < d.rejections.size(); ++i) {
        if (i)
            out << ',';
        const auto &r = d.rejections[i];
        out << "{\"first_distance\":" << r.first_distance
            << ",\"last_distance\":" << r.last_distance << ",\"reason\":" << std::quoted(r.reason)
            << '}';
    }
    out << "]}";
    return out.str();
}
std::string routesJson(const std::vector<CompleteRoute> &routes) {
    std::ostringstream out;
    out << std::setprecision(17) << '[';
    for (size_t j = 0; j < routes.size(); ++j) {
        if (j)
            out << ',';
        const auto &r = routes[j];
        out << "{\"id\":" << r.id << ",\"path\":";
        writePath(out, r.path);
        out << ",\"physical_path\":";
        writePath(out, r.physical_path);
        out << ",\"edge_ids\":[";
        for (size_t i = 0; i < r.edge_ids.size(); ++i) {
            if (i)
                out << ',';
            out << r.edge_ids[i];
        }
        out << "],\"directions\":[";
        for (size_t i = 0; i < r.directions.size(); ++i) {
            if (i)
                out << ',';
            out << r.directions[i];
        }
        out << "],\"modes\":[";
        for (size_t i = 0; i < r.modes.size(); ++i) {
            if (i)
                out << ',';
            out << '"' << modeName(r.modes[i]) << '"';
        }
        out << "],\"resource_ids\":[";
        for (size_t i = 0; i < r.resource_ids.size(); ++i) {
            if (i)
                out << ',';
            out << r.resource_ids[i];
        }
        out << "],\"edge_traversals\":[";
        for (size_t i = 0; i < r.edge_traversals.size(); ++i) {
            if (i)
                out << ',';
            const auto &e = r.edge_traversals[i];
            out << "{\"edge_id\":" << e.edge_id << ",\"direction\":" << e.direction
                << ",\"mode\":\"" << modeName(e.mode)
                << "\",\"first_distance\":" << e.first_distance
                << ",\"last_distance\":" << e.last_distance << '}';
        }
        out << "],\"segments\":[";
        for (size_t i = 0; i < r.segments.size(); ++i) {
            if (i)
                out << ',';
            const auto &s = r.segments[i];
            out << "{\"physical_edge_id\":" << s.physical_edge_id
                << ",\"traffic_resource_id\":" << s.traffic_resource_id
                << ",\"forward\":" << (s.forward ? "true" : "false") << ",\"state\":\""
                << stateName(s.state) << "\",\"mode\":\"" << modeName(s.mode) << "\",\"kind\":\""
                << kindName(s.kind) << "\",\"geometry\":";
            writePath(out, s.geometry);
            out << ",\"opposing_geometry\":";
            writePath(out, s.opposing_geometry);
            out << ",\"reference_geometry\":";
            writePath(out, s.reference_geometry);
            out << ",\"reference_distances\":[";
            for (size_t k = 0; k < s.reference_distances.size(); ++k) {
                if (k)
                    out << ',';
                out << s.reference_distances[k];
            }
            out << "],\"certificate_distances\":[";
            for (size_t k = 0; k < s.certificate_distances.size(); ++k) {
                if (k)
                    out << ',';
                out << s.certificate_distances[k];
            }
            out << "],\"traffic_resource_ids\":[";
            for (size_t k = 0; k < s.traffic_resource_ids.size(); ++k) {
                if (k)
                    out << ',';
                out << s.traffic_resource_ids[k];
            }
            out << "],\"min_clearance\":" << s.min_clearance
                << ",\"min_separation\":" << s.min_separation << '}';
        }
        out << "],\"resource_traversals\":";
        writeUsage(out, r.resource_traversals);
        out << ",\"physical_resource_traversals\":";
        writeUsage(out, r.physical_resource_traversals);
        out << ",\"metrics\":";
        writeMetrics(out, r.metrics);
        out << ",\"physical_metrics\":";
        writeMetrics(out, r.physical_metrics);
        out << ",\"lane_rejections\":[";
        for (size_t i = 0; i < r.lane_rejections.size(); ++i) {
            if (i)
                out << ',';
            const auto &reject = r.lane_rejections[i];
            out << "{\"first_distance\":" << reject.first_distance
                << ",\"last_distance\":" << reject.last_distance
                << ",\"reason\":" << std::quoted(reject.reason) << '}';
        }
        out << "],\"lane_diagnostics\":[";
        for (size_t i = 0; i < r.lane_diagnostics.size(); ++i) {
            if (i)
                out << ',';
            out << std::quoted(r.lane_diagnostics[i]);
        }
        out << ']';
        out << '}';
    }
    out << ']';
    return out.str();
}
} // namespace corridor
