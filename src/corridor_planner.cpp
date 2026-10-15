#include "corridor_planner.hpp"
#include "corridor_profile.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <iomanip>
#include <limits>
#include <map>
#include <mutex>
#include <numeric>
#include <queue>
#include <set>
#include <sstream>
#include <stdexcept>
#include <unordered_map>

namespace corridor {
namespace {
constexpr double eps = 1e-8;
constexpr double inf = std::numeric_limits<double>::infinity();
double distance(const Vec2 &a, const Vec2 &b) {
    return std::hypot(a[0] - b[0], a[1] - b[1]);
}
bool finite(const Vec2 &p) {
    return std::isfinite(p[0]) && std::isfinite(p[1]);
}
Vec2 lerp(const Vec2 &a, const Vec2 &b, double t) {
    return {a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])};
}
double projection(const Vec2 &p, const Vec2 &a, const Vec2 &b) {
    const double dx = b[0] - a[0], dy = b[1] - a[1], d = dx * dx + dy * dy;
    return d == 0 ? 0 : std::clamp(((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / d, 0.0, 1.0);
}
struct Box {
    double x0 = inf, y0 = inf, x1 = -inf, y1 = -inf;
    void add(const Vec2 &p) {
        x0 = std::min(x0, p[0]);
        y0 = std::min(y0, p[1]);
        x1 = std::max(x1, p[0]);
        y1 = std::max(y1, p[1]);
    }
    void add(const Box &b) {
        add(Vec2{b.x0, b.y0});
        add(Vec2{b.x1, b.y1});
    }
    double squaredDistance(const Box &b) const {
        double dx = std::max({0.0, x0 - b.x1, b.x0 - x1}),
               dy = std::max({0.0, y0 - b.y1, b.y0 - y1});
        return dx * dx + dy * dy;
    }
    bool contains(const Vec2 &p) const {
        return p[0] >= x0 && p[0] <= x1 && p[1] >= y0 && p[1] <= y1;
    }
};
Box box(const Vec2 &a, const Vec2 &b) {
    Box result;
    result.add(a);
    result.add(b);
    return result;
}
// Small immutable AABB tree; shared by all robot profiles and queries.
struct Tree {
    struct Node {
        Box box;
        int first, last, left = -1, right = -1;
    };
    std::vector<Box> boxes;
    std::vector<int> order;
    std::vector<Node> nodes;
    explicit Tree(std::vector<Box> b = {}) : boxes(std::move(b)), order(boxes.size()) {
        std::iota(order.begin(), order.end(), 0);
        if (!order.empty())
            build(0, static_cast<int>(order.size()));
    }
    int build(int first, int last) {
        Box b;
        for (int i = first; i < last; ++i)
            b.add(boxes[order[i]]);
        int index = static_cast<int>(nodes.size());
        nodes.push_back({b, first, last});
        if (last - first > 8) {
            const bool x = b.x1 - b.x0 > b.y1 - b.y0;
            const int mid = (first + last) / 2;
            std::nth_element(order.begin() + first, order.begin() + mid, order.begin() + last,
                             [&](int a, int c) {
                                 return x ? boxes[a].x0 + boxes[a].x1 < boxes[c].x0 + boxes[c].x1
                                          : boxes[a].y0 + boxes[a].y1 < boxes[c].y0 + boxes[c].y1;
                             });
            const int left = build(first, mid), right = build(mid, last);
            nodes[index].left = left;
            nodes[index].right = right;
        }
        return index;
    }
    template <typename F> double nearest(const Box &query, F exact) const {
        double best = inf;
        auto visit = [&](auto &&self, int i) -> void {
            const auto &n = nodes[i];
            if (n.box.squaredDistance(query) >= best)
                return;
            if (n.left < 0) {
                for (int j = n.first; j < n.last; ++j)
                    if (boxes[order[j]].squaredDistance(query) < best)
                        best = std::min(best, exact(order[j]));
            } else {
                int a = n.left, b = n.right;
                if (nodes[a].box.squaredDistance(query) > nodes[b].box.squaredDistance(query))
                    std::swap(a, b);
                self(self, a);
                self(self, b);
            }
        };
        if (!nodes.empty())
            visit(visit, 0);
        return best;
    }
    template <typename F> bool anyContainingEndpoint(const Vec2 &a, const Vec2 &b, F exact) const {
        auto visit = [&](auto &&self, int i) -> bool {
            const auto &n = nodes[i];
            if (!n.box.contains(a) && !n.box.contains(b))
                return false;
            if (n.left < 0) {
                for (int j = n.first; j < n.last; ++j) {
                    const int id = order[j];
                    if ((boxes[id].contains(a) || boxes[id].contains(b)) && exact(id))
                        return true;
                }
                return false;
            }
            return self(self, n.left) || self(self, n.right);
        };
        return !nodes.empty() && visit(visit, 0);
    }
    // Best-first nearby segment enumeration, bounded independently of graph size.
    std::vector<int> nearby(const Vec2 &p, int limit, double radius,
                            const std::function<double(int)> &exact) const {
        using Item = std::tuple<double, bool, int>;
        std::priority_queue<Item, std::vector<Item>, std::greater<Item>> queue;
        std::vector<int> result;
        const auto q = box(p, p);
        if (!nodes.empty())
            queue.emplace(nodes[0].box.squaredDistance(q), false, 0);
        while (!queue.empty() && static_cast<int>(result.size()) < limit) {
            auto [d, leaf, id] = queue.top();
            queue.pop();
            if (d > radius * radius)
                break;
            if (leaf) {
                result.push_back(id);
                continue;
            }
            const auto &n = nodes[id];
            if (n.left < 0)
                for (int j = n.first; j < n.last; ++j)
                    queue.emplace(exact(order[j]), true, order[j]);
            else {
                queue.emplace(nodes[n.left].box.squaredDistance(q), false, n.left);
                queue.emplace(nodes[n.right].box.squaredDistance(q), false, n.right);
            }
        }
        return result;
    }
};
struct Segment {
    Vec2 a, b;
    int edge = -1, index = -1;
};
Vec2 at(const RoutingEdge &e, double d) {
    d = std::clamp(d, 0.0, e.reference_distances.back());
    auto it = std::upper_bound(e.reference_distances.begin(), e.reference_distances.end(), d);
    if (it == e.reference_distances.end())
        return e.reference_geometry.back();
    const auto i = static_cast<size_t>(it - e.reference_distances.begin());
    return lerp(e.reference_geometry[i - 1], e.reference_geometry[i],
                (d - e.reference_distances[i - 1]) /
                    (e.reference_distances[i] - e.reference_distances[i - 1]));
}
std::vector<Vec2> slice(const RoutingEdge &e, double a, double b) {
    const bool reverse = b < a;
    double lo = std::min(a, b), hi = std::max(a, b);
    std::vector<Vec2> p{at(e, lo)};
    auto first =
        std::upper_bound(e.reference_distances.begin(), e.reference_distances.end(), lo + eps);
    for (auto it = first; it != e.reference_distances.end() && *it < hi - eps; ++it)
        p.push_back(e.reference_geometry[it - e.reference_distances.begin()]);
    if (distance(p.back(), at(e, hi)) > eps)
        p.push_back(at(e, hi));
    if (reverse)
        std::reverse(p.begin(), p.end());
    return p;
}
void checkRobot(const RobotProfile &r) {
    if (!std::isfinite(r.radius) || !std::isfinite(r.safety_margin) || r.radius < 0 ||
        r.safety_margin < 0 || !std::isfinite(r.radius + r.safety_margin))
        throw std::invalid_argument(
            "robot radius and safety margin must be finite and nonnegative");
}
struct Use {
    int resource, sign;
    double start, end;
    Vec2 entry, exit;
};
struct Arc {
    int from, to, edge = -1;
    bool direct_reference = false;
    double first = 0, last = 0, length = 0, clearance = inf, narrow = 0, bottleneck = 0;
    Vec2 a{}, b{};
    std::vector<Use> uses;
};
void arcMetrics(Arc &a, const std::unordered_map<int, int> &resourceIndex, const MapData &data,
                double radius) {
    std::vector<std::pair<double, double>> ranges;
    for (const auto &u : a.uses) {
        const auto &r = data.resources[resourceIndex.at(u.resource)];
        if (!r.narrow)
            continue;
        ranges.emplace_back(u.start, u.end);
        a.bottleneck += (u.end - u.start) / std::max({r.min_width, 2 * radius, eps});
    }
    std::sort(ranges.begin(), ranges.end());
    double end = 0;
    for (const auto &r : ranges) {
        a.narrow += std::max(0.0, r.second - std::max(end, r.first));
        end = std::max(end, r.second);
    }
    // A connector can pass through a narrow gap without overlapping its ECM
    // chain. Use the already measured continuous clearance to conservatively
    // expose the complete connector; do not invent an area/resource mapping.
    if (a.edge < 0 && 2 * a.clearance < data.narrow_width) {
        a.narrow = a.length;
        a.bottleneck =
            std::max(a.bottleneck, a.length / std::max({2 * a.clearance, 2 * radius, eps}));
    }
}
// Exact polygon clipping and line intersections for off-graph connector events.
// Narrow resources have no area in schema v2: record only actual centerline
// intersections/overlaps, never manufacture a conflict polygon.
std::vector<std::pair<double, double>> intersections(const Vec2 &a, const Vec2 &b, const Vec2 &c,
                                                     const Vec2 &d) {
    double rx = b[0] - a[0], ry = b[1] - a[1], sx = d[0] - c[0], sy = d[1] - c[1],
           den = rx * sy - ry * sx;
    const double length = distance(a, b);
    if (length <= eps)
        return {};
    if (std::abs(den) > 1e-12 * length * std::max(distance(c, d), eps)) {
        double t = ((c[0] - a[0]) * sy - (c[1] - a[1]) * sx) / den;
        double u = ((c[0] - a[0]) * ry - (c[1] - a[1]) * rx) / den;
        if (t >= -eps && t <= 1 + eps && u >= -eps && u <= 1 + eps) {
            t = std::clamp(t, 0.0, 1.0);
            return {{t, t}};
        }
        return {};
    }
    if (pointSegmentDistanceSquared(c, a, b) > eps * eps &&
        pointSegmentDistanceSquared(d, a, b) > eps * eps) {
        const double cross = (c[0] - a[0]) * ry - (c[1] - a[1]) * rx;
        if (std::abs(cross) > eps * length)
            return {};
    }
    double t = ((c[0] - a[0]) * rx + (c[1] - a[1]) * ry) / (length * length),
           u = ((d[0] - a[0]) * rx + (d[1] - a[1]) * ry) / (length * length);
    double lo = std::max(0.0, std::min(t, u)), hi = std::min(1.0, std::max(t, u));
    return hi >= lo ? std::vector<std::pair<double, double>>{{lo, hi}}
                    : std::vector<std::pair<double, double>>{};
}
std::vector<Use> connectorUses(const Vec2 &a, const Vec2 &b, const MapData &data) {
    std::vector<Use> uses;
    const double length = distance(a, b);
    if (length <= eps)
        return uses;
    const auto q = box(a, b);
    for (const auto &r : data.resources) {
        std::vector<std::pair<double, double>> ranges;
        if (!r.region.empty()) {
            Box bounds;
            for (const auto &p : r.region)
                bounds.add(p);
            if (bounds.squaredDistance(q) > eps * eps)
                continue;
            std::vector<double> cuts{0, 1};
            for (size_t i = 0; i < r.region.size(); ++i)
                for (const auto &t :
                     intersections(a, b, r.region[i], r.region[(i + 1) % r.region.size()])) {
                    cuts.push_back(t.first);
                    cuts.push_back(t.second);
                }
            std::sort(cuts.begin(), cuts.end());
            for (size_t i = 1; i < cuts.size(); ++i)
                if (cuts[i] - cuts[i - 1] > eps &&
                    pointInsidePolygon(lerp(a, b, (cuts[i] + cuts[i - 1]) / 2), r.region))
                    ranges.emplace_back(cuts[i - 1], cuts[i]);
        } else if (r.narrow) {
            for (const auto &line : r.geometry)
                for (size_t i = 1; i < line.size(); ++i)
                    if (box(line[i - 1], line[i]).squaredDistance(q) <= eps * eps) {
                        auto hits = intersections(a, b, line[i - 1], line[i]);
                        ranges.insert(ranges.end(), hits.begin(), hits.end());
                    }
        }
        std::sort(ranges.begin(), ranges.end());
        std::vector<std::pair<double, double>> merged;
        for (auto range : ranges)
            if (!merged.empty() && range.first <= merged.back().second + eps)
                merged.back().second = std::max(merged.back().second, range.second);
            else
                merged.push_back(range);
        for (const auto &t : merged) {
            if (t.second - t.first <= eps && (t.first <= eps || t.first >= 1 - eps))
                continue;
            uses.push_back({r.id, 0, t.first * length, t.second * length, lerp(a, b, t.first),
                            lerp(a, b, t.second)});
        }
    }
    return uses;
}
std::map<int, double> resourceWeights(const CompleteRoute &r) {
    std::map<int, double> weights;
    for (const auto &u :
         (r.physical_path.empty() ? r.resource_traversals : r.physical_resource_traversals))
        weights[u.resource_id] += u.traversal_length;
    for (auto &w : weights)
        w.second += 1;
    return weights;
}
std::map<int, double> edgeWeights(const CompleteRoute &r) {
    std::map<int, double> weights;
    for (const auto &e : r.edge_traversals)
        weights[e.edge_id] += std::abs(e.last_distance - e.first_distance);
    return weights;
}
double overlap(const std::map<int, double> &a, const std::map<int, double> &b) {
    double shared = 0, la = 0, lb = 0;
    for (auto v : a) {
        la += v.second;
        auto it = b.find(v.first);
        if (it != b.end())
            shared += std::min(v.second, it->second);
    }
    for (auto v : b)
        lb += v.second;
    if (std::min(la, lb) <= eps)
        return a.empty() && b.empty() ? 1 : 0;
    return std::clamp(shared / std::min(la, lb), 0.0, 1.0);
}
bool geometricallyNear(const std::vector<Vec2> &a, const std::vector<Vec2> &b, double separation) {
    auto contained = [&](const std::vector<Vec2> &source, const std::vector<Vec2> &target) {
        std::vector<Box> boxes;
        for (size_t i = 1; i < target.size(); ++i)
            boxes.push_back(box(target[i - 1], target[i]));
        if (boxes.empty())
            return false;
        Tree tree(std::move(boxes));
        for (const auto &p : source)
            if (tree.nearest(box(p, p), [&](int i) {
                    return pointSegmentDistanceSquared(p, target[i], target[i + 1]);
                }) > separation * separation)
                return false;
        return true;
    };
    return contained(a, b) && contained(b, a);
}
std::vector<int> signature(const CompleteRoute &r) {
    std::vector<int> ids = r.resource_ids.empty() ? r.edge_ids : r.resource_ids;
    std::sort(ids.begin(), ids.end());
    ids.erase(std::unique(ids.begin(), ids.end()), ids.end());
    return ids;
}
} // namespace

struct CorridorMap::Impl {
    MapData data;
    std::unordered_map<int, int> nodeIndex, resourceIndex, edgeIndex;
    mutable std::mutex cacheMutex;
    mutable std::map<std::string, std::shared_ptr<const LaneDecomposition>> laneCache;
    mutable std::map<std::string, std::vector<Vec2>> transitionCache;
    mutable std::map<std::string, bool> shortcutCache;
    std::vector<Segment> boundary, reference;
    std::vector<Box> polygonBoxes;
    Tree boundaryTree, referenceTree, polygonTree;
    std::vector<std::vector<double>> segmentClearances;
    double clearance(const Vec2 &a, const Vec2 &b) const {
        detail::StageScope scope(PlanningStage::CollisionValidation);
        if (auto *p = detail::profile())
            ++p->clearance_queries;
        if (!finite(a) || !finite(b))
            return 0;
        const auto &w = data.bounds;
        double outer = std::min({a[0] - w[0], a[1] - w[1], w[2] - a[0], w[3] - a[1], b[0] - w[0],
                                 b[1] - w[1], w[2] - b[0], w[3] - b[1]});
        if (outer <= 0)
            return 0;
        if (polygonTree.anyContainingEndpoint(a, b, [&](int i) {
                return pointInsidePolygon(a, data.obstacles[i]) ||
                       pointInsidePolygon(b, data.obstacles[i]);
            }))
            return 0;
        return std::min(outer, std::sqrt(boundaryTree.nearest(box(a, b), [&](int i) {
                            const auto &s = boundary[i];
                            return segmentDistanceSquared(a, b, s.a, s.b);
                        })));
    }
    double edgeClearance(int index, double lo, double hi) const {
        const auto &e = data.edges[index];
        double result = inf;
        auto it = std::upper_bound(e.reference_distances.begin(), e.reference_distances.end(), lo);
        size_t i = it == e.reference_distances.end() ? e.reference_distances.size() - 1
                                                     : it - e.reference_distances.begin();
        for (; i < e.reference_distances.size() && e.reference_distances[i - 1] < hi - eps; ++i) {
            if (lo <= e.reference_distances[i - 1] + eps && hi >= e.reference_distances[i] - eps)
                result = std::min(result, segmentClearances[index][i - 1]);
            else
                result =
                    std::min(result, clearance(at(e, std::max(lo, e.reference_distances[i - 1])),
                                               at(e, std::min(hi, e.reference_distances[i]))));
        }
        if (lo <= eps && hi >= e.reference_distances.back() - eps)
            result = std::min({result, e.stored_min_clearance, e.min_width / 2});
        return result;
    }
};

CorridorMap::CorridorMap(MapData data) {
    auto m = std::make_shared<Impl>();
    m->data = std::move(data);
    auto &d = m->data;
    if (!std::isfinite(d.narrow_width) || d.narrow_width <= 0 ||
        !std::all_of(d.bounds.begin(), d.bounds.end(), [](double v) { return std::isfinite(v); }) ||
        d.bounds[0] >= d.bounds[2] || d.bounds[1] >= d.bounds[3])
        throw std::invalid_argument("invalid map bounds");
    for (const auto &polygon : d.obstacles) {
        if (polygon.size() < 3)
            throw std::invalid_argument("invalid obstacle polygon");
        Box b;
        for (size_t i = 0; i < polygon.size(); ++i) {
            if (!finite(polygon[i]))
                throw std::invalid_argument("nonfinite polygon");
            b.add(polygon[i]);
            m->boundary.push_back({polygon[i], polygon[(i + 1) % polygon.size()]});
        }
        m->polygonBoxes.push_back(b);
    }
    m->polygonTree = Tree(m->polygonBoxes);
    std::vector<Box> boxes;
    for (const auto &s : m->boundary)
        boxes.push_back(box(s.a, s.b));
    m->boundaryTree = Tree(std::move(boxes));
    for (size_t i = 0; i < d.nodes.size(); ++i)
        if (!finite(d.nodes[i].position) || !m->nodeIndex.emplace(d.nodes[i].id, i).second)
            throw std::invalid_argument("invalid/duplicate node");
    for (size_t i = 0; i < d.resources.size(); ++i) {
        const auto &r = d.resources[i];
        if (!m->resourceIndex.emplace(r.id, i).second || !std::isfinite(r.min_width) ||
            r.min_width < 0)
            throw std::invalid_argument("invalid/duplicate resource");
        for (const auto &line : r.geometry)
            for (const auto &p : line)
                if (!finite(p))
                    throw std::invalid_argument("invalid resource geometry");
        for (const auto &p : r.region)
            if (!finite(p))
                throw std::invalid_argument("invalid resource region");
    }
    std::set<int> ids;
    for (size_t i = 0; i < d.edges.size(); ++i) {
        auto &e = d.edges[i];
        m->edgeIndex.emplace(e.id, static_cast<int>(i));
        const auto n = e.reference_geometry.size();
        if (!ids.insert(e.id).second || !m->nodeIndex.count(e.u) || !m->nodeIndex.count(e.v) ||
            n < 2 || e.reference_distances.size() != n || !std::isfinite(e.stored_min_clearance) ||
            e.stored_min_clearance < 0 || !std::isfinite(e.min_width) || e.min_width < 0)
            throw std::invalid_argument("invalid edge/reference metadata");
        if (distance(e.reference_geometry.front(), d.nodes[m->nodeIndex.at(e.u)].position) > eps ||
            distance(e.reference_geometry.back(), d.nodes[m->nodeIndex.at(e.v)].position) > eps ||
            e.reference_distances.front() != 0)
            throw std::invalid_argument("disconnected reference geometry");
        std::vector<double> clearances;
        for (size_t j = 0; j < n; ++j) {
            if (!finite(e.reference_geometry[j]) || !std::isfinite(e.reference_distances[j]))
                throw std::invalid_argument("invalid reference point");
            if (j == 0)
                continue;
            const double length = distance(e.reference_geometry[j - 1], e.reference_geometry[j]);
            if (e.reference_distances[j] <= e.reference_distances[j - 1] ||
                !std::isfinite(length) || length <= 0 ||
                std::abs(length - (e.reference_distances[j] - e.reference_distances[j - 1])) >
                    1e-6 * std::max(1.0, length))
                throw std::invalid_argument("invalid reference arclength");
            m->reference.push_back({e.reference_geometry[j - 1], e.reference_geometry[j],
                                    static_cast<int>(i), static_cast<int>(j - 1)});
            clearances.push_back(
                m->clearance(e.reference_geometry[j - 1], e.reference_geometry[j]));
        }
        m->segmentClearances.push_back(std::move(clearances));
        for (const auto &r : e.resources)
            if (!m->resourceIndex.count(r.resource_id) || !std::isfinite(r.start) ||
                !std::isfinite(r.end) || r.start < -eps || r.end < r.start ||
                r.end > e.reference_distances.back() + eps || r.direction < -1 || r.direction > 1)
                throw std::invalid_argument("invalid resource interval");
    }
    boxes.clear();
    for (const auto &s : m->reference)
        boxes.push_back(box(s.a, s.b));
    m->referenceTree = Tree(std::move(boxes));
    impl_ = std::move(m);
}
const MapData &CorridorMap::data() const {
    return impl_->data;
}
double CorridorMap::segmentClearance(const Vec2 &a, const Vec2 &b) const {
    return impl_->clearance(a, b);
}
namespace {
std::string laneKey(double radius, const LaneConfig &c) {
    std::ostringstream out;
    out << std::setprecision(17) << radius << ':' << c.sample_spacing << ':' << c.max_offset << ':'
        << c.clearance_guard << ':' << c.separation_margin << ':' << c.hysteresis_length << ':'
        << c.min_split_length << ':' << c.transition_length << ':' << c.max_offset_slope << ':'
        << c.miter_limit << ':' << c.offset_iterations << ':' << c.refinement_iterations << ':'
        << c.max_samples_per_edge << ':' << c.taper_endpoints;
    return out.str();
}
} // namespace
std::shared_ptr<const LaneDecomposition>
CorridorMap::laneDecomposition(int edge_id, const RobotProfile &robot,
                               const LaneConfig &config) const {
    detail::StageScope scope(PlanningStage::LaneDecomposition);
    checkRobot(robot);
    if (!validateLaneConfig(config))
        throw std::invalid_argument("invalid lane configuration");
    const std::string key =
        std::to_string(edge_id) + ":" + laneKey(robot.radius + robot.safety_margin, config);
    {
        std::lock_guard<std::mutex> lock(impl_->cacheMutex);
        auto it = impl_->laneCache.find(key);
        if (it != impl_->laneCache.end()) {
            if (auto *p = detail::profile())
                ++p->lane_cache_hits;
            return it->second;
        }
    }
    if (auto *p = detail::profile())
        ++p->lane_builds;
    const auto &edge = impl_->data.edges.at(impl_->edgeIndex.at(edge_id));
    auto built = std::make_shared<const LaneDecomposition>(
        buildLaneDecomposition(*this, edge, robot, config));
    std::lock_guard<std::mutex> lock(impl_->cacheMutex);
    if (impl_->laneCache.size() >= 1024)
        impl_->laneCache.clear();
    return impl_->laneCache.emplace(key, built).first->second;
}
std::shared_ptr<const LaneDecomposition>
CorridorMap::laneDecomposition(const std::vector<Vec2> &reference, const RobotProfile &robot,
                               const LaneConfig &config) const {
    detail::StageScope scope(PlanningStage::LaneDecomposition);
    checkRobot(robot);
    if (!validateLaneConfig(config) || reference.size() < 2)
        throw std::invalid_argument("invalid virtual corridor");
    std::ostringstream out;
    out << std::setprecision(17)
        << "virtual:" << laneKey(robot.radius + robot.safety_margin, config);
    for (const auto &p : reference)
        out << ':' << p[0] << ':' << p[1];
    const auto key = out.str();
    {
        std::lock_guard<std::mutex> lock(impl_->cacheMutex);
        auto it = impl_->laneCache.find(key);
        if (it != impl_->laneCache.end()) {
            if (auto *p = detail::profile())
                ++p->lane_cache_hits;
            return it->second;
        }
    }
    if (auto *p = detail::profile())
        ++p->lane_builds;
    RoutingEdge edge;
    edge.id = -1;
    edge.reference_geometry = reference;
    edge.reference_distances = {0};
    for (size_t i = 1; i < reference.size(); ++i) {
        const double ds = distance(reference[i - 1], reference[i]);
        if (ds <= 0 || !finite(reference[i]))
            throw std::invalid_argument("invalid connector reference");
        edge.reference_distances.push_back(edge.reference_distances.back() + ds);
    }
    auto built = std::make_shared<const LaneDecomposition>(
        buildLaneDecomposition(*this, edge, robot, config));
    std::lock_guard<std::mutex> lock(impl_->cacheMutex);
    if (impl_->laneCache.size() >= 1024)
        impl_->laneCache.clear();
    return impl_->laneCache.emplace(key, built).first->second;
}
std::vector<Vec2> CorridorMap::laneConnection(const Vec2 &from, const Vec2 &to, const Vec2 &via,
                                              const RobotProfile &robot,
                                              const LaneConfig &config) const {
    detail::StageScope scope(PlanningStage::Transitions);
    checkRobot(robot);
    if (!validateLaneConfig(config))
        throw std::invalid_argument("invalid lane configuration");
    std::ostringstream out;
    out << std::setprecision(17) << laneKey(robot.radius + robot.safety_margin, config);
    for (const auto &p : {from, to, via})
        out << ':' << p[0] << ':' << p[1];
    const auto key = out.str();
    {
        std::lock_guard<std::mutex> lock(impl_->cacheMutex);
        auto it = impl_->transitionCache.find(key);
        if (it != impl_->transitionCache.end()) {
            if (auto *p = detail::profile())
                ++p->transition_cache_hits;
            return it->second;
        }
    }
    if (auto *p = detail::profile())
        ++p->transition_builds;
    auto result = buildLaneConnection(*this, from, to, via, robot, config);
    std::lock_guard<std::mutex> lock(impl_->cacheMutex);
    if (impl_->transitionCache.size() >= 512)
        impl_->transitionCache.clear();
    return impl_->transitionCache.emplace(key, result).first->second;
}
std::shared_ptr<const LaneDecomposition>
CorridorMap::physicalSectionDecomposition(const std::vector<EdgeTraversal> &traversals,
                                          const RobotProfile &robot,
                                          const LaneConfig &config) const {
    std::vector<Vec2> reference;
    for (const auto &t : traversals) {
        const auto &e = impl_->data.edges.at(impl_->edgeIndex.at(t.edge_id));
        if (!std::isfinite(t.first_distance) || !std::isfinite(t.last_distance) ||
            std::min(t.first_distance, t.last_distance) < 0 ||
            std::max(t.first_distance, t.last_distance) > e.reference_distances.back() + eps ||
            (t.last_distance - t.first_distance) * t.direction <= eps)
            throw std::invalid_argument("invalid diagnostic physical section");
        const auto part = slice(e, t.first_distance, t.last_distance);
        if (!reference.empty() && distance(reference.back(), part.front()) > eps)
            throw std::invalid_argument("discontinuous diagnostic physical section");
        for (const auto &p : part)
            if (reference.empty() || distance(reference.back(), p) > eps)
                reference.push_back(p);
    }
    return laneDecomposition(reference, robot, config);
}
size_t CorridorMap::laneCacheEntries() const {
    std::lock_guard<std::mutex> lock(impl_->cacheMutex);
    return impl_->laneCache.size();
}
size_t CorridorMap::transitionCacheEntries() const {
    std::lock_guard<std::mutex> lock(impl_->cacheMutex);
    return impl_->transitionCache.size();
}
bool CorridorMap::validatePath(const std::vector<Vec2> &path, const RobotProfile &robot) const {
    detail::StageScope scope(PlanningStage::CollisionValidation);
    auto *p = detail::profile();
    if (p)
        ++p->path_validation_calls;
    auto failed = [&]() {
        if (p)
            ++p->path_validation_failures;
        return false;
    };
    checkRobot(robot);
    if (path.empty())
        return failed();
    const double radius = robot.radius + robot.safety_margin;
    if (impl_->clearance(path.front(), path.front()) <= radius + eps)
        return failed();
    for (size_t i = 1; i < path.size(); ++i)
        if (impl_->clearance(path[i - 1], path[i]) <= radius + eps)
            return failed();
    return true;
}
std::vector<int> CorridorMap::feasiblePhysicalEdges(const RobotProfile &robot) const {
    checkRobot(robot);
    const double R = robot.radius + robot.safety_margin;
    std::vector<int> result;
    for (size_t i = 0; i < impl_->data.edges.size(); ++i) {
        const auto &e = impl_->data.edges[i];
        if (impl_->edgeClearance(static_cast<int>(i), 0, e.reference_distances.back()) > R + eps)
            result.push_back(e.id);
    }
    return result;
}
double routeOverlap(const CompleteRoute &a, const CompleteRoute &b) {
    auto wa = resourceWeights(a), wb = resourceWeights(b);
    return wa.empty() && wb.empty() ? overlap(edgeWeights(a), edgeWeights(b)) : overlap(wa, wb);
}

namespace {
void finalizeLaneResources(CompleteRoute &route, const CorridorMap &map, const PlannerConfig &cfg,
                           const RobotProfile &robot) {
    const auto &data = map.data();
    route.resource_traversals.clear();
    route.resource_ids.clear();
    double offset = 0;
    std::vector<std::pair<double, double>> narrow;
    for (auto &s : route.segments) {
        s.traffic_resource_ids.clear();
        s.min_clearance = map.segmentClearance(s.geometry.front(), s.geometry.front());
        const RoutingEdge *edge = nullptr;
        for (const auto &e : data.edges)
            if (e.id == s.physical_edge_id) {
                edge = &e;
                break;
            }
        double local = 0;
        for (size_t i = 1; i < s.geometry.size(); ++i) {
            const auto &a = s.geometry[i - 1];
            const auto &b = s.geometry[i];
            double ds = distance(a, b);
            s.min_clearance = std::min(s.min_clearance, map.segmentClearance(a, b));
            std::vector<Use> uses;
            if (edge) {
                double d0 = s.reference_distances[i - 1], d1 = s.reference_distances[i];
                for (const auto &r : edge->resources) {
                    double lo = std::max(std::min(d0, d1), r.start),
                           hi = std::min(std::max(d0, d1), r.end);
                    if (hi - lo <= eps)
                        continue;
                    double t0 = (lo - d0) / (d1 - d0), t1 = (hi - d0) / (d1 - d0);
                    if (t1 < t0)
                        std::swap(t0, t1);
                    uses.push_back({r.resource_id, r.direction * (s.forward ? 1 : -1), t0 * ds,
                                    t1 * ds, lerp(a, b, t0), lerp(a, b, t1)});
                }
            } else if ((s.kind == SegmentKind::Corridor || s.kind == SegmentKind::ModeTransition) &&
                       s.reference_distances.size() == s.geometry.size()) {
                double d0 = s.reference_distances[i - 1], d1 = s.reference_distances[i];
                for (const auto &r : route.physical_resource_traversals) {
                    double lo = std::max(std::min(d0, d1), r.route_start),
                           hi = std::min(std::max(d0, d1), r.route_end);
                    if (hi < lo - eps)
                        continue;
                    double t0 = (lo - d0) / (d1 - d0), t1 = (hi - d0) / (d1 - d0);
                    if (t1 < t0)
                        std::swap(t0, t1);
                    uses.push_back({r.resource_id, r.direction, t0 * ds, t1 * ds, lerp(a, b, t0),
                                    lerp(a, b, t1)});
                }
            } else
                uses = connectorUses(a, b, data);
            for (const auto &u : uses) {
                route.resource_traversals.push_back({u.resource, u.sign, u.entry, u.exit,
                                                     offset + local + u.start,
                                                     offset + local + u.end, u.end - u.start,
                                                     s.physical_edge_id, s.physical_edge_id});
                s.traffic_resource_ids.push_back(u.resource);
                for (const auto &r : data.resources)
                    if (r.id == u.resource && r.narrow)
                        narrow.emplace_back(offset + local + u.start, offset + local + u.end);
            }
            if (!edge && s.narrow_reference)
                narrow.emplace_back(offset + local, offset + local + ds);
            local += ds;
        }
        std::sort(s.traffic_resource_ids.begin(), s.traffic_resource_ids.end());
        s.traffic_resource_ids.erase(
            std::unique(s.traffic_resource_ids.begin(), s.traffic_resource_ids.end()),
            s.traffic_resource_ids.end());
        s.traffic_resource_id =
            s.traffic_resource_ids.empty() ? -1 : s.traffic_resource_ids.front();
        offset += local;
    }
    auto &events = route.resource_traversals;
    std::sort(events.begin(), events.end(), [](const auto &a, const auto &b) {
        return std::tie(a.route_start, a.route_end, a.resource_id) <
               std::tie(b.route_start, b.route_end, b.resource_id);
    });
    std::vector<ResourceTraversal> merged;
    std::map<std::pair<int, int>, size_t> last;
    for (const auto &event : events) {
        auto key = std::make_pair(event.resource_id, event.direction);
        auto it = last.find(key);
        if (it != last.end() && std::abs(merged[it->second].route_end - event.route_start) <= eps &&
            distance(merged[it->second].exit, event.entry) <= eps) {
            auto &old = merged[it->second];
            old.exit = event.exit;
            old.route_end = event.route_end;
            old.traversal_length += event.traversal_length;
            old.exit_edge_id = event.exit_edge_id;
        } else {
            last[key] = merged.size();
            merged.push_back(event);
        }
    }
    events = std::move(merged);
    for (const auto &u : events)
        route.resource_ids.push_back(u.resource_id);
    std::sort(narrow.begin(), narrow.end());
    double end = 0;
    for (const auto &range : narrow) {
        route.metrics.narrow_exposure += std::max(0.0, range.second - std::max(end, range.first));
        end = std::max(end, range.second);
    }
    for (const auto &u : events)
        for (const auto &r : data.resources)
            if (r.id == u.resource_id && r.narrow)
                route.metrics.bottleneck_penalty +=
                    u.traversal_length /
                    std::max({r.min_width, 2 * (robot.radius + robot.safety_margin), eps});
    route.metrics.score = route.metrics.length + cfg.lambda_narrow * route.metrics.narrow_exposure +
                          cfg.lambda_bottleneck * route.metrics.bottleneck_penalty;
}
} // namespace
std::vector<CompleteRoute> planKRoutes(const CorridorMap &map, const Vec2 &start, const Vec2 &goal,
                                       const RobotProfile &robot, int k,
                                       const PlannerConfig &config, PlanningTimings *timings) {
    struct Timer {
        PlanningTimings *value;
        std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
        ~Timer() {
            if (value) {
                value->total_seconds =
                    std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
                value->route_search_seconds =
                    std::max(0.0, value->total_seconds - value->lane_placement_seconds);
            }
        }
    } timer{timings};
    if (timings)
        *timings = {};
    detail::ProfileContext profile(timings);
    checkRobot(robot);
    if (!validateLaneConfig(config.lanes) || config.max_candidates < 1 ||
        config.max_attachments < 1 || config.max_attachment_segments < 1 ||
        !std::isfinite(config.max_detour_ratio) || config.max_detour_ratio < 1 ||
        !std::isfinite(config.diversity_threshold) || config.diversity_threshold < 0 ||
        config.diversity_threshold > 1 || !std::isfinite(config.lambda_narrow) ||
        config.lambda_narrow < 0 || !std::isfinite(config.lambda_bottleneck) ||
        config.lambda_bottleneck < 0 || !std::isfinite(config.min_route_separation) ||
        config.min_route_separation < 0 || !std::isfinite(config.penalty_factor) ||
        config.penalty_factor <= 0 || !std::isfinite(config.max_attachment_distance) ||
        config.max_attachment_distance <= 0)
        throw std::invalid_argument("invalid planner configuration");
    if (k <= 0)
        return {};
    const auto &m = *map.impl_;
    const auto &data = m.data;
    const double radius = robot.radius + robot.safety_margin;
    detail::StageScope attachmentScope(PlanningStage::Attachment);
    if (m.clearance(start, start) <= radius + eps || m.clearance(goal, goal) <= radius + eps) {
        if (timings)
            timings->lane_failures.push_back("query_endpoint_footprint_blocked");
        return {};
    }
    if (distance(start, goal) <= eps) {
        CompleteRoute r;
        r.path = {start, goal};
        r.metrics.min_clearance = m.clearance(start, goal);
        if (timings)
            timings->physical_route_reachable = true;
        return {r};
    }
    struct Attachment {
        int edge, node;
        double d;
        Vec2 p;
        double clearance;
    };
    std::vector<Attachment> starts, goals;
    std::vector<std::vector<std::pair<double, int>>> cuts(data.edges.size());
    std::vector<Vec2> nodes;
    for (const auto &n : data.nodes)
        nodes.push_back(n.position);
    for (size_t i = 0; i < data.edges.size(); ++i) {
        const auto &e = data.edges[i];
        cuts[i] = {{0, m.nodeIndex.at(e.u)}, {e.reference_distances.back(), m.nodeIndex.at(e.v)}};
    }
    auto attach = [&](const Vec2 &p, std::vector<Attachment> &out) {
        auto nearby = m.referenceTree.nearby(p, config.max_attachment_segments,
                                             config.max_attachment_distance, [&](int i) {
                                                 const auto &s = m.reference[i];
                                                 return pointSegmentDistanceSquared(p, s.a, s.b);
                                             });
        std::set<int> selected;
        for (int id : nearby) {
            const auto &s = m.reference[id];
            if (selected.count(s.edge))
                continue;
            const auto &e = data.edges[s.edge];
            const double t = projection(p, s.a, s.b);
            Vec2 q = lerp(s.a, s.b, t);
            double d = e.reference_distances[s.index] +
                       t * (e.reference_distances[s.index + 1] - e.reference_distances[s.index]);
            if (m.clearance(q, q) <= radius + eps)
                continue;
            double clearance = m.clearance(p, q);
            if (clearance <= radius + eps)
                continue;
            // Do not spend a candidate on a feasible point isolated by unsafe
            // geometry in both directions of its host edge.
            const double delta = std::min(1e-4, e.reference_distances.back() / 1000);
            if ((d <= eps ||
                 m.edgeClearance(s.edge, std::max(0.0, d - delta), d) <= radius + eps) &&
                (d >= e.reference_distances.back() - eps ||
                 m.edgeClearance(s.edge, d, std::min(e.reference_distances.back(), d + delta)) <=
                     radius + eps))
                continue;
            int node = -1;
            for (const auto &cut : cuts[s.edge])
                if (std::abs(cut.first - d) <= eps) {
                    node = cut.second;
                    d = cut.first;
                    q = nodes[node];
                    break;
                }
            if (node < 0) {
                node = static_cast<int>(nodes.size());
                nodes.push_back(q);
                cuts[s.edge].emplace_back(d, node);
            }
            out.push_back({s.edge, node, d, q, clearance});
            selected.insert(s.edge);
            if (static_cast<int>(out.size()) >= config.max_attachments)
                break;
        }
    };
    attach(start, starts);
    attach(goal, goals);
    const int source = static_cast<int>(nodes.size());
    nodes.push_back(start);
    const int target = static_cast<int>(nodes.size());
    nodes.push_back(goal);
    std::vector<Arc> arcs;
    std::vector<std::vector<int>> adjacency(nodes.size());
    auto add = [&](Arc arc) {
        arcMetrics(arc, m.resourceIndex, data, radius);
        adjacency[arc.from].push_back(static_cast<int>(arcs.size()));
        arcs.push_back(std::move(arc));
    };
    {
        detail::StageScope filteringScope(PlanningStage::GraphFiltering);
        for (size_t i = 0; i < data.edges.size(); ++i) {
            auto &c = cuts[i];
            std::sort(c.begin(), c.end());
            const auto &e = data.edges[i];
            for (size_t j = 1; j < c.size(); ++j) {
                double lo = c[j - 1].first, hi = c[j].first;
                if (hi - lo <= eps)
                    continue;
                const double clearance = m.edgeClearance(i, lo, hi);
                if (clearance <= radius + eps)
                    continue;
                for (int direction : {1, -1}) {
                    Arc a;
                    a.from = direction == 1 ? c[j - 1].second : c[j].second;
                    a.to = direction == 1 ? c[j].second : c[j - 1].second;
                    a.edge = static_cast<int>(i);
                    a.first = direction == 1 ? lo : hi;
                    a.last = direction == 1 ? hi : lo;
                    a.length = hi - lo;
                    a.clearance = clearance;
                    for (const auto &interval : e.resources) {
                        double first = std::max(lo, interval.start),
                               last = std::min(hi, interval.end);
                        if (last - first <= eps)
                            continue;
                        if (direction == 1)
                            a.uses.push_back({interval.resource_id, interval.direction, first - lo,
                                              last - lo, at(e, first), at(e, last)});
                        else
                            a.uses.push_back({interval.resource_id, -interval.direction, hi - last,
                                              hi - first, at(e, last), at(e, first)});
                    }
                    add(std::move(a));
                }
            }
        }
    }
    auto connector = [&](int from, int to, double clearance) {
        Arc a;
        a.from = from;
        a.to = to;
        a.a = nodes[from];
        a.b = nodes[to];
        a.length = distance(a.a, a.b);
        a.clearance = clearance;
        a.uses = connectorUses(a.a, a.b, data);
        add(std::move(a));
    };
    for (const auto &a : starts)
        connector(source, a.node, a.clearance);
    for (const auto &a : goals)
        connector(a.node, target, a.clearance);
    const double direct = m.clearance(start, goal);
    if (direct > radius + eps) {
        bool onReference = false;
        auto equivalentCorridor = [&](const Attachment &a, const Attachment &b) {
            const double span = std::abs(a.d - b.d), chord = distance(start, goal);
            if (span <= eps || span > chord * config.max_detour_ratio + eps)
                return false;
            std::ostringstream shortcutKey;
            shortcutKey << std::setprecision(17) << a.edge << ':' << a.d << ':' << b.d << ':'
                        << start[0] << ':' << start[1] << ':' << goal[0] << ':' << goal[1] << ':'
                        << radius;
            const auto key = shortcutKey.str();
            {
                std::lock_guard<std::mutex> lock(m.cacheMutex);
                auto found = m.shortcutCache.find(key);
                if (found != m.shortcutCache.end())
                    return found->second;
            }
            auto remember = [&](bool valid) {
                std::lock_guard<std::mutex> lock(m.cacheMutex);
                if (m.shortcutCache.size() >= 512)
                    m.shortcutCache.clear();
                m.shortcutCache.emplace(key, valid);
                return valid;
            };
            const auto reference = slice(data.edges[a.edge], a.d, b.d);
            auto freeTriangle = [&](const Vec2 &p, const Vec2 &q, const Vec2 &r) {
                if (m.clearance(p, q) <= radius + eps || m.clearance(q, r) <= radius + eps ||
                    m.clearance(r, p) <= radius + eps)
                    return false;
                const Polygon triangle{p, q, r};
                for (const auto &polygon : data.obstacles)
                    for (const auto &point : polygon)
                        if (pointInsidePolygon(point, triangle))
                            return false;
                return true;
            };
            // Certify the whole ruled ribbon, not just sampled connector points.
            // A direct shortcut in this ribbon is the same physical corridor,
            // and must not become a second K-route merely by cutting a bend.
            double d = 0;
            for (size_t i = 1; i < reference.size(); ++i) {
                const double next = d + distance(reference[i - 1], reference[i]);
                const auto p = lerp(start, goal, d / span), q = lerp(start, goal, next / span);
                if (!freeTriangle(reference[i - 1], reference[i], q) ||
                    !freeTriangle(reference[i - 1], q, p))
                    return remember(false);
                d = next;
            }
            return remember(true);
        };
        for (const auto &a : starts)
            for (const auto &b : goals)
                if (!onReference && a.edge == b.edge && equivalentCorridor(a, b)) {
                    Arc arc;
                    arc.from = source;
                    arc.to = target;
                    arc.edge = a.edge;
                    arc.direct_reference = true;
                    arc.first = a.d;
                    arc.last = b.d;
                    arc.length = distance(start, goal);
                    arc.clearance = direct;
                    const auto &e = data.edges[a.edge];
                    double lo = std::min(a.d, b.d), hi = std::max(a.d, b.d);
                    int sign = b.d > a.d ? 1 : -1;
                    for (const auto &interval : e.resources) {
                        double first = std::max(lo, interval.start),
                               last = std::min(hi, interval.end);
                        if (last - first <= eps)
                            continue;
                        const double t0 =
                            sign == 1 ? (first - lo) / (hi - lo) : (hi - last) / (hi - lo);
                        const double t1 =
                            sign == 1 ? (last - lo) / (hi - lo) : (hi - first) / (hi - lo);
                        arc.uses.push_back({interval.resource_id, sign * interval.direction,
                                            t0 * arc.length, t1 * arc.length, lerp(start, goal, t0),
                                            lerp(start, goal, t1)});
                    }
                    add(std::move(arc));
                    onReference = true;
                }
        if (!onReference)
            connector(source, target, direct);
    }
    std::vector<double> penalties(data.resources.size()), edgePenalties(data.edges.size()),
        arcPenalties(arcs.size()), cost(nodes.size());
    std::vector<int> previous(nodes.size());
    auto search = [&](bool baseline) {
        detail::StageScope searchScope(PlanningStage::TopologySearch);
        std::fill(cost.begin(), cost.end(), inf);
        std::fill(previous.begin(), previous.end(), -1);
        cost[source] = 0;
        using Item = std::pair<double, int>;
        std::priority_queue<Item, std::vector<Item>, std::greater<Item>> queue;
        queue.emplace(0, source);
        while (!queue.empty()) {
            auto [value, node] = queue.top();
            queue.pop();
            if (value != cost[node])
                continue;
            if (node == target)
                break;
            for (int id : adjacency[node]) {
                const auto &a = arcs[id];
                double weight = a.length;
                if (!baseline) {
                    weight += config.lambda_narrow * a.narrow +
                              config.lambda_bottleneck * a.bottleneck + arcPenalties[id] * a.length;
                    if (a.edge >= 0)
                        weight += edgePenalties[a.edge] * a.length;
                    for (const auto &u : a.uses)
                        weight += penalties[m.resourceIndex.at(u.resource)] *
                                  (u.end - u.start + std::min(1.0, a.length));
                }
                if (value + weight < cost[a.to]) {
                    cost[a.to] = value + weight;
                    previous[a.to] = id;
                    queue.emplace(cost[a.to], a.to);
                }
            }
        }
        std::vector<int> walk;
        if (!std::isfinite(cost[target]))
            return walk;
        for (int n = target; n != source;) {
            int id = previous[n];
            if (id < 0)
                throw std::logic_error("disconnected search result");
            walk.push_back(id);
            n = arcs[id].from;
        }
        std::reverse(walk.begin(), walk.end());
        return walk;
    };
    auto reconstruct = [&](const std::vector<int> &walk, bool geometry) {
        detail::StageScope reconstructionScope(PlanningStage::Reconstruction);
        CompleteRoute r;
        r.metrics.min_clearance = inf;
        double offset = 0;
        for (int id : walk) {
            const auto &a = arcs[id];
            r.metrics.min_clearance = std::min(r.metrics.min_clearance, a.clearance);
            r.metrics.narrow_exposure += a.narrow;
            r.metrics.bottleneck_penalty += a.bottleneck;
            if (a.edge >= 0) {
                const auto &e = data.edges[a.edge];
                int sign = a.last > a.first ? 1 : -1;
                if (!r.edge_traversals.empty() && r.edge_traversals.back().edge_id == e.id &&
                    r.edge_traversals.back().direction == sign &&
                    std::abs(r.edge_traversals.back().last_distance - a.first) <= eps)
                    r.edge_traversals.back().last_distance = a.last;
                else {
                    r.edge_traversals.push_back(
                        {e.id, sign, CorridorMode::CenterBidirectional, a.first, a.last});
                    r.edge_ids.push_back(e.id);
                    r.directions.push_back(sign);
                    r.modes.push_back(CorridorMode::CenterBidirectional);
                }
            }
            for (const auto &u : a.uses)
                r.resource_traversals.push_back({u.resource, u.sign, u.entry, u.exit,
                                                 offset + u.start, offset + u.end, u.end - u.start,
                                                 a.edge < 0 ? -1 : data.edges[a.edge].id,
                                                 a.edge < 0 ? -1 : data.edges[a.edge].id});
            if (geometry) {
                auto p = a.direct_reference ? std::vector<Vec2>{start, goal}
                         : a.edge >= 0      ? slice(data.edges[a.edge], a.first, a.last)
                                            : std::vector<Vec2>{a.a, a.b};
                if (!r.path.empty() && distance(r.path.back(), p.front()) > eps)
                    throw std::logic_error("discontinuous route");
                RouteSegment segment;
                segment.geometry = p;
                segment.reference_geometry = p;
                segment.physical_edge_id = a.edge < 0 ? -1 : data.edges[a.edge].id;
                segment.forward = a.edge < 0 || a.last > a.first;
                segment.kind =
                    a.edge >= 0 ? SegmentKind::Corridor
                                : (a.from == source && a.to == target ? SegmentKind::DirectConnector
                                   : a.from == source                 ? SegmentKind::StartConnector
                                                                      : SegmentKind::GoalConnector);
                segment.narrow_reference = a.narrow > eps;
                if (a.direct_reference)
                    segment.reference_distances = {a.first, a.last};
                else if (a.edge >= 0) {
                    double d = a.first;
                    segment.reference_distances.push_back(d);
                    for (size_t j = 1; j < p.size(); ++j) {
                        d += (segment.forward ? 1 : -1) * distance(p[j - 1], p[j]);
                        segment.reference_distances.push_back(d);
                    }
                }
                r.segments.push_back(std::move(segment));
                for (const auto &point : p)
                    if (r.path.empty() || distance(r.path.back(), point) > eps)
                        r.path.push_back(point);
            }
            offset += a.length;
        }
        auto &events = r.resource_traversals;
        std::sort(events.begin(), events.end(), [](const auto &a, const auto &b) {
            return std::tie(a.route_start, a.route_end, a.resource_id) <
                   std::tie(b.route_start, b.route_end, b.resource_id);
        });
        std::vector<ResourceTraversal> merged;
        std::map<std::pair<int, int>, size_t> last;
        for (const auto &event : events) {
            auto key = std::make_pair(event.resource_id, event.direction);
            auto it = last.find(key);
            if (it != last.end() &&
                std::abs(merged[it->second].route_end - event.route_start) <= eps &&
                distance(merged[it->second].exit, event.entry) <= eps) {
                auto &old = merged[it->second];
                old.exit = event.exit;
                old.route_end = event.route_end;
                old.traversal_length += event.traversal_length;
                old.exit_edge_id = event.exit_edge_id;
            } else {
                last[key] = merged.size();
                merged.push_back(event);
            }
        }
        events = std::move(merged);
        for (const auto &event : events)
            r.resource_ids.push_back(event.resource_id);
        r.metrics.length = offset;
        r.metrics.score = offset + config.lambda_narrow * r.metrics.narrow_exposure +
                          config.lambda_bottleneck * r.metrics.bottleneck_penalty;
        if (geometry) {
            if (r.path.empty())
                r.path = {start, goal};
            r.path.front() = start;
            r.path.back() = goal;
        }
        return r;
    };
    std::vector<CompleteRoute> result;
    std::set<std::pair<bool, std::vector<int>>> signatures;
    double shortest = 0;
    detail::StageScope selectionScope(PlanningStage::TopologySearch);
    for (int attempt = 0; attempt < config.max_candidates && static_cast<int>(result.size()) < k;
         ++attempt) {
        auto walk = search(attempt == 0);
        if (walk.empty())
            break;
        if (timings) {
            ++timings->physical_candidates;
            timings->physical_route_reachable = true;
        }
        auto candidate = reconstruct(walk, false);
        if (attempt == 0)
            shortest = candidate.metrics.length;
        auto sig = std::make_pair(!candidate.resource_ids.empty(), signature(candidate));
        bool accept = candidate.metrics.length <= shortest * config.max_detour_ratio + eps &&
                      !signatures.count(sig);
        if (accept)
            for (const auto &old : result)
                if (1 - routeOverlap(candidate, old) + eps < config.diversity_threshold) {
                    accept = false;
                    break;
                }
        if (accept) {
            auto complete = reconstruct(walk, true);
            for (const auto &old : result)
                if (geometricallyNear(complete.path, old.physical_path,
                                      config.min_route_separation)) {
                    accept = false;
                    break;
                }
            if (accept) {
                if (!map.validatePath(complete.path, robot))
                    throw std::runtime_error(
                        "shortlisted reconstructed path failed collision validation");
                const auto laneStart = std::chrono::steady_clock::now();
                detail::StageScope reconstructionScope(PlanningStage::Reconstruction);
                const bool lanesValid = applyLanePlacement(map, robot, config, complete);
                if (timings) {
                    timings->rejected_split_candidates +=
                        static_cast<int>(complete.lane_rejections.size());
                    for (const auto &rejection : complete.lane_rejections)
                        timings->split_failure_reasons.push_back(rejection.reason);
                }
                if (lanesValid) {
                    finalizeLaneResources(complete, map, config, robot);
                    complete.id = static_cast<int>(result.size());
                    result.push_back(std::move(complete));
                    signatures.insert(std::move(sig));
                } else if (timings) {
                    ++timings->rejected_lane_routes;
                    if (complete.lane_diagnostics.empty())
                        timings->lane_failures.push_back("complete_lane_conversion_failed");
                    else
                        timings->lane_failures.insert(timings->lane_failures.end(),
                                                      complete.lane_diagnostics.begin(),
                                                      complete.lane_diagnostics.end());
                }
                if (timings)
                    timings->lane_placement_seconds +=
                        std::chrono::duration<double>(std::chrono::steady_clock::now() - laneStart)
                            .count();
            }
        }
        // Penalize rejected repeats as well, so bounded searches can escape a
        // shared neck. Penalties use physical IDs, independent of lane modes.
        for (auto [id, weight] : resourceWeights(candidate)) {
            (void)weight;
            auto &p = penalties[m.resourceIndex.at(id)];
            p = std::min(1e8, p + config.penalty_factor * (1 + attempt / 4));
        }
        for (auto [id, length] : edgeWeights(candidate)) {
            (void)length;
            auto it = std::find_if(data.edges.begin(), data.edges.end(),
                                   [&](const auto &e) { return e.id == id; });
            edgePenalties[it - data.edges.begin()] += config.penalty_factor * 0.2;
        }
        for (int id : walk)
            if (arcs[id].edge < 0)
                arcPenalties[id] += config.penalty_factor * 0.2;
    }
    if (timings && result.empty() && timings->lane_failures.empty())
        timings->lane_failures.push_back("no_attached_feasible_physical_route_within_budget");
    return result;
}
} // namespace corridor
