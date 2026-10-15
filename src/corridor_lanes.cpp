#include "corridor_planner.hpp"
#include "corridor_profile.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace corridor {
namespace {
constexpr double eps = 1e-8;
double length(const Vec2 &a, const Vec2 &b) {
    return std::hypot(a[0] - b[0], a[1] - b[1]);
}
Vec2 mix(const Vec2 &a, const Vec2 &b, double t) {
    return {a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])};
}
Vec2 moved(const Vec2 &p, const Vec2 &normal, double offset) {
    return {p[0] + normal[0] * offset, p[1] + normal[1] * offset};
}
Vec2 unit(const Vec2 &a, const Vec2 &b) {
    double d = length(a, b);
    return d > eps ? Vec2{(b[0] - a[0]) / d, (b[1] - a[1]) / d} : Vec2{0, 0};
}
double pathLength(const std::vector<Vec2> &p) {
    double d = 0;
    for (size_t i = 1; i < p.size(); ++i)
        d += length(p[i - 1], p[i]);
    return d;
}
double clearance(const CorridorMap &map, const std::vector<Vec2> &p) {
    if (p.empty())
        return 0;
    double c = map.segmentClearance(p.front(), p.front());
    for (size_t i = 1; i < p.size(); ++i)
        c = std::min(c, map.segmentClearance(p[i - 1], p[i]));
    return c;
}
Vec2 at(const std::vector<double> &d, const std::vector<Vec2> &p, double s) {
    auto it = std::upper_bound(d.begin(), d.end(), s);
    if (it == d.begin())
        return p.front();
    if (it == d.end())
        return p.back();
    size_t i = it - d.begin();
    return mix(p[i - 1], p[i], (s - d[i - 1]) / (d[i] - d[i - 1]));
}
void smooth(std::vector<double> &v, const std::vector<double> &d, double slope) {
    for (size_t i = 1; i < v.size(); ++i)
        v[i] = std::min(v[i], v[i - 1] + slope * (d[i] - d[i - 1]));
    for (size_t i = v.size() - 1; i > 0; --i)
        v[i - 1] = std::min(v[i - 1], v[i] + slope * (d[i] - d[i - 1]));
}
void addPoint(std::vector<Vec2> &p, const Vec2 &q) {
    if (p.empty() || length(p.back(), q) > eps)
        p.push_back(q);
}
bool progresses(const Vec2 &a, const Vec2 &b, const Vec2 &ca, const Vec2 &cb) {
    const double dx = cb[0] - ca[0], dy = cb[1] - ca[1];
    return (b[0] - a[0]) * dx + (b[1] - a[1]) * dy > eps * std::hypot(dx, dy);
}
bool rightSide(const Vec2 &a, const Vec2 &b, const Vec2 &ca, const Vec2 &cb, int sign) {
    const auto t = unit(ca, cb);
    return sign * ((a[0] - ca[0]) * t[1] - (a[1] - ca[1]) * t[0]) > eps &&
           sign * ((b[0] - cb[0]) * t[1] - (b[1] - cb[1]) * t[0]) > eps;
}
struct ChordBox {
    size_t index;
    double minx, maxx, miny, maxy;
};
std::vector<ChordBox> chordBoxes(const std::vector<Vec2> &p) {
    std::vector<ChordBox> boxes;
    for (size_t i = 1; i < p.size(); ++i)
        boxes.push_back({i - 1, std::min(p[i - 1][0], p[i][0]), std::max(p[i - 1][0], p[i][0]),
                         std::min(p[i - 1][1], p[i][1]), std::max(p[i - 1][1], p[i][1])});
    std::sort(boxes.begin(), boxes.end(), [](const auto &a, const auto &b) {
        return a.minx < b.minx || (a.minx == b.minx && a.index < b.index);
    });
    return boxes;
}
template <typename F>
void nearbyChords(const std::vector<ChordBox> &a, const std::vector<ChordBox> &b, double margin,
                  F visit) {
    double span = 0;
    for (const auto &box : b)
        span = std::max(span, box.maxx - box.minx);
    for (const auto &box : a) {
        auto first = std::lower_bound(b.begin(), b.end(), box.minx - margin - span,
                                      [](const auto &other, double x) { return other.minx < x; });
        for (auto it = first; it != b.end() && it->minx <= box.maxx + margin; ++it)
            if (it->maxx + margin >= box.minx && it->maxy + margin >= box.miny &&
                box.maxy + margin >= it->miny)
                visit(box.index, it->index);
    }
}
} // namespace

int pathSelfIntersections(const std::vector<Vec2> &path) {
    detail::StageScope scope(PlanningStage::CollisionValidation);
    int count = 0;
    const auto boxes = chordBoxes(path);
    nearbyChords(boxes, boxes, eps, [&](size_t i, size_t j) {
        if (j >= i + 2 &&
            segmentDistanceSquared(path[i], path[i + 1], path[j], path[j + 1]) <= eps * eps)
            ++count;
    });
    // Adjacent collinear reversals overlap even though ordinary adjacent joins are legal.
    for (size_t i = 1; i + 1 < path.size(); ++i) {
        const auto a = unit(path[i - 1], path[i]), b = unit(path[i], path[i + 1]);
        if (a[0] * b[0] + a[1] * b[1] < -1 + 1e-8)
            ++count;
    }
    return count;
}

bool validateLaneConfig(const LaneConfig &c) {
    const double positive[] = {c.sample_spacing,    c.max_offset,       c.clearance_guard,
                               c.separation_margin, c.min_split_length, c.transition_length,
                               c.max_offset_slope,  c.miter_limit};
    for (double v : positive)
        if (!std::isfinite(v) || v <= 0)
            return false;
    return std::isfinite(c.hysteresis_length) && c.hysteresis_length >= 0 && c.miter_limit >= 1 &&
           c.max_samples_per_edge >= 16 && c.max_samples_per_edge <= 65536 &&
           c.offset_iterations >= 1 && c.offset_iterations <= 24 && c.refinement_iterations >= 0 &&
           c.refinement_iterations <= 8;
}
double pathSeparation(const std::vector<Vec2> &a, const std::vector<Vec2> &b) {
    detail::StageScope scope(PlanningStage::CollisionValidation);
    if (a.empty() || b.empty())
        return 0;
    double best = std::pow(length(a.front(), b.front()), 2);
    if (a.size() == 1 || b.size() == 1) {
        for (size_t i = 0; i < std::max<size_t>(1, a.size() - 1); ++i)
            for (size_t j = 0; j < std::max<size_t>(1, b.size() - 1); ++j)
                best =
                    std::min(best, segmentDistanceSquared(a[i], a[std::min(i + 1, a.size() - 1)],
                                                          b[j], b[std::min(j + 1, b.size() - 1)]));
    } else {
        const auto ab = chordBoxes(a), bb = chordBoxes(b);
        // Endpoint distance is an upper bound on the minimum. Search every pair
        // whose boxes can improve it; no sampled-only separation approximation.
        nearbyChords(ab, bb, std::sqrt(best) + eps, [&](size_t i, size_t j) {
            best = std::min(best, segmentDistanceSquared(a[i], a[i + 1], b[j], b[j + 1]));
        });
    }
    return std::sqrt(best);
}

static LaneDecomposition buildLaneDecompositionImpl(const CorridorMap &map, const RoutingEdge &edge,
                                                    const RobotProfile &robot,
                                                    const LaneConfig &cfg, bool compact) {
    detail::StageScope scope(PlanningStage::LaneDecomposition);
    if (!validateLaneConfig(cfg))
        throw std::invalid_argument("invalid lane configuration");
    const double R = robot.radius + robot.safety_margin;
    if (!std::isfinite(R) || robot.radius < 0 || robot.safety_margin < 0)
        throw std::invalid_argument("invalid robot profile");
    if (edge.reference_geometry.size() < 2 ||
        edge.reference_distances.size() != edge.reference_geometry.size())
        throw std::invalid_argument("invalid lane reference");
    if (edge.reference_distances.front() != 0)
        throw std::invalid_argument("nonzero lane arclength origin");
    if (edge.reference_geometry.size() > static_cast<size_t>(cfg.max_samples_per_edge))
        throw std::length_error("lane reference exceeds sample budget");
    for (size_t i = 0; i < edge.reference_geometry.size(); ++i) {
        if (!std::isfinite(edge.reference_geometry[i][0]) ||
            !std::isfinite(edge.reference_geometry[i][1]) ||
            !std::isfinite(edge.reference_distances[i]) ||
            (i && (edge.reference_distances[i] < edge.reference_distances[i - 1] ||
                   (edge.reference_distances[i] == edge.reference_distances[i - 1] &&
                    length(edge.reference_geometry[i], edge.reference_geometry[i - 1]) > eps))))
            throw std::invalid_argument("invalid lane station");
    }
    LaneDecomposition out;
    out.physical_edge_id = edge.id;
    out.source_reference = edge.reference_geometry;
    out.effective_radius = R;
    const double nominal = std::min(cfg.sample_spacing, std::max(.1, R * .5));
    const size_t extra =
        static_cast<size_t>(cfg.max_samples_per_edge) - edge.reference_geometry.size();
    const double spacing = extra ? std::max(nominal, edge.reference_distances.back() / extra)
                                 : std::numeric_limits<double>::infinity();
    // Clean near-zero chords before any tangent calculation. Preserve original
    // arclength stations for canonical resource mapping; never alter the graph.
    std::vector<Vec2> reference{edge.reference_geometry.front()};
    std::vector<double> stations{0};
    const double tiny = std::max(1e-6, nominal * .01);
    for (size_t j = 1; j < edge.reference_geometry.size(); ++j) {
        const auto &p = edge.reference_geometry[j];
        if (length(reference.back(), p) < tiny && j + 1 < edge.reference_geometry.size())
            continue;
        if (j + 1 == edge.reference_geometry.size() && reference.size() > 1 &&
            length(reference.back(), p) < tiny) {
            reference.pop_back();
            stations.pop_back();
        }
        reference.push_back(p);
        stations.push_back(edge.reference_distances[j]);
    }
    if (reference.size() < 2 || pathLength(reference) <= eps)
        throw std::invalid_argument("degenerate lane reference");
    out.center.push_back(reference.front());
    out.distances.push_back(0);
    for (size_t j = 1; j < reference.size(); ++j) {
        const double ds = stations[j] - stations[j - 1];
        const int count = std::max(1, static_cast<int>(std::ceil(ds / spacing)));
        for (int k = 1; k <= count; ++k) {
            double t = static_cast<double>(k) / count;
            out.center.push_back(mix(reference[j - 1], reference[j], t));
            out.distances.push_back(stations[j - 1] + t * ds);
        }
    }
    const size_t n = out.center.size();
    std::vector<Vec2> normals(n);
    std::vector<double> right(n), left(n), rightCapacity(n), leftCapacity(n), radialSum(n);
    const double guard = std::max(cfg.clearance_guard, eps * 10), required = 2 * R + eps * 4;
    const double minimumPreferred = R + cfg.separation_margin / 2 + guard;
    // The compact retry cannot use more than a separation-sized displacement
    // followed by the configured bounded expansion steps. Primary profiles still
    // measure full available space up to max_offset.
    const double compactBound = std::max(required, std::max(1.2 * R, minimumPreferred)) *
                                    std::pow(1.15, cfg.refinement_iterations) +
                                2 * guard + cfg.separation_margin;
    const double searchBound = compact ? std::min(cfg.max_offset, compactBound) : cfg.max_offset;
    std::vector<bool> centerSafe(n);
    std::vector<double> centerClearance(n);
    for (size_t i = 0; i < n; ++i) {
        centerClearance[i] = map.segmentClearance(out.center[i], out.center[i]);
        centerSafe[i] = centerClearance[i] > R + eps;
    }
    // The compact retry also tests a longer finite frame near short bends.
    // It changes the geometric candidate family, not footprint tolerance.
    const double minimumWindow =
        std::max({cfg.sample_spacing, 2 * R, compact ? cfg.transition_length : 0.0});
    const auto axis = unit(out.center.front(), out.center.back());
    const Vec2 axisNormal{axis[1], -axis[0]};
    std::vector<double> axisOffset(n);
    bool nearStraight = compact;
    for (size_t i = 0; i < n; ++i) {
        axisOffset[i] = (out.center[i][0] - out.center.front()[0]) * axisNormal[0] +
                        (out.center[i][1] - out.center.front()[1]) * axisNormal[1];
        nearStraight = nearStraight && std::abs(axisOffset[i]) <= .25 * cfg.sample_spacing;
        if (i)
            nearStraight =
                nearStraight && (out.center[i][0] - out.center[i - 1][0]) * axis[0] +
                                        (out.center[i][1] - out.center[i - 1][1]) * axis[1] >
                                    eps;
    }
    auto normalAt = [&](size_t i, double window) {
        if (nearStraight)
            return axisNormal;
        const auto a = at(out.distances, out.center, std::max(0.0, out.distances[i] - window));
        const auto b = at(out.distances, out.center,
                          std::min(out.distances.back(), out.distances[i] + window));
        const auto tangent = unit(a, b);
        return Vec2{tangent[1], -tangent[0]};
    };
    auto refreshCapacity = [&](size_t i) {
        if (!centerSafe[i])
            return;
        const auto normal = normals[i];
        auto capacity = [&](int sign) {
            // Distance-to-obstacle is 1-Lipschitz: this whole radial sweep is
            // certified by the existing center clearance, with contact reserve.
            double lo = std::min(searchBound, std::max(0.0, centerClearance[i] - R - 2 * eps));
            double hi = searchBound;
            if (lo >= hi)
                return hi;
            const double probe = std::min(hi, lo + 1e-7);
            if (map.segmentClearance(out.center[i], moved(out.center[i], normal, sign * probe)) <=
                R + eps)
                return lo;
            // In open space the entire configured search bound is already feasible.
            // Avoid twenty repeated obstacle queries to rediscover that fact.
            if (map.segmentClearance(out.center[i], moved(out.center[i], normal, sign * hi)) >
                R + eps)
                return hi;
            for (int attempt = 0; attempt < cfg.offset_iterations; ++attempt) {
                double mid = (lo + hi) / 2;
                // Checking the entire radial segment makes the capacity monotone
                // and prevents jumping across a wall into another free-space component.
                if (map.segmentClearance(out.center[i], moved(out.center[i], normal, sign * mid)) >
                    R + eps)
                    lo = mid;
                else
                    hi = mid;
            }
            return lo;
        };
        right[i] = capacity(1);
        left[i] = capacity(-1);
        // Prefer a wall guard, but do not turn a legal near-threshold pair into
        // SHARED merely because an extra (non-footprint) reserve would not fit.
        const double reserve = std::min(
            guard, std::max(0.0, (right[i] + left[i] - required - cfg.separation_margin) / 4));
        // An asymmetric reference may have very little room on one side.
        // A comfort reserve must not erase a usable positive directional offset.
        right[i] -= std::min(right[i] * .5, reserve);
        left[i] -= std::min(left[i] * .5, reserve);
        // Start halfway through actual usable displacement on each side. Keep
        // at least a footprint-sized displacement where possible, compensating
        // on the other side when the stored reference is asymmetric.
        const double rc = right[i], lc = left[i];
        radialSum[i] = rc + lc;
        rightCapacity[i] = rc;
        leftCapacity[i] = lc;
        const double target = compact ? std::max(1.2 * R, minimumPreferred) : minimumPreferred;
        right[i] = std::min(rc, compact ? target : std::max(target, rc * .5));
        left[i] = std::min(lc, compact ? target : std::max(target, lc * .5));
        if (right[i] + left[i] < required) {
            right[i] = std::min(rc, std::max(right[i], required - left[i]));
            left[i] = std::min(lc, std::max(left[i], required - right[i]));
        }
    };
    for (size_t i = 0; i < n; ++i) {
        normals[i] = normalAt(i, minimumWindow);
        refreshCapacity(i);
    }
    // A footprint-sized normal window is insufficient for a wide lane offset.
    // Two bounded passes round the frame over the intended displacement, retaining
    // unit normals and recomputing actual radial capacities when the frame changes.
    for (int pass = 0; pass < 2; ++pass) {
        smooth(right, out.distances, cfg.max_offset_slope);
        smooth(left, out.distances, cfg.max_offset_slope);
        std::vector<double> windows(n);
        for (size_t i = 0; i < n; ++i)
            windows[i] = std::max({minimumWindow, 2 * right[i], 2 * left[i]});
        smooth(windows, out.distances, 2 * cfg.max_offset_slope);
        for (size_t i = 0; i < n; ++i) {
            const auto normal = normalAt(i, windows[i]);
            if (length(normal, normals[i]) > 1e-8) {
                normals[i] = normal;
                refreshCapacity(i);
            }
        }
    }
    // Bound bend displacement by the longitudinal motion of the normal field.
    // This limits inner-lane foldbacks without clamping long straight sections.
    for (size_t i = 0; i + 1 < n; ++i) {
        Vec2 chord{out.center[i + 1][0] - out.center[i][0],
                   out.center[i + 1][1] - out.center[i][1]};
        const double turn = std::abs((normals[i + 1][0] - normals[i][0]) * chord[0] +
                                     (normals[i + 1][1] - normals[i][1]) * chord[1]);
        if (turn > eps * eps) {
            const double cap = .6 * (chord[0] * chord[0] + chord[1] * chord[1]) / turn;
            for (size_t k : {i, i + 1}) {
                rightCapacity[k] = std::min(rightCapacity[k], 1.5 * cap);
                leftCapacity[k] = std::min(leftCapacity[k], 1.5 * cap);
                right[k] = std::min(right[k], cap);
                left[k] = std::min(left[k], cap);
            }
        }
    }
    smooth(right, out.distances, cfg.max_offset_slope);
    smooth(left, out.distances, cfg.max_offset_slope);
    if (compact) {
        // Near 4R, bulging into a wider pocket and shrinking a failed join can
        // erase the tiny separation reserve. Try a consistent pair over each
        // capacity-feasible run, if its componentwise minimum capacities fit it.
        // This is an additional checked candidate, never relaxed clearance.
        for (size_t begin = 0; begin < n;) {
            if (!centerSafe[begin] || rightCapacity[begin] + leftCapacity[begin] < required) {
                ++begin;
                continue;
            }
            size_t end = begin;
            double rc = rightCapacity[begin], lc = leftCapacity[begin];
            double minRight = -std::numeric_limits<double>::infinity();
            double maxRight = std::numeric_limits<double>::infinity();
            double minLeft = minRight, maxLeft = maxRight;
            while (end < n && centerSafe[end] &&
                   rightCapacity[end] + leftCapacity[end] >= required) {
                rc = std::min(rc, rightCapacity[end]);
                lc = std::min(lc, leftCapacity[end]);
                minRight = std::max(minRight, axisOffset[end] + 2 * eps);
                maxRight = std::min(maxRight, axisOffset[end] + rightCapacity[end]);
                minLeft = std::max(minLeft, axisOffset[end] - leftCapacity[end]);
                maxLeft = std::min(maxLeft, axisOffset[end] - 2 * eps);
                ++end;
            }
            if (nearStraight && minRight <= maxRight && minLeft <= maxLeft &&
                maxRight - minLeft >= required) {
                // Compensate tiny reference wiggles instead of copying them into
                // opposing paths. The actual original reference still supplies
                // every station's mandatory-side and forward-progress checks.
                const double target = std::max(1.2 * R, minimumPreferred);
                double r = std::clamp(target, minRight, maxRight);
                double l = std::clamp(-target, minLeft, maxLeft);
                r = std::min(maxRight, std::max(r, l + required));
                l = std::max(minLeft, std::min(l, r - required));
                for (size_t k = begin; k < end; ++k) {
                    right[k] = r - axisOffset[k];
                    left[k] = axisOffset[k] - l;
                }
            } else if (rc + lc >= required) {
                const double target = std::max(1.2 * R, minimumPreferred);
                double r = std::min(rc, target), l = std::min(lc, target);
                r = std::min(rc, std::max(r, required - l));
                l = std::min(lc, std::max(l, required - r));
                for (size_t k = begin; k < end; ++k) {
                    right[k] = r;
                    left[k] = l;
                }
            }
            begin = end;
        }
        smooth(right, out.distances, cfg.max_offset_slope);
        smooth(left, out.distances, cfg.max_offset_slope);
    }
    std::vector<Vec2> rawRight(n), rawLeft(n);
    std::vector<bool> valid(n - 1), blocked(n - 1), safeCells(n - 1), progressCells(n - 1),
        separateCells(n - 1), enoughCells(n - 1), sideCells(n - 1);
    // Bounded local refinement of joins, always followed by continuous segment checks.
    for (int pass = 0; pass <= cfg.refinement_iterations; ++pass) {
        for (size_t i = 0; i < n; ++i) {
            rawRight[i] = moved(out.center[i], normals[i], right[i]);
            rawLeft[i] = moved(out.center[i], normals[i], -left[i]);
        }
        std::vector<bool> shrink(n), expand(n);
        for (size_t i = 0; i + 1 < n; ++i) {
            blocked[i] = map.segmentClearance(out.center[i], out.center[i + 1]) <= R + eps;
            bool enough = right[i] + left[i] >= required && right[i + 1] + left[i + 1] >= required;
            bool safe = map.segmentClearance(rawRight[i], rawRight[i + 1]) > R + eps &&
                        map.segmentClearance(rawLeft[i], rawLeft[i + 1]) > R + eps;
            bool separate = segmentDistanceSquared(rawRight[i], rawRight[i + 1], rawLeft[i],
                                                   rawLeft[i + 1]) >= required * required;
            bool monotone =
                progresses(rawRight[i], rawRight[i + 1], out.center[i], out.center[i + 1]) &&
                progresses(rawLeft[i], rawLeft[i + 1], out.center[i], out.center[i + 1]);
            bool sides =
                rightSide(rawRight[i], rawRight[i + 1], out.center[i], out.center[i + 1], 1) &&
                rightSide(rawLeft[i], rawLeft[i + 1], out.center[i], out.center[i + 1], -1);
            safeCells[i] = safe;
            progressCells[i] = monotone;
            separateCells[i] = separate;
            enoughCells[i] = enough;
            sideCells[i] = sides;
            valid[i] = !blocked[i] && enough && safe && separate && monotone && sides;
            if (enough && (!safe || !monotone))
                shrink[i] = shrink[i + 1] = true;
            else if (safe && monotone && (!separate || !enough) && radialSum[i] >= required &&
                     radialSum[i + 1] >= required)
                expand[i] = expand[i + 1] = true;
        }
        const auto rightBoxes = chordBoxes(rawRight), leftBoxes = chordBoxes(rawLeft);
        nearbyChords(rightBoxes, leftBoxes, required, [&](size_t i, size_t j) {
            if (valid[i] && valid[j] &&
                segmentDistanceSquared(rawRight[i], rawRight[i + 1], rawLeft[j], rawLeft[j + 1]) <
                    required * required)
                expand[i] = expand[i + 1] = expand[j] = expand[j + 1] = true;
        });
        if (pass == cfg.refinement_iterations ||
            (std::none_of(shrink.begin(), shrink.end(), [](bool v) { return v; }) &&
             std::none_of(expand.begin(), expand.end(), [](bool v) { return v; })))
            break;
        auto nextRight = right, nextLeft = left;
        const double window = std::max(cfg.sample_spacing, 2 * R);
        // A corner may need more separation than a straight section. Expand a
        // finite neighbourhood within actual radial capacities; shrinking a
        // separated-but-too-close pair would only worsen the failed certificate.
        for (size_t i = 0; i < n; ++i)
            if (expand[i]) {
                auto begin = std::lower_bound(out.distances.begin(), out.distances.end(),
                                              out.distances[i] - window);
                for (size_t j = begin - out.distances.begin();
                     j < n && out.distances[j] <= out.distances[i] + window; ++j) {
                    double factor =
                        1 + .15 * (1 - std::abs(out.distances[j] - out.distances[i]) / window);
                    nextRight[j] =
                        std::max(nextRight[j], std::min(rightCapacity[j], right[j] * factor));
                    nextLeft[j] =
                        std::max(nextLeft[j], std::min(leftCapacity[j], left[j] * factor));
                }
            }
        for (size_t i = 0; i < n; ++i)
            if (shrink[i]) {
                nextRight[i] = right[i] * .97;
                nextLeft[i] = left[i] * .97;
            }
        right = std::move(nextRight);
        left = std::move(nextLeft);
        smooth(right, out.distances, cfg.max_offset_slope);
        smooth(left, out.distances, cfg.max_offset_slope);
    }
    // Opposing streams can collide at nonadjacent pieces of a bend/loop too.
    std::vector<bool> bad(n - 1);
    const auto rightBoxes = chordBoxes(rawRight), leftBoxes = chordBoxes(rawLeft);
    nearbyChords(rightBoxes, leftBoxes, required, [&](size_t i, size_t j) {
        if (valid[i] && valid[j] &&
            segmentDistanceSquared(rawRight[i], rawRight[i + 1], rawLeft[j], rawLeft[j + 1]) <
                required * required)
            bad[i] = bad[j] = true;
    });
    for (size_t i = 0; i < valid.size(); ++i)
        valid[i] = valid[i] && !bad[i];
    std::vector<std::string> reasons(n - 1);
    for (size_t i = 0; i < reasons.size(); ++i)
        reasons[i] = blocked[i]          ? "center_footprint_blocked"
                     : !enoughCells[i]   ? (radialSum[i] < required || radialSum[i + 1] < required
                                                ? "insufficient_lateral_capacity"
                                                : "bounded_bend_offset_or_refinement")
                     : !safeCells[i]     ? "lane_footprint_clearance"
                     : !progressCells[i] ? "backward_lane_progress"
                     : !sideCells[i]     ? "wrong_side_or_centerline_lane"
                     : !separateCells[i] || bad[i] ? "opposing_swept_areas_overlap"
                                                   : "insufficient_core_and_transition_room";
    // Reject both sides of any same-lane crossing before choosing split runs.
    for (const auto *lane : {&rawRight, &rawLeft}) {
        const auto boxes = chordBoxes(*lane);
        nearbyChords(boxes, boxes, eps, [&](size_t i, size_t j) {
            if (j >= i + 2 && segmentDistanceSquared((*lane)[i], (*lane)[i + 1], (*lane)[j],
                                                     (*lane)[j + 1]) <= eps * eps) {
                valid[i] = valid[j] = false;
                reasons[i] = reasons[j] = "lane_self_intersection";
            }
        });
    }
    out.forward_lane = out.reverse_lane = out.center;
    out.cells.resize(n - 1, LaneState::Shared);
    for (size_t i = 0; i < blocked.size(); ++i)
        if (blocked[i])
            out.cells[i] = LaneState::Blocked;
    // Length hysteresis delays entry into a new mode, while exits occur as
    // soon as actual collision/separation checks fail. It never substitutes a
    // maximum-width sample for a geometric separation certificate.
    for (size_t begin = 0; begin < valid.size();) {
        detail::StageScope transitionScope(PlanningStage::Transitions);
        if (!valid[begin]) {
            ++begin;
            continue;
        }
        size_t end = begin;
        while (end < valid.size() && valid[end])
            ++end;
        size_t localBegin = begin, localEnd = end;
        // Retry only the affected endpoint neighbourhood. Failure at a junction
        // does not discard a long certified core farther along the corridor.
        for (int attempt = 0; attempt <= cfg.refinement_iterations && localBegin < localEnd;
             ++attempt) {
            const double lo = out.distances[localBegin], hi = out.distances[localEnd];
            auto taperDistance = [&](bool entry) {
                if ((entry && localBegin == 0 && !cfg.taper_endpoints) ||
                    (!entry && localEnd == valid.size() && !cfg.taper_endpoints))
                    return 0.0;
                double result = std::max(cfg.transition_length, 2 * R);
                double maximum = 0;
                for (size_t k = localBegin; k <= localEnd; ++k)
                    maximum = std::max({maximum, right[k], left[k]});
                for (int pass = 0; pass < 8; ++pass) {
                    double nearby = 0;
                    for (size_t k = localBegin; k <= localEnd; ++k)
                        if ((entry ? out.distances[k] - lo : hi - out.distances[k]) <= result + eps)
                            nearby = std::max({nearby, right[k], left[k]});
                    const double need = 1.5 * nearby / cfg.max_offset_slope;
                    if (need <= result + eps)
                        return result;
                    result = std::max(result, need);
                }
                return std::max(result, 1.5 * maximum / cfg.max_offset_slope);
            };
            const double enter = taperDistance(true), exit = taperDistance(false);
            if (hi - lo <
                enter + exit + std::max(cfg.min_split_length, 2 * R) + cfg.hysteresis_length)
                break;
            auto trialRight = out.forward_lane, trialLeft = out.reverse_lane;
            for (size_t k = localBegin; k <= localEnd; ++k) {
                double factor = 1;
                if (enter > 0)
                    factor = std::min(factor, (out.distances[k] - lo) / enter);
                if (exit > 0)
                    factor = std::min(factor, (hi - out.distances[k]) / exit);
                factor = std::clamp(factor, 0.0, 1.0);
                factor = factor * factor * (3 - 2 * factor);
                trialRight[k] = moved(out.center[k], normals[k], right[k] * factor);
                trialLeft[k] = moved(out.center[k], normals[k], -left[k] * factor);
            }
            size_t failed = localEnd;
            std::string why;
            for (size_t k = localBegin; k < localEnd; ++k) {
                if (map.segmentClearance(trialRight[k], trialRight[k + 1]) <= R + eps ||
                    map.segmentClearance(trialLeft[k], trialLeft[k + 1]) <= R + eps) {
                    failed = k;
                    why = "local_transition_footprint_clearance";
                    break;
                }
                if (!progresses(trialRight[k], trialRight[k + 1], out.center[k],
                                out.center[k + 1]) ||
                    !progresses(trialLeft[k], trialLeft[k + 1], out.center[k], out.center[k + 1])) {
                    failed = k;
                    why = "local_transition_backward_progress";
                    break;
                }
            }
            if (failed == localEnd &&
                (pathSelfIntersections(trialRight) || pathSelfIntersections(trialLeft)))
                why = "transition_self_intersection";
            if (why.empty()) {
                out.forward_lane = std::move(trialRight);
                out.reverse_lane = std::move(trialLeft);
                for (size_t k = localBegin; k < localEnd; ++k) {
                    reasons[k].clear();
                    out.cells[k] = out.distances[k] >= lo + enter - eps &&
                                           out.distances[k + 1] <= hi - exit + eps
                                       ? LaneState::Split
                                       : LaneState::Transition;
                }
                break;
            }
            for (size_t k = localBegin; k < localEnd; ++k)
                reasons[k] = why;
            if (failed < localEnd && out.distances[failed] < lo + enter)
                localBegin = failed + 1;
            else if (failed < localEnd && out.distances[failed + 1] > hi - exit)
                localEnd = failed;
            else
                break;
        }
        if (!compact && std::none_of(out.cells.begin() + begin, out.cells.begin() + end,
                                     [](LaneState state) { return state == LaneState::Split; })) {
            // Preferred half-space offsets may require excessive taper room.
            // Try one footprint-sized pair locally, never shrinking a successful
            // wide corridor or treating a failed preferred candidate as infeasible.
            out.rejections.push_back({out.distances[begin], out.distances[end],
                                      "preferred_pair_has_no_usable_core_or_transition"});
            RoutingEdge local;
            local.id = -1;
            local.reference_geometry.assign(out.center.begin() + begin,
                                            out.center.begin() + end + 1);
            for (size_t k = begin; k <= end; ++k)
                local.reference_distances.push_back(out.distances[k] - out.distances[begin]);
            auto localConfig = cfg;
            localConfig.taper_endpoints = true;
            const auto alternative =
                buildLaneDecompositionImpl(map, local, robot, localConfig, true);
            auto a = out.forward_lane, b = out.reverse_lane;
            auto states = out.cells;
            bool hasSplit = false;
            for (size_t k = begin; k <= end; ++k) {
                const double s = out.distances[k] - out.distances[begin];
                a[k] = at(alternative.distances, alternative.forward_lane, s);
                b[k] = at(alternative.distances, alternative.reverse_lane, s);
                if (k < end) {
                    const double next = out.distances[k + 1] - out.distances[begin];
                    LaneState state = LaneState::Shared;
                    for (const auto &interval : alternative.intervals)
                        if (std::min(next, interval.last_distance) -
                                std::max(s, interval.first_distance) >
                            eps) {
                            if (interval.state == LaneState::Split &&
                                s >= interval.first_distance - eps &&
                                next <= interval.last_distance + eps)
                                state = LaneState::Split;
                            else if (interval.state != LaneState::Shared)
                                state = LaneState::Transition;
                        }
                    states[k] = state;
                    hasSplit = hasSplit || state == LaneState::Split;
                }
            }
            bool safe = hasSplit && !pathSelfIntersections(a) && !pathSelfIntersections(b);
            for (size_t k = begin; k < end && safe; ++k)
                safe = map.segmentClearance(a[k], a[k + 1]) > R + eps &&
                       map.segmentClearance(b[k], b[k + 1]) > R + eps &&
                       progresses(a[k], a[k + 1], out.center[k], out.center[k + 1]) &&
                       progresses(b[k], b[k + 1], out.center[k], out.center[k + 1]);
            const auto ab = chordBoxes(a), bb = chordBoxes(b);
            nearbyChords(ab, bb, required, [&](size_t i, size_t j) {
                if (states[i] == LaneState::Split && states[j] == LaneState::Split &&
                    segmentDistanceSquared(a[i], a[i + 1], b[j], b[j + 1]) < required * required)
                    safe = false;
            });
            if (safe) {
                out.forward_lane = std::move(a);
                out.reverse_lane = std::move(b);
                out.cells = std::move(states);
                for (size_t k = begin; k < end; ++k)
                    if (out.cells[k] != LaneState::Shared)
                        reasons[k].clear();
            }
        }
        begin = end;
    }
    // Recover preferred-frame failures from a complete compact certificate.
    // Blending is local and may never invalidate an existing separated core.
    const bool fragmented =
        !out.rejections.empty() ||
        std::any_of(reasons.begin(), reasons.end(), [](const std::string &reason) {
            return !reason.empty() && reason != "insufficient_lateral_capacity" &&
                   reason != "center_footprint_blocked";
        });
    if (!compact && fragmented &&
        out.distances.back() >= std::max(cfg.min_split_length, 2 * R) + cfg.hysteresis_length) {
        const auto alternative = buildLaneDecompositionImpl(map, edge, robot, cfg, true);
        if (alternative.distances == out.distances) {
            std::vector<double> weights(n, 0);
            bool recover = false;
            for (size_t i = 0; i < out.cells.size();) {
                if (out.cells[i] == LaneState::Split || out.cells[i] == LaneState::Blocked ||
                    alternative.cells[i] != LaneState::Split) {
                    ++i;
                    continue;
                }
                size_t j = i + 1;
                while (j < out.cells.size() && out.cells[j] != LaneState::Split &&
                       out.cells[j] != LaneState::Blocked &&
                       alternative.cells[j] == LaneState::Split)
                    ++j;
                double difference = 0;
                for (size_t k = 0; k < n; ++k)
                    if (out.distances[k] >= out.distances[i] - 2 * cfg.max_offset &&
                        out.distances[k] <= out.distances[j] + 2 * cfg.max_offset)
                        difference = std::max(
                            {difference, length(out.forward_lane[k], alternative.forward_lane[k]),
                             length(out.reverse_lane[k], alternative.reverse_lane[k])});
                const double ramp = std::max(
                    {cfg.transition_length, 2 * R, 1.5 * difference / cfg.max_offset_slope});
                for (size_t k = 0; k < n; ++k) {
                    const double distance = std::max({0.0, out.distances[i] - out.distances[k],
                                                      out.distances[k] - out.distances[j]});
                    double w = std::clamp(1 - distance / ramp, 0.0, 1.0);
                    w = w * w * (3 - 2 * w);
                    weights[k] = std::max(weights[k], w);
                }
                recover = true;
                i = j;
            }
            if (recover) {
                auto a = out.forward_lane, b = out.reverse_lane;
                for (size_t i = 0; i < n; ++i) {
                    a[i] = mix(a[i], alternative.forward_lane[i], weights[i]);
                    b[i] = mix(b[i], alternative.reverse_lane[i], weights[i]);
                }
                bool safe = !pathSelfIntersections(a) && !pathSelfIntersections(b);
                std::vector<bool> eligible(n - 1);
                for (size_t i = 0; i + 1 < n; ++i) {
                    safe = safe && map.segmentClearance(a[i], a[i + 1]) > R + eps &&
                           map.segmentClearance(b[i], b[i + 1]) > R + eps &&
                           progresses(a[i], a[i + 1], out.center[i], out.center[i + 1]) &&
                           progresses(b[i], b[i + 1], out.center[i], out.center[i + 1]);
                    eligible[i] = rightSide(a[i], a[i + 1], out.center[i], out.center[i + 1], 1) &&
                                  rightSide(b[i], b[i + 1], out.center[i], out.center[i + 1], -1) &&
                                  segmentDistanceSquared(a[i], a[i + 1], b[i], b[i + 1]) >=
                                      required * required;
                }
                const auto ab = chordBoxes(a), bb = chordBoxes(b);
                std::vector<bool> conflicts(n - 1);
                nearbyChords(ab, bb, required, [&](size_t i, size_t j) {
                    if (eligible[i] && eligible[j] &&
                        segmentDistanceSquared(a[i], a[i + 1], b[j], b[j + 1]) <
                            required * required)
                        conflicts[i] = conflicts[j] = true;
                });
                for (size_t i = 0; i < eligible.size(); ++i)
                    eligible[i] = eligible[i] && !conflicts[i];
                for (size_t i = 0; i < eligible.size();) {
                    if (!eligible[i]) {
                        ++i;
                        continue;
                    }
                    size_t j = i + 1;
                    while (j < eligible.size() && eligible[j])
                        ++j;
                    if (out.distances[j] - out.distances[i] <
                        std::max(cfg.min_split_length, 2 * R) + cfg.hysteresis_length)
                        for (size_t k = i; k < j; ++k)
                            eligible[k] = false;
                    i = j;
                }
                for (size_t i = 0; i < eligible.size(); ++i)
                    if (out.cells[i] == LaneState::Split && !eligible[i])
                        safe = false;
                if (safe) {
                    out.forward_lane = std::move(a);
                    out.reverse_lane = std::move(b);
                    std::vector<bool> recovered(eligible.size());
                    for (size_t i = 0; i < eligible.size(); ++i) {
                        if (eligible[i]) {
                            recovered[i] = out.cells[i] != LaneState::Split;
                            out.cells[i] = LaneState::Split;
                            reasons[i].clear();
                        } else if (length(out.forward_lane[i], out.center[i]) > eps ||
                                   length(out.forward_lane[i + 1], out.center[i + 1]) > eps ||
                                   length(out.reverse_lane[i], out.center[i]) > eps ||
                                   length(out.reverse_lane[i + 1], out.center[i + 1]) > eps) {
                            out.cells[i] = LaneState::Transition;
                            reasons[i].clear();
                        }
                    }
                    for (size_t i = 0; i < recovered.size();) {
                        if (!recovered[i]) {
                            ++i;
                            continue;
                        }
                        size_t j = i + 1;
                        while (j < recovered.size() && recovered[j])
                            ++j;
                        out.rejections.push_back({out.distances[i], out.distances[j],
                                                  "preferred_frame_recovered_with_compact_pair"});
                        i = j;
                    }
                } else {
                    out.rejections.push_back(
                        {0, out.distances.back(), "compact_gap_connection_not_certified"});
                }
            }
        }
    }
    // Round actual lane joins, rather than only their normal field. Keep all
    // reference stations/modes fixed and certify the paired correction globally.
    // Unsafe local rounding is suppressed without abandoning a valid SPLIT.
    auto roundedRight = out.forward_lane, roundedLeft = out.reverse_lane;
    std::vector<double> blend(n, 0);
    const double joinRadius = std::max(cfg.sample_spacing, R);
    for (size_t i = 0; i < out.cells.size();) {
        size_t j = i + 1;
        while (j < out.cells.size() && out.cells[j] == out.cells[i])
            ++j;
        if (out.cells[i] == LaneState::Split)
            for (size_t k = i + 1; k < j; ++k) {
                const double radius = std::min({joinRadius, out.distances[k] - out.distances[i],
                                                out.distances[j] - out.distances[k]});
                for (auto pair : {std::make_pair(&roundedRight, &out.forward_lane),
                                  std::make_pair(&roundedLeft, &out.reverse_lane)}) {
                    const auto a = at(out.distances, *pair.second, out.distances[k] - radius);
                    const auto b = at(out.distances, *pair.second, out.distances[k] + radius);
                    (*pair.first)[k] = mix((*pair.second)[k], mix(a, b, .5), .5);
                }
                blend[k] = 1;
            }
        i = j;
    }
    for (int pass = 0; pass <= cfg.refinement_iterations; ++pass) {
        auto a = out.forward_lane, b = out.reverse_lane;
        for (size_t i = 0; i < n; ++i) {
            a[i] = mix(a[i], roundedRight[i], blend[i]);
            b[i] = mix(b[i], roundedLeft[i], blend[i]);
        }
        std::vector<bool> badJoin(n);
        for (size_t i = 0; i + 1 < n; ++i)
            if (out.cells[i] == LaneState::Split &&
                (map.segmentClearance(a[i], a[i + 1]) <= R + eps ||
                 map.segmentClearance(b[i], b[i + 1]) <= R + eps ||
                 !progresses(a[i], a[i + 1], out.center[i], out.center[i + 1]) ||
                 !progresses(b[i], b[i + 1], out.center[i], out.center[i + 1]) ||
                 !rightSide(a[i], a[i + 1], out.center[i], out.center[i + 1], 1) ||
                 !rightSide(b[i], b[i + 1], out.center[i], out.center[i + 1], -1)))
                badJoin[i] = badJoin[i + 1] = true;
        const auto ab = chordBoxes(a), bb = chordBoxes(b);
        nearbyChords(ab, bb, required, [&](size_t i, size_t j) {
            if (out.cells[i] == LaneState::Split && out.cells[j] == LaneState::Split &&
                segmentDistanceSquared(a[i], a[i + 1], b[j], b[j + 1]) < required * required)
                badJoin[i] = badJoin[i + 1] = badJoin[j] = badJoin[j + 1] = true;
        });
        for (const auto *lane : {&a, &b}) {
            const auto boxes = chordBoxes(*lane);
            nearbyChords(boxes, boxes, eps, [&](size_t i, size_t j) {
                if (j >= i + 2 && segmentDistanceSquared((*lane)[i], (*lane)[i + 1], (*lane)[j],
                                                         (*lane)[j + 1]) <= eps * eps)
                    badJoin[i] = badJoin[i + 1] = badJoin[j] = badJoin[j + 1] = true;
            });
        }
        if (std::none_of(badJoin.begin(), badJoin.end(), [](bool v) { return v; })) {
            out.forward_lane = std::move(a);
            out.reverse_lane = std::move(b);
            break;
        }
        for (size_t i = 0; i < n; ++i)
            if (badJoin[i]) {
                auto first = std::lower_bound(out.distances.begin(), out.distances.end(),
                                              out.distances[i] - joinRadius);
                for (size_t j = first - out.distances.begin();
                     j < n && out.distances[j] <= out.distances[i] + joinRadius; ++j)
                    blend[j] = std::min(blend[j],
                                        std::abs(out.distances[j] - out.distances[i]) / joinRadius);
            }
    }
    // Certify the actual final curves, including every nonadjacent opposing pair.
    for (size_t i = 0; i < out.cells.size();) {
        size_t j = i + 1;
        while (j < out.cells.size() && out.cells[j] == out.cells[i])
            ++j;
        LaneInterval interval{out.distances[i], out.distances[j], out.cells[i]};
        std::vector<Vec2> a(out.forward_lane.begin() + i, out.forward_lane.begin() + j + 1);
        std::vector<Vec2> b(out.reverse_lane.begin() + i, out.reverse_lane.begin() + j + 1);
        interval.forward_length = pathLength(a);
        interval.reverse_length = pathLength(b);
        interval.forward_clearance = clearance(map, a);
        interval.reverse_clearance = clearance(map, b);
        interval.validated =
            interval.forward_clearance > R + eps && interval.reverse_clearance > R + eps;
        if (interval.state == LaneState::Split) {
            interval.min_separation = pathSeparation(a, b);
            interval.sweeps_disjoint = interval.min_separation > 2 * R + eps;
            interval.forward_right = interval.reverse_right = true;
            interval.forward_offset_min = interval.reverse_offset_min =
                std::numeric_limits<double>::infinity();
            for (size_t k = i; k < j; ++k) {
                const auto t = unit(out.center[k], out.center[k + 1]);
                for (size_t h : {k, k + 1}) {
                    const double r = (out.forward_lane[h][0] - out.center[h][0]) * t[1] -
                                     (out.forward_lane[h][1] - out.center[h][1]) * t[0];
                    const double l = -(out.reverse_lane[h][0] - out.center[h][0]) * t[1] +
                                     (out.reverse_lane[h][1] - out.center[h][1]) * t[0];
                    interval.forward_offset_min = std::min(interval.forward_offset_min, r);
                    interval.forward_offset_max = std::max(interval.forward_offset_max, r);
                    interval.reverse_offset_min = std::min(interval.reverse_offset_min, l);
                    interval.reverse_offset_max = std::max(interval.reverse_offset_max, l);
                }
                interval.forward_right = interval.forward_right &&
                                         rightSide(out.forward_lane[k], out.forward_lane[k + 1],
                                                   out.center[k], out.center[k + 1], 1);
                interval.reverse_right = interval.reverse_right &&
                                         rightSide(out.reverse_lane[k], out.reverse_lane[k + 1],
                                                   out.center[k], out.center[k + 1], -1);
            }
            interval.validated = interval.validated && interval.sweeps_disjoint &&
                                 interval.forward_right && interval.reverse_right;
            if (!interval.validated)
                throw std::runtime_error("constructed SPLIT interval failed final certificate");
        }
        out.intervals.push_back(interval);
        i = j;
    }
    for (size_t i = 0; i < out.intervals.size(); ++i) {
        auto &interval = out.intervals[i];
        interval.entry_transition_present =
            i > 0 && out.intervals[i - 1].state == LaneState::Transition;
        interval.exit_transition_present =
            i + 1 < out.intervals.size() && out.intervals[i + 1].state == LaneState::Transition;
        interval.entry_transition_valid = interval.validated && interval.entry_transition_present &&
                                          out.intervals[i - 1].validated;
        interval.exit_transition_valid = interval.validated && interval.exit_transition_present &&
                                         out.intervals[i + 1].validated;
    }
    for (size_t i = 0; i < reasons.size();) {
        size_t j = i + 1;
        while (j < reasons.size() && reasons[j] == reasons[i])
            ++j;
        if (!reasons[i].empty())
            out.rejections.push_back({out.distances[i], out.distances[j], reasons[i]});
        i = j;
    }
    double separation = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < out.cells.size(); ++i)
        if (out.cells[i] == LaneState::Split) {
            separation = length(out.forward_lane[i], out.reverse_lane[i]);
            break;
        }
    if (std::isfinite(separation)) {
        const auto a = chordBoxes(out.forward_lane), b = chordBoxes(out.reverse_lane);
        nearbyChords(a, b, separation + eps, [&](size_t i, size_t j) {
            if (out.cells[i] == LaneState::Split && out.cells[j] == LaneState::Split)
                separation =
                    std::min(separation, std::sqrt(segmentDistanceSquared(
                                             out.forward_lane[i], out.forward_lane[i + 1],
                                             out.reverse_lane[j], out.reverse_lane[j + 1])));
        });
        out.min_split_separation = separation;
        if (separation <= 2 * R + eps)
            throw std::runtime_error("global opposing SPLIT certificate failed");
    }
    return out;
}

LaneDecomposition buildLaneDecomposition(const CorridorMap &map, const RoutingEdge &edge,
                                         const RobotProfile &robot, const LaneConfig &cfg) {
    return buildLaneDecompositionImpl(map, edge, robot, cfg, false);
}

std::vector<Vec2> buildLaneConnection(const CorridorMap &map, const Vec2 &from, const Vec2 &to,
                                      const Vec2 &via, const RobotProfile &robot,
                                      const LaneConfig &cfg) {
    if (length(from, to) <= eps)
        return {from};
    // Prefer the shortest checked local link. A quadratic via the center can
    // leave two almost-coincident endpoints, double back, and create a loop.
    std::vector<Vec2> direct{from, to};
    if (map.validatePath(direct, robot))
        return direct;
    const double span = length(from, via) + length(via, to);
    int count = std::clamp(static_cast<int>(std::ceil(span / cfg.sample_spacing)), 2, 128);
    std::vector<Vec2> smoothPath;
    for (int i = 0; i <= count; ++i) {
        double t = static_cast<double>(i) / count;
        smoothPath.push_back(mix(mix(from, via, t), mix(via, to, t), t));
    }
    if (map.validatePath(smoothPath, robot) && pathSelfIntersections(smoothPath) == 0 &&
        pathLength(smoothPath) <= 1.5 * span)
        return smoothPath;
    std::vector<Vec2> through{from};
    addPoint(through, via);
    addPoint(through, to);
    if (map.validatePath(through, robot) && pathSelfIntersections(through) == 0)
        return through;
    // Direct is a final checked local connector, never an unchecked offset join.
    return {};
}

bool applyLanePlacement(const CorridorMap &map, const RobotProfile &robot, const PlannerConfig &cfg,
                        CompleteRoute &route) {
    auto fail = [&](const std::string &reason) {
        route.lane_diagnostics.push_back(reason);
        return false;
    };
    route.lane_diagnostics.clear();
    route.lane_rejections.clear();
    route.physical_path = route.path;
    route.physical_metrics = route.metrics;
    route.physical_resource_traversals = route.resource_traversals;
    const auto physical = route.segments;
    std::vector<RouteSegment> placed, corridors;
    struct SectionPart {
        RouteSegment source;
        double first, last, physical_origin;
    };
    std::vector<SectionPart> parts;
    std::vector<Vec2> reference;
    double station = 0, physicalOffset = 0;
    // Stitch the *selected* physical walk before constructing any normals. This
    // changes geometry only: graph IDs, search, and physical weights are untouched.
    for (const auto &source : physical) {
        const double physicalLength = pathLength(source.geometry);
        if (source.geometry.size() < 2 || physicalLength <= eps)
            continue;
        std::vector<Vec2> p = source.geometry;
        if (source.physical_edge_id >= 0) {
            const RoutingEdge *edge = nullptr;
            for (const auto &e : map.data().edges)
                if (e.id == source.physical_edge_id) {
                    edge = &e;
                    break;
                }
            if (!edge || source.reference_distances.empty())
                return fail("physical_edge_reference_missing");
            const double first = source.reference_distances.front();
            const double last = source.reference_distances.back();
            const bool extendedShortcut =
                source.geometry.size() == 2 && physicalLength > std::abs(last - first) + eps;
            // A certified direct shortcut can extend beyond its parent-edge
            // projections. Keep its actual reference instead of turning the
            // remainder into a long unlaned connector. IDs/stations stay canonical.
            if (!extendedShortcut) {
                p = {
                    at(edge->reference_distances, edge->reference_geometry, std::min(first, last))};
                for (size_t i = 0; i < edge->reference_distances.size(); ++i)
                    if (edge->reference_distances[i] > std::min(first, last) + eps &&
                        edge->reference_distances[i] < std::max(first, last) - eps)
                        addPoint(p, edge->reference_geometry[i]);
                addPoint(p, at(edge->reference_distances, edge->reference_geometry,
                               std::max(first, last)));
                if (last < first)
                    std::reverse(p.begin(), p.end());
            }
        }
        if (!reference.empty() && length(reference.back(), p.front()) > eps)
            return fail("discontinuous_physical_reference");
        const double begin = station;
        for (const auto &point : p) {
            if (!reference.empty())
                station += length(reference.back(), point);
            addPoint(reference, point);
        }
        parts.push_back({source, begin, station, physicalOffset});
        physicalOffset += physicalLength;
    }
    try {
        if (reference.size() >= 2) {
            auto sectionConfig = cfg.lanes;
            sectionConfig.taper_endpoints = true;
            auto dec = map.laneDecomposition(reference, robot, sectionConfig);
            // A short, already-certified physical shortcut has no room for a
            // split core and two tapers. Keep its SHARED reference at the actual
            // poses instead of making an out-and-back trip to a remote projection.
            // The parent ID/canonical resource stations remain exactly the same.
            if (parts.size() == 1 && parts.front().source.physical_edge_id >= 0 &&
                parts.front().source.geometry.size() == 2 &&
                pathLength(parts.front().source.geometry) <
                    cfg.lanes.min_split_length +
                        2 * std::max(cfg.lanes.transition_length,
                                     2 * (robot.radius + robot.safety_margin)) &&
                std::all_of(dec->cells.begin(), dec->cells.end(),
                            [](LaneState state) { return state == LaneState::Shared; })) {
                reference = parts.front().source.geometry;
                dec = map.laneDecomposition(reference, robot, sectionConfig);
                parts.front().first = 0;
                parts.front().last = pathLength(reference);
            }
            route.lane_rejections = dec->rejections;
            // The section uses travel-oriented stations. Original canonical edge
            // stations are retained separately for resource cuts and direction IDs.
            for (size_t partIndex = 0; partIndex < parts.size(); ++partIndex) {
                const auto &part = parts[partIndex];
                const auto &source = part.source;
                const double window =
                    std::max(cfg.lanes.sample_spacing, 2 * (robot.radius + robot.safety_margin));
                const bool beforeJunction =
                    partIndex + 1 < parts.size() && source.physical_edge_id >= 0 &&
                    parts[partIndex + 1].source.physical_edge_id >= 0 &&
                    parts[partIndex + 1].source.physical_edge_id != source.physical_edge_id;
                const bool afterJunction =
                    partIndex > 0 && source.physical_edge_id >= 0 &&
                    parts[partIndex - 1].source.physical_edge_id >= 0 &&
                    parts[partIndex - 1].source.physical_edge_id != source.physical_edge_id;
                for (const auto &interval : dec->intervals) {
                    double first = std::max(part.first, interval.first_distance);
                    double last = std::min(part.last, interval.last_distance);
                    if (last - first <= eps)
                        continue;
                    if (interval.state == LaneState::Blocked)
                        return fail("reference_center_footprint_blocked");
                    std::vector<double> cuts{first, last};
                    if (afterJunction && part.first + window > first + eps &&
                        part.first + window < last - eps)
                        cuts.push_back(part.first + window);
                    if (beforeJunction && part.last - window > first + eps &&
                        part.last - window < last - eps)
                        cuts.push_back(part.last - window);
                    std::sort(cuts.begin(), cuts.end());
                    cuts.erase(
                        std::unique(cuts.begin(), cuts.end(),
                                    [](double a, double b) { return std::abs(a - b) <= eps; }),
                        cuts.end());
                    for (size_t cut = 1; cut < cuts.size(); ++cut) {
                        RouteSegment s;
                        s.physical_edge_id = source.physical_edge_id;
                        s.forward = source.forward;
                        s.state = interval.state;
                        s.kind = s.state == LaneState::Transition ? SegmentKind::ModeTransition
                                                                  : SegmentKind::Corridor;
                        const double middle = (cuts[cut - 1] + cuts[cut]) / 2;
                        if ((afterJunction && middle < part.first + window) ||
                            (beforeJunction && middle > part.last - window))
                            s.kind = SegmentKind::JunctionConnector;
                        s.mode =
                            s.state == LaneState::Split
                                ? (s.forward ? CorridorMode::RightOneWay : CorridorMode::LeftOneWay)
                                : CorridorMode::CenterBidirectional;
                        s.narrow_reference = source.narrow_reference;
                        s.decomposition = dec;
                        s.reference_origin = part.physical_origin;
                        s.certificate_distances.push_back(cuts[cut - 1]);
                        for (double d : dec->distances)
                            if (d > cuts[cut - 1] + eps && d < cuts[cut] - eps)
                                s.certificate_distances.push_back(d);
                        s.certificate_distances.push_back(cuts[cut]);
                        const double originalFirst = source.physical_edge_id >= 0
                                                         ? source.reference_distances.front()
                                                         : part.physical_origin;
                        const double originalLast =
                            source.physical_edge_id >= 0
                                ? source.reference_distances.back()
                                : part.physical_origin + pathLength(source.geometry);
                        for (double d : s.certificate_distances) {
                            s.geometry.push_back(at(dec->distances, dec->forward_lane, d));
                            s.reference_geometry.push_back(at(dec->distances, dec->center, d));
                            s.reference_distances.push_back(
                                originalFirst + (originalLast - originalFirst) * (d - part.first) /
                                                    (part.last - part.first));
                            if (s.state == LaneState::Split)
                                s.opposing_geometry.push_back(
                                    at(dec->distances, dec->reverse_lane, d));
                        }
                        if (s.state == LaneState::Split) {
                            s.min_separation = pathSeparation(s.geometry, s.opposing_geometry);
                            if (s.min_separation < 2 * (robot.radius + robot.safety_margin) + eps)
                                return fail("cropped_opposing_sweeps_overlap");
                        }
                        corridors.push_back(std::move(s));
                    }
                }
            }
        }
    } catch (const std::length_error &) {
        return fail("continuous_section_sample_budget_exceeded");
    }
    if (corridors.empty()) {
        if (route.path.size() < 2 || pathLength(route.path) <= eps) {
            route.segments.clear();
            return true;
        }
        return fail("no_usable_lane_section");
    }
    auto connector = [&](const Vec2 &a, const Vec2 &b, const Vec2 &via, SegmentKind kind) {
        RouteSegment s;
        s.kind = kind;
        s.geometry = map.laneConnection(a, b, via, robot, cfg.lanes);
        s.reference_geometry = {a, via, b};
        if (s.geometry.empty())
            return fail("local_junction_connection_failed_after_checked_alternatives");
        s.narrow_reference = map.segmentClearance(via, via) * 2 < map.data().narrow_width;
        placed.push_back(std::move(s));
        return true;
    };
    if (!connector(route.physical_path.front(), corridors.front().geometry.front(),
                   corridors.front().reference_geometry.front(), SegmentKind::StartConnector))
        return false;
    const Vec2 goalVia = corridors.back().reference_geometry.back();
    for (auto &s : corridors) {
        if (length(placed.back().geometry.back(), s.geometry.front()) > eps)
            if (!connector(placed.back().geometry.back(), s.geometry.front(),
                           s.reference_geometry.front(), SegmentKind::JunctionConnector))
                return false;
        placed.push_back(std::move(s));
    }
    if (!connector(placed.back().geometry.back(), route.physical_path.back(), goalVia,
                   SegmentKind::GoalConnector))
        return false;
    route.path.clear();
    route.segments = std::move(placed);
    route.metrics = RouteMetrics{};
    route.metrics.min_clearance = std::numeric_limits<double>::infinity();
    for (const auto &s : route.segments) {
        double d = pathLength(s.geometry);
        if (s.decomposition)
            route.metrics.lane_reference_length += pathLength(s.reference_geometry);
        route.metrics.length += d;
        if (s.state == LaneState::Transition) {
            route.metrics.transition_length += d;
            if (&s == &route.segments.front() || (&s - 1)->state != LaneState::Transition)
                ++route.metrics.mode_transition_count;
        } else if (s.kind != SegmentKind::Corridor)
            route.metrics.connector_length += d;
        if (s.state == LaneState::Split)
            route.metrics.split_length += d;
        else
            route.metrics.shared_exposure += d;
        for (const auto &p : s.geometry)
            addPoint(route.path, p);
    }
    route.path.front() = route.physical_path.front();
    route.path.back() = route.physical_path.back();
    route.metrics.physical_length = route.physical_metrics.length;
    route.metrics.length_inflation_ratio =
        route.physical_metrics.length > eps ? route.metrics.length / route.physical_metrics.length
                                            : 1;
    route.metrics.self_intersection_count = pathSelfIntersections(route.path);
    for (const auto &s : route.segments)
        // Only certificates have corresponding reference stations. A residual
        // connector's {from, via, to} is not indexed like its direct/curved path.
        if (s.decomposition)
            for (size_t i = 0; i < s.geometry.size() && i < s.reference_geometry.size(); ++i)
                route.metrics.max_lateral_deviation =
                    std::max(route.metrics.max_lateral_deviation,
                             length(s.geometry[i], s.reference_geometry[i]));
    if (!map.validatePath(route.path, robot))
        return fail("complete_lane_path_footprint_clearance");
    if (route.metrics.self_intersection_count != 0)
        return fail("complete_lane_path_self_intersection");
    route.metrics.min_clearance = clearance(map, route.path);
    route.metrics.split_fraction =
        route.metrics.length > eps ? route.metrics.split_length / route.metrics.length : 0;
    double offsetIntegral = 0;
    route.metrics.split_offset_min = std::numeric_limits<double>::infinity();
    route.metrics.min_opposing_clearance = std::numeric_limits<double>::infinity();
    route.metrics.min_split_clearance = std::numeric_limits<double>::infinity();
    route.metrics.min_opposing_separation = std::numeric_limits<double>::infinity();
    std::vector<const LaneDecomposition *> certificates;
    LaneState previous = LaneState::Blocked;
    for (const auto &s : route.segments) {
        if (s.geometry.size() < 2 || pathLength(s.geometry) <= eps)
            continue;
        if (s.state == LaneState::Shared) {
            route.metrics.shared_length += pathLength(s.geometry);
            if (previous != LaneState::Shared)
                ++route.metrics.shared_interval_count;
        } else if (s.state == LaneState::Split) {
            if (previous != LaneState::Split)
                ++route.metrics.split_interval_count;
            route.metrics.min_opposing_clearance =
                std::min(route.metrics.min_opposing_clearance, clearance(map, s.opposing_geometry));
            route.metrics.min_split_clearance =
                std::min(route.metrics.min_split_clearance, clearance(map, s.geometry));
            route.metrics.min_opposing_separation = std::min(route.metrics.min_opposing_separation,
                                                             s.decomposition->min_split_separation);
            for (size_t i = 1; i < s.geometry.size(); ++i) {
                const auto t = unit(s.reference_geometry[i - 1], s.reference_geometry[i]);
                const double a = (s.geometry[i - 1][0] - s.reference_geometry[i - 1][0]) * t[1] -
                                 (s.geometry[i - 1][1] - s.reference_geometry[i - 1][1]) * t[0];
                const double b = (s.geometry[i][0] - s.reference_geometry[i][0]) * t[1] -
                                 (s.geometry[i][1] - s.reference_geometry[i][1]) * t[0];
                offsetIntegral += .5 * (a + b) * length(s.geometry[i - 1], s.geometry[i]);
                route.metrics.split_offset_min = std::min({route.metrics.split_offset_min, a, b});
                route.metrics.split_offset_max = std::max({route.metrics.split_offset_max, a, b});
            }
        }
        previous = s.state;
        if (s.decomposition && std::find(certificates.begin(), certificates.end(),
                                         s.decomposition.get()) == certificates.end()) {
            certificates.push_back(s.decomposition.get());
        }
    }
    route.metrics.rejected_split_candidates = static_cast<int>(route.lane_rejections.size());
    if (route.metrics.split_length > eps)
        route.metrics.split_offset_mean = offsetIntegral / route.metrics.split_length;
    else {
        route.metrics.split_offset_min = route.metrics.min_opposing_clearance =
            route.metrics.min_opposing_separation = 0;
        route.metrics.min_split_clearance = 0;
        route.lane_diagnostics.push_back(
            "zero_SPLIT: inspect lane_rejections for local capacity, bend or transition limits");
    }
    for (size_t i = 0; i < route.edge_traversals.size(); ++i) {
        auto mode = CorridorMode::CenterBidirectional;
        for (const auto &s : route.segments)
            if (s.physical_edge_id == route.edge_ids[i] && s.state == LaneState::Split) {
                mode = s.mode;
                break;
            }
        route.modes[i] = mode;
        route.edge_traversals[i].mode = mode;
    }
    return true;
}

bool validateCompleteRoute(const CorridorMap &map, const CompleteRoute &route,
                           const RobotProfile &robot, const PlannerConfig &cfg) {
    if (!validateLaneConfig(cfg.lanes) || !map.validatePath(route.path, robot) ||
        pathSelfIntersections(route.path) != 0)
        return false;
    double total = 0;
    std::vector<Vec2> reconstructed;
    for (const auto &s : route.segments) {
        if (s.geometry.empty() || !map.validatePath(s.geometry, robot) ||
            s.state == LaneState::Blocked)
            return false;
        if (!reconstructed.empty() && length(reconstructed.back(), s.geometry.front()) > eps)
            return false;
        for (const auto &p : s.geometry)
            addPoint(reconstructed, p);
        total += pathLength(s.geometry);
        if (s.decomposition || s.kind == SegmentKind::Corridor ||
            s.kind == SegmentKind::ModeTransition) {
            if (!s.decomposition || s.reference_distances.size() != s.geometry.size() ||
                s.reference_geometry.size() != s.geometry.size())
                return false;
            std::shared_ptr<const LaneDecomposition> current;
            try {
                auto sectionConfig = cfg.lanes;
                sectionConfig.taper_endpoints = true;
                current = !s.certificate_distances.empty()
                              ? map.laneDecomposition(s.decomposition->source_reference, robot,
                                                      sectionConfig)
                          : s.physical_edge_id >= 0
                              ? map.laneDecomposition(s.physical_edge_id, robot, cfg.lanes)
                              : map.laneDecomposition(s.decomposition->source_reference, robot,
                                                      cfg.lanes);
            } catch (const std::exception &) {
                return false;
            }
            const auto &d = *current;
            if (std::abs(d.effective_radius - (robot.radius + robot.safety_margin)) > eps)
                return false;
            const bool section = !s.certificate_distances.empty();
            if (section && s.certificate_distances.size() != s.geometry.size())
                return false;
            const auto &chosen = section || s.forward ? d.forward_lane : d.reverse_lane;
            const auto &stations = section ? s.certificate_distances : s.reference_distances;
            const double origin = section ? 0 : s.reference_origin;
            for (size_t i = 0; i < s.geometry.size(); ++i)
                if (length(at(d.distances, chosen, stations[i] - origin), s.geometry[i]) > eps ||
                    length(at(d.distances, d.center, stations[i] - origin),
                           s.reference_geometry[i]) > eps ||
                    (section && i && stations[i] <= stations[i - 1]))
                    return false;
            if (s.state == LaneState::Split) {
                if (s.opposing_geometry.size() != s.geometry.size())
                    return false;
                const auto &opposite = section || s.forward ? d.reverse_lane : d.forward_lane;
                for (size_t i = 0; i < s.opposing_geometry.size(); ++i)
                    if (length(at(d.distances, opposite, stations[i] - origin),
                               s.opposing_geometry[i]) > eps)
                        return false;
            }
            double lo = std::min(stations.front(), stations.back()) - origin;
            double hi = std::max(stations.front(), stations.back()) - origin;
            for (size_t i = 1; i < s.geometry.size(); ++i)
                if (!progresses(s.geometry[i - 1], s.geometry[i], s.reference_geometry[i - 1],
                                s.reference_geometry[i]))
                    return false;
            for (const auto &interval : d.intervals)
                if (std::min(hi, interval.last_distance) - std::max(lo, interval.first_distance) >
                        eps &&
                    s.state != interval.state)
                    return false;
        }
        if (s.state == LaneState::Split) {
            if (s.mode != (s.forward ? CorridorMode::RightOneWay : CorridorMode::LeftOneWay) ||
                !map.validatePath(s.opposing_geometry, robot) ||
                pathSeparation(s.geometry, s.opposing_geometry) <
                    2 * (robot.radius + robot.safety_margin) + eps)
                return false;
            if (s.geometry.size() != s.reference_geometry.size())
                return false;
            for (size_t i = 0; i + 1 < s.geometry.size(); ++i) {
                auto t = unit(s.reference_geometry[i], s.reference_geometry[i + 1]);
                double side = (s.geometry[i][0] - s.reference_geometry[i][0]) * t[1] -
                              (s.geometry[i][1] - s.reference_geometry[i][1]) * t[0];
                if (side <= eps ||
                    !rightSide(s.geometry[i], s.geometry[i + 1], s.reference_geometry[i],
                               s.reference_geometry[i + 1], 1))
                    return false;
                if (!rightSide(s.opposing_geometry[i], s.opposing_geometry[i + 1],
                               s.reference_geometry[i], s.reference_geometry[i + 1], -1))
                    return false;
            }
        } else if (s.state == LaneState::Shared && s.kind == SegmentKind::Corridor) {
            if (s.geometry.size() != s.reference_geometry.size())
                return false;
            for (size_t i = 0; i < s.geometry.size(); ++i)
                if (length(s.geometry[i], s.reference_geometry[i]) > eps)
                    return false;
        }
    }
    if (!route.segments.empty() && (reconstructed.size() != route.path.size() ||
                                    std::abs(total - route.metrics.length) > 1e-6))
        return false;
    for (size_t i = 0; i < reconstructed.size(); ++i)
        if (length(reconstructed[i], route.path[i]) > eps)
            return false;
    std::vector<double> distances{0};
    for (size_t i = 1; i < route.path.size(); ++i)
        distances.push_back(distances.back() + length(route.path[i - 1], route.path[i]));
    double previous = -1;
    for (const auto &u : route.resource_traversals) {
        if (u.route_start < previous - eps || u.route_start < -eps || u.route_end < u.route_start ||
            u.route_end > route.metrics.length + eps ||
            std::abs(u.traversal_length - (u.route_end - u.route_start)) > 1e-6)
            return false;
        if (length(at(distances, route.path, u.route_start), u.entry) > 1e-6 ||
            length(at(distances, route.path, u.route_end), u.exit) > 1e-6)
            return false;
        previous = u.route_start;
    }
    return true;
}
} // namespace corridor
