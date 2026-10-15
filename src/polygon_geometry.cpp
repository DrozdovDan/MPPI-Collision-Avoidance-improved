#include "polygon_geometry.hpp"
#include <algorithm>

namespace {
double cross(const Point2 &a, const Point2 &b, const Point2 &c) {
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]);
}
} // namespace

double pointSegmentDistanceSquared(const Point2 &p, const Point2 &a, const Point2 &b) {
    const double dx = b[0] - a[0], dy = b[1] - a[1], length = dx * dx + dy * dy;
    const double t =
        length == 0 ? 0 : std::clamp(((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / length, 0.0, 1.0);
    const double x = p[0] - a[0] - t * dx, y = p[1] - a[1] - t * dy;
    return x * x + y * y;
}

double segmentDistanceSquared(const Point2 &a, const Point2 &b, const Point2 &c, const Point2 &d) {
    const double c1 = cross(a, b, c), c2 = cross(a, b, d), c3 = cross(c, d, a), c4 = cross(c, d, b);
    if (((c1 < 0 && c2 > 0) || (c1 > 0 && c2 < 0)) && ((c3 < 0 && c4 > 0) || (c3 > 0 && c4 < 0)))
        return 0;
    return std::min({pointSegmentDistanceSquared(a, c, d), pointSegmentDistanceSquared(b, c, d),
                     pointSegmentDistanceSquared(c, a, b), pointSegmentDistanceSquared(d, a, b)});
}

bool pointInsidePolygon(const Point2 &p, const Polygon &polygon) {
    if (polygon.size() < 3)
        return false;
    bool result = false;
    for (size_t i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++) {
        const auto &a = polygon[i];
        const auto &b = polygon[j];
        if ((a[1] > p[1]) != (b[1] > p[1]) &&
            p[0] < (b[0] - a[0]) * (p[1] - a[1]) / (b[1] - a[1]) + a[0])
            result = !result;
    }
    return result;
}

bool polygonCollision(const Point2 &a, const Point2 &b, const Polygon &polygon, double radius) {
    if (polygon.size() < 3)
        return false;
    if (pointInsidePolygon(a, polygon) || pointInsidePolygon(b, polygon))
        return true;
    for (size_t i = 0; i < polygon.size(); ++i)
        if (segmentDistanceSquared(a, b, polygon[i], polygon[(i + 1) % polygon.size()]) <=
            radius * radius + 1e-12)
            return true;
    return false;
}
