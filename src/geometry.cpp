#include "geometry.hpp"
#include <algorithm>

namespace {
double cross(const Point2 &a, const Point2 &b, const Point2 &c) {
    return (b[0]-a[0])*(c[1]-a[1]) - (b[1]-a[1])*(c[0]-a[0]);
}
double distanceSquared(const Point2 &p, const Point2 &a, const Point2 &b) {
    double dx = b[0]-a[0], dy = b[1]-a[1];
    double length = dx*dx + dy*dy;
    double t = length == 0 ? 0 : std::clamp(((p[0]-a[0])*dx + (p[1]-a[1])*dy)/length, 0.0, 1.0);
    double x = p[0]-a[0]-t*dx, y = p[1]-a[1]-t*dy;
    return x*x + y*y;
}
bool inside(const Point2 &p, const Polygon &polygon) {
    bool result = false;
    for (size_t i = 0, j = polygon.size()-1; i < polygon.size(); j = i++) {
        const auto &a = polygon[i];
        const auto &b = polygon[j];
        if ((a[1] > p[1]) != (b[1] > p[1]) &&
            p[0] < (b[0]-a[0])*(p[1]-a[1])/(b[1]-a[1])+a[0]) result = !result;
    }
    return result;
}
}

bool polygonCollision(const Point2 &a, const Point2 &b, const Polygon &polygon, double radius) {
    if (polygon.size() < 3) return false;
    if (inside(a, polygon) || inside(b, polygon)) return true;
    for (size_t i = 0; i < polygon.size(); ++i) {
        const auto &c = polygon[i];
        const auto &d = polygon[(i+1)%polygon.size()];
        double c1 = cross(a,b,c), c2 = cross(a,b,d), c3 = cross(c,d,a), c4 = cross(c,d,b);
        if (((c1 < 0 && c2 > 0) || (c1 > 0 && c2 < 0)) &&
            ((c3 < 0 && c4 > 0) || (c3 > 0 && c4 < 0))) return true;
        if (std::min({distanceSquared(a,c,d), distanceSquared(b,c,d),
                      distanceSquared(c,a,b), distanceSquared(d,a,b)}) <= radius*radius + 1e-12) return true;
    }
    return false;
}

double pointToLineSegmentDistance(const xt::xtensor<double, 1> &line_p1,
								  const xt::xtensor<double, 1> &line_p2,
								  const xt::xtensor<double, 1> &point) {
	if (xt::allclose(line_p1, point) or xt::allclose(line_p2, point))
		return 0.0;

	auto v = line_p2 - line_p1;
	auto w = point - line_p1;
	double c1, c2;

	c1 = xt::linalg::dot(w, v)[0];

	if (c1 <= 0.0)
		return xt::linalg::norm(w, 2);
	c2 = xt::linalg::dot(v, v)[0];
	if (c2 <= c1)
		return xt::linalg::norm(point - line_p2, 2);

	double b = c1 / c2;
	auto pb = line_p1 + v * b;
	return xt::linalg::norm(point - pb, 2);
}

double determinant2D(const xt::xtensor<double, 1> &ab, const xt::xtensor<double, 1> &cd) {
	return ab[0] * cd[1] - ab[1] * cd[0];
}


std::tuple<bool, xt::xtensor<double, 1>> lineIntersection(const xt::xtensor<double, 1> &line1, const xt::xtensor<double, 1> &line2) {
	double a1 = line1[0], b1 = line1[1], c1 = line1[2];

	double a2 = line2[0], b2 = line2[1], c2 = line2[2];
	double determinant = a1*b2 - a2*b1;

	if (determinant == 0)
		return {false, xt::zeros<double>({0})};
	else {
		double x = (b2 * -c1 - b1 * -c2) / determinant;
		double y = (a1 * -c2 - a2 * -c1) / determinant;
		return {true, xt::xtensor<double, 1>({x, y})};
	}
}
