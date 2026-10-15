#pragma once
#include <array>
#include <vector>

using Point2 = std::array<double, 2>;
using Polygon = std::vector<Point2>;
// Standalone circular-footprint geometry, shared with the controller.
double pointSegmentDistanceSquared(const Point2 &p, const Point2 &a, const Point2 &b);
double segmentDistanceSquared(const Point2 &a, const Point2 &b, const Point2 &c, const Point2 &d);
bool pointInsidePolygon(const Point2 &p, const Polygon &polygon);
bool polygonCollision(const Point2 &a, const Point2 &b, const Polygon &polygon, double radius);
