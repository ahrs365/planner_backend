#include "reference_line.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace ahrs {

ReferenceLine::ReferenceLine(const Vec2d& start, const Vec2d& end,
                             double interval) {
  BuildFrom(start, end, interval);
}

void ReferenceLine::BuildFrom(const Vec2d& start, const Vec2d& end,
                              double interval) {
  points_.clear();
  const double dx = end.x() - start.x();
  const double dy = end.y() - start.y();
  const double length = std::sqrt(dx * dx + dy * dy);
  const double theta = std::atan2(dy, dx);

  if (length < 1e-6 || interval <= 1e-6) {
    Point p(start.x(), start.y());
    p.s_ = 0.0;
    p.theta_ = theta;
    points_.push_back(p);
    return;
  }

  const size_t steps = static_cast<size_t>(std::ceil(length / interval));
  points_.reserve(steps + 1);
  for (size_t i = 0; i <= steps; ++i) {
    const double s = std::min(static_cast<double>(i) * interval, length);
    const double ratio = s / length;
    const double x = start.x() + ratio * dx;
    const double y = start.y() + ratio * dy;
    Point p(x, y);
    p.s_ = s;
    p.theta_ = theta;
    points_.push_back(p);
  }
}

const std::vector<Point>& ReferenceLine::GetPoints() const { return points_; }

size_t ReferenceLine::FindNearestIndex(double x, double y) const {
  if (points_.empty()) {
    return 0;
  }
  const Vec2d target(x, y);
  double best_dist = std::numeric_limits<double>::infinity();
  size_t best_index = 0;
  for (size_t i = 0; i < points_.size(); ++i) {
    const double dist = DistanceSquared(points_[i].pose_, target);
    if (dist < best_dist) {
      best_dist = dist;
      best_index = i;
    }
  }
  return best_index;
}

size_t ReferenceLine::FindNearestIndexAtLength(double s) const {
  if (points_.empty()) {
    return 0;
  }
  for (size_t i = 0; i < points_.size(); ++i) {
    if (points_[i].s_ >= s) {
      return i;
    }
  }
  return points_.size() - 1;
}

}  // namespace ahrs
