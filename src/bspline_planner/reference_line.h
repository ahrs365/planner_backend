#pragma once

#include <vector>

#include "bspline_planner_types.h"

namespace ahrs {

class ReferenceLine {
 public:
  ReferenceLine() = default;
  ReferenceLine(const Vec2d& start, const Vec2d& end, double interval);

  void BuildFrom(const Vec2d& start, const Vec2d& end, double interval);
  const std::vector<Point>& GetPoints() const;
  size_t FindNearestIndex(double x, double y) const;
  size_t FindNearestIndexAtLength(double s) const;

 private:
  std::vector<Point> points_;
};

}  // namespace ahrs
