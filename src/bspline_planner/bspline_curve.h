#pragma once

#include <vector>

#include "bspline_planner_types.h"

namespace ahrs {

class BsplineCurve {
 public:
  explicit BsplineCurve(const double& interval);
  ~BsplineCurve();

  std::vector<Point> GenerateCurve();
  void SetControlPoints(const std::vector<Vec2d>& control_points);

 private:
  Point GetPos(const size_t& k, const double& ti);

  double interval_ = 0.01;
  size_t ctp_size_ = 0;
  std::vector<Vec2d> ctp_;
};

}  // namespace ahrs
