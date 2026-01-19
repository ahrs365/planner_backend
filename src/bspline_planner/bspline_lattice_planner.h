#pragma once

#include <vector>

#include "bspline_planner_types.h"
#include "reference_line.h"

namespace ahrs {

class BsplineLatticePlanner {
 public:
  bool Plan(const RobotState& state, const Vec2d& goal, Environment& env,
            Curve& trajectory, const Config& config);

  const DebugInfo& GetDebugInfo() const { return debug_info_; }

 private:
  void SampleControlPoints(
      const RobotState& state, const ReferenceLine& reference_line,
      Environment& env, std::vector<std::vector<Vec2d>>& control_point_samples,
      std::vector<Vec2d>& layer_centers);

  std::vector<Vec2d> ChooseControlPoints(
      const RobotState& state, const Vec2d& goal,
      const std::vector<std::vector<Vec2d>>& control_point_samples,
      const std::vector<Vec2d>& layer_centers, const Environment& env);

  std::vector<std::vector<Vec2d>> GenerateControlPointSequences(
      const Vec2d& goal,
      const std::vector<std::vector<Vec2d>>& control_point_samples,
      const std::vector<Vec2d>& layer_centers, const Environment& env);

  std::vector<Point> BuildControlPointPath(
      const std::vector<Vec2d>& control_points) const;

  Config config_;
  DebugInfo debug_info_;
};

}  // namespace ahrs
