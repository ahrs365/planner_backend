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
  void EnsureGlobalSamples(const RobotState& state, const Vec2d& goal,
                           Environment& env);
  void BuildGlobalSamples(const RobotState& state, const Vec2d& goal,
                          Environment& env);
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
  Curve last_trajectory_;
  bool has_last_ = false;
  bool has_cache_ = false;
  Vec2d cached_goal_;
  std::size_t cached_config_sig_ = 0;
  ReferenceLine cached_reference_line_;
  std::vector<std::vector<Vec2d>> cached_all_control_points_;
  std::vector<std::vector<Vec2d>> cached_blocked_control_points_;
  std::vector<std::vector<Vec2d>> cached_valid_control_points_;
  std::vector<Vec2d> cached_layer_centers_;
  std::vector<double> cached_layer_s_;
  std::vector<bool> cached_layer_has_obstacle_;
  std::vector<bool> window_layer_force_full_;
};

}  // namespace ahrs
