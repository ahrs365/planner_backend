#include <algorithm>
#include <cmath>
#include <limits>

#include "bspline_curve.h"
#include "bspline_lattice_planner.h"

namespace ahrs {

bool BsplineLatticePlanner::Plan(const RobotState& state, const Vec2d& goal,
                                 Environment& env, Curve& trajectory,
                                 const Config& config) {
  config_ = config;
  ReferenceLine reference_line(state.pose_, goal, config_.reference_interval_);

  std::vector<std::vector<Vec2d>> control_point_samples;
  std::vector<Vec2d> layer_centers;
  SampleControlPoints(state, reference_line, env, control_point_samples,
                      layer_centers);

  std::vector<std::vector<Vec2d>> sequences = GenerateControlPointSequences(
      goal, control_point_samples, layer_centers, env);
  if (sequences.empty()) {
    return false;
  }

  BsplineCurve bspline(config_.bspline_interval_);
  std::vector<std::vector<Point>> bspline_samples;
  bspline_samples.reserve(sequences.size());

  const auto rect_collision = [&](const Vec2d& pos, double heading) -> bool {
    const double half_l = 0.5 * config_.vehicle_length_ + config_.collision_margin_;
    const double half_w = 0.5 * config_.vehicle_width_ + config_.collision_margin_;
    const double c = std::cos(heading);
    const double s = std::sin(heading);
    for (const auto& obs : env.obstacles_) {
      const double dx = obs.center.x() - pos.x();
      const double dy = obs.center.y() - pos.y();
      const double local_x = c * dx + s * dy;
      const double local_y = -s * dx + c * dy;
      const double clamp_x = std::max(-half_l, std::min(local_x, half_l));
      const double clamp_y = std::max(-half_w, std::min(local_y, half_w));
      const double diff_x = local_x - clamp_x;
      const double diff_y = local_y - clamp_y;
      if (diff_x * diff_x + diff_y * diff_y <= obs.radius * obs.radius) {
        return true;
      }
    }
    return false;
  };

  for (auto& seq : sequences) {
    if (seq.size() < 4) {
      while (seq.size() < 4) {
        seq.push_back(goal);
      }
    }
    bspline.SetControlPoints(seq);
    std::vector<Point> points = bspline.GenerateCurve();
    if (points.empty()) {
      continue;
    }

    bool collision = false;
    for (const auto& p : points) {
      if (env.IsCollision(p.pose_, config_.collision_margin_) ||
          rect_collision(p.pose_, p.theta_)) {
        collision = true;
        break;
      }
    }
    if (!collision) {
      bspline_samples.push_back(points);
    }
  }

  if (bspline_samples.empty()) {
    return false;
  }

  debug_info_.bspline_samples_ = bspline_samples;
  debug_info_.chosen_control_points_ = sequences.front();
  std::vector<Point> control_path =
      BuildControlPointPath(sequences.front());
  trajectory = Curve(bspline_samples.front(), control_path);
  return true;
}

void BsplineLatticePlanner::SampleControlPoints(
    const RobotState& state, const ReferenceLine& reference_line,
    Environment& env, std::vector<std::vector<Vec2d>>& control_point_samples,
    std::vector<Vec2d>& layer_centers) {
  const std::vector<Point>& reference_points = reference_line.GetPoints();
  if (reference_points.empty()) {
    return;
  }

  // Layer 1/2/3: back point, current position, front point (single point each).
  const Vec2d heading(std::cos(state.theta_), std::sin(state.theta_));
  const Vec2d back_ctp = state.pose_ - heading * config_.zero_layer_interval_;
  const Vec2d front_ctp = state.pose_ + heading * config_.zero_layer_interval_;

  control_point_samples.push_back({back_ctp});
  layer_centers.push_back(back_ctp);
  control_point_samples.push_back({state.pose_});
  layer_centers.push_back(state.pose_);
  control_point_samples.push_back({front_ctp});
  layer_centers.push_back(front_ctp);

  const size_t start_index =
      reference_line.FindNearestIndex(state.pose_.x(), state.pose_.y());
  const double start_s = reference_points.at(start_index).s_;
  const double end_s = start_s + config_.sample_length_;
  const size_t end_index = reference_line.FindNearestIndexAtLength(end_s);

  double last_s = start_s + config_.zero_layer_interval_;
  for (size_t i = start_index; i <= end_index; ++i) {
    const Point& center_point = reference_points.at(i);
    if (center_point.s_ - last_s < config_.ctp_interval_x_) {
      continue;
    }

    const double theta = center_point.theta_;
    std::vector<Vec2d> one_layer_sample;
    for (double dy = -config_.sample_half_width_;
         dy < config_.sample_half_width_ + 1e-6;
         dy += config_.ctp_interval_y_) {
      const double sample_x =
          center_point.pose_.x() - dy * std::sin(theta);
      const double sample_y =
          center_point.pose_.y() + dy * std::cos(theta);
      Vec2d sample(sample_x, sample_y);
      if (!env.IsCollision(sample, config_.collision_margin_)) {
        one_layer_sample.emplace_back(sample);
      }
    }

    if (!one_layer_sample.empty()) {
      control_point_samples.push_back(one_layer_sample);
      layer_centers.push_back(center_point.pose_);
      last_s = center_point.s_;
    }
  }

  debug_info_.sample_control_points_ = control_point_samples;
  debug_info_.reference_line_points_.assign(
      reference_points.begin() + start_index,
      reference_points.begin() + end_index + 1);
}

std::vector<Vec2d> BsplineLatticePlanner::ChooseControlPoints(
    const RobotState& state, const Vec2d& goal,
    const std::vector<std::vector<Vec2d>>& control_point_samples,
    const std::vector<Vec2d>& layer_centers, const Environment& env) {
  std::vector<Vec2d> chosen;
  chosen.reserve(control_point_samples.size() + 1);

  for (size_t i = 0; i < control_point_samples.size(); ++i) {
    const auto& layer = control_point_samples[i];
    if (layer.empty()) {
      continue;
    }
    const Vec2d& center = layer_centers[i];

    auto best_it = std::min_element(
        layer.begin(), layer.end(),
        [&](const Vec2d& a, const Vec2d& b) {
          return DistanceSquared(a, center) < DistanceSquared(b, center);
        });

    Vec2d selected = *best_it;
    if (env.IsCollision(selected)) {
      for (const auto& candidate : layer) {
        if (!env.IsCollision(candidate)) {
          selected = candidate;
          break;
        }
      }
    }
    chosen.push_back(selected);
  }

  chosen.push_back(goal);
  return chosen;
}

std::vector<std::vector<Vec2d>>
BsplineLatticePlanner::GenerateControlPointSequences(
    const Vec2d& goal,
    const std::vector<std::vector<Vec2d>>& control_point_samples,
    const std::vector<Vec2d>& layer_centers, const Environment& env) {
  const auto rect_collision = [&](const Vec2d& pos, double heading) -> bool {
    const double half_l = 0.5 * config_.vehicle_length_ + config_.collision_margin_;
    const double half_w = 0.5 * config_.vehicle_width_ + config_.collision_margin_;
    const double c = std::cos(heading);
    const double s = std::sin(heading);
    for (const auto& obs : env.obstacles_) {
      const double dx = obs.center.x() - pos.x();
      const double dy = obs.center.y() - pos.y();
      const double local_x = c * dx + s * dy;
      const double local_y = -s * dx + c * dy;
      const double clamp_x = std::max(-half_l, std::min(local_x, half_l));
      const double clamp_y = std::max(-half_w, std::min(local_y, half_w));
      const double diff_x = local_x - clamp_x;
      const double diff_y = local_y - clamp_y;
      if (diff_x * diff_x + diff_y * diff_y <= obs.radius * obs.radius) {
        return true;
      }
    }
    return false;
  };

  struct PathCandidate {
    std::vector<Vec2d> points;
    double cost = 0.0;
  };

  if (control_point_samples.empty()) {
    return {};
  }

  std::vector<PathCandidate> beam;
  const auto& first_layer = control_point_samples.front();
  beam.reserve(first_layer.size());
  for (const auto& p : first_layer) {
    PathCandidate c;
    c.points.push_back(p);
    c.cost = 0.0;
    beam.push_back(std::move(c));
  }

  for (size_t layer = 1; layer < control_point_samples.size(); ++layer) {
    const auto& candidates = control_point_samples[layer];
    if (candidates.empty()) {
      continue;
    }

    std::vector<PathCandidate> next_beam;
    next_beam.reserve(beam.size() * candidates.size());

    for (const auto& prev : beam) {
      const Vec2d& last = prev.points.back();
      Vec2d last_dir(0.0, 0.0);
      bool has_last_dir = false;
      if (prev.points.size() >= 2) {
        last_dir = last - prev.points[prev.points.size() - 2];
        has_last_dir = last_dir.Length() > 1e-6;
      }

      for (const auto& p : candidates) {
        PathCandidate c = prev;
        c.points.push_back(p);

        double cost = prev.cost;
        const Vec2d& center = layer_centers[layer];
        cost += config_.w_center_ * DistanceSquared(p, center);

        const Vec2d new_dir = p - last;
        const double new_len = new_dir.Length();
        double heading = 0.0;
        if (new_len > 1e-6) {
          heading = new_dir.Angle();
        } else if (has_last_dir) {
          heading = last_dir.Angle();
        } else {
          heading = (goal - p).Angle();
        }

        if (env.IsCollision(p, config_.collision_margin_) ||
            rect_collision(p, heading)) {
          continue;
        }

        if (has_last_dir && new_len > 1e-6) {
          const double dtheta = std::fabs(
              std::atan2(last_dir.x() * new_dir.y() - last_dir.y() * new_dir.x(),
                         last_dir.x() * new_dir.x() + last_dir.y() * new_dir.y()));
          cost += config_.w_smooth_ * dtheta * dtheta;
        }

        c.cost = cost;
        next_beam.push_back(std::move(c));
      }
    }

    std::sort(next_beam.begin(), next_beam.end(),
              [](const PathCandidate& a, const PathCandidate& b) {
                return a.cost < b.cost;
              });
    if (next_beam.size() > config_.beam_width_) {
      next_beam.resize(config_.beam_width_);
    }
    beam = std::move(next_beam);
  }

  std::sort(beam.begin(), beam.end(),
            [](const PathCandidate& a, const PathCandidate& b) {
              return a.cost < b.cost;
            });
  if (beam.size() > config_.max_candidate_paths_) {
    beam.resize(config_.max_candidate_paths_);
  }

  std::vector<std::vector<Vec2d>> sequences;
  sequences.reserve(beam.size());
  for (auto& c : beam) {
    c.points.push_back(goal);
    sequences.push_back(std::move(c.points));
  }
  return sequences;
}

std::vector<Point> BsplineLatticePlanner::BuildControlPointPath(
    const std::vector<Vec2d>& control_points) const {
  std::vector<Point> path;
  if (control_points.empty()) {
    return path;
  }

  path.reserve(control_points.size());
  for (size_t i = 0; i + 1 < control_points.size(); ++i) {
    const Vec2d& cur = control_points[i];
    const Vec2d& next = control_points[i + 1];
    const Vec2d diff = next - cur;
    Point point(cur.x(), cur.y());
    point.theta_ = diff.Angle();
    path.push_back(point);
  }

  Point last(control_points.back().x(), control_points.back().y());
  if (!path.empty()) {
    last.theta_ = path.back().theta_;
  }
  path.push_back(last);
  return path;
}

}  // namespace ahrs
