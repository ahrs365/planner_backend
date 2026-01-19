#include <algorithm>
#include <cmath>
#include <limits>
#include <iostream>
#include <chrono>

#include "bspline_curve.h"
#include "bspline_lattice_planner.h"

namespace ahrs {

bool BsplineLatticePlanner::Plan(const RobotState& state, const Vec2d& goal,
                                 Environment& env, Curve& trajectory,
                                 const Config& config) {
  const auto t0 = std::chrono::steady_clock::now();
  const auto return_last_or_empty = [&]() -> bool {
    if (has_last_) {
      trajectory = last_trajectory_;
      return true;
    }
    trajectory = Curve();
    return false;
  };
  const double dx_goal = goal.x() - state.pose_.x();
  const double dy_goal = goal.y() - state.pose_.y();
  if (dx_goal * dx_goal + dy_goal * dy_goal <= 1.0 * 1.0) {
    std::cout << "[bspline] Close to goal, stop planning" << std::endl;
    return return_last_or_empty();
  }
  std::cout << "[bspline] Plan start, goal=(" << goal.x() << "," << goal.y()
            << ")" << std::endl;
  config_ = config;
  const auto t1 = std::chrono::steady_clock::now();
  EnsureGlobalSamples(state, goal, env);
  const auto t2 = std::chrono::steady_clock::now();
  const ReferenceLine& reference_line = cached_reference_line_;

  std::vector<std::vector<Vec2d>> control_point_samples;
  std::vector<Vec2d> layer_centers;
  SampleControlPoints(state, reference_line, env, control_point_samples,
                      layer_centers);
  const auto t2b = std::chrono::steady_clock::now();
  if (control_point_samples.size() <= 4) {
    std::cout << "[bspline] Only 4 control point layers left, stop planning"
              << std::endl;
    return return_last_or_empty();
  }
  const auto t3 = std::chrono::steady_clock::now();
  std::cout << "[bspline] Layers in window=" << control_point_samples.size()
            << std::endl;

  std::vector<std::vector<Vec2d>> sequences = GenerateControlPointSequences(
      goal, control_point_samples, layer_centers, env);
  const auto t3b = std::chrono::steady_clock::now();
  const auto t4 = std::chrono::steady_clock::now();
  std::cout << "[bspline] Control point sequences=" << sequences.size()
            << std::endl;
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
  const auto t5 = std::chrono::steady_clock::now();

  if (bspline_samples.empty()) {
    std::cout << "[bspline] All samples filtered (collision)" << std::endl;
    return false;
  }

  debug_info_.bspline_samples_ = bspline_samples;
  debug_info_.chosen_control_points_ = sequences.front();
  std::vector<Point> control_path =
      BuildControlPointPath(sequences.front());
  trajectory = Curve(bspline_samples.front(), control_path);
  last_trajectory_ = trajectory;
  has_last_ = true;
  const auto t6 = std::chrono::steady_clock::now();
  const auto ms_total =
      std::chrono::duration_cast<std::chrono::milliseconds>(t6 - t0).count();
  const auto ms_cache =
      std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
  const auto ms_sample =
      std::chrono::duration_cast<std::chrono::milliseconds>(t2b - t2).count();
  const auto ms_window =
      std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t2b).count();
  const auto ms_sequence =
      std::chrono::duration_cast<std::chrono::milliseconds>(t3b - t3).count();
  const auto ms_search =
      std::chrono::duration_cast<std::chrono::milliseconds>(t4 - t3b).count();
  const auto ms_curve =
      std::chrono::duration_cast<std::chrono::milliseconds>(t5 - t4).count();
  const auto ms_build =
      std::chrono::duration_cast<std::chrono::milliseconds>(t6 - t5).count();
  std::cout << "[bspline] ms total=" << ms_total
            << " cache=" << ms_cache
            << " sample=" << ms_sample
            << " window=" << ms_window
            << " sequence=" << ms_sequence
            << " search=" << ms_search
            << " curve=" << ms_curve
            << " build=" << ms_build << std::endl;
  std::cout << "[bspline] Plan done, curves=" << bspline_samples.size()
            << std::endl;
  return true;
}

void BsplineLatticePlanner::EnsureGlobalSamples(const RobotState& state,
                                                const Vec2d& goal,
                                                Environment& env) {
  auto hash_combine = [](std::size_t& seed, double value) {
    const std::size_t h = std::hash<double>{}(value);
    seed ^= h + 0x9e3779b9 + (seed << 6) + (seed >> 2);
  };
  std::size_t config_sig = 0;
  hash_combine(config_sig, config_.reference_interval_);
  hash_combine(config_sig, config_.sample_length_);
  hash_combine(config_sig, config_.ctp_interval_x_);
  hash_combine(config_sig, config_.sample_half_width_);
  hash_combine(config_sig, config_.ctp_interval_y_);
  hash_combine(config_sig, config_.collision_margin_);
  std::size_t env_sig = 0;
  for (const auto& obs : env.obstacles_) {
    hash_combine(env_sig, obs.center.x());
    hash_combine(env_sig, obs.center.y());
    hash_combine(env_sig, obs.radius);
  }

  const double dx = goal.x() - cached_goal_.x();
  const double dy = goal.y() - cached_goal_.y();
  const bool goal_changed = !has_cache_ || (dx * dx + dy * dy) > 1e-6;
  const bool config_changed = !has_cache_ || config_sig != cached_config_sig_;
  const bool env_changed = !has_cache_ || env_sig != cached_env_sig_;
  if (goal_changed || config_changed || env_changed) {
    BuildGlobalSamples(state, goal, env);
    cached_goal_ = goal;
    cached_config_sig_ = config_sig;
    cached_env_sig_ = env_sig;
    has_cache_ = true;
  }
}

void BsplineLatticePlanner::BuildGlobalSamples(const RobotState& state,
                                               const Vec2d& goal,
                                               Environment& env) {
  cached_reference_line_ =
      ReferenceLine(state.pose_, goal, config_.reference_interval_);
  const std::vector<Point>& reference_points = cached_reference_line_.GetPoints();
  cached_all_control_points_.clear();
  cached_blocked_control_points_.clear();
  cached_valid_control_points_.clear();
  cached_layer_centers_.clear();
  cached_layer_s_.clear();
  cached_layer_has_obstacle_.clear();

  if (reference_points.empty()) {
    return;
  }

  const double robot_radius =
      0.5 * std::max(config_.vehicle_length_, config_.vehicle_width_) +
      config_.collision_margin_;

  double last_s = reference_points.front().s_ - config_.ctp_interval_x_;
  for (size_t i = 0; i < reference_points.size(); ++i) {
    const Point& center_point = reference_points[i];
    if (center_point.s_ - last_s < config_.ctp_interval_x_) {
      continue;
    }

    const double theta = center_point.theta_;
    std::vector<Vec2d> one_layer_sample;
    std::vector<Vec2d> one_layer_all;
    std::vector<Vec2d> one_layer_blocked;
    const bool is_last = (i + 1) >= reference_points.size();
    if (is_last) {
      Vec2d sample(center_point.pose_.x(), center_point.pose_.y());
      one_layer_all.emplace_back(sample);
      if (!env.IsCollision(sample, config_.collision_margin_)) {
        one_layer_sample.emplace_back(sample);
      } else {
        one_layer_blocked.emplace_back(sample);
      }
    } else {
      for (double dy = -config_.sample_half_width_;
           dy < config_.sample_half_width_ + 1e-6;
           dy += config_.ctp_interval_y_) {
        const double sample_x =
            center_point.pose_.x() - dy * std::sin(theta);
        const double sample_y =
            center_point.pose_.y() + dy * std::cos(theta);
        Vec2d sample(sample_x, sample_y);
        one_layer_all.emplace_back(sample);
        if (!env.IsCollision(sample, config_.collision_margin_)) {
          one_layer_sample.emplace_back(sample);
        } else {
          one_layer_blocked.emplace_back(sample);
        }
      }
    }

    cached_all_control_points_.push_back(one_layer_all);
    cached_blocked_control_points_.push_back(one_layer_blocked);
    cached_valid_control_points_.push_back(one_layer_sample);
    cached_layer_centers_.push_back(center_point.pose_);
    cached_layer_s_.push_back(center_point.s_);
    bool has_obstacle = !one_layer_blocked.empty();
    if (!has_obstacle) {
      has_obstacle = env.IsCollision(center_point.pose_, robot_radius);
    }
    cached_layer_has_obstacle_.push_back(has_obstacle);
    last_s = center_point.s_;
  }
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
  window_layer_force_full_.clear();
  window_layer_force_full_.push_back(false);
  window_layer_force_full_.push_back(false);
  window_layer_force_full_.push_back(false);

  // Sliding window: start after the front control point (layer 3).
  const size_t front_index =
      reference_line.FindNearestIndex(front_ctp.x(), front_ctp.y());
  const double front_s = reference_points.at(front_index).s_;
  size_t window_start = cached_layer_s_.size();
  for (size_t i = 0; i < cached_layer_s_.size(); ++i) {
    if (cached_layer_s_[i] - front_s >= config_.ctp_interval_x_) {
      window_start = i;
      break;
    }
  }
  if (window_start >= cached_layer_s_.size()) {
    const size_t last_layer = cached_valid_control_points_.empty()
                                  ? 0
                                  : cached_valid_control_points_.size() - 1;
    window_start = last_layer;
  }
  size_t window_count = 0;
  std::vector<std::vector<Vec2d>> active_control_points;
  double min_x = std::numeric_limits<double>::infinity();
  double min_y = std::numeric_limits<double>::infinity();
  double max_x = -std::numeric_limits<double>::infinity();
  double max_y = -std::numeric_limits<double>::infinity();
  for (size_t i = window_start; i < cached_valid_control_points_.size(); ++i) {
    if (window_count >= config_.max_layers_) {
      break;
    }
    if (cached_valid_control_points_[i].empty()) {
      continue;
    }
    const std::vector<Vec2d>& layer_points = cached_valid_control_points_[i];
    control_point_samples.push_back(layer_points);
    layer_centers.push_back(cached_layer_centers_[i]);
    active_control_points.push_back(layer_points);
    const bool force_full =
        i < cached_layer_has_obstacle_.size() && cached_layer_has_obstacle_[i];
    window_layer_force_full_.push_back(force_full);
    for (const auto& p : layer_points) {
      min_x = std::min(min_x, p.x());
      min_y = std::min(min_y, p.y());
      max_x = std::max(max_x, p.x());
      max_y = std::max(max_y, p.y());
    }
    window_count += 1;
  }

  debug_info_.sample_control_points_.clear();
  debug_info_.sample_control_points_.push_back({back_ctp});
  debug_info_.sample_control_points_.push_back({state.pose_});
  debug_info_.sample_control_points_.push_back({front_ctp});
  debug_info_.sample_control_points_.insert(
      debug_info_.sample_control_points_.end(),
      cached_all_control_points_.begin(),
      cached_all_control_points_.end());
  debug_info_.blocked_control_points_ = cached_blocked_control_points_;
  debug_info_.active_control_points_ = active_control_points;
  debug_info_.active_bounds_.clear();
  if (min_x <= max_x && min_y <= max_y) {
    debug_info_.active_bounds_.emplace_back(min_x, min_y);
    debug_info_.active_bounds_.emplace_back(max_x, max_y);
  }
  debug_info_.reference_line_points_ = reference_points;
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
  const auto t0 = std::chrono::steady_clock::now();
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

  auto compute_cost = [&](const std::vector<Vec2d>& seq) -> double {
    if (seq.empty()) {
      return std::numeric_limits<double>::infinity();
    }
    double cost = 0.0;
    for (size_t i = 0; i < seq.size(); ++i) {
      const Vec2d& p = seq[i];
      const Vec2d& center = layer_centers[i];
      cost += config_.w_center_ * DistanceSquared(p, center);
      if (i >= 3 && i < 3 + config_.spread_layers_) {
        cost -= config_.w_spread_ * DistanceSquared(p, center);
      }
      Vec2d heading_dir(0.0, 0.0);
      if (i + 1 < seq.size()) {
        heading_dir = seq[i + 1] - p;
      } else if (i > 0) {
        heading_dir = p - seq[i - 1];
      }
      const double heading = heading_dir.Length() > 1e-6
                                  ? heading_dir.Angle()
                                  : (goal - p).Angle();
      if (env.IsCollision(p, config_.collision_margin_) ||
          rect_collision(p, heading)) {
        return std::numeric_limits<double>::infinity();
      }
      if (i >= 1) {
        const Vec2d prev_dir = p - seq[i - 1];
        if (prev_dir.Length() > 1e-6 && heading_dir.Length() > 1e-6) {
          const double dtheta = std::fabs(
              std::atan2(prev_dir.x() * heading_dir.y() - prev_dir.y() * heading_dir.x(),
                         prev_dir.x() * heading_dir.x() + prev_dir.y() * heading_dir.y()));
          cost += config_.w_smooth_ * dtheta * dtheta;
        }
      }
    }
    return cost;
  };

  // Step 1: Backbone search with full expansion on layer 4/5.
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
    const auto t_layer0 = std::chrono::steady_clock::now();
    const auto& candidates = control_point_samples[layer];
    if (candidates.empty()) {
      continue;
    }
    std::vector<Vec2d> layer_candidates = candidates;
    if (config_.max_candidate_paths_ > 0 &&
        layer_candidates.size() > config_.max_candidate_paths_) {
      const Vec2d& center = layer_centers[layer];
      std::nth_element(
          layer_candidates.begin(),
          layer_candidates.begin() + config_.max_candidate_paths_,
          layer_candidates.end(),
          [&](const Vec2d& a, const Vec2d& b) {
            return DistanceSquared(a, center) < DistanceSquared(b, center);
          });
      layer_candidates.resize(config_.max_candidate_paths_);
    }
    std::vector<PathCandidate> next_beam;
    next_beam.reserve(beam.size() * layer_candidates.size());
    const size_t hard_cap = std::max<size_t>(config_.max_sequences_, 1);
    const size_t soft_cap = hard_cap * 4;
    for (const auto& prev : beam) {
      const bool force_full = (layer == 3 || layer == 4);
      const bool has_obstacle =
          layer < window_layer_force_full_.size() && window_layer_force_full_[layer];
      if (!force_full && !has_obstacle) {
        const Vec2d& center = layer_centers[layer];
        PathCandidate c = prev;
        c.points.push_back(center);
        c.cost = compute_cost(c.points);
        if (!std::isfinite(c.cost)) {
          continue;
        }
        next_beam.push_back(std::move(c));
      } else {
        for (const auto& p : layer_candidates) {
          PathCandidate c = prev;
          c.points.push_back(p);
          c.cost = compute_cost(c.points);
          if (!std::isfinite(c.cost)) {
            continue;
          }
          next_beam.push_back(std::move(c));
          if (next_beam.size() >= soft_cap) {
            std::nth_element(next_beam.begin(),
                             next_beam.begin() + hard_cap,
                             next_beam.end(),
                             [](const PathCandidate& a, const PathCandidate& b) {
                               return a.cost < b.cost;
                             });
            next_beam.resize(hard_cap);
          }
        }
      }
    }
    std::sort(next_beam.begin(), next_beam.end(),
              [](const PathCandidate& a, const PathCandidate& b) {
                return a.cost < b.cost;
              });
    if (next_beam.size() > config_.max_sequences_) {
      std::nth_element(next_beam.begin(),
                       next_beam.begin() + config_.max_sequences_,
                       next_beam.end(),
                       [](const PathCandidate& a, const PathCandidate& b) {
                         return a.cost < b.cost;
                       });
      next_beam.resize(config_.max_sequences_);
      std::sort(next_beam.begin(), next_beam.end(),
                [](const PathCandidate& a, const PathCandidate& b) {
                  return a.cost < b.cost;
                });
    }
    beam = std::move(next_beam);
    const auto t_layer1 = std::chrono::steady_clock::now();
    const auto ms_layer =
        std::chrono::duration_cast<std::chrono::milliseconds>(t_layer1 - t_layer0).count();
    if (ms_layer > 0) {
      std::cout << "[bspline] layer " << layer
                << " candidates=" << layer_candidates.size()
                << " beam=" << beam.size()
                << " ms=" << ms_layer << std::endl;
    }
  }

  std::sort(beam.begin(), beam.end(),
            [](const PathCandidate& a, const PathCandidate& b) {
              return a.cost < b.cost;
            });
  if (beam.size() > config_.max_sequences_) {
    beam.resize(config_.max_sequences_);
  }

  std::vector<std::vector<Vec2d>> sequences;
  sequences.reserve(beam.size());
  for (auto& c : beam) {
    c.points.push_back(goal);
    sequences.push_back(std::move(c.points));
  }
  const auto t1 = std::chrono::steady_clock::now();
  const auto ms_total =
      std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();
  if (ms_total > 0) {
    std::cout << "[bspline] sequences total ms=" << ms_total << std::endl;
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
