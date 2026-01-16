#include "dwa_planner.h"

#include <algorithm>
#include <cmath>
#include <limits>

std::vector<double> DWAPlanner::plan(double x, double y, double theta,
                                     double v, double w,
                                     double gx, double gy,
                                     const std::vector<double> &obs_flat) {
  const Window win = calcDynamicWindow(v, w);

  // Calculate velocity and yawrate resolution based on sample counts
  const double v_reso = std::max((win.v_max - win.v_min) / std::max(velocity_samples_ - 1, 1), 1e-6);
  const double w_reso = std::max((win.w_max - win.w_min) / std::max(yawrate_samples_ - 1, 1), 1e-6);

  std::vector<Cost> costs;
  std::vector<std::vector<double>> trajectories;
  std::vector<std::pair<double, double>> velocities;  // (v, w) pairs

  // Sample trajectories
  for (int i = 0; i < velocity_samples_; i++) {
    const double vt = win.v_min + v_reso * i;
    
    for (int j = 0; j < yawrate_samples_; j++) {
      double wt = win.w_min + w_reso * j;
      
      std::vector<double> traj;
      simulateTrajectory(x, y, theta, vt, wt, traj);

      Cost cost;
      cost.to_goal_cost = calcToGoalCost(traj, gx, gy);
      Cost obs_result = calcObstacleCost(traj, obs_flat);
      cost.obs_cost = obs_result.obs_cost;
      cost.is_collision = obs_result.is_collision;
      cost.speed_cost = win.v_max - vt;  // Use dynamic window max

      costs.push_back(cost);
      trajectories.push_back(traj);
      velocities.push_back({vt, wt});
    }

    // Always sample w=0 if it's in the window
    if (win.w_min < 0.0 && 0.0 < win.w_max) {
      const double wt = 0.0;
      std::vector<double> traj;
      simulateTrajectory(x, y, theta, vt, wt, traj);

      Cost cost;
      cost.to_goal_cost = calcToGoalCost(traj, gx, gy);
      Cost obs_result = calcObstacleCost(traj, obs_flat);
      cost.obs_cost = obs_result.obs_cost;
      cost.is_collision = obs_result.is_collision;
      cost.speed_cost = win.v_max - vt;

      costs.push_back(cost);
      trajectories.push_back(traj);
      velocities.push_back({vt, wt});
    }
  }

  // Normalize costs before applying weights
  normalizeCosts(costs);

  // Find best trajectory
  double min_total_cost = std::numeric_limits<double>::infinity();
  int best_idx = -1;
  
  for (size_t i = 0; i < costs.size(); i++) {
    if (!costs[i].is_collision) {
      // Apply weights after normalization
      costs[i].total_cost = to_goal_cost_gain_ * costs[i].to_goal_cost +
                            obstacle_cost_gain_ * costs[i].obs_cost +
                            speed_cost_gain_ * costs[i].speed_cost;
      
      if (costs[i].total_cost < min_total_cost) {
        min_total_cost = costs[i].total_cost;
        best_idx = static_cast<int>(i);
      }
    }
  }

  double best_v = 0.0;
  double best_w = 0.0;
  best_traj_.clear();

  if (best_idx >= 0) {
    best_v = velocities[best_idx].first;
    best_w = velocities[best_idx].second;
    best_traj_ = trajectories[best_idx];
    best_cost_ = min_total_cost;
  } else {
    // No valid trajectory, stop
    best_cost_ = std::numeric_limits<double>::infinity();
  }

  return {best_v, best_w};
}

void DWAPlanner::normalizeCosts(std::vector<Cost> &costs) const {
  if (costs.empty()) return;

  double min_obs = std::numeric_limits<double>::infinity();
  double max_obs = 0.0;
  double min_goal = std::numeric_limits<double>::infinity();
  double max_goal = 0.0;
  double min_speed = std::numeric_limits<double>::infinity();
  double max_speed = 0.0;

  // Find min/max for non-collision trajectories
  for (const auto &cost : costs) {
    if (!cost.is_collision) {
      min_obs = std::min(min_obs, cost.obs_cost);
      max_obs = std::max(max_obs, cost.obs_cost);
      min_goal = std::min(min_goal, cost.to_goal_cost);
      max_goal = std::max(max_goal, cost.to_goal_cost);
      min_speed = std::min(min_speed, cost.speed_cost);
      max_speed = std::max(max_speed, cost.speed_cost);
    }
  }

  // Normalize to [0, 1]
  const double eps = 1e-9;
  for (auto &cost : costs) {
    if (!cost.is_collision) {
      cost.obs_cost = (cost.obs_cost - min_obs) / (max_obs - min_obs + eps);
      cost.to_goal_cost = (cost.to_goal_cost - min_goal) / (max_goal - min_goal + eps);
      cost.speed_cost = (cost.speed_cost - min_speed) / (max_speed - min_speed + eps);
    }
  }
}

std::vector<double> DWAPlanner::sampleTrajectories(double x, double y, double theta,
                                                   double v, double w,
                                                   const std::vector<double> &obs_flat) {
  (void)obs_flat;
  const Window win = calcDynamicWindow(v, w);
  const int steps = predictSteps();

  const double v_reso = std::max((win.v_max - win.v_min) / std::max(velocity_samples_ - 1, 1), 1e-6);
  const double w_reso = std::max((win.w_max - win.w_min) / std::max(yawrate_samples_ - 1, 1), 1e-6);

  std::vector<double> out;
  int traj_count = 0;

  for (int i = 0; i < velocity_samples_; i++) {
    const double vt = win.v_min + v_reso * i;
    
    for (int j = 0; j < yawrate_samples_; j++) {
      const double wt = win.w_min + w_reso * j;
      std::vector<double> traj;
      simulateTrajectory(x, y, theta, vt, wt, traj);
      if (traj.size() / 2 != static_cast<size_t>(steps)) {
        continue;
      }
      out.insert(out.end(), traj.begin(), traj.end());
      traj_count++;
    }
    
    if (win.w_min < 0.0 && 0.0 < win.w_max) {
      const double wt = 0.0;
      std::vector<double> traj;
      simulateTrajectory(x, y, theta, vt, wt, traj);
      if (traj.size() / 2 != static_cast<size_t>(steps)) {
        continue;
      }
      out.insert(out.end(), traj.begin(), traj.end());
      traj_count++;
    }
  }

  std::vector<double> header;
  header.reserve(2 + out.size());
  header.push_back(static_cast<double>(traj_count));
  header.push_back(static_cast<double>(steps));
  header.insert(header.end(), out.begin(), out.end());
  return header;
}

std::vector<double> DWAPlanner::bestTrajectory() const { return best_traj_; }

double DWAPlanner::bestCost() const { return best_cost_; }

void DWAPlanner::setLimits(double max_speed, double min_speed,
                           double max_yawrate, double max_accel,
                           double max_dyawrate) {
  max_speed_ = max_speed;
  min_speed_ = min_speed;
  max_yawrate_ = max_yawrate;
  max_accel_ = max_accel;
  max_dyawrate_ = max_dyawrate;
}

void DWAPlanner::setResolution(double v_reso, double yawrate_reso) {
  // Convert resolution to sample counts (for backward compatibility)
  velocity_samples_ = std::max(static_cast<int>(max_speed_ / v_reso), 5);
  yawrate_samples_ = std::max(static_cast<int>(2 * max_yawrate_ / yawrate_reso), 10);
}

void DWAPlanner::setSamples(int velocity_samples, int yawrate_samples) {
  velocity_samples_ = std::max(velocity_samples, 3);
  yawrate_samples_ = std::max(yawrate_samples, 5);
}

void DWAPlanner::setPredict(double dt, double predict_time) {
  dt_ = dt;
  predict_time_ = predict_time;
}

void DWAPlanner::setWeights(double to_goal, double obstacle, double speed) {
  to_goal_cost_gain_ = to_goal;
  obstacle_cost_gain_ = obstacle;
  speed_cost_gain_ = speed;
}

void DWAPlanner::setRobotRadius(double r) { robot_radius_ = r; }

void DWAPlanner::setObsRange(double range) { obs_range_ = range; }

DWAPlanner::Window DWAPlanner::calcDynamicWindow(double v, double w) const {
  const double v_min = std::max(min_speed_, v - max_accel_ * dt_);
  const double v_max = std::min(max_speed_, v + max_accel_ * dt_);
  const double w_min = std::max(-max_yawrate_, w - max_dyawrate_ * dt_);
  const double w_max = std::min(max_yawrate_, w + max_dyawrate_ * dt_);
  return {v_min, v_max, w_min, w_max};
}

int DWAPlanner::predictSteps() const {
  return static_cast<int>(predict_time_ / dt_);
}

void DWAPlanner::simulateTrajectory(double x, double y, double theta,
                                    double v, double w,
                                    std::vector<double> &traj) const {
  const int steps = predictSteps();
  traj.reserve(steps * 2);

  double px = x;
  double py = y;
  double ptheta = theta;

  for (int i = 0; i < steps; ++i) {
    px += v * std::cos(ptheta) * dt_;
    py += v * std::sin(ptheta) * dt_;
    ptheta += w * dt_;
    traj.push_back(px);
    traj.push_back(py);
  }
}

double DWAPlanner::calcToGoalCost(const std::vector<double> &traj,
                                  double gx, double gy) const {
  if (traj.size() < 2) return std::numeric_limits<double>::infinity();
  const double last_x = traj[traj.size() - 2];
  const double last_y = traj[traj.size() - 1];
  const double dx = gx - last_x;
  const double dy = gy - last_y;
  return std::sqrt(dx * dx + dy * dy);
}

DWAPlanner::Cost DWAPlanner::calcObstacleCost(const std::vector<double> &traj,
                                              const std::vector<double> &obs_flat) const {
  Cost result;
  if (obs_flat.empty()) {
    result.obs_cost = 0.0;
    result.is_collision = false;
    return result;
  }

  double min_dist = obs_range_;

  for (size_t i = 0; i + 1 < traj.size(); i += 2) {
    const double tx = traj[i];
    const double ty = traj[i + 1];
    const double t = (static_cast<double>(i / 2) + 1.0) * dt_;
    
    for (size_t j = 0; j + 4 < obs_flat.size(); j += 5) {
      const double ox0 = obs_flat[j];
      const double oy0 = obs_flat[j + 1];
      const double orad = obs_flat[j + 2];
      const double ovx = obs_flat[j + 3];
      const double ovy = obs_flat[j + 4];
      const double ox = ox0 + ovx * t;
      const double oy = oy0 + ovy * t;
      const double dx = tx - ox;
      const double dy = ty - oy;
      const double dist = std::sqrt(dx * dx + dy * dy);
      const double edge_dist = dist - orad - robot_radius_;
      
      if (edge_dist <= 0.0) {
        result.is_collision = true;
        result.obs_cost = 1e6;
        return result;
      }
      
      min_dist = std::min(min_dist, edge_dist);
    }
  }

  // Use inverse distance but with a safety margin
  // Only significant cost when closer than safety_margin
  const double safety_margin = 0.5;  // Start caring when closer than 0.5m
  if (min_dist > obs_range_) {
    result.obs_cost = 0.0;
  } else if (min_dist > safety_margin) {
    // Gentle cost increase when far from obstacles
    result.obs_cost = 1.0 / (min_dist + 1.0);
  } else {
    // Steeper cost increase when close to obstacles
    result.obs_cost = 1.0 / (min_dist + 0.1);
  }
  
  result.is_collision = false;
  return result;
}

