#include "dwa_planner.h"

#include <algorithm>
#include <cmath>
#include <limits>

std::vector<double> DWAPlanner::plan(double x, double y, double theta,
                                     double v, double w,
                                     double gx, double gy,
                                     const std::vector<double> &obs_flat) {
  const Window win = calcDynamicWindow(v, w);

  double best_cost = std::numeric_limits<double>::infinity();
  double best_v = 0.0;
  double best_w = 0.0;

  best_cost_ = std::numeric_limits<double>::infinity();
  best_traj_.clear();
  for (double vt = win.v_min; vt <= win.v_max + 1e-6; vt += v_reso_) {
    bool has_zero_w = false;
    for (double wt = win.w_min; wt <= win.w_max + 1e-6; wt += yawrate_reso_) {
      if (std::abs(wt) < 1e-9) has_zero_w = true;
      std::vector<double> traj;
      simulateTrajectory(x, y, theta, vt, wt, traj);

      const double to_goal = calcToGoalCost(traj, gx, gy);
      const double obs_cost = calcObstacleCost(traj, obs_flat);
      if (obs_cost >= kCollisionCost) {
        continue;
      }
      const double speed_cost = max_speed_ - vt;

      const double cost = to_goal_cost_gain_ * to_goal +
                          obstacle_cost_gain_ * obs_cost +
                          speed_cost_gain_ * speed_cost;
      if (cost < best_cost) {
        best_cost = cost;
        best_v = vt;
        best_w = wt;
        best_traj_ = traj;
      }
    }
    if (!has_zero_w && win.w_min <= 0.0 && win.w_max >= 0.0) {
      const double wt = 0.0;
      std::vector<double> traj;
      simulateTrajectory(x, y, theta, vt, wt, traj);

      const double to_goal = calcToGoalCost(traj, gx, gy);
      const double obs_cost = calcObstacleCost(traj, obs_flat);
      if (obs_cost >= kCollisionCost) {
        continue;
      }
      const double speed_cost = max_speed_ - vt;

      const double cost = to_goal_cost_gain_ * to_goal +
                          obstacle_cost_gain_ * obs_cost +
                          speed_cost_gain_ * speed_cost;
      if (cost < best_cost) {
        best_cost = cost;
        best_v = vt;
        best_w = wt;
        best_traj_ = traj;
      }
    }
  }

  best_cost_ = best_cost;
  return {best_v, best_w};
}

std::vector<double> DWAPlanner::sampleTrajectories(double x, double y, double theta,
                                                   double v, double w,
                                                   const std::vector<double> &obs_flat) {
  (void)obs_flat;
  const Window win = calcDynamicWindow(v, w);
  const int steps = predictSteps();

  std::vector<double> out;
  int traj_count = 0;

  for (double vt = win.v_min; vt <= win.v_max + 1e-6; vt += v_reso_) {
    bool has_zero_w = false;
    for (double wt = win.w_min; wt <= win.w_max + 1e-6; wt += yawrate_reso_) {
      if (std::abs(wt) < 1e-9) has_zero_w = true;
      std::vector<double> traj;
      simulateTrajectory(x, y, theta, vt, wt, traj);
      if (traj.size() / 2 != static_cast<size_t>(steps)) {
        continue;
      }
      out.insert(out.end(), traj.begin(), traj.end());
      traj_count++;
    }
    if (!has_zero_w && win.w_min <= 0.0 && win.w_max >= 0.0) {
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
  v_reso_ = v_reso;
  yawrate_reso_ = yawrate_reso;
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

double DWAPlanner::calcObstacleCost(const std::vector<double> &traj,
                                    const std::vector<double> &obs_flat) const {
  if (obs_flat.empty()) return 0.0;
  double min_dist = std::numeric_limits<double>::infinity();

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
      const double edge_dist = dist - orad;
      if (edge_dist < min_dist) min_dist = edge_dist;
    }
  }

  if (min_dist <= robot_radius_) {
    return kCollisionCost;
  }
  return 1.0 / std::max(min_dist, 1e-6);
}
