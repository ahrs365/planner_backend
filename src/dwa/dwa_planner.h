#pragma once

#include <vector>

class DWAPlanner {
 public:
  DWAPlanner() = default;

  std::vector<double> plan(double x, double y, double theta,
                           double v, double w,
                           double gx, double gy,
                           const std::vector<double> &obs_flat);

  std::vector<double> sampleTrajectories(double x, double y, double theta,
                                         double v, double w,
                                         const std::vector<double> &obs_flat);

  std::vector<double> bestTrajectory() const;
  double bestCost() const;

  void setLimits(double max_speed, double min_speed,
                 double max_yawrate, double max_accel,
                 double max_dyawrate);

  void setResolution(double v_reso, double yawrate_reso);
  void setSamples(int velocity_samples, int yawrate_samples);
  void setPredict(double dt, double predict_time);
  void setWeights(double to_goal, double obstacle, double speed);
  void setRobotRadius(double r);
  void setObsRange(double range);

 private:
  struct Window {
    double v_min;
    double v_max;
    double w_min;
    double w_max;
  };

  struct Cost {
    double obs_cost = 0.0;
    double to_goal_cost = 0.0;
    double speed_cost = 0.0;
    double total_cost = 0.0;
    bool is_collision = false;
  };

  Window calcDynamicWindow(double v, double w) const;
  int predictSteps() const;
  void simulateTrajectory(double x, double y, double theta,
                          double v, double w,
                          std::vector<double> &traj) const;
  double calcToGoalCost(const std::vector<double> &traj,
                        double gx, double gy) const;
  Cost calcObstacleCost(const std::vector<double> &traj,
                        const std::vector<double> &obs_flat) const;
  void normalizeCosts(std::vector<Cost> &costs) const;

  double max_speed_ = 2.0;
  double min_speed_ = -0.2;
  double max_yawrate_ = 2.5;
  double max_accel_ = 1.0;
  double max_dyawrate_ = 3.0;
  
  // Use sample counts instead of resolution for more uniform sampling
  int velocity_samples_ = 10;
  int yawrate_samples_ = 20;
  
  double dt_ = 0.1;
  double predict_time_ = 2.0;
  double to_goal_cost_gain_ = 1.0;
  double obstacle_cost_gain_ = 0.3;  // Reduced: avoid over-reaction to obstacles
  double speed_cost_gain_ = 0.3;
  double robot_radius_ = 0.35;
  double obs_range_ = 3.0;  // Obstacle sensing range

  double best_cost_ = 0.0;
  std::vector<double> best_traj_;
};
