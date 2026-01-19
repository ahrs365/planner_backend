#pragma once

#include <cmath>
#include <cstddef>
#include <vector>

namespace ahrs {

class Vec2d {
 public:
  Vec2d() = default;
  Vec2d(double x, double y) : x_(x), y_(y) {}

  double x() const { return x_; }
  double y() const { return y_; }
  void set_x(double x) { x_ = x; }
  void set_y(double y) { y_ = y; }

  Vec2d operator+(const Vec2d& other) const {
    return Vec2d(x_ + other.x_, y_ + other.y_);
  }
  Vec2d operator-(const Vec2d& other) const {
    return Vec2d(x_ - other.x_, y_ - other.y_);
  }
  Vec2d operator*(double scale) const { return Vec2d(x_ * scale, y_ * scale); }

  double Length() const { return std::sqrt(x_ * x_ + y_ * y_); }
  double Angle() const { return std::atan2(y_, x_); }

 private:
  double x_ = 0.0;
  double y_ = 0.0;
};

inline double DistanceSquared(const Vec2d& a, const Vec2d& b) {
  const double dx = a.x() - b.x();
  const double dy = a.y() - b.y();
  return dx * dx + dy * dy;
}

struct Point {
  Vec2d pose_;
  double s_ = 0.0;
  double theta_ = 0.0;
  double kappa_ = 0.0;

  Point() = default;
  Point(double x, double y) : pose_(x, y) {}
};

struct Curve {
  std::vector<Point> points_;
  std::vector<Point> control_points_;

  Curve() = default;
  Curve(const std::vector<Point>& points,
        const std::vector<Point>& control_points)
      : points_(points), control_points_(control_points) {}
};

struct RobotState {
  Vec2d pose_;
  double theta_ = 0.0;
};

struct Config {
  double reference_interval_ = 0.2;
  double sample_length_ = 10.0;
  double ctp_interval_x_ = 0.6;
  double sample_half_width_ = 1.5;
  double ctp_interval_y_ = 0.5;
  double zero_layer_interval_ = 0.5;
  double bspline_interval_ = 0.02;
  double vehicle_length_ = 0.7;
  double vehicle_width_ = 0.7;
  double collision_margin_ = 0.05;
  size_t max_layers_ = 10;
  size_t spread_layers_ = 2;
  double w_spread_ = 0.6;
  size_t beam_width_ = 8;
  size_t max_candidate_paths_ = 12;
  double w_center_ = 1.0;
  double w_smooth_ = 1.5;
  double w_collision_ = 1000.0;
  size_t max_sequences_ = 2000;
};

struct Obstacle {
  Vec2d center;
  double radius = 0.0;
};

struct Environment {
  std::vector<Obstacle> obstacles_;

  bool IsCollision(const Vec2d& point, double buffer = 0.0) const {
    for (const auto& obs : obstacles_) {
      const double dx = point.x() - obs.center.x();
      const double dy = point.y() - obs.center.y();
      const double r = obs.radius + buffer;
      if (dx * dx + dy * dy <= r * r) {
        return true;
      }
    }
    return false;
  }
};

struct DebugInfo {
  std::vector<std::vector<Vec2d>> sample_control_points_;
  std::vector<std::vector<Vec2d>> blocked_control_points_;
  std::vector<std::vector<Vec2d>> active_control_points_;
  std::vector<Vec2d> active_bounds_;
  std::vector<Point> reference_line_points_;
  std::vector<Vec2d> chosen_control_points_;
  std::vector<std::vector<Point>> bspline_samples_;
};

}  // namespace ahrs
