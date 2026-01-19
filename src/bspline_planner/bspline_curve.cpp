#include "bspline_curve.h"

#include <Eigen/Dense>

#include "bspline_math.h"
namespace ahrs {
BsplineCurve::BsplineCurve(const double& interval)
    : interval_(interval), ctp_size_(0) {}

BsplineCurve::~BsplineCurve() {}

std::vector<Point> BsplineCurve::GenerateCurve() {
  std::vector<Point> trajectory;
  if (ctp_size_ < 4) {
    return trajectory;
  }
  for (size_t i = 0; i < ctp_size_ - 3; ++i) {
    for (double j = 0; j < 1.0; j += interval_) {
      Point p = GetPos(i, j);
      trajectory.push_back(p);
    }
  }

  return trajectory;
}

void BsplineCurve::SetControlPoints(const std::vector<Vec2d>& control_points) {
  ctp_ = control_points;
  ctp_size_ = ctp_.size();
}

Point BsplineCurve::GetPos(const size_t& k, const double& ti) {
  Eigen::MatrixXd m(4, 4);
  m << -1, 3, -3, 1, 3, -6, 3, 0, -3, 0, 3, 0, 1, 4, 1, 0;

  Eigen::MatrixXd t(1, 4);
  Eigen::MatrixXd dt(1, 4);
  Eigen::MatrixXd ddt(1, 4);
  t << ti * ti * ti, ti * ti, ti, 1;
  dt << 3 * ti * ti, 2 * ti, 1, 0;
  ddt << 6 * ti, 2, 0, 0;

  Eigen::MatrixXd p(4, 2);
  for (size_t i = 0; i < 4; i++) {
    p(i, 0) = ctp_[(k + i) % ctp_size_].x() / 6.0;
    p(i, 1) = ctp_[(k + i) % ctp_size_].y() / 6.0;
  }

  Eigen::MatrixXd pos = t * m * p;
  Eigen::MatrixXd first_derivative = dt * m * p;
  Eigen::MatrixXd second_derivative = ddt * m * p;

  double f_x = first_derivative(0, 0);
  double f_y = first_derivative(0, 1);
  double s_x = second_derivative(0, 0);
  double s_y = second_derivative(0, 1);
  double kappa = std::fabs(f_x * s_y - s_x * f_y) /
                 std::sqrt(std::pow(f_x * f_x + f_y * f_y, 3));

  Point res(pos(0, 0), pos(0, 1));
  res.kappa_ = kappa;
  res.theta_ = NormalizeAngle(std::atan2(f_y, f_x));

  return res;
}

}  // namespace ahrs