#include "bspline_curve.h"

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
  const size_t segments = ctp_size_ - 3;
  const size_t samples_per_segment =
      static_cast<size_t>(std::ceil(1.0 / std::max(interval_, 1e-6)));
  trajectory.reserve(segments * samples_per_segment);
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
  const double t = ti;
  const double t2 = t * t;
  const double t3 = t2 * t;

  const double b0 = (-t3 + 3.0 * t2 - 3.0 * t + 1.0) / 6.0;
  const double b1 = (3.0 * t3 - 6.0 * t2 + 4.0) / 6.0;
  const double b2 = (-3.0 * t3 + 3.0 * t2 + 3.0 * t + 1.0) / 6.0;
  const double b3 = t3 / 6.0;

  const double db0 = (-3.0 * t2 + 6.0 * t - 3.0) / 6.0;
  const double db1 = (9.0 * t2 - 12.0 * t) / 6.0;
  const double db2 = (-9.0 * t2 + 6.0 * t + 3.0) / 6.0;
  const double db3 = (3.0 * t2) / 6.0;

  const double ddb0 = (1.0 - t);
  const double ddb1 = (3.0 * t - 2.0);
  const double ddb2 = (-3.0 * t + 1.0);
  const double ddb3 = t;

  const Vec2d& p0 = ctp_[k];
  const Vec2d& p1 = ctp_[k + 1];
  const Vec2d& p2 = ctp_[k + 2];
  const Vec2d& p3 = ctp_[k + 3];

  const double x = b0 * p0.x() + b1 * p1.x() + b2 * p2.x() + b3 * p3.x();
  const double y = b0 * p0.y() + b1 * p1.y() + b2 * p2.y() + b3 * p3.y();

  const double f_x = db0 * p0.x() + db1 * p1.x() + db2 * p2.x() + db3 * p3.x();
  const double f_y = db0 * p0.y() + db1 * p1.y() + db2 * p2.y() + db3 * p3.y();
  const double s_x = ddb0 * p0.x() + ddb1 * p1.x() + ddb2 * p2.x() + ddb3 * p3.x();
  const double s_y = ddb0 * p0.y() + ddb1 * p1.y() + ddb2 * p2.y() + ddb3 * p3.y();
  const double denom = std::pow(f_x * f_x + f_y * f_y, 1.5);
  const double kappa =
      denom > 1e-9 ? std::fabs(f_x * s_y - s_x * f_y) / denom : 0.0;

  Point res(x, y);
  res.kappa_ = kappa;
  res.theta_ = NormalizeAngle(std::atan2(f_y, f_x));

  return res;
}

}  // namespace ahrs