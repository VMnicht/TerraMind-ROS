#include "terramind_navigation_driver/coordinates.hpp"
#include <cmath>
#include <stdexcept>

namespace terramind::navigation
{
namespace
{
constexpr double a = 6378137.0, flattening = 1.0 / 298.257223563;
constexpr double e2 = flattening * (2 - flattening);
constexpr double pi = 3.14159265358979323846;
void validate(const Geodetic & p)
{
  if (!std::isfinite(p.latitude_rad) || !std::isfinite(p.longitude_rad) ||
    !std::isfinite(p.height_m) || std::abs(p.latitude_rad) > pi / 2 ||
    std::abs(p.longitude_rad) > pi) {throw std::invalid_argument("invalid WGS84 coordinate");}
}
}
Eigen::Vector3d to_ecef(const Geodetic & p)
{
  validate(p);
  const double s = std::sin(p.latitude_rad), c = std::cos(p.latitude_rad);
  const double n = a / std::sqrt(1 - e2 * s * s);
  return {(n + p.height_m) * c * std::cos(p.longitude_rad),
    (n + p.height_m) * c * std::sin(p.longitude_rad), (n * (1 - e2) + p.height_m) * s};
}
Geodetic from_ecef(const Eigen::Vector3d & p)
{
  if (!p.allFinite() || p.norm() < 1) {throw std::invalid_argument("undefined geodetic coordinate");}
  const double horizontal = std::hypot(p.x(), p.y());
  const double longitude = std::atan2(p.y(), p.x());
  if (horizontal < 1e-9) {
    return {std::copysign(pi / 2, p.z()), longitude, std::abs(p.z()) - a * (1 - flattening)};
  }
  double latitude = std::atan2(p.z(), horizontal * (1 - e2));
  for (int i = 0; i < 20; ++i) {
    const double s = std::sin(latitude), n = a / std::sqrt(1 - e2 * s * s);
    const double next = std::atan2(p.z() + e2 * n * s, horizontal);
    if (std::abs(next - latitude) < 1e-15) {latitude = next; break;}
    latitude = next;
  }
  // 沿椭球法线计算高度，避免接近极点时除以 cos(latitude)。
  const double s = std::sin(latitude), c = std::cos(latitude);
  const double height = horizontal * c + p.z() * s - a * std::sqrt(1 - e2 * s * s);
  return {latitude, longitude, height};
}
Eigen::Matrix3d ecef_to_enu(double lat, double lon)
{
  const double s = std::sin(lat), c = std::cos(lat), sl = std::sin(lon), cl = std::cos(lon);
  Eigen::Matrix3d r;
  r << -sl, cl, 0, -s * cl, -s * sl, c, c * cl, c * sl, s;
  return r;
}
void Coordinates::set_origin(const Geodetic & origin)
{
  validate(origin); origin_ = origin; origin_ecef_ = to_ecef(origin);
  map_from_ecef_ = ecef_to_enu(origin.latitude_rad, origin.longitude_rad);
}
Eigen::Vector3d Coordinates::forward(const Geodetic & p) const
{
  if (!origin_) {throw std::logic_error("navigation origin not set");}
  return map_from_ecef_ * (to_ecef(p) - origin_ecef_);
}
Geodetic Coordinates::reverse(const Eigen::Vector3d & p) const
{
  if (!origin_) {throw std::logic_error("navigation origin not set");}
  return from_ecef(origin_ecef_ + map_from_ecef_.transpose() * p);
}
void Coordinates::set_extrinsics(const Eigen::Vector3d & t, const Eigen::Quaterniond & q)
{
  if (!t.allFinite() || !q.coeffs().allFinite() || std::abs(q.squaredNorm() - 1) > 0.001) {
    throw std::invalid_argument("invalid navigation extrinsics");
  }
  translation_ = t; rotation_ = q.normalized();
}
Solution Coordinates::convert(const Frame & f) const
{
  if (!invalid_reason(f).empty()) {throw std::invalid_argument("invalid navigation measurement");}
  Solution out;
  out.imu_position = forward({f.latitude_rad, f.longitude_rad, f.height_m});
  Eigen::Matrix3d enu_from_ned, frd_from_flu;
  enu_from_ned << 0, 1, 0, 1, 0, 0, 0, 0, -1;
  frd_from_flu = Eigen::Vector3d(1, -1, -1).asDiagonal();
  // 当前位置的切平面与固定原点切平面不同；速度和姿态都转到固定 map。
  const Eigen::Matrix3d map_from_ned = map_from_ecef_ *
    ecef_to_enu(f.latitude_rad, f.longitude_rad).transpose() * enu_from_ned;
  const auto & v = f.velocity_ned;
  out.imu_velocity = map_from_ned * Eigen::Vector3d(v[0], v[1], v[2]);
  const auto & q = f.quaternion_wxyz;
  const Eigen::Quaterniond q_ned_frd = Eigen::Quaterniond(q[0], q[1], q[2], q[3]).normalized();
  out.imu_orientation = Eigen::Quaterniond(map_from_ned * q_ned_frd.toRotationMatrix() * frd_from_flu).normalized();
  out.base_orientation = (out.imu_orientation * rotation_.conjugate()).normalized();
  out.base_position = out.imu_position - out.base_orientation * translation_;
  return out;
}
}
