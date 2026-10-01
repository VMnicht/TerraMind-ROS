#include "terramind_sim/differential_drive_model.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
namespace terramind::sim
{
constexpr double PI = 3.14159265358979323846;
DifferentialDriveModel::DifferentialDriveModel(Geometry g, double a, double aa)
: geometry_(g), linear_acceleration_(a), angular_acceleration_(aa)
{
  if (!std::isfinite(g.wheel_diameter) || !std::isfinite(g.wheel_separation) ||
    g.wheel_diameter <= 0 || g.wheel_separation <= 0 || !std::isfinite(a) || !std::isfinite(aa) ||
    a <= 0 || aa <= 0)
  {
    throw std::invalid_argument("positive finite simulation geometry and acceleration required");
  }
}
void DifferentialDriveModel::reset()
{
  x = y = yaw = linear = angular = left_position = right_position = 0;
}
double DifferentialDriveModel::left_rpm(double v, double w)const
{
  return (v - w * geometry_.wheel_separation / 2) * 60 / (PI * geometry_.wheel_diameter);
}
double DifferentialDriveModel::right_rpm(double v, double w)const
{
  return (v + w * geometry_.wheel_separation / 2) * 60 / (PI * geometry_.wheel_diameter);
}
void DifferentialDriveModel::step(double v, double w, double dt)
{
  if (!std::isfinite(dt) || dt < 0 || !std::isfinite(v) || !std::isfinite(w)) {
    throw std::invalid_argument("invalid dynamics input");
  }
  while (dt > 1e-9) {
    double h = std::min(dt, 0.01);
    dt -= h;
    double previous_v = linear, previous_w = angular;
    linear += std::clamp(v - linear, -linear_acceleration_ * h, linear_acceleration_ * h);
    angular += std::clamp(w - angular, -angular_acceleration_ * h, angular_acceleration_ * h);
    double vm = (linear + previous_v) / 2, wm = (angular + previous_w) / 2;
    double dyaw = wm * h;
    if (std::abs(wm) > 1e-8) {
      x += vm / wm * (std::sin(yaw + dyaw) - std::sin(yaw));
      y += vm / wm * (std::cos(yaw) - std::cos(yaw + dyaw));
    } else {
      x += vm * std::cos(yaw) * h;
      y += vm * std::sin(yaw) * h;
    }
    yaw = std::remainder(yaw + dyaw, 2 * PI);
    left_position += left_rpm(vm, wm) * 2 * PI / 60 * h;
    right_position += right_rpm(vm, wm) * 2 * PI / 60 * h;
  }
}
}
