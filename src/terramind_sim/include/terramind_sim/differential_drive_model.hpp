#pragma once
namespace terramind::sim
{
struct Geometry
{
  double wheel_diameter = 0.20, wheel_separation = 0.45;
};
class DifferentialDriveModel
{
public:
  explicit DifferentialDriveModel(
    Geometry geometry = {}, double linear_acceleration = 0.7,
    double angular_acceleration = 4.0);
  void step(double target_linear, double target_angular, double dt);
  void reset();
  double left_rpm(double linear, double angular)const;
  double right_rpm(double linear, double angular)const;
  double x = 0, y = 0, yaw = 0, linear = 0, angular = 0, left_position = 0, right_position = 0;

private:
  Geometry geometry_;
  double linear_acceleration_, angular_acceleration_;
};
}
