#pragma once
namespace terramind::sim
{
struct Geometry
{
  // 单位 m；默认值仅供演示，不能当作真实机器标定尺寸。
  double wheel_diameter = 0.20, wheel_separation = 0.45;
};
// 平面无侧滑运动学 + 加速度限制；不模拟碰撞、坡面、轮胎打滑或电机电气特性。
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
  // x/y 单位 m，yaw/轮转角单位 rad，linear 为 m/s，angular 为 rad/s。
  double x = 0, y = 0, yaw = 0, linear = 0, angular = 0, left_position = 0, right_position = 0;

private:
  Geometry geometry_;
  double linear_acceleration_, angular_acceleration_;
};
}
