#pragma once
#include "terramind_navigation_driver/protocol.hpp"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <optional>

namespace terramind::navigation
{
struct Geodetic {double latitude_rad, longitude_rad, height_m;};
Eigen::Vector3d to_ecef(const Geodetic & point);
Geodetic from_ecef(const Eigen::Vector3d & point);
Eigen::Matrix3d ecef_to_enu(double latitude_rad, double longitude_rad);
struct Solution
{
  Eigen::Vector3d imu_position, base_position, imu_velocity;
  Eigen::Quaterniond imu_orientation, base_orientation;
};
class Coordinates
{
public:
  void set_origin(const Geodetic & origin);
  bool has_origin() const {return origin_.has_value();}
  const Geodetic & origin() const {return origin_.value();}
  Eigen::Vector3d forward(const Geodetic & point) const;
  Geodetic reverse(const Eigen::Vector3d & point) const;
  // R_base_imu 将轴向换成 FLU 后的导航机体系向量转到车辆 base_link。
  // t_base_imu 是 IMU 中心在 base_link 下的位置，单位 m。
  void set_extrinsics(const Eigen::Vector3d & t_base_imu, const Eigen::Quaterniond & q_base_imu);
  Solution convert(const Frame & frame) const;
private:
  std::optional<Geodetic> origin_;
  Eigen::Vector3d origin_ecef_ = Eigen::Vector3d::Zero();
  Eigen::Matrix3d map_from_ecef_ = Eigen::Matrix3d::Identity();
  Eigen::Vector3d translation_ = Eigen::Vector3d::Zero();
  Eigen::Quaterniond rotation_ = Eigen::Quaterniond::Identity();
};
}
