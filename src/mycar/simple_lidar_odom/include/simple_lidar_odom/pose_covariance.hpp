#ifndef SIMPLE_LIDAR_ODOM__POSE_COVARIANCE_HPP_
#define SIMPLE_LIDAR_ODOM__POSE_COVARIANCE_HPP_

#include <array>
#include <cstddef>

#include <Eigen/Core>

namespace simple_lidar_odom
{

double sigma2FromResiduals(double weighted_squared_residual_sum, std::size_t dof, double min_sigma2);

Eigen::Matrix3d poseCovarianceFromHessian(
  const Eigen::Matrix3d & hessian,
  double sigma2,
  double eigenvalue_floor);

std::array<double, 36> mapPoseCovToOdom(const Eigen::Matrix3d & pose_covariance, double large_variance);

}  // namespace simple_lidar_odom

#endif  // SIMPLE_LIDAR_ODOM__POSE_COVARIANCE_HPP_
