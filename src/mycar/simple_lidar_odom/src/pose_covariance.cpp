#include "simple_lidar_odom/pose_covariance.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include <Eigen/Eigenvalues>

namespace simple_lidar_odom
{

double sigma2FromResiduals(double weighted_squared_residual_sum, std::size_t dof, double min_sigma2)
{
  const double clamped_min_sigma2 = std::max(min_sigma2, 0.0);
  if (!std::isfinite(weighted_squared_residual_sum) || weighted_squared_residual_sum < 0.0 || dof == 0U) {
    return clamped_min_sigma2;
  }

  return std::max(weighted_squared_residual_sum / static_cast<double>(dof), clamped_min_sigma2);
}

Eigen::Matrix3d poseCovarianceFromHessian(
  const Eigen::Matrix3d & hessian,
  double sigma2,
  double eigenvalue_floor)
{
  const double clamped_sigma2 = std::max(sigma2, 0.0);
  const double clamped_eigenvalue_floor = std::max(eigenvalue_floor, std::numeric_limits<double>::min());

  const Eigen::Matrix3d sym_hessian = 0.5 * (hessian + hessian.transpose());
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(sym_hessian);
  if (solver.info() != Eigen::Success) {
    return Eigen::Matrix3d::Identity() * (clamped_sigma2 / clamped_eigenvalue_floor);
  }

  Eigen::Vector3d inv_eigenvalues = solver.eigenvalues();
  for (int i = 0; i < inv_eigenvalues.size(); ++i) {
    inv_eigenvalues(i) = 1.0 / std::max(inv_eigenvalues(i), clamped_eigenvalue_floor);
  }

  const Eigen::Matrix3d covariance =
    solver.eigenvectors() * inv_eigenvalues.asDiagonal() * solver.eigenvectors().transpose();
  return clamped_sigma2 * 0.5 * (covariance + covariance.transpose());
}

std::array<double, 36> mapPoseCovToOdom(const Eigen::Matrix3d & pose_covariance, double large_variance)
{
  std::array<double, 36> odom_covariance{};
  odom_covariance.fill(0.0);

  odom_covariance[0 * 6 + 0] = pose_covariance(0, 0);
  odom_covariance[0 * 6 + 1] = pose_covariance(0, 1);
  odom_covariance[0 * 6 + 5] = pose_covariance(0, 2);
  odom_covariance[1 * 6 + 0] = pose_covariance(1, 0);
  odom_covariance[1 * 6 + 1] = pose_covariance(1, 1);
  odom_covariance[1 * 6 + 5] = pose_covariance(1, 2);
  odom_covariance[5 * 6 + 0] = pose_covariance(2, 0);
  odom_covariance[5 * 6 + 1] = pose_covariance(2, 1);
  odom_covariance[5 * 6 + 5] = pose_covariance(2, 2);

  odom_covariance[2 * 6 + 2] = large_variance;
  odom_covariance[3 * 6 + 3] = large_variance;
  odom_covariance[4 * 6 + 4] = large_variance;
  return odom_covariance;
}

}  // namespace simple_lidar_odom
