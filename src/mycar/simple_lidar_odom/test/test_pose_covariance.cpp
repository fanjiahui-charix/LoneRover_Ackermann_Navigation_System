#include <gtest/gtest.h>

#include <Eigen/Core>
#include <Eigen/Eigenvalues>

#include "simple_lidar_odom/pose_covariance.hpp"

namespace simple_lidar_odom
{
namespace
{

TEST(PoseCovarianceTest, Sigma2UsesResidualMeanSquareWithFloor)
{
  EXPECT_DOUBLE_EQ(sigma2FromResiduals(12.0, 4U, 0.1), 3.0);
  EXPECT_DOUBLE_EQ(sigma2FromResiduals(0.01, 10U, 0.1), 0.1);
}

TEST(PoseCovarianceTest, Sigma2FallsBackForInvalidInputs)
{
  EXPECT_DOUBLE_EQ(sigma2FromResiduals(-1.0, 4U, 0.2), 0.2);
  EXPECT_DOUBLE_EQ(sigma2FromResiduals(5.0, 0U, 0.2), 0.2);
}

TEST(PoseCovarianceTest, HessianInverseMatchesWellConditionedCase)
{
  Eigen::Matrix3d hessian = Eigen::Matrix3d::Zero();
  hessian(0, 0) = 4.0;
  hessian(1, 1) = 2.0;
  hessian(2, 2) = 8.0;
  hessian(0, 1) = 0.6;
  hessian(1, 0) = 0.6;
  hessian(0, 2) = -0.4;
  hessian(2, 0) = -0.4;
  hessian(1, 2) = 0.2;
  hessian(2, 1) = 0.2;

  const double sigma2 = 0.5;
  const Eigen::Matrix3d covariance = poseCovarianceFromHessian(hessian, sigma2, 1.0e-6);
  const Eigen::Matrix3d expected = sigma2 * hessian.inverse();

  EXPECT_TRUE(covariance.isApprox(expected, 1.0e-9));
}

TEST(PoseCovarianceTest, HessianInverseFloorsDegenerateEigenvalues)
{
  Eigen::Matrix3d hessian = Eigen::Matrix3d::Zero();
  hessian(0, 0) = 4.0;
  hessian(1, 1) = 1.0e-10;
  hessian(2, 2) = -2.0;
  hessian(0, 1) = 0.3;
  hessian(1, 0) = 0.1;

  const double sigma2 = 2.0;
  const double eigenvalue_floor = 0.5;
  const Eigen::Matrix3d covariance = poseCovarianceFromHessian(hessian, sigma2, eigenvalue_floor);

  EXPECT_TRUE(covariance.isApprox(covariance.transpose(), 1.0e-12));

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> covariance_solver(covariance);
  ASSERT_EQ(covariance_solver.info(), Eigen::Success);
  EXPECT_GE(covariance_solver.eigenvalues().minCoeff(), -1.0e-12);
  EXPECT_LE(covariance_solver.eigenvalues().maxCoeff(), sigma2 / eigenvalue_floor + 1.0e-9);
}

TEST(PoseCovarianceTest, OdomMappingPlacesPlanarTermsInRosSlots)
{
  Eigen::Matrix3d covariance;
  covariance << 1.0, 0.1, 0.2,
    0.1, 2.0, 0.3,
    0.2, 0.3, 3.0;

  const auto odom_covariance = mapPoseCovToOdom(covariance, 1.0e6);

  EXPECT_DOUBLE_EQ(odom_covariance[0], 1.0);
  EXPECT_DOUBLE_EQ(odom_covariance[1], 0.1);
  EXPECT_DOUBLE_EQ(odom_covariance[5], 0.2);
  EXPECT_DOUBLE_EQ(odom_covariance[6], 0.1);
  EXPECT_DOUBLE_EQ(odom_covariance[7], 2.0);
  EXPECT_DOUBLE_EQ(odom_covariance[11], 0.3);
  EXPECT_DOUBLE_EQ(odom_covariance[30], 0.2);
  EXPECT_DOUBLE_EQ(odom_covariance[31], 0.3);
  EXPECT_DOUBLE_EQ(odom_covariance[35], 3.0);
  EXPECT_DOUBLE_EQ(odom_covariance[14], 1.0e6);
  EXPECT_DOUBLE_EQ(odom_covariance[21], 1.0e6);
  EXPECT_DOUBLE_EQ(odom_covariance[28], 1.0e6);
}

}  // namespace
}  // namespace simple_lidar_odom
