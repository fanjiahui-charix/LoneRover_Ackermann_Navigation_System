#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "mycar_navigation/nav_core/arc.hpp"

namespace mycar_navigation::nav_core
{
namespace
{
constexpr double kTolerance = 1e-6;
constexpr double kRadiusTolerance = 1e-5;
}  // namespace

TEST(ArcIntegratorTest, StraightLineKeepsZeroLateralDrift)
{
  ArcIntegrator integrator;
  const Pose2D start{0.0, 0.0, 0.0};

  const auto trajectory = integrator.integrate(start, 1.0, 0.0, 1.0, 0.1);

  ASSERT_EQ(trajectory.size(), 11U);
  double previous_x = trajectory.front().x;
  for (const auto & pose : trajectory) {
    EXPECT_NEAR(pose.y, 0.0, kTolerance);
    EXPECT_GE(pose.x + kTolerance, previous_x);
    previous_x = pose.x;
  }
  EXPECT_NEAR(trajectory.back().x, 1.0, kTolerance);
}

TEST(ArcIntegratorTest, MaxCurvatureTracksCircleOfMinimumRadius)
{
  const double minimum_turning_radius = 0.6;
  const double kappa = 1.0 / minimum_turning_radius;
  ArcIntegrator integrator;
  const Pose2D start{1.2, -0.4, 0.0};

  const auto trajectory = integrator.integrate(start, 0.3, kappa, 1.0, 0.05);

  ASSERT_GT(trajectory.size(), 1U);
  const double center_x = start.x;
  const double center_y = start.y + minimum_turning_radius;

  for (const auto & pose : trajectory) {
    const double dx = pose.x - center_x;
    const double dy = pose.y - center_y;
    EXPECT_NEAR(std::hypot(dx, dy), minimum_turning_radius, kRadiusTolerance);
  }
}

// Negative curvature curves to the right; radius still equals 1/|kappa| with
// the centre on the opposite side (centre_y = start.y + 1/kappa).
TEST(ArcIntegratorTest, NegativeCurvatureTracksCircleOnOppositeSide)
{
  const double kappa = -1.0 / 0.6;
  ArcIntegrator integrator;
  const Pose2D start{0.0, 0.0, 0.0};

  const auto trajectory = integrator.integrate(start, 0.3, kappa, 1.0, 0.05);

  ASSERT_GT(trajectory.size(), 1U);
  const double center_x = start.x;
  const double center_y = start.y + 1.0 / kappa;  // negative -> centre below
  const double radius = 1.0 / std::abs(kappa);

  for (const auto & pose : trajectory) {
    EXPECT_NEAR(std::hypot(pose.x - center_x, pose.y - center_y), radius, kRadiusTolerance);
  }
}

// The Ackermann contract omega = v * kappa must hold on every integrated step,
// for both signs of v and kappa.
TEST(ArcIntegratorTest, YawRateEqualsVelocityTimesCurvature)
{
  ArcIntegrator integrator;
  const double dt = 0.05;
  const struct
  {
    double v;
    double kappa;
  } cases[] = {{0.4, 1.2}, {0.4, -1.2}, {-0.3, 0.8}, {-0.3, -0.8}};

  for (const auto & c : cases) {
    const auto trajectory = integrator.integrate(Pose2D{0.0, 0.0, 0.0}, c.v, c.kappa, 0.5, dt);
    ASSERT_GT(trajectory.size(), 1U);
    const double expected_omega = c.v * c.kappa;
    for (std::size_t k = 1; k < trajectory.size(); ++k) {
      const double measured_omega = (trajectory[k].yaw - trajectory[k - 1].yaw) / dt;
      EXPECT_NEAR(measured_omega, expected_omega, 1e-9);
    }
  }
}

TEST(CurvatureSamplerTest, UniformSamplingStaysWithinRadiusBound)
{
  const double minimum_turning_radius = 0.6;
  CurvatureSampler sampler(minimum_turning_radius);
  const auto samples = sampler.sample(31);
  const double max_curvature = 1.0 / minimum_turning_radius;

  ASSERT_EQ(samples.size(), 31U);
  EXPECT_NEAR(samples.front(), -max_curvature, kTolerance);
  EXPECT_NEAR(samples.back(), max_curvature, kTolerance);

  for (const double sample_value : samples) {
    EXPECT_LE(std::abs(sample_value), max_curvature + kTolerance);
  }

  // Every sampled curvature corresponds to |omega/v| <= 1/Rmin for any v != 0.
  const double v = 1.0;
  for (const double sample_value : samples) {
    const double omega = v * sample_value;
    EXPECT_LE(std::abs(omega / v), max_curvature + kTolerance);
  }
}

TEST(CurvatureSamplerTest, RefineAroundCapsSpacingAtFiveCentimetersInverse)
{
  CurvatureSampler sampler(0.6);
  const auto refined = sampler.refineAround(0.3, 0.4, 5);

  ASSERT_GT(refined.size(), 1U);
  for (std::size_t index = 1; index < refined.size(); ++index) {
    EXPECT_LE(refined[index] - refined[index - 1], 0.05 + 1e-9);
  }
}

}  // namespace mycar_navigation::nav_core
