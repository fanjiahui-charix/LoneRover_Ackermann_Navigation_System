#include <cmath>
#include <limits>
#include <vector>

#include <gtest/gtest.h>

#include "mycar_navigation/navigator/scan_conversion.hpp"

namespace mycar_navigation::navigator
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr float kInf = std::numeric_limits<float>::infinity();
constexpr float kNan = std::numeric_limits<float>::quiet_NaN();

TEST(ScanConversionTest, ForwardRayMapsToPlusXWhenLaserAtOrigin)
{
  // Single ray straight ahead (angle 0), laser at base origin.
  const std::vector<float> ranges = {2.0f};
  const auto pts = laserScanToBasePoints(ranges, 0.0, kPi / 2.0, 0.1, 10.0, Pose2D{0, 0, 0});

  ASSERT_EQ(pts.size(), 1U);
  EXPECT_NEAR(pts[0].x, 2.0, 1e-9);
  EXPECT_NEAR(pts[0].y, 0.0, 1e-9);
}

TEST(ScanConversionTest, FiltersNanInfAndOutOfRange)
{
  // 5 rays: valid, NaN, inf, below range_min, above range_max.
  const std::vector<float> ranges = {1.0f, kNan, kInf, 0.05f, 99.0f};
  const auto pts = laserScanToBasePoints(ranges, 0.0, 0.1, 0.1, 10.0, Pose2D{0, 0, 0});

  ASSERT_EQ(pts.size(), 1U);  // only the first ray survives
  EXPECT_NEAR(pts[0].x, 1.0, 1e-9);
}

TEST(ScanConversionTest, AppliesLaserMountTranslationAndRotation)
{
  // Laser mounted 0.1 m ahead of base origin, rotated +90 deg (laser +x -> base +y).
  // A ray straight ahead in the laser frame (angle 0, range 1.0) should land at
  // base (0.1, 1.0).
  const std::vector<float> ranges = {1.0f};
  const auto pts = laserScanToBasePoints(
    ranges, 0.0, 0.1, 0.1, 10.0, Pose2D{0.1, 0.0, kPi / 2.0});

  ASSERT_EQ(pts.size(), 1U);
  EXPECT_NEAR(pts[0].x, 0.1, 1e-9);
  EXPECT_NEAR(pts[0].y, 1.0, 1e-9);
}

TEST(ScanConversionTest, AngleIndexingUsesAngleMinPlusIncrement)
{
  // Two rays: angle_min = -pi/2 (points to -y), increment pi/2 (second ray to +x).
  const std::vector<float> ranges = {1.0f, 1.0f};
  const auto pts = laserScanToBasePoints(ranges, -kPi / 2.0, kPi / 2.0, 0.1, 10.0, Pose2D{0, 0, 0});

  ASSERT_EQ(pts.size(), 2U);
  EXPECT_NEAR(pts[0].x, 0.0, 1e-9);
  EXPECT_NEAR(pts[0].y, -1.0, 1e-9);
  EXPECT_NEAR(pts[1].x, 1.0, 1e-9);
  EXPECT_NEAR(pts[1].y, 0.0, 1e-9);
}

}  // namespace
}  // namespace mycar_navigation::navigator
