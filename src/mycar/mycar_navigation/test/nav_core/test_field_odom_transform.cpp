#include <gtest/gtest.h>

#include <cmath>

#include "mycar_navigation/nav_core/field_odom_transform.hpp"

namespace mycar_navigation::nav_core
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
}  // namespace

TEST(FieldOdomTransformTest, MapsOdomOriginToAnchor)
{
  const FieldOdomTransform transform(Pose2D{0.5, 0.3, 0.0});

  const Pose2D field_pose = transform.odomToField(Pose2D{0.0, 0.0, 0.0});

  EXPECT_NEAR(field_pose.x, 0.5, 1e-9);
  EXPECT_NEAR(field_pose.y, 0.3, 1e-9);
  EXPECT_NEAR(field_pose.yaw, 0.0, 1e-9);
}

TEST(FieldOdomTransformTest, RotatesOdomPoseByAnchorYaw)
{
  const FieldOdomTransform transform(Pose2D{0.0, 0.0, kPi / 2.0});

  const Pose2D field_pose = transform.odomToField(Pose2D{1.0, 0.0, 0.0});

  EXPECT_NEAR(field_pose.x, 0.0, 1e-9);
  EXPECT_NEAR(field_pose.y, 1.0, 1e-9);
  EXPECT_NEAR(field_pose.yaw, kPi / 2.0, 1e-9);
}

// fieldToBase and the class's baseToField must be exact inverses.
TEST(FieldOdomTransformTest, FieldToBaseAndBaseToFieldAreInverses)
{
  const FieldOdomTransform transform(Pose2D{0.2, -0.1, -0.3});
  const Pose2D robot_field_pose = transform.odomToField(Pose2D{0.7, -0.4, 0.8});
  const Point2D field_point{1.8, -0.2};

  const Point2D base_point = transform.fieldToBase(field_point, robot_field_pose);
  const Point2D reconstructed = transform.baseToField(base_point, robot_field_pose);

  EXPECT_NEAR(reconstructed.x, field_point.x, 1e-9);
  EXPECT_NEAR(reconstructed.y, field_point.y, 1e-9);
}

// Direct numerical check of fieldToBase: robot at (1,1) facing +y (yaw=pi/2),
// a field point 1m ahead in +x is on the robot's right -> base (0, -1).
TEST(FieldOdomTransformTest, FieldToBaseProducesExpectedBaseCoordinates)
{
  const FieldOdomTransform transform(Pose2D{0.0, 0.0, 0.0});
  const Pose2D robot_field_pose{1.0, 1.0, kPi / 2.0};

  const Point2D base_point = transform.fieldToBase(Point2D{2.0, 1.0}, robot_field_pose);

  EXPECT_NEAR(base_point.x, 0.0, 1e-9);
  EXPECT_NEAR(base_point.y, -1.0, 1e-9);
}

// odomToField yaw must be wrapped to (-pi, pi] even when anchor.yaw + odom.yaw
// leaves that range in either direction.
TEST(FieldOdomTransformTest, OutputYawIsNormalizedAcrossWraps)
{
  const double two_pi = 2.0 * kPi;

  const FieldOdomTransform positive(Pose2D{0.0, 0.0, 3.0});
  EXPECT_NEAR(positive.odomToField(Pose2D{0.0, 0.0, 3.0}).yaw, 6.0 - two_pi, 1e-9);

  const FieldOdomTransform negative(Pose2D{0.0, 0.0, -2.0});
  EXPECT_NEAR(negative.odomToField(Pose2D{0.0, 0.0, -2.0}).yaw, -4.0 + two_pi, 1e-9);

  // Result always lands within [-pi, pi].
  const FieldOdomTransform spun(Pose2D{0.0, 0.0, 0.0});
  for (double y = -4.0 * kPi; y <= 4.0 * kPi; y += 0.37) {
    const double wrapped = spun.odomToField(Pose2D{0.0, 0.0, y}).yaw;
    EXPECT_LE(wrapped, kPi + 1e-9);
    EXPECT_GE(wrapped, -kPi - 1e-9);
  }
}

// The anchor yaw itself is normalized at construction.
TEST(FieldOdomTransformTest, AnchorYawIsNormalizedAtConstruction)
{
  const FieldOdomTransform transform(Pose2D{0.0, 0.0, 4.0});
  EXPECT_NEAR(transform.fieldAnchor().yaw, 4.0 - 2.0 * kPi, 1e-9);
}

}  // namespace mycar_navigation::nav_core
