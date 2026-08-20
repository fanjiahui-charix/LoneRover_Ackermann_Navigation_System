#include "mycar_navigation/nav_core/field_odom_transform.hpp"

#include <cmath>

namespace mycar_navigation::nav_core
{

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;
}  // namespace

FieldOdomTransform::FieldOdomTransform(const Pose2D & field_anchor)
: field_anchor_(field_anchor)
{
  field_anchor_.yaw = normalizeAngle(field_anchor_.yaw);
}

Pose2D FieldOdomTransform::odomToField(const Pose2D & odom_pose) const
{
  const double cos_yaw = std::cos(field_anchor_.yaw);
  const double sin_yaw = std::sin(field_anchor_.yaw);

  Pose2D field_pose;
  field_pose.x = field_anchor_.x + odom_pose.x * cos_yaw - odom_pose.y * sin_yaw;
  field_pose.y = field_anchor_.y + odom_pose.x * sin_yaw + odom_pose.y * cos_yaw;
  field_pose.yaw = normalizeAngle(field_anchor_.yaw + odom_pose.yaw);
  return field_pose;
}

Point2D FieldOdomTransform::fieldToBase(
  const Point2D & field_point,
  const Pose2D & robot_field_pose) const
{
  const double dx = field_point.x - robot_field_pose.x;
  const double dy = field_point.y - robot_field_pose.y;
  const double cos_yaw = std::cos(robot_field_pose.yaw);
  const double sin_yaw = std::sin(robot_field_pose.yaw);

  Point2D base_point;
  base_point.x = dx * cos_yaw + dy * sin_yaw;
  base_point.y = -dx * sin_yaw + dy * cos_yaw;
  return base_point;
}

Point2D FieldOdomTransform::baseToField(
  const Point2D & base_point,
  const Pose2D & robot_field_pose) const
{
  const double cos_yaw = std::cos(robot_field_pose.yaw);
  const double sin_yaw = std::sin(robot_field_pose.yaw);

  Point2D field_point;
  field_point.x = robot_field_pose.x + base_point.x * cos_yaw - base_point.y * sin_yaw;
  field_point.y = robot_field_pose.y + base_point.x * sin_yaw + base_point.y * cos_yaw;
  return field_point;
}

const Pose2D & FieldOdomTransform::fieldAnchor() const
{
  return field_anchor_;
}

double FieldOdomTransform::normalizeAngle(double angle)
{
  const double wrapped = std::fmod(angle + kPi, kTwoPi);
  if (wrapped < 0.0) {
    return wrapped + kPi;
  }
  return wrapped - kPi;
}

}  // namespace mycar_navigation::nav_core
