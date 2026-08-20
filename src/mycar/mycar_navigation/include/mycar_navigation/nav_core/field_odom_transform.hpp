#ifndef MYCAR_NAVIGATION_NAV_CORE_FIELD_ODOM_TRANSFORM_HPP_
#define MYCAR_NAVIGATION_NAV_CORE_FIELD_ODOM_TRANSFORM_HPP_

#include "mycar_navigation/nav_core/types.hpp"

namespace mycar_navigation::nav_core
{

class FieldOdomTransform
{
public:
  explicit FieldOdomTransform(const Pose2D & field_anchor);

  Pose2D odomToField(const Pose2D & odom_pose) const;
  Point2D fieldToBase(const Point2D & field_point, const Pose2D & robot_field_pose) const;
  // Inverse of fieldToBase: a point expressed in the robot base frame back to
  // the field frame. Used to classify local-grid cells against the static map.
  Point2D baseToField(const Point2D & base_point, const Pose2D & robot_field_pose) const;

  const Pose2D & fieldAnchor() const;

private:
  static double normalizeAngle(double angle);

  Pose2D field_anchor_;
};

}  // namespace mycar_navigation::nav_core

#endif  // MYCAR_NAVIGATION_NAV_CORE_FIELD_ODOM_TRANSFORM_HPP_
