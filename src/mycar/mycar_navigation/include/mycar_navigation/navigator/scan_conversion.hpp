#ifndef MYCAR_NAVIGATION_NAVIGATOR_SCAN_CONVERSION_HPP_
#define MYCAR_NAVIGATION_NAVIGATOR_SCAN_CONVERSION_HPP_

#include <vector>

#include "mycar_navigation/nav_core/types.hpp"

namespace mycar_navigation::navigator
{

using nav_core::Point2D;
using nav_core::Pose2D;

// Convert one LaserScan (already unpacked into a ranges array) into base-frame
// Cartesian points, applying the laser's mounting pose in base_link.
// - ray k has angle = angle_min + k * angle_increment (laser frame).
// - NaN/inf ranges and ranges outside [range_min, range_max] are dropped.
// - laser_pose_in_base {x,y,yaw}: laser origin relative to base_link (default 0;
//   fill from static TF / measurement on the vehicle).
// Pure function — no ROS dependency.
std::vector<Point2D> laserScanToBasePoints(
  const std::vector<float> & ranges, double angle_min, double angle_increment,
  double range_min, double range_max, const Pose2D & laser_pose_in_base);

}  // namespace mycar_navigation::navigator

#endif  // MYCAR_NAVIGATION_NAVIGATOR_SCAN_CONVERSION_HPP_
