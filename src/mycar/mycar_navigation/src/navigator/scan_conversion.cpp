#include "mycar_navigation/navigator/scan_conversion.hpp"

#include <cmath>

namespace mycar_navigation::navigator
{

std::vector<Point2D> laserScanToBasePoints(
  const std::vector<float> & ranges, double angle_min, double angle_increment,
  double range_min, double range_max, const Pose2D & laser_pose_in_base)
{
  std::vector<Point2D> points;
  points.reserve(ranges.size());

  const double cos_yaw = std::cos(laser_pose_in_base.yaw);
  const double sin_yaw = std::sin(laser_pose_in_base.yaw);

  for (std::size_t k = 0; k < ranges.size(); ++k) {
    const double r = static_cast<double>(ranges[k]);
    if (!std::isfinite(r) || r < range_min || r > range_max) {
      continue;
    }

    const double angle = angle_min + static_cast<double>(k) * angle_increment;
    // Point in the laser frame.
    const double lx = r * std::cos(angle);
    const double ly = r * std::sin(angle);
    // Transform into base_link: rotate by laser yaw, then translate.
    Point2D base_point;
    base_point.x = laser_pose_in_base.x + lx * cos_yaw - ly * sin_yaw;
    base_point.y = laser_pose_in_base.y + lx * sin_yaw + ly * cos_yaw;
    points.push_back(base_point);
  }

  return points;
}

}  // namespace mycar_navigation::navigator
