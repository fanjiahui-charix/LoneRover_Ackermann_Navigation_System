#ifndef MYCAR_NAVIGATION_NAV_CORE_TYPES_HPP_
#define MYCAR_NAVIGATION_NAV_CORE_TYPES_HPP_

#include <cstdint>

namespace mycar_navigation::nav_core
{

struct Point2D
{
  double x{0};
  double y{0};
};

struct Pose2D
{
  double x{0};
  double y{0};
  double yaw{0};
};

struct GridIndex
{
  int i{0};
  int j{0};
};

enum class MapClass : uint8_t
{
  DRIVABLE,
  HARD_FORBIDDEN,
  SOFT_COST,
  UNKNOWN
};

}  // namespace mycar_navigation::nav_core

#endif  // MYCAR_NAVIGATION_NAV_CORE_TYPES_HPP_
