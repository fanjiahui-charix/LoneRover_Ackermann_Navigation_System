#ifndef MYCAR_NAVIGATION_NAV_CORE_FOOTPRINT_HPP_
#define MYCAR_NAVIGATION_NAV_CORE_FOOTPRINT_HPP_

#include <vector>

#include "mycar_navigation/nav_core/types.hpp"

namespace mycar_navigation::nav_core
{

class Footprint
{
public:
  Footprint();
  Footprint(double half_length, double half_width);

  double halfLength() const noexcept;
  double halfWidth() const noexcept;

  // Returns every grid cell whose square area is intersected by the rotated
  // rectangular footprint (conservative: a cell the body merely clips at a
  // corner is included). This is the collision primitive for spec §4.5 — it
  // must never under-report a cell the body overlaps.
  std::vector<GridIndex> coveredCells(const Pose2D & pose_in_world, double resolution) const;

private:
  // Separating Axis Theorem test between the rotated footprint rectangle and an
  // axis-aligned cell square (half size = resolution/2 centred on cell_center).
  bool intersectsCell(
    const Point2D & cell_center, double cell_half, const Pose2D & pose_in_world,
    double cos_yaw, double sin_yaw) const noexcept;

  double half_length_;
  double half_width_;
};

}  // namespace mycar_navigation::nav_core

#endif  // MYCAR_NAVIGATION_NAV_CORE_FOOTPRINT_HPP_
