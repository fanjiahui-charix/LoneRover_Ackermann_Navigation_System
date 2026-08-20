#ifndef MYCAR_NAVIGATION_PLANNER_LOCAL_GRID_HPP_
#define MYCAR_NAVIGATION_PLANNER_LOCAL_GRID_HPP_

#include <vector>

#include "mycar_navigation/nav_core/field_odom_transform.hpp"
#include "mycar_navigation/nav_core/map_mask.hpp"
#include "mycar_navigation/nav_core/types.hpp"

namespace mycar_navigation::planner
{

using nav_core::GridIndex;
using nav_core::MapClass;
using nav_core::Point2D;
using nav_core::Pose2D;

class LocalGrid
{
public:
  LocalGrid(double resolution, double half_extent_m);

  void clear();
  void addScanPoints(const std::vector<Point2D> & pts_base, double inflation_radius);
  void addMapForbidden(
    const nav_core::MapMask & map, const nav_core::FieldOdomTransform & tf,
    const Pose2D & robot_field_pose, bool treat_unknown_as_forbidden);
  void addMapLayers(
    const nav_core::MapMask & map, const nav_core::FieldOdomTransform & tf,
    const Pose2D & robot_field_pose, bool treat_unknown_as_forbidden);

  bool inBounds(const GridIndex & idx) const;
  bool isScanOccupied(const GridIndex & idx) const;
  bool isMapForbidden(const GridIndex & idx) const;
  bool isMapSoftCost(const GridIndex & idx) const;
  double resolution() const;
  int halfCells() const;

private:
  std::size_t flatIndex(const GridIndex & idx) const;
  Point2D cellCenterBase(const GridIndex & idx) const;

  double resolution_;
  double half_extent_m_;
  int half_cells_;
  int side_cells_;
  std::vector<uint8_t> scan_occupied_;
  std::vector<uint8_t> map_forbidden_;
  std::vector<uint8_t> map_soft_cost_;
};

}  // namespace mycar_navigation::planner

#endif  // MYCAR_NAVIGATION_PLANNER_LOCAL_GRID_HPP_
