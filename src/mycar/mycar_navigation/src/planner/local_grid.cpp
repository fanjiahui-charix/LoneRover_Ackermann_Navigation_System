#include "mycar_navigation/planner/local_grid.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace mycar_navigation::planner
{
namespace
{

bool shouldMarkForbidden(MapClass klass, bool treat_unknown_as_forbidden)
{
  return klass == MapClass::HARD_FORBIDDEN ||
         (treat_unknown_as_forbidden && klass == MapClass::UNKNOWN);
}

}  // namespace

LocalGrid::LocalGrid(double resolution, double half_extent_m)
: resolution_(resolution),
  half_extent_m_(half_extent_m),
  half_cells_(0),
  side_cells_(0)
{
  if (!(resolution_ > 0.0)) {
    throw std::invalid_argument("LocalGrid resolution must be positive");
  }
  if (!(half_extent_m_ > 0.0)) {
    throw std::invalid_argument("LocalGrid half extent must be positive");
  }

  half_cells_ = static_cast<int>(std::floor(half_extent_m_ / resolution_));
  side_cells_ = 2 * half_cells_ + 1;
  const std::size_t cell_count = static_cast<std::size_t>(side_cells_) * static_cast<std::size_t>(side_cells_);
  scan_occupied_.assign(cell_count, 0U);
  map_forbidden_.assign(cell_count, 0U);
  map_soft_cost_.assign(cell_count, 0U);
}

void LocalGrid::clear()
{
  std::fill(scan_occupied_.begin(), scan_occupied_.end(), 0U);
  std::fill(map_forbidden_.begin(), map_forbidden_.end(), 0U);
  std::fill(map_soft_cost_.begin(), map_soft_cost_.end(), 0U);
}

void LocalGrid::addScanPoints(const std::vector<Point2D> & pts_base, double inflation_radius)
{
  if (inflation_radius < 0.0) {
    throw std::invalid_argument("LocalGrid inflation radius must be non-negative");
  }

  const int inflation_cells = static_cast<int>(std::ceil(inflation_radius / resolution_));
  const double inflation_radius_sq = inflation_radius * inflation_radius;

  for (const Point2D & point : pts_base) {
    const GridIndex center{
      static_cast<int>(std::floor(point.x / resolution_)),
      static_cast<int>(std::floor(point.y / resolution_))};
    for (int dj = -inflation_cells; dj <= inflation_cells; ++dj) {
      for (int di = -inflation_cells; di <= inflation_cells; ++di) {
        const GridIndex idx{center.i + di, center.j + dj};
        if (!inBounds(idx)) {
          continue;
        }
        const Point2D cell_center = cellCenterBase(idx);
        const double dx = cell_center.x - point.x;
        const double dy = cell_center.y - point.y;
        if (dx * dx + dy * dy <= inflation_radius_sq + 1e-12 || inflation_cells == 0) {
          scan_occupied_[flatIndex(idx)] = 1U;
        }
      }
    }
  }
}

void LocalGrid::addMapForbidden(
  const nav_core::MapMask & map, const nav_core::FieldOdomTransform & tf,
  const Pose2D & robot_field_pose, bool treat_unknown_as_forbidden)
{
  addMapLayers(map, tf, robot_field_pose, treat_unknown_as_forbidden);
}

void LocalGrid::addMapLayers(
  const nav_core::MapMask & map, const nav_core::FieldOdomTransform & tf,
  const Pose2D & robot_field_pose, bool treat_unknown_as_forbidden)
{
  const double sample_step = std::min(resolution_, std::max(1e-6, map.resolution()));
  const int subdivisions = std::max(1, static_cast<int>(std::ceil(resolution_ / sample_step)));
  const double offset_step = resolution_ / static_cast<double>(subdivisions);
  const double start_offset = -0.5 * resolution_ + 0.5 * offset_step;

  for (int j = -half_cells_; j <= half_cells_; ++j) {
    for (int i = -half_cells_; i <= half_cells_; ++i) {
      const GridIndex idx{i, j};
      bool forbidden = false;
      bool soft_cost = false;
      for (int sy = 0; sy < subdivisions && !forbidden; ++sy) {
        for (int sx = 0; sx < subdivisions; ++sx) {
          const Point2D sample_base{
            static_cast<double>(i) * resolution_ + start_offset + static_cast<double>(sx) * offset_step +
              0.5 * resolution_,
            static_cast<double>(j) * resolution_ + start_offset + static_cast<double>(sy) * offset_step +
              0.5 * resolution_};
          const Point2D sample_field = tf.baseToField(sample_base, robot_field_pose);
          const MapClass klass = map.classifyWorld(sample_field);
          if (shouldMarkForbidden(klass, treat_unknown_as_forbidden)) {
            forbidden = true;
            break;
          }
          soft_cost = soft_cost || klass == MapClass::SOFT_COST;
        }
      }
      if (forbidden) {
        map_forbidden_[flatIndex(idx)] = 1U;
      } else if (soft_cost) {
        map_soft_cost_[flatIndex(idx)] = 1U;
      }
    }
  }
}

bool LocalGrid::inBounds(const GridIndex & idx) const
{
  return idx.i >= -half_cells_ && idx.i <= half_cells_ && idx.j >= -half_cells_ && idx.j <= half_cells_;
}

bool LocalGrid::isScanOccupied(const GridIndex & idx) const
{
  if (!inBounds(idx)) {
    return false;
  }
  return scan_occupied_[flatIndex(idx)] != 0U;
}

bool LocalGrid::isMapForbidden(const GridIndex & idx) const
{
  if (!inBounds(idx)) {
    return true;
  }
  return map_forbidden_[flatIndex(idx)] != 0U;
}

bool LocalGrid::isMapSoftCost(const GridIndex & idx) const
{
  if (!inBounds(idx)) {
    return false;
  }
  return map_soft_cost_[flatIndex(idx)] != 0U;
}

double LocalGrid::resolution() const
{
  return resolution_;
}

int LocalGrid::halfCells() const
{
  return half_cells_;
}

std::size_t LocalGrid::flatIndex(const GridIndex & idx) const
{
  return static_cast<std::size_t>(idx.j + half_cells_) * static_cast<std::size_t>(side_cells_) +
         static_cast<std::size_t>(idx.i + half_cells_);
}

Point2D LocalGrid::cellCenterBase(const GridIndex & idx) const
{
  return Point2D{
    static_cast<double>(idx.i) * resolution_ + 0.5 * resolution_,
    static_cast<double>(idx.j) * resolution_ + 0.5 * resolution_};
}

}  // namespace mycar_navigation::planner
