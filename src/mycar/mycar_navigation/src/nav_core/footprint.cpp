#include "mycar_navigation/nav_core/footprint.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace mycar_navigation::nav_core
{
namespace
{
constexpr double kDefaultHalfLength = 0.22;  // 模板待实测
constexpr double kDefaultHalfWidth = 0.14;   // 模板待实测
constexpr double kContainmentEpsilon = 1e-9;

Point2D makeCorner(double local_x, double local_y, const Pose2D & pose, double cos_yaw, double sin_yaw)
{
  return Point2D{
    pose.x + local_x * cos_yaw - local_y * sin_yaw,
    pose.y + local_x * sin_yaw + local_y * cos_yaw};
}
}  // namespace

Footprint::Footprint()
: half_length_(kDefaultHalfLength), half_width_(kDefaultHalfWidth)
{
}

Footprint::Footprint(double half_length, double half_width)
: half_length_(half_length), half_width_(half_width)
{
  if (!(half_length_ > 0.0) || !(half_width_ > 0.0)) {
    throw std::invalid_argument("Footprint half dimensions must be positive");
  }
}

double Footprint::halfLength() const noexcept
{
  return half_length_;
}

double Footprint::halfWidth() const noexcept
{
  return half_width_;
}

std::vector<GridIndex> Footprint::coveredCells(const Pose2D & pose_in_world, double resolution) const
{
  if (!(resolution > 0.0)) {
    throw std::invalid_argument("Footprint resolution must be positive");
  }

  const double cos_yaw = std::cos(pose_in_world.yaw);
  const double sin_yaw = std::sin(pose_in_world.yaw);

  const Point2D corners[4] = {
    makeCorner(half_length_, half_width_, pose_in_world, cos_yaw, sin_yaw),
    makeCorner(half_length_, -half_width_, pose_in_world, cos_yaw, sin_yaw),
    makeCorner(-half_length_, half_width_, pose_in_world, cos_yaw, sin_yaw),
    makeCorner(-half_length_, -half_width_, pose_in_world, cos_yaw, sin_yaw)};

  double min_x = std::numeric_limits<double>::infinity();
  double max_x = -std::numeric_limits<double>::infinity();
  double min_y = std::numeric_limits<double>::infinity();
  double max_y = -std::numeric_limits<double>::infinity();
  for (const Point2D & corner : corners) {
    min_x = std::min(min_x, corner.x);
    max_x = std::max(max_x, corner.x);
    min_y = std::min(min_y, corner.y);
    max_y = std::max(max_y, corner.y);
  }

  // Candidate cells = bounding box of the rotated rectangle, widened by one
  // cell on each side so any cell the rectangle clips is tested (SAT then
  // rejects the ones that do not actually intersect).
  const int min_i = static_cast<int>(std::floor(min_x / resolution)) - 1;
  const int max_i = static_cast<int>(std::floor(max_x / resolution)) + 1;
  const int min_j = static_cast<int>(std::floor(min_y / resolution)) - 1;
  const int max_j = static_cast<int>(std::floor(max_y / resolution)) + 1;

  const double cell_half = 0.5 * resolution;

  std::vector<GridIndex> cells;
  cells.reserve(static_cast<std::size_t>(std::max(0, max_i - min_i + 1) * std::max(0, max_j - min_j + 1)));

  for (int j = min_j; j <= max_j; ++j) {
    for (int i = min_i; i <= max_i; ++i) {
      const Point2D cell_center{
        (static_cast<double>(i) + 0.5) * resolution,
        (static_cast<double>(j) + 0.5) * resolution};
      if (intersectsCell(cell_center, cell_half, pose_in_world, cos_yaw, sin_yaw)) {
        cells.push_back(GridIndex{i, j});
      }
    }
  }

  return cells;
}

bool Footprint::intersectsCell(
  const Point2D & cell_center, double cell_half, const Pose2D & pose_in_world,
  double cos_yaw, double sin_yaw) const noexcept
{
  // Separating Axis Theorem between the rotated footprint rectangle (axes
  // u=(cos,sin), v=(-sin,cos), half-extents half_length_/half_width_) and the
  // axis-aligned cell square (axes (1,0)/(0,1), half-extent cell_half).
  // The two rectangles overlap iff none of the four candidate axes separates
  // them. Conservative: borderline (touching) is treated as overlap.
  const double dx = cell_center.x - pose_in_world.x;
  const double dy = cell_center.y - pose_in_world.y;
  const double abs_cos = std::abs(cos_yaw);
  const double abs_sin = std::abs(sin_yaw);

  // Footprint local axes.
  const double dist_u = std::abs(dx * cos_yaw + dy * sin_yaw);
  if (dist_u > half_length_ + cell_half * (abs_cos + abs_sin) + kContainmentEpsilon) {
    return false;
  }
  const double dist_v = std::abs(-dx * sin_yaw + dy * cos_yaw);
  if (dist_v > half_width_ + cell_half * (abs_sin + abs_cos) + kContainmentEpsilon) {
    return false;
  }

  // World axes (cell aligned). Footprint projection radius onto each.
  if (std::abs(dx) > half_length_ * abs_cos + half_width_ * abs_sin + cell_half + kContainmentEpsilon) {
    return false;
  }
  if (std::abs(dy) > half_length_ * abs_sin + half_width_ * abs_cos + cell_half + kContainmentEpsilon) {
    return false;
  }

  return true;
}

}  // namespace mycar_navigation::nav_core
