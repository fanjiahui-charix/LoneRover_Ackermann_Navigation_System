#include "association.hpp"

#include <cmath>

#include "local_landmark_map.hpp"

namespace simple_lidar_odom
{

int associateCone(const Point & scan_world, const std::vector<ConeEntry> & map, double max_dist2)
{
  int best_index = -1;
  double best_dist2 = max_dist2;

  for (std::size_t i = 0; i < map.size(); ++i) {
    const double dist2 = (scan_world - map[i].pos).squaredNorm();
    if (dist2 <= best_dist2) {
      best_dist2 = dist2;
      best_index = static_cast<int>(i);
    }
  }

  return best_index;
}

int associateFenceLine(
  const Point & normal_world, double d_world, const std::vector<FenceEntry> & map,
  double angle_tol, double dist_tol)
{
  int best_index = -1;
  double best_angle = angle_tol;

  for (std::size_t i = 0; i < map.size(); ++i) {
    const Point & map_normal = map[i].normal;
    if (map_normal.dot(normal_world) <= 0.0) {
      continue;
    }

    const double sin_delta = std::abs(
      map_normal.x() * normal_world.y() - map_normal.y() * normal_world.x());
    if (sin_delta >= angle_tol) {
      continue;
    }
    if (std::abs(map[i].d - d_world) >= dist_tol) {
      continue;
    }
    if (sin_delta < best_angle) {
      best_angle = sin_delta;
      best_index = static_cast<int>(i);
    }
  }

  return best_index;
}

}  // namespace simple_lidar_odom
