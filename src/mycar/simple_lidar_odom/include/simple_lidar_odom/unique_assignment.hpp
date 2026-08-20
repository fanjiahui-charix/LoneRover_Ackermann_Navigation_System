#ifndef SIMPLE_LIDAR_ODOM__UNIQUE_ASSIGNMENT_HPP_
#define SIMPLE_LIDAR_ODOM__UNIQUE_ASSIGNMENT_HPP_

#include <cstddef>
#include <vector>

#include <Eigen/Dense>

namespace simple_lidar_odom
{

using Point2 = Eigen::Vector2d;

struct UniqueMatch
{
  std::size_t observation_index{0};
  std::size_t landmark_index{0};
  double squared_distance{0.0};
};

// Deterministic nearest-first assignment. Each observation and landmark can
// appear at most once, so correspondence count cannot be inflated by duplicate
// observations selecting the same map landmark.
std::vector<UniqueMatch> greedyUniqueAssignment(
  const std::vector<Point2> & observations,
  const std::vector<Point2> & landmarks,
  double max_distance);

}  // namespace simple_lidar_odom

#endif  // SIMPLE_LIDAR_ODOM__UNIQUE_ASSIGNMENT_HPP_
