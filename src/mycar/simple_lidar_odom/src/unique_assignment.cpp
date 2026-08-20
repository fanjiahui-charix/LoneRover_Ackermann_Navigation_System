#include "simple_lidar_odom/unique_assignment.hpp"

#include <algorithm>
#include <cmath>

namespace simple_lidar_odom
{

std::vector<UniqueMatch> greedyUniqueAssignment(
  const std::vector<Point2> & observations,
  const std::vector<Point2> & landmarks,
  double max_distance)
{
  std::vector<UniqueMatch> candidates;
  if (!std::isfinite(max_distance) || max_distance <= 0.0) {
    return candidates;
  }

  const double max_distance_squared = max_distance * max_distance;
  for (std::size_t observation = 0; observation < observations.size(); ++observation) {
    for (std::size_t landmark = 0; landmark < landmarks.size(); ++landmark) {
      const double distance_squared =
        (observations[observation] - landmarks[landmark]).squaredNorm();
      if (std::isfinite(distance_squared) && distance_squared <= max_distance_squared) {
        candidates.push_back({observation, landmark, distance_squared});
      }
    }
  }

  std::sort(
    candidates.begin(), candidates.end(),
    [](const UniqueMatch & lhs, const UniqueMatch & rhs) {
      if (lhs.squared_distance != rhs.squared_distance) {
        return lhs.squared_distance < rhs.squared_distance;
      }
      if (lhs.observation_index != rhs.observation_index) {
        return lhs.observation_index < rhs.observation_index;
      }
      return lhs.landmark_index < rhs.landmark_index;
    });

  std::vector<bool> observation_used(observations.size(), false);
  std::vector<bool> landmark_used(landmarks.size(), false);
  std::vector<UniqueMatch> matches;
  matches.reserve(std::min(observations.size(), landmarks.size()));
  for (const auto & candidate : candidates) {
    if (observation_used[candidate.observation_index] ||
      landmark_used[candidate.landmark_index])
    {
      continue;
    }
    observation_used[candidate.observation_index] = true;
    landmark_used[candidate.landmark_index] = true;
    matches.push_back(candidate);
  }
  return matches;
}

}  // namespace simple_lidar_odom
