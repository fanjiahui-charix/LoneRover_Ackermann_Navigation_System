#ifndef MYCAR_NAVIGATION_CLUSTERING_CONE_MEMORY_HPP_
#define MYCAR_NAVIGATION_CLUSTERING_CONE_MEMORY_HPP_

#include <cstddef>
#include <vector>

#include "mycar_navigation/nav_core/types.hpp"

namespace mycar_navigation::clustering
{
using nav_core::Point2D;

struct MemoryConfig
{
  int confirm_observations = 2;
  double ttl_sec = 5.0;
  double match_distance = 0.20;
};

struct ConeTrack
{
  Point2D centroid_field;
  int observations;
  double last_seen_sec;
  bool confirmed;
};

class ConeMemory
{
public:
  explicit ConeMemory(const MemoryConfig & cfg);

  void update(const std::vector<Point2D> & cone_centroids_field, double now_sec);
  void decay(double now_sec);
  void clear();
  std::vector<Point2D> confirmedCones() const;
  std::size_t trackCount() const;

private:
  MemoryConfig cfg_;
  std::vector<ConeTrack> tracks_;
};

}  // namespace mycar_navigation::clustering

#endif  // MYCAR_NAVIGATION_CLUSTERING_CONE_MEMORY_HPP_
