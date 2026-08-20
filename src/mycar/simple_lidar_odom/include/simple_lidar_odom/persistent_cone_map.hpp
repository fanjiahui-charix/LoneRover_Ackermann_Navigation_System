#ifndef SIMPLE_LIDAR_ODOM_PERSISTENT_CONE_MAP_HPP_
#define SIMPLE_LIDAR_ODOM_PERSISTENT_CONE_MAP_HPP_

#include <vector>

#include <Eigen/Dense>

namespace simple_lidar_odom
{

// Same alias as landmark_extractor.hpp; an identical re-declaration in the same
// namespace is legal C++ and keeps this unit ROS-free (landmark_extractor.hpp
// pulls in sensor_msgs).
using Point2 = Eigen::Vector2d;

enum class ConeState { Tentative, Confirmed };

struct MapCone
{
  Point2 pos{Point2::Zero()};            // odom frame; frozen once Confirmed
  ConeState state{ConeState::Tentative};
  int hits{0};                           // observation count == running-mean N
  int confirmed_hits{0};                 // reobservations after confirmation
  int last_seen_frame{0};
};

// Persistent confirmed-cone world map. While Tentative a cone's position is the
// running mean of its observations; on reaching confirm_hits it becomes
// Confirmed and its position is frozen as a rigid anchor. Confirmed anchors are
// persistent by default (`confirmed_prune_misses=0`), but optional weak-anchor
// pruning remains available. Stale tentatives are dropped as noise.
class PersistentConeMap
{
public:
  void configure(
    double assoc_dist,
    int confirm_hits,
    int prune_misses,
    int confirmed_min_reobserve_hits = 2,
    int confirmed_prune_misses = 0,
    std::size_t max_cones = 30);

  // cones_odom: this frame's cone centroids already transformed into odom using
  // the accepted pose. frame_idx: a monotonic accepted-frame counter.
  void update(const std::vector<Point2> & cones_odom, int frame_idx);

  const std::vector<MapCone> & cones() const { return cones_; }
  void clear() { cones_.clear(); }

private:
  double assoc_dist2_{0.09};   // 0.30^2
  int confirm_hits_{5};
  int prune_misses_{10};
  int confirmed_min_reobserve_hits_{2};
  int confirmed_prune_misses_{0};
  std::size_t max_cones_{30};
  std::vector<MapCone> cones_;
};

}  // namespace simple_lidar_odom
#endif  // SIMPLE_LIDAR_ODOM_PERSISTENT_CONE_MAP_HPP_
