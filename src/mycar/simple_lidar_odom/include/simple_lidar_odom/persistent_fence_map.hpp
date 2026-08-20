#ifndef SIMPLE_LIDAR_ODOM_PERSISTENT_FENCE_MAP_HPP_
#define SIMPLE_LIDAR_ODOM_PERSISTENT_FENCE_MAP_HPP_

#include <vector>

#include <Eigen/Dense>

namespace simple_lidar_odom
{

// Same alias as landmark_extractor.hpp / persistent_cone_map.hpp; an identical
// re-declaration in the same namespace is legal C++ and keeps this unit
// ROS-free (landmark_extractor.hpp pulls in sensor_msgs).
using Point2 = Eigen::Vector2d;

// Canonicalize an infinite line (normal, d) to the d>=0 half so that the two
// equivalent representations (n,d) and (-n,-d) of the same physical line map to
// one. The extractor's normal sign comes from the min eigenvector and flips
// frame to frame; without this a wall's two-frame observations cancel in the
// running mean. For lines through the origin (|d|~0) we fall back to orienting
// by n.x() then n.y() so the choice is still deterministic.
// Single source of truth: PersistentFenceMap::update calls it on every obs, and
// the node's associateLandmarks calls it on the world-frame observation before
// the association gates, keeping map side and query side sign-consistent.
void canonicalizeLine(Point2 & n, double & d);

enum class FenceState { Tentative, Confirmed };

struct MapFence
{
  Point2 normal{Point2::UnitX()};   // unit normal (odom frame); frozen once Confirmed
  double d{0.0};                    // offset: normal.x = d
  FenceState state{FenceState::Tentative};
  int hits{0};                      // observation count == running-mean N
  int last_seen_frame{0};
};

// One frame's fence observation: an infinite line (normal, d) already
// transformed into the odom frame. Only normal/d are carried (no sensor_msgs).
struct LineObs
{
  Point2 normal{Point2::UnitX()};
  double d{0.0};
};

// Persistent confirmed-fence world map, mirroring PersistentConeMap but with
// infinite-line entries. While Tentative a fence's (normal, d) is the running
// mean of its observations (renormalized to keep the normal unit-length); on
// reaching confirm_hits it becomes Confirmed and its parameters are frozen
// forever (a rigid anchor, never pruned, never moved). Stale Tentatives (unseen
// for > prune_misses frames) are dropped as noise.
class PersistentFenceMap
{
public:
  // assoc_angle_tol: |sin| threshold between unit normals (rad ~= sin for small).
  // assoc_dist_tol: |d_obs - d_map| threshold to fold into the same wall (m).
  void configure(double assoc_angle_tol, double assoc_dist_tol,
                 int confirm_hits, int prune_misses);

  // lines_odom: this frame's fence lines already transformed into odom using the
  // accepted pose. frame_idx: a monotonic accepted-frame counter.
  void update(const std::vector<LineObs> & lines_odom, int frame_idx);

  const std::vector<MapFence> & fences() const { return fences_; }
  void clear() { fences_.clear(); }

private:
  double assoc_angle_tol_{0.10};   // |sin| between normals
  double assoc_dist_tol_{0.30};    // |d_obs - d_map|, m
  int confirm_hits_{5};
  int prune_misses_{10};
  std::vector<MapFence> fences_;
};

}  // namespace simple_lidar_odom
#endif  // SIMPLE_LIDAR_ODOM_PERSISTENT_FENCE_MAP_HPP_
