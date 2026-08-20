#include "simple_lidar_odom/persistent_cone_map.hpp"
#include "simple_lidar_odom/unique_assignment.hpp"

#include <algorithm>
#include <cmath>

namespace simple_lidar_odom
{

void PersistentConeMap::configure(
  double assoc_dist,
  int confirm_hits,
  int prune_misses,
  int confirmed_min_reobserve_hits,
  int confirmed_prune_misses,
  std::size_t max_cones)
{
  assoc_dist2_ = assoc_dist * assoc_dist;
  confirm_hits_ = std::max(1, confirm_hits);
  prune_misses_ = std::max(0, prune_misses);
  confirmed_min_reobserve_hits_ = std::max(0, confirmed_min_reobserve_hits);
  confirmed_prune_misses_ = std::max(0, confirmed_prune_misses);
  max_cones_ = std::max<std::size_t>(1, max_cones);
}

void PersistentConeMap::update(const std::vector<Point2> & cones_odom, int frame_idx)
{
  // Associate only against cones that existed at frame start, so two distinct
  // observations in the same sweep can't be merged into a cone created moments
  // earlier this frame.
  const std::size_t n_before = cones_.size();

  std::vector<Point2> existing_positions;
  existing_positions.reserve(n_before);
  for (std::size_t i = 0; i < n_before; ++i) {
    existing_positions.push_back(cones_[i].pos);
  }
  const auto matches = greedyUniqueAssignment(
    cones_odom, existing_positions, std::sqrt(assoc_dist2_));
  std::vector<bool> observation_matched(cones_odom.size(), false);

  for (const auto & match : matches) {
    const Point2 & obs = cones_odom[match.observation_index];
    MapCone & c = cones_[match.landmark_index];
    observation_matched[match.observation_index] = true;
    if (c.state == ConeState::Tentative) {
      // Running mean denoise, then promote+freeze on reaching confirm_hits.
      c.pos += (obs - c.pos) / static_cast<double>(c.hits + 1);
      ++c.hits;
      if (c.hits >= confirm_hits_) {
        c.state = ConeState::Confirmed;  // pos frozen from here on
        c.confirmed_hits = 0;
      }
    } else {
      ++c.hits;  // Confirmed: count it, but position stays frozen
      ++c.confirmed_hits;
    }
    c.last_seen_frame = frame_idx;
  }

  for (std::size_t observation = 0; observation < cones_odom.size(); ++observation) {
    if (observation_matched[observation] || cones_.size() >= max_cones_) {
      continue;
    }
    // A second observation that competed for the same existing anchor is a
    // duplicate for this frame, not a new map cone.
    bool near_existing = false;
    for (const auto & position : existing_positions) {
      if ((cones_odom[observation] - position).squaredNorm() <= assoc_dist2_) {
        near_existing = true;
        break;
      }
    }
    if (near_existing) {
      continue;
    }
    const Point2 & obs = cones_odom[observation];
    MapCone nc;
    nc.pos = obs;
    nc.hits = 1;
    nc.last_seen_frame = frame_idx;
    nc.state = (1 >= confirm_hits_) ? ConeState::Confirmed : ConeState::Tentative;
    nc.confirmed_hits = (nc.state == ConeState::Confirmed) ? 1 : 0;
    cones_.push_back(nc);
  }

  // Drop stale tentatives. Confirmed anchors are kept by default; if explicitly
  // enabled, only weakly reobserved confirmed anchors can be pruned.
  const int prune = prune_misses_;
  const int confirmed_prune = confirmed_prune_misses_;
  const int min_reobserve = confirmed_min_reobserve_hits_;
  cones_.erase(
    std::remove_if(
      cones_.begin(), cones_.end(),
      [frame_idx, prune, confirmed_prune, min_reobserve](const MapCone & c) {
        if (c.state == ConeState::Tentative) {
          return (frame_idx - c.last_seen_frame) > prune;
        }
        return confirmed_prune > 0 &&
          c.confirmed_hits < min_reobserve &&
          (frame_idx - c.last_seen_frame) > confirmed_prune;
      }),
    cones_.end());
}

}  // namespace simple_lidar_odom
