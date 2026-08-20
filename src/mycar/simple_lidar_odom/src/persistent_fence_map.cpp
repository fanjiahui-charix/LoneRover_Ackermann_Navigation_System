#include "simple_lidar_odom/persistent_fence_map.hpp"

#include <algorithm>
#include <cmath>

namespace simple_lidar_odom
{

void canonicalizeLine(Point2 & n, double & d)
{
  constexpr double kEps = 1e-6;
  if (d > kEps) {
    return;  // already on the d>0 half
  }
  if (d < -kEps) {
    n = -n;
    d = -d;
    return;
  }
  // |d| ~ 0: line through (near) the origin; the sign can't be fixed by d, so
  // orient deterministically by the normal direction (n.x() first, then n.y()).
  d = 0.0;
  if (n.x() > kEps) {
    return;
  }
  if (n.x() < -kEps) {
    n = -n;
    return;
  }
  if (n.y() < 0.0) {
    n = -n;
  }
}

void PersistentFenceMap::configure(double assoc_angle_tol, double assoc_dist_tol,
                                   int confirm_hits, int prune_misses)
{
  assoc_angle_tol_ = assoc_angle_tol;
  assoc_dist_tol_ = assoc_dist_tol;
  confirm_hits_ = std::max(1, confirm_hits);
  prune_misses_ = std::max(0, prune_misses);
}

void PersistentFenceMap::update(const std::vector<LineObs> & lines_odom, int frame_idx)
{
  // Associate only against fences that existed at frame start (n_before), so two
  // observations in the same sweep can't merge into a fence created this frame.
  const std::size_t n_before = fences_.size();

  for (LineObs obs : lines_odom) {
    canonicalizeLine(obs.normal, obs.d);   // single source of truth for line sign

    int best = -1;
    double best_dist = assoc_dist_tol_;
    for (std::size_t i = 0; i < n_before; ++i) {
      const MapFence & f = fences_[i];
      // Gate 1: same-facing. Separates anti-parallel opposite walls that the
      // angle gate alone (|sin|~0, looks parallel) would wrongly merge.
      if (obs.normal.dot(f.normal) <= 0.0) {
        continue;
      }
      // Gate 2: angle, via |sin| = |cross| between unit normals.
      const double sin_ang = std::abs(obs.normal.x() * f.normal.y() -
                                      obs.normal.y() * f.normal.x());
      if (sin_ang >= assoc_angle_tol_) {
        continue;
      }
      // Gate 3: perpendicular distance; keep the closest among candidates so
      // same-angle parallel walls are split by d.
      const double dd = std::abs(obs.d - f.d);
      if (dd <= best_dist) {
        best_dist = dd;
        best = static_cast<int>(i);
      }
    }

    if (best >= 0) {
      MapFence & f = fences_[static_cast<std::size_t>(best)];
      if (f.state == FenceState::Tentative) {
        // Running-mean denoise of the line params, then renormalize so |normal|=1
        // and d scales by the same factor (stays the same physical line).
        f.normal += (obs.normal - f.normal) / static_cast<double>(f.hits + 1);
        f.d += (obs.d - f.d) / static_cast<double>(f.hits + 1);
        const double len = f.normal.norm();
        f.normal /= len;
        f.d /= len;
        ++f.hits;
        if (f.hits >= confirm_hits_) {
          f.state = FenceState::Confirmed;  // (normal,d) frozen from here on
        }
      } else {
        ++f.hits;  // Confirmed: count it, but parameters stay frozen
      }
      f.last_seen_frame = frame_idx;
    } else {
      MapFence nf;
      nf.normal = obs.normal;
      nf.d = obs.d;
      nf.hits = 1;
      nf.last_seen_frame = frame_idx;
      nf.state = (1 >= confirm_hits_) ? FenceState::Confirmed : FenceState::Tentative;
      fences_.push_back(nf);
    }
  }

  // Drop stale tentatives; confirmed anchors are never pruned.
  const int prune = prune_misses_;
  fences_.erase(
    std::remove_if(
      fences_.begin(), fences_.end(),
      [frame_idx, prune](const MapFence & f) {
        return f.state == FenceState::Tentative && (frame_idx - f.last_seen_frame) > prune;
      }),
    fences_.end());
}

}  // namespace simple_lidar_odom
