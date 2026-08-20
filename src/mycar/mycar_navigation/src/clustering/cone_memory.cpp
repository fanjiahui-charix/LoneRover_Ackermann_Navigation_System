#include "mycar_navigation/clustering/cone_memory.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace mycar_navigation::clustering
{
namespace
{
double squaredDistance(const Point2D & lhs, const Point2D & rhs)
{
  const double dx = lhs.x - rhs.x;
  const double dy = lhs.y - rhs.y;
  return dx * dx + dy * dy;
}
}  // namespace

ConeMemory::ConeMemory(const MemoryConfig & cfg)
: cfg_(cfg)
{
}

void ConeMemory::update(const std::vector<Point2D> & cone_centroids_field, double now_sec)
{
  const double match_distance_sq = cfg_.match_distance * cfg_.match_distance;
  std::vector<bool> track_claimed(tracks_.size(), false);

  for (const Point2D & centroid : cone_centroids_field) {
    std::size_t best_index = tracks_.size();
    double best_distance_sq = std::numeric_limits<double>::max();

    for (std::size_t index = 0; index < tracks_.size(); ++index) {
      if (track_claimed[index]) {
        continue;
      }
      const double distance_sq = squaredDistance(tracks_[index].centroid_field, centroid);
      if (distance_sq <= match_distance_sq && distance_sq < best_distance_sq) {
        best_distance_sq = distance_sq;
        best_index = index;
      }
    }

    if (best_index == tracks_.size()) {
      ConeTrack track{};
      track.centroid_field = centroid;
      track.observations = 1;
      track.last_seen_sec = now_sec;
      track.confirmed = (track.observations >= cfg_.confirm_observations);
      tracks_.push_back(track);
      track_claimed.push_back(true);
      continue;
    }

    ConeTrack & track = tracks_[best_index];
    const int next_observations = track.observations + 1;
    const double blend = 1.0 / static_cast<double>(next_observations);
    track.centroid_field.x += (centroid.x - track.centroid_field.x) * blend;
    track.centroid_field.y += (centroid.y - track.centroid_field.y) * blend;
    track.observations = next_observations;
    track.last_seen_sec = now_sec;
    track.confirmed = (track.observations >= cfg_.confirm_observations);
    track_claimed[best_index] = true;
  }
}

void ConeMemory::decay(double now_sec)
{
  tracks_.erase(
    std::remove_if(
      tracks_.begin(), tracks_.end(),
      [&](const ConeTrack & track) {
        return (now_sec - track.last_seen_sec) > cfg_.ttl_sec;
      }),
    tracks_.end());
}

void ConeMemory::clear()
{
  tracks_.clear();
}

std::vector<Point2D> ConeMemory::confirmedCones() const
{
  std::vector<Point2D> cones;
  for (const ConeTrack & track : tracks_) {
    if (track.confirmed) {
      cones.push_back(track.centroid_field);
    }
  }
  return cones;
}

std::size_t ConeMemory::trackCount() const
{
  return tracks_.size();
}

}  // namespace mycar_navigation::clustering
