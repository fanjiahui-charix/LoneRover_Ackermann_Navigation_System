#include "simple_lidar_odom/local_landmark_map.hpp"

#include <algorithm>

namespace simple_lidar_odom
{

LocalLandmarkMap::LocalLandmarkMap(int max_age)
: max_age_(std::max(1, max_age))
{
}

void LocalLandmarkMap::setMaxAge(int max_age)
{
  max_age_ = std::max(1, max_age);
}

void LocalLandmarkMap::addFrame(
  const PointList & cone_centers_odom,
  const std::vector<LineLandmark> & fences_odom)
{
  for (auto & cone : cones_) {
    ++cone.age;
  }
  for (auto & fence : fences_) {
    ++fence.age;
  }

  for (const auto & cone : cone_centers_odom) {
    cones_.push_back(ConeEntry{cone, 0});
  }

  for (const auto & fence : fences_odom) {
    FenceEntry entry;
    entry.normal = fence.normal;
    entry.d = fence.d;
    entry.centroid_odom = fence.centroid;
    entry.support_points_odom = fence.support_points;
    entry.age = 0;
    fences_.push_back(std::move(entry));
  }

  evict();
}

void LocalLandmarkMap::evict()
{
  cones_.erase(
    std::remove_if(
      cones_.begin(), cones_.end(),
      [this](const ConeEntry & entry) {return entry.age > max_age_;}),
    cones_.end());

  fences_.erase(
    std::remove_if(
      fences_.begin(), fences_.end(),
      [this](const FenceEntry & entry) {return entry.age > max_age_;}),
    fences_.end());
}

void LocalLandmarkMap::clear()
{
  cones_.clear();
  fences_.clear();
}

const std::vector<ConeEntry> & LocalLandmarkMap::cones() const
{
  return cones_;
}

const std::vector<FenceEntry> & LocalLandmarkMap::fences() const
{
  return fences_;
}

}  // namespace simple_lidar_odom
