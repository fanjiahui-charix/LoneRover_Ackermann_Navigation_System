#pragma once

#include <vector>

#include <Eigen/Dense>

#include "simple_lidar_odom/landmark_extractor.hpp"

namespace simple_lidar_odom
{

struct ConeEntry
{
  Point2 pos_odom{Point2::Zero()};
  int age{0};
};

struct FenceEntry
{
  Point2 normal{Point2::UnitX()};
  double d{0.0};
  Point2 centroid_odom{Point2::Zero()};
  PointList support_points_odom;
  int age{0};
};

class LocalLandmarkMap
{
public:
  explicit LocalLandmarkMap(int max_age = 30);

  void setMaxAge(int max_age);

  void addFrame(
    const PointList & cone_centers_odom,
    const std::vector<LineLandmark> & fences_odom);

  void evict();
  void clear();

  const std::vector<ConeEntry> & cones() const;
  const std::vector<FenceEntry> & fences() const;

private:
  int max_age_{30};
  std::vector<ConeEntry> cones_;
  std::vector<FenceEntry> fences_;
};

}  // namespace simple_lidar_odom
