#ifndef MYCAR_NAVIGATION_CLUSTERING_CLUSTER_EXTRACTOR_HPP_
#define MYCAR_NAVIGATION_CLUSTERING_CLUSTER_EXTRACTOR_HPP_

#include <vector>

#include "mycar_navigation/nav_core/types.hpp"

namespace mycar_navigation::clustering
{
using nav_core::Point2D;

struct ClusterConfig
{
  double cluster_tolerance = 0.10;
  int min_points = 2;
  int max_points = 60;
  double max_cluster_span = 0.40;
  double max_range = 6.0;
};

struct Cluster
{
  Point2D centroid;
  int point_count;
  double span;
};

std::vector<Cluster> extractClusters(
  const std::vector<Point2D> & pts_base,
  const ClusterConfig & cfg);

}  // namespace mycar_navigation::clustering

#endif  // MYCAR_NAVIGATION_CLUSTERING_CLUSTER_EXTRACTOR_HPP_
