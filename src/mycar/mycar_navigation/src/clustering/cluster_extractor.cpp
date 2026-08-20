#include "mycar_navigation/clustering/cluster_extractor.hpp"

#include <cmath>
#include <cstddef>
#include <limits>
#include <queue>
#include <vector>

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

double pointRange(const Point2D & point)
{
  return std::hypot(point.x, point.y);
}

double computeSpan(const std::vector<Point2D> & points)
{
  double max_span_sq = 0.0;
  for (std::size_t i = 0; i < points.size(); ++i) {
    for (std::size_t j = i + 1; j < points.size(); ++j) {
      max_span_sq = std::max(max_span_sq, squaredDistance(points[i], points[j]));
    }
  }
  return std::sqrt(max_span_sq);
}

Point2D computeCentroid(const std::vector<Point2D> & points)
{
  Point2D centroid{};
  for (const Point2D & point : points) {
    centroid.x += point.x;
    centroid.y += point.y;
  }
  const double scale = points.empty() ? 0.0 : 1.0 / static_cast<double>(points.size());
  centroid.x *= scale;
  centroid.y *= scale;
  return centroid;
}
}  // namespace

std::vector<Cluster> extractClusters(
  const std::vector<Point2D> & pts_base,
  const ClusterConfig & cfg)
{
  std::vector<Point2D> filtered;
  filtered.reserve(pts_base.size());
  for (const Point2D & point : pts_base) {
    if (pointRange(point) <= cfg.max_range) {
      filtered.push_back(point);
    }
  }

  std::vector<Cluster> clusters;
  if (filtered.empty()) {
    return clusters;
  }

  const double tolerance_sq = cfg.cluster_tolerance * cfg.cluster_tolerance;
  std::vector<bool> visited(filtered.size(), false);

  for (std::size_t seed = 0; seed < filtered.size(); ++seed) {
    if (visited[seed]) {
      continue;
    }

    std::queue<std::size_t> frontier;
    std::vector<Point2D> members;
    frontier.push(seed);
    visited[seed] = true;

    while (!frontier.empty()) {
      const std::size_t current = frontier.front();
      frontier.pop();
      members.push_back(filtered[current]);

      for (std::size_t next = 0; next < filtered.size(); ++next) {
        if (visited[next]) {
          continue;
        }
        if (squaredDistance(filtered[current], filtered[next]) <= tolerance_sq) {
          visited[next] = true;
          frontier.push(next);
        }
      }
    }

    const int point_count = static_cast<int>(members.size());
    if (point_count < cfg.min_points || point_count > cfg.max_points) {
      continue;
    }

    const double span = computeSpan(members);
    if (span > cfg.max_cluster_span) {
      continue;
    }

    clusters.push_back(Cluster{computeCentroid(members), point_count, span});
  }

  return clusters;
}

}  // namespace mycar_navigation::clustering
