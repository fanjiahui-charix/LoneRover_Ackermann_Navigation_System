#include <vector>

#include <gtest/gtest.h>

#include "mycar_navigation/clustering/cluster_extractor.hpp"

namespace mycar_navigation::clustering
{
namespace
{
using nav_core::Point2D;

TEST(ClusterExtractorTest, GroupsNearbyPointsAndIgnoresFarNoise)
{
  ClusterConfig cfg;
  cfg.cluster_tolerance = 0.12;
  cfg.min_points = 2;
  cfg.max_points = 10;
  cfg.max_cluster_span = 0.35;
  cfg.max_range = 5.0;

  const std::vector<Point2D> points{{1.0, 0.0}, {1.05, 0.02}, {3.0, 0.0}, {3.06, 0.01}, {6.5, 0.0}};
  const auto clusters = extractClusters(points, cfg);

  ASSERT_EQ(clusters.size(), 2U);
  EXPECT_EQ(clusters[0].point_count, 2);
  EXPECT_NEAR(clusters[0].centroid.x, 1.025, 1e-6);
  EXPECT_EQ(clusters[1].point_count, 2);
  EXPECT_NEAR(clusters[1].centroid.x, 3.03, 1e-6);
}

TEST(ClusterExtractorTest, RejectsElongatedWallLikeClusters)
{
  ClusterConfig cfg;
  cfg.cluster_tolerance = 0.20;
  cfg.min_points = 2;
  cfg.max_cluster_span = 0.25;

  const std::vector<Point2D> points{{1.0, 0.0}, {1.15, 0.0}, {1.30, 0.0}};
  const auto clusters = extractClusters(points, cfg);

  EXPECT_TRUE(clusters.empty());
}

TEST(ClusterExtractorTest, RejectsOversizedClusters)
{
  ClusterConfig cfg;
  cfg.cluster_tolerance = 0.10;
  cfg.min_points = 2;
  cfg.max_points = 3;
  cfg.max_cluster_span = 1.0;

  const std::vector<Point2D> points{{1.0, 0.0}, {1.03, 0.0}, {1.06, 0.0}, {1.09, 0.0}};
  const auto clusters = extractClusters(points, cfg);

  EXPECT_TRUE(clusters.empty());
}

}  // namespace
}  // namespace mycar_navigation::clustering
