#include <gtest/gtest.h>

#include "sensor_msgs/msg/laser_scan.hpp"
#include "simple_lidar_odom/landmark_extractor.hpp"
#include "simple_lidar_odom/local_landmark_map.hpp"

namespace
{

sensor_msgs::msg::LaserScan makeScan(const std::vector<float> & ranges, float angle_increment)
{
  sensor_msgs::msg::LaserScan scan;
  scan.angle_min = -0.5f * static_cast<float>(ranges.size() - 1) * angle_increment;
  scan.angle_max = -scan.angle_min;
  scan.angle_increment = angle_increment;
  scan.ranges = ranges;
  return scan;
}

}  // namespace

TEST(LandmarkExtractor, SegmentScanSplitsOnInvalidAndJump)
{
  auto scan = makeScan({1.0F, 1.02F, std::numeric_limits<float>::infinity(), 1.0F, 1.6F}, 0.05F);
  const auto segments = simple_lidar_odom::segmentScan(scan, 0.05, 5.0, 0.20);
  ASSERT_EQ(segments.size(), 3U);
  EXPECT_EQ(segments[0].pts.size(), 2U);
  EXPECT_EQ(segments[1].pts.size(), 1U);
  EXPECT_EQ(segments[2].pts.size(), 1U);
}

TEST(LandmarkExtractor, FitCircleFixedRRecoversCenter)
{
  simple_lidar_odom::PointList pts;
  const simple_lidar_odom::Point2 center(1.0, 0.5);
  const double radius = 0.1;
  for (double a : {-0.4, -0.2, 0.0, 0.2, 0.4}) {
    pts.emplace_back(center.x() + radius * std::cos(a), center.y() + radius * std::sin(a));
  }

  const auto fit = simple_lidar_odom::fitCircleFixedR(pts, radius, 0.02);
  EXPECT_TRUE(fit.valid);
  EXPECT_NEAR(fit.center.x(), center.x(), 0.02);
  EXPECT_NEAR(fit.center.y(), center.y(), 0.02);
}

TEST(LandmarkExtractor, ExtractFenceLinesMergesCollinearSegments)
{
  simple_lidar_odom::Segment a;
  simple_lidar_odom::Segment b;
  for (double x = 0.0; x <= 0.4; x += 0.1) {
    a.pts.emplace_back(x, 1.0);
  }
  for (double x = 0.5; x <= 0.9; x += 0.1) {
    b.pts.emplace_back(x, 1.0);
  }

  const auto lines = simple_lidar_odom::extractFenceLines({a, b}, 0.1, 0.05);
  ASSERT_EQ(lines.size(), 1U);
  EXPECT_NEAR(std::abs(lines.front().normal.y()), 1.0, 1.0e-3);
  EXPECT_GT(lines.front().support_points.size(), a.pts.size());
}

TEST(LocalLandmarkMap, EvictsOldEntriesByAge)
{
  simple_lidar_odom::LocalLandmarkMap map(1);
  simple_lidar_odom::PointList cones{{0.0, 0.0}};
  simple_lidar_odom::LineLandmark fence;
  fence.normal = simple_lidar_odom::Point2(0.0, -1.0);
  fence.centroid = simple_lidar_odom::Point2(0.0, 1.0);
  fence.d = fence.normal.dot(fence.centroid);
  fence.support_points = {{0.0, 1.0}, {1.0, 1.0}};

  map.addFrame(cones, {fence});
  map.addFrame({}, {});
  map.addFrame({}, {});

  EXPECT_TRUE(map.cones().empty());
  EXPECT_TRUE(map.fences().empty());
}
