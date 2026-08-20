#include <gtest/gtest.h>

#include "simple_lidar_odom/unique_assignment.hpp"

namespace
{

using simple_lidar_odom::Point2;

TEST(UniqueAssignment, DoesNotReuseMapLandmark)
{
  const std::vector<Point2> observations{{1.00, 0.0}, {1.02, 0.0}, {2.0, 0.0}};
  const std::vector<Point2> landmarks{{1.01, 0.0}, {2.0, 0.0}};
  const auto matches = simple_lidar_odom::greedyUniqueAssignment(
    observations, landmarks, 0.2);

  ASSERT_EQ(matches.size(), 2U);
  EXPECT_NE(matches[0].landmark_index, matches[1].landmark_index);
  EXPECT_NE(matches[0].observation_index, matches[1].observation_index);
}

TEST(UniqueAssignment, RejectsOutsideGate)
{
  const auto matches = simple_lidar_odom::greedyUniqueAssignment(
    std::vector<Point2>{{0.0, 0.0}}, std::vector<Point2>{{1.0, 0.0}}, 0.2);
  EXPECT_TRUE(matches.empty());
}

}  // namespace
