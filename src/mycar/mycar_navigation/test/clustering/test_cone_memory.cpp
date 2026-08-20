#include <vector>

#include <gtest/gtest.h>

#include "mycar_navigation/clustering/cone_memory.hpp"

namespace mycar_navigation::clustering
{
namespace
{
using nav_core::Point2D;

TEST(ConeMemoryTest, ConfirmsTrackAfterRequiredObservations)
{
  MemoryConfig cfg;
  cfg.confirm_observations = 2;
  cfg.match_distance = 0.25;

  ConeMemory memory(cfg);
  memory.update({Point2D{1.0, 2.0}}, 1.0);
  EXPECT_TRUE(memory.confirmedCones().empty());

  memory.update({Point2D{1.1, 2.0}}, 2.0);
  const auto confirmed = memory.confirmedCones();
  ASSERT_EQ(confirmed.size(), 1U);
  EXPECT_NEAR(confirmed[0].x, 1.05, 1e-6);
  EXPECT_NEAR(confirmed[0].y, 2.0, 1e-6);
}

TEST(ConeMemoryTest, DecayRemovesExpiredTracks)
{
  MemoryConfig cfg;
  cfg.ttl_sec = 2.0;

  ConeMemory memory(cfg);
  memory.update({Point2D{0.0, 0.0}}, 1.0);
  EXPECT_EQ(memory.trackCount(), 1U);

  memory.decay(3.1);
  EXPECT_EQ(memory.trackCount(), 0U);
}

TEST(ConeMemoryTest, ClearDropsAllTracks)
{
  ConeMemory memory(MemoryConfig{});
  memory.update({Point2D{0.0, 0.0}, Point2D{1.0, 1.0}}, 1.0);
  EXPECT_EQ(memory.trackCount(), 2U);

  memory.clear();
  EXPECT_EQ(memory.trackCount(), 0U);
  EXPECT_TRUE(memory.confirmedCones().empty());
}

}  // namespace
}  // namespace mycar_navigation::clustering
