#include <gtest/gtest.h>
#include <cmath>
#include "simple_lidar_odom/persistent_cone_map.hpp"
using namespace simple_lidar_odom;

static Point2 P(double x, double y) { return Point2(x, y); }

TEST(PersistentConeMap, NewObservationCreatesTentative) {
  PersistentConeMap m; m.configure(0.30, 5, 10);
  m.update({P(1, 0)}, 0);
  ASSERT_EQ(m.cones().size(), 1u);
  EXPECT_EQ(m.cones()[0].state, ConeState::Tentative);
  EXPECT_EQ(m.cones()[0].hits, 1);
  EXPECT_DOUBLE_EQ(m.cones()[0].pos.x(), 1.0);
}

TEST(PersistentConeMap, RepeatedObservationsConfirmAndFreeze) {
  PersistentConeMap m; m.configure(0.30, 5, 10);
  for (int f = 0; f < 5; ++f) m.update({P(1.0, 0.0)}, f);
  ASSERT_EQ(m.cones().size(), 1u);
  EXPECT_EQ(m.cones()[0].state, ConeState::Confirmed);
  m.update({P(1.20, 0.0)}, 5);   // within gate, but frozen -> must not move
  EXPECT_EQ(m.cones().size(), 1u);
  EXPECT_NEAR(m.cones()[0].pos.x(), 1.0, 1e-9);
}

TEST(PersistentConeMap, RunningMeanDenoisesTentative) {
  PersistentConeMap m; m.configure(0.30, 100, 50);   // never confirm here
  m.update({P(1.00, 0.0)}, 0);
  m.update({P(1.10, 0.0)}, 1);    // mean(1.00,1.10) = 1.05
  EXPECT_NEAR(m.cones()[0].pos.x(), 1.05, 1e-9);
  m.update({P(1.20, 0.0)}, 2);    // mean(1.00,1.10,1.20) = 1.10
  EXPECT_NEAR(m.cones()[0].pos.x(), 1.10, 1e-9);
}

TEST(PersistentConeMap, StaleTentativePruned) {
  PersistentConeMap m; m.configure(0.30, 5, 10);
  m.update({P(1, 0)}, 0);         // cone A, last_seen 0
  m.update({P(5, 0)}, 5);         // cone B; A not re-seen, 5-0 !> 10 -> kept
  EXPECT_EQ(m.cones().size(), 2u);
  m.update({P(5, 0)}, 11);        // A: 11-0=11 > 10 -> pruned; B refreshed
  ASSERT_EQ(m.cones().size(), 1u);
  EXPECT_NEAR(m.cones()[0].pos.x(), 5.0, 1e-9);
}

TEST(PersistentConeMap, ConfirmedNeverPruned) {
  PersistentConeMap m; m.configure(0.30, 3, 5);
  for (int f = 0; f < 3; ++f) m.update({P(2, 0)}, f);   // confirmed at frame 2
  ASSERT_EQ(m.cones()[0].state, ConeState::Confirmed);
  m.update({P(9, 9)}, 100);       // long gap; confirmed must survive
  bool has_confirmed = false;
  for (const auto & c : m.cones())
    if (c.state == ConeState::Confirmed) has_confirmed = true;
  EXPECT_TRUE(has_confirmed);
}

TEST(PersistentConeMap, OneMapConeIsUpdatedAtMostOncePerFrame) {
  PersistentConeMap m; m.configure(0.30, 2, 10);
  m.update({P(1.0, 0.0)}, 0);
  m.update({P(1.0, 0.0)}, 1);
  ASSERT_EQ(m.cones().size(), 1u);
  const int hits_before = m.cones()[0].hits;
  m.update({P(0.99, 0.0), P(1.01, 0.0)}, 2);
  ASSERT_EQ(m.cones().size(), 1u);
  EXPECT_EQ(m.cones()[0].hits, hits_before + 1);
}
