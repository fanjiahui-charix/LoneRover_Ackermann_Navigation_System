#include <gtest/gtest.h>
#include <cmath>
#include "simple_lidar_odom/persistent_fence_map.hpp"
using namespace simple_lidar_odom;

// Build a LineObs from (normal, d). Normal is normalized here so tests state
// directions loosely; the map relies on canonicalizeLine for sign.
static LineObs L(double nx, double ny, double d)
{
  Point2 n(nx, ny);
  const double len = n.norm();
  return LineObs{n / len, d / len};
}

TEST(PersistentFenceMap, NewObservationCreatesTentative) {
  PersistentFenceMap m; m.configure(0.10, 0.30, 5, 10);
  m.update({L(1, 0, 2.0)}, 0);
  ASSERT_EQ(m.fences().size(), 1u);
  EXPECT_EQ(m.fences()[0].state, FenceState::Tentative);
  EXPECT_EQ(m.fences()[0].hits, 1);
  EXPECT_NEAR(m.fences()[0].normal.x(), 1.0, 1e-9);
  EXPECT_NEAR(m.fences()[0].d, 2.0, 1e-9);
}

TEST(PersistentFenceMap, RepeatedObservationConfirmsAndFreezes) {
  PersistentFenceMap m; m.configure(0.10, 0.30, 5, 10);
  for (int f = 0; f < 5; ++f) m.update({L(1, 0, 2.0)}, f);
  ASSERT_EQ(m.fences().size(), 1u);
  EXPECT_EQ(m.fences()[0].state, FenceState::Confirmed);
  // Offset observation within the gate, but the anchor is frozen -> must not move.
  m.update({L(1, 0, 2.2)}, 5);
  ASSERT_EQ(m.fences().size(), 1u);
  EXPECT_NEAR(m.fences()[0].d, 2.0, 1e-9);
  EXPECT_EQ(m.fences()[0].hits, 6);
  EXPECT_EQ(m.fences()[0].last_seen_frame, 5);
}

TEST(PersistentFenceMap, RunningMeanDenoisesTentative) {
  PersistentFenceMap m; m.configure(0.10, 0.30, 100, 50);  // never confirm here
  m.update({L(1, 0, 2.00)}, 0);
  m.update({L(1, 0, 2.10)}, 1);   // mean(2.00,2.10) = 2.05
  EXPECT_NEAR(m.fences()[0].d, 2.05, 1e-9);
  EXPECT_NEAR(m.fences()[0].normal.norm(), 1.0, 1e-9);
  m.update({L(1, 0, 2.20)}, 2);   // mean(2.00,2.10,2.20) = 2.10
  EXPECT_NEAR(m.fences()[0].d, 2.10, 1e-9);
  EXPECT_NEAR(m.fences()[0].normal.norm(), 1.0, 1e-9);
}

TEST(PersistentFenceMap, LineGeometricConsistencyAfterMean) {
  PersistentFenceMap m; m.configure(0.10, 0.30, 100, 50);
  m.update({L(1.0, 0.0, 2.0)}, 0);
  // ~0.05 rad off; |sin| ~ 0.05 < angle_tol -> associates and means in.
  m.update({L(std::cos(0.05), std::sin(0.05), 2.0)}, 1);
  ASSERT_EQ(m.fences().size(), 1u);
  // After running-mean + renormalize the normal must stay unit-length; there is
  // no exact "point satisfies n.x == d" invariant for a mean of line params.
  EXPECT_NEAR(m.fences()[0].normal.norm(), 1.0, 1e-9);
}

TEST(PersistentFenceMap, ParallelWallsDistinguishedByD) {
  PersistentFenceMap m; m.configure(0.10, 0.30, 5, 10);
  m.update({L(1, 0, 2.0)}, 0);
  m.update({L(1, 0, 3.0)}, 0);   // same normal, d differs by 1.0 > assoc_dist_tol
  EXPECT_EQ(m.fences().size(), 2u);
}

TEST(PersistentFenceMap, OppositeSignCanonicalizedAndMerged) {
  PersistentFenceMap m; m.configure(0.10, 0.30, 5, 10);
  m.update({L(1, 0, 2.0)}, 0);
  // Same physical line, opposite sign representation: (-n, -d). Entry-side
  // canonicalization to d>=0 makes both identical -> merge, hits=2.
  m.update({L(-1, 0, -2.0)}, 1);
  ASSERT_EQ(m.fences().size(), 1u);
  EXPECT_EQ(m.fences()[0].hits, 2);
  EXPECT_NEAR(m.fences()[0].normal.x(), 1.0, 1e-9);
  EXPECT_NEAR(m.fences()[0].d, 2.0, 1e-9);
}

TEST(PersistentFenceMap, AntiparallelOppositeWallSeparate) {
  PersistentFenceMap m; m.configure(0.10, 0.30, 5, 10);
  // Corridor: two facing walls in SEPARATE frames so association is actually
  // attempted. Both have d>0 after canonicalize and |sin|~0 between normals (so
  // the angle + dist gates would merge them); only the same-facing gate
  // (n_obs.n_map > 0) keeps them as two distinct walls.
  m.update({L(1, 0, 2.0)}, 0);
  m.update({L(-1, 0, 2.0)}, 1);
  EXPECT_EQ(m.fences().size(), 2u);
}

TEST(PersistentFenceMap, SameFrameNoSelfMerge) {
  PersistentFenceMap m; m.configure(0.10, 0.30, 5, 10);
  // Two ASSOCIABLE lines in one frame (same normal, d within assoc_dist_tol):
  // without the n_before snapshot the second would fold into the one created
  // moments earlier this frame. The snapshot keeps them separate -> 2 entries.
  m.update({L(1, 0, 2.0), L(1, 0, 2.05)}, 0);
  ASSERT_EQ(m.fences().size(), 2u);
  EXPECT_EQ(m.fences()[0].hits, 1);
  EXPECT_EQ(m.fences()[1].hits, 1);
}

TEST(PersistentFenceMap, StaleTentativePruned) {
  PersistentFenceMap m; m.configure(0.10, 0.30, 5, 10);
  m.update({L(1, 0, 2.0)}, 0);    // fence A, last_seen 0
  m.update({L(0, 1, 5.0)}, 5);    // fence B; A not re-seen, 5-0 !> 10 -> kept
  EXPECT_EQ(m.fences().size(), 2u);
  m.update({L(0, 1, 5.0)}, 11);   // A: 11-0=11 > 10 -> pruned; B refreshed
  ASSERT_EQ(m.fences().size(), 1u);
  EXPECT_NEAR(m.fences()[0].normal.y(), 1.0, 1e-9);
}

TEST(PersistentFenceMap, ConfirmedNeverPruned) {
  PersistentFenceMap m; m.configure(0.10, 0.30, 3, 5);
  for (int f = 0; f < 3; ++f) m.update({L(1, 0, 2.0)}, f);  // confirmed at frame 2
  ASSERT_EQ(m.fences()[0].state, FenceState::Confirmed);
  m.update({L(0, 1, 9.0)}, 100);  // long gap; confirmed must survive
  bool has_confirmed = false;
  for (const auto & f : m.fences())
    if (f.state == FenceState::Confirmed) has_confirmed = true;
  EXPECT_TRUE(has_confirmed);
}
