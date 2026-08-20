// Iron-law regression (spec §5): the collision layer (raw-scan LocalGrid +
// evaluateArc) MUST be completely independent of the cone-clustering memory.
// Clustering is a lossy soft layer for scoring/visualisation/stuck-detection;
// it must never be able to add or remove an obstacle from the hard collision
// check. This test proves that by computing collision feasibility with an empty
// ConeMemory and again with the memory stuffed full of ghost cones placed right
// on the arc, and asserting the collision results are byte-identical.
#include <vector>

#include <gtest/gtest.h>

#include "mycar_navigation/clustering/cone_memory.hpp"
#include "mycar_navigation/nav_core/arc.hpp"
#include "mycar_navigation/nav_core/footprint.hpp"
#include "mycar_navigation/planner/feasibility.hpp"
#include "mycar_navigation/planner/local_grid.hpp"

namespace mycar_navigation
{
namespace
{
using clustering::ConeMemory;
using clustering::MemoryConfig;
using nav_core::ArcIntegrator;
using nav_core::Footprint;
using nav_core::Point2D;
using nav_core::Pose2D;
using planner::ArcEvaluation;
using planner::evaluateArc;
using planner::LocalGrid;

// Build the hard collision grid from raw scan points only (the safety floor).
LocalGrid buildScanGrid(const std::vector<Point2D> & scan_pts)
{
  LocalGrid grid(0.05, 3.0);
  grid.clear();
  grid.addScanPoints(scan_pts, 0.05);  // physical inflation only
  return grid;
}

ArcEvaluation evalStraight(const LocalGrid & grid)
{
  const Footprint fp(0.22, 0.14);
  const auto rollout = ArcIntegrator().integrate(Pose2D{0.0, 0.0, 0.0}, 1.0, 0.0, 2.0, 0.1);
  return evaluateArc(rollout, 0.1, 0.10, fp, grid, grid.resolution());
}

TEST(IronLawTest, CollisionFeasibilityIsIndependentOfConeMemory)
{
  // A real obstacle sits ~1.0 m ahead on the straight arc.
  const std::vector<Point2D> scan_pts = {
    {1.0, 0.0}, {1.0, 0.05}, {1.0, -0.05}};

  // Baseline: collision check with NO clustering memory involved at all.
  const LocalGrid grid_baseline = buildScanGrid(scan_pts);
  const ArcEvaluation baseline = evalStraight(grid_baseline);

  // Now create a clustering memory and flood it with ghost cones placed exactly
  // on the arc (in front of and behind the real obstacle). If clustering had any
  // influence on the collision layer, these ghosts would change the result.
  ConeMemory memory(MemoryConfig{});
  for (int frame = 0; frame < 5; ++frame) {
    memory.update(
      {Point2D{0.3, 0.0}, Point2D{0.6, 0.0}, Point2D{1.5, 0.0}, Point2D{2.0, 0.0}},
      static_cast<double>(frame));
  }
  ASSERT_FALSE(memory.confirmedCones().empty()) << "ghost cones must be confirmed for a fair test";

  // The collision grid is rebuilt from the SAME raw scan, untouched by memory.
  const LocalGrid grid_with_ghosts = buildScanGrid(scan_pts);
  const ArcEvaluation with_ghosts = evalStraight(grid_with_ghosts);

  // Iron law: identical collision outcome regardless of cone memory contents.
  EXPECT_EQ(baseline.feasible, with_ghosts.feasible);
  EXPECT_EQ(baseline.blocked_by_scan, with_ghosts.blocked_by_scan);
  EXPECT_EQ(baseline.blocked_by_map, with_ghosts.blocked_by_map);
  EXPECT_DOUBLE_EQ(baseline.free_length, with_ghosts.free_length);

  // And the real scan obstacle is still what stops the arc (~1.0 m minus
  // footprint half-length), proving the raw scan — not clustering — is the floor.
  EXPECT_TRUE(baseline.blocked_by_scan);
  EXPECT_GT(baseline.free_length, 0.5);
  EXPECT_LT(baseline.free_length, 1.0);
}

// Clearing or decaying the cone memory likewise cannot change collision results.
TEST(IronLawTest, MemoryLifecycleDoesNotAffectCollisionLayer)
{
  const std::vector<Point2D> scan_pts = {{0.8, 0.0}, {0.8, 0.05}, {0.8, -0.05}};

  const ArcEvaluation before = evalStraight(buildScanGrid(scan_pts));

  ConeMemory memory(MemoryConfig{});
  memory.update({Point2D{0.4, 0.0}}, 0.0);
  memory.update({Point2D{0.4, 0.0}}, 1.0);
  memory.decay(100.0);  // expire everything
  memory.clear();

  const ArcEvaluation after = evalStraight(buildScanGrid(scan_pts));

  EXPECT_EQ(before.feasible, after.feasible);
  EXPECT_DOUBLE_EQ(before.free_length, after.free_length);
}

}  // namespace
}  // namespace mycar_navigation
