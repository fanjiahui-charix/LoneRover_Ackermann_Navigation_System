#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "mycar_navigation/nav_core/arc.hpp"
#include "mycar_navigation/nav_core/field_odom_transform.hpp"
#include "mycar_navigation/nav_core/footprint.hpp"
#include "mycar_navigation/nav_core/map_mask.hpp"
#include "mycar_navigation/nav_core/types.hpp"
#include "mycar_navigation/planner/feasibility.hpp"
#include "mycar_navigation/planner/local_grid.hpp"

namespace mycar_navigation::planner
{
namespace
{
constexpr double kResolution = 0.05;
constexpr double kHalfExtent = 3.0;
constexpr double kTolerance = 0.12;
constexpr double kStepLength = 0.1;
constexpr double kHorizon = 1.0;
constexpr double kInflationRadius = 0.0;
constexpr double kMinFeasibleLength = 0.1;

std::vector<Pose2D> makeStraightRollout()
{
  const nav_core::ArcIntegrator integrator;
  return integrator.integrate(Pose2D{}, 1.0, 0.0, kHorizon, kStepLength);
}

nav_core::Footprint makeCompactFootprint()
{
  return nav_core::Footprint(0.02, 0.02);
}

void expectNearLength(double actual, double expected)
{
  EXPECT_NEAR(actual, expected, kTolerance);
}

}  // namespace

TEST(FeasibilityTest, EmptyGridRemainsFeasibleToRolloutEnd)
{
  LocalGrid grid(kResolution, kHalfExtent);
  const auto rollout = makeStraightRollout();

  const ArcEvaluation evaluation = evaluateArc(
    rollout, kStepLength, kMinFeasibleLength, makeCompactFootprint(), grid, kResolution);

  EXPECT_TRUE(evaluation.feasible);
  EXPECT_FALSE(evaluation.blocked_by_scan);
  EXPECT_FALSE(evaluation.blocked_by_map);
  expectNearLength(evaluation.free_length, (static_cast<double>(rollout.size()) - 1.0) * kStepLength);
}

TEST(FeasibilityTest, ScanObstacleBlocksAtFirstCollidingPose)
{
  LocalGrid grid(kResolution, kHalfExtent);
  grid.addScanPoints({Point2D{0.36, 0.0}}, kInflationRadius);
  const auto rollout = makeStraightRollout();

  const ArcEvaluation evaluation = evaluateArc(
    rollout, kStepLength, kMinFeasibleLength, makeCompactFootprint(), grid, kResolution);

  EXPECT_TRUE(evaluation.feasible);
  EXPECT_TRUE(evaluation.blocked_by_scan);
  EXPECT_FALSE(evaluation.blocked_by_map);
  expectNearLength(evaluation.free_length, 0.3);
}

TEST(FeasibilityTest, MapForbiddenBlocksAtFirstCollidingPose)
{
  LocalGrid grid(kResolution, kHalfExtent);
  std::vector<MapClass> cells(200U * 200U, MapClass::DRIVABLE);
  const int forbidden_i = static_cast<int>(std::floor(0.36 / kResolution)) + 100;
  const int row_from_bottom = static_cast<int>(std::floor(0.0 / kResolution)) + 100;
  const int forbidden_j = 199 - row_from_bottom;
  cells[static_cast<std::size_t>(forbidden_j * 200 + forbidden_i)] = MapClass::HARD_FORBIDDEN;

  const nav_core::MapMask forbidden_map = nav_core::MapMask::fromCells(
    200U, 200U, kResolution, -5.0, -5.0, std::move(cells));
  const nav_core::FieldOdomTransform tf(Pose2D{0.0, 0.0, 0.0});
  grid.addMapForbidden(forbidden_map, tf, Pose2D{}, true);
  const auto rollout = makeStraightRollout();

  const ArcEvaluation evaluation = evaluateArc(
    rollout, kStepLength, kMinFeasibleLength, makeCompactFootprint(), grid, kResolution);

  EXPECT_TRUE(evaluation.feasible);
  EXPECT_FALSE(evaluation.blocked_by_scan);
  EXPECT_TRUE(evaluation.blocked_by_map);
  expectNearLength(evaluation.free_length, 0.3);
}

TEST(FeasibilityTest, CollisionAtStartIsInfeasible)
{
  LocalGrid grid(kResolution, kHalfExtent);
  grid.addScanPoints({Point2D{0.0, 0.0}}, kInflationRadius);
  const auto rollout = makeStraightRollout();

  const ArcEvaluation evaluation = evaluateArc(
    rollout, kStepLength, kMinFeasibleLength, makeCompactFootprint(), grid, kResolution);

  EXPECT_FALSE(evaluation.feasible);
  EXPECT_TRUE(evaluation.blocked_by_scan);
  EXPECT_FALSE(evaluation.blocked_by_map);
  EXPECT_DOUBLE_EQ(evaluation.free_length, 0.0);
}

}  // namespace mycar_navigation::planner
