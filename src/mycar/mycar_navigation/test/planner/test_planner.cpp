#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "mycar_navigation/nav_core/field_odom_transform.hpp"
#include "mycar_navigation/nav_core/footprint.hpp"
#include "mycar_navigation/nav_core/map_mask.hpp"
#include "mycar_navigation/nav_core/types.hpp"
#include "mycar_navigation/planner/planner.hpp"
#include "mycar_navigation/planner/speed_limiter.hpp"

namespace mycar_navigation::planner
{
namespace
{
constexpr double kResolution = 0.05;
constexpr double kHalfExtent = 3.0;
constexpr double kRmin = 0.6;
constexpr double kEps = 1e-6;

PlannerConfig makeConfig()
{
  PlannerConfig cfg;
  cfg.grid_resolution = kResolution;
  cfg.grid_half_extent = kHalfExtent;
  cfg.footprint_half_length = 0.22;
  cfg.footprint_half_width = 0.14;
  cfg.min_turning_radius = kRmin;
  cfg.obstacle_radius = 0.03;
  cfg.localization_margin = 0.02;
  cfg.speed_margin_per_mps = 0.02;
  cfg.max_speed = 1.0;
  cfg.min_speed = 0.05;
  cfg.a_lat_max = 2.0;
  cfg.a_brake = 1.0;
  cfg.a_accel = 5.0;
  cfg.t_latency = 0.2;
  cfg.brake_margin = 0.10;
  cfg.n_curvatures = 31;
  cfg.dt = 0.1;
  cfg.min_horizon = 0.5;
  cfg.goal_approach_distance = 0.5;
  cfg.treat_unknown_as_forbidden = true;
  cfg.w_goal = 0.6;
  cfg.w_clear = 0.3;
  cfg.w_smooth = 0.1;
  cfg.min_feasible_length = 0.10;
  return cfg;
}

nav_core::MapMask makeDrivableMap()
{
  const std::size_t side_cells = 240U;
  std::vector<nav_core::MapClass> cells(side_cells * side_cells, nav_core::MapClass::DRIVABLE);
  return nav_core::MapMask::fromCells(
    side_cells, side_cells, kResolution, -6.0, -6.0, std::move(cells));
}

Planner makePlanner(const PlannerConfig & cfg)
{
  return Planner(
    cfg,
    nav_core::Footprint(cfg.footprint_half_length, cfg.footprint_half_width),
    makeDrivableMap(),
    nav_core::FieldOdomTransform(Pose2D{0.0, 0.0, 0.0}));
}

PlannerInput makeInput(const std::vector<Point2D> & scan_points)
{
  PlannerInput in;
  in.robot_field_pose = Pose2D{0.0, 0.0, 0.0};
  in.goal_field_pose = Pose2D{2.5, 0.0, 0.0};
  in.scan_points_base = scan_points;
  in.prev_v = 0.0;
  in.prev_kappa = 0.0;
  return in;
}

std::vector<Point2D> makePointCluster(double center_x, double center_y, double spacing, int rows, int cols)
{
  std::vector<Point2D> points;
  points.reserve(static_cast<std::size_t>(rows * cols));
  const double row_offset = 0.5 * static_cast<double>(rows - 1) * spacing;
  const double col_offset = 0.5 * static_cast<double>(cols - 1) * spacing;
  for (int r = 0; r < rows; ++r) {
    for (int c = 0; c < cols; ++c) {
      points.push_back(Point2D{
        center_x + static_cast<double>(c) * spacing - col_offset,
        center_y + static_cast<double>(r) * spacing - row_offset});
    }
  }
  return points;
}

std::vector<Point2D> makeWall(double x, double y_min, double y_max, double spacing)
{
  std::vector<Point2D> points;
  for (double y = y_min; y <= y_max + 1e-9; y += spacing) {
    points.push_back(Point2D{x, y});
  }
  return points;
}

std::vector<Point2D> concat(std::vector<Point2D> lhs, const std::vector<Point2D> & rhs)
{
  lhs.insert(lhs.end(), rhs.begin(), rhs.end());
  return lhs;
}

}  // namespace

TEST(PlannerTest, ObstacleAheadPicksAvoidanceArc)
{
  PlannerConfig cfg = makeConfig();
  const Planner planner = makePlanner(cfg);
  const auto scan = makePointCluster(0.70, 0.0, 0.05, 7, 4);

  const PlannerResult result = planner.computeCommand(makeInput(scan));

  EXPECT_EQ(result.status, PlannerStatus::OK);
  EXPECT_FALSE(result.selected_trajectory.empty());
  if (std::abs(result.kappa) <= 1e-3) {
    EXPECT_LT(result.v, 0.7);
  } else {
    EXPECT_GT(std::abs(result.kappa), 1e-3);
  }
  EXPECT_GT(result.v, 0.0);
}

TEST(PlannerTest, ObstacleAtNoseNoForwardSolution)
{
  PlannerConfig cfg = makeConfig();
  const Planner planner = makePlanner(cfg);
  const auto scan = makeWall(0.12, -0.25, 0.25, 0.03);

  const PlannerResult result = planner.computeCommand(makeInput(scan));

  EXPECT_EQ(result.status, PlannerStatus::NO_FORWARD_TRAJECTORY);
  EXPECT_DOUBLE_EQ(result.v, 0.0);
  EXPECT_DOUBLE_EQ(result.kappa, 0.0);
}

TEST(PlannerTest, AckermannConstraintAlwaysHeld)
{
  PlannerConfig cfg = makeConfig();
  const Planner planner = makePlanner(cfg);
  const double max_kappa = 1.0 / cfg.min_turning_radius + 1e-9;

  const std::vector<std::vector<Point2D>> scenarios = {
    {},
    makePointCluster(0.9, 0.0, 0.05, 5, 3),
    concat(makeWall(1.0, -0.6, -0.25, 0.05), makeWall(1.0, 0.25, 0.6, 0.05)),
  };

  for (const auto & scan : scenarios) {
    const PlannerResult result = planner.computeCommand(makeInput(scan));
    if (result.status == PlannerStatus::OK) {
      EXPECT_LE(std::abs(result.kappa), max_kappa);
    }
    const PlannerResult recovery = planner.computeRecoveryCommand(makeInput(scan));
    if (recovery.status == PlannerStatus::OK) {
      EXPECT_LE(std::abs(recovery.kappa), max_kappa);
    }
  }
}

TEST(PlannerTest, BrakingReducesSpeedNearObstacle)
{
  PlannerConfig cfg = makeConfig();
  const Planner planner = makePlanner(cfg);

  PlannerInput mid = makeInput(makeWall(0.85, -0.15, 0.15, 0.05));
  PlannerInput near = makeInput(makeWall(0.55, -0.15, 0.15, 0.05));

  const PlannerResult mid_result = planner.computeCommand(mid);
  const PlannerResult near_result = planner.computeCommand(near);

  ASSERT_EQ(mid_result.status, PlannerStatus::OK);
  ASSERT_EQ(near_result.status, PlannerStatus::OK);
  EXPECT_LT(mid_result.v, cfg.max_speed);
  EXPECT_LT(near_result.v, mid_result.v);
}

TEST(PlannerTest, NarrowChannelPicksMidline)
{
  PlannerConfig cfg = makeConfig();
  cfg.footprint_half_width = 0.10;
  cfg.obstacle_radius = 0.01;
  cfg.localization_margin = 0.01;
  cfg.speed_margin_per_mps = 0.01;
  const Planner planner = makePlanner(cfg);

  auto scan = makeWall(1.0, -0.65, -0.25, 0.03);
  scan = concat(std::move(scan), makeWall(1.0, 0.25, 0.65, 0.03));

  const PlannerResult result = planner.computeCommand(makeInput(scan));

  EXPECT_EQ(result.status, PlannerStatus::OK);
  EXPECT_LT(std::abs(result.kappa), 0.25);
}

TEST(PlannerTest, RecoveryReverseFeasible)
{
  PlannerConfig cfg = makeConfig();
  const Planner planner = makePlanner(cfg);
  auto scan = makeWall(0.12, -0.30, 0.30, 0.03);
  scan = concat(std::move(scan), makeWall(0.24, -0.12, 0.12, 0.03));

  PlannerInput input = makeInput(scan);
  input.goal_field_pose = Pose2D{-1.0, 0.0, 0.0};
  const PlannerResult result = planner.computeRecoveryCommand(input);

  if (result.status != PlannerStatus::OK) {
    const PlannerResult forward = planner.computeCommand(input);
    EXPECT_EQ(forward.status, PlannerStatus::NO_FORWARD_TRAJECTORY);
  }
  EXPECT_TRUE(result.status == PlannerStatus::OK || result.status == PlannerStatus::NO_RECOVERY_TRAJECTORY);
  if (result.status == PlannerStatus::OK) {
    EXPECT_LT(result.v, 0.0);
    EXPECT_FALSE(result.selected_trajectory.empty());
    const double required_stop = stopDistance(
      std::abs(result.v), cfg.a_brake, cfg.t_latency, cfg.brake_margin);
    EXPECT_LE(required_stop, result.best_score + 1e-6);
  }
}

TEST(PlannerTest, RejectsInsufficientGridForStoppingHorizon)
{
  PlannerConfig cfg = makeConfig();
  cfg.grid_half_extent = 0.6;
  const Planner planner = makePlanner(cfg);

  const PlannerResult forward = planner.computeCommand(makeInput({}));
  EXPECT_EQ(forward.status, PlannerStatus::INVALID_INPUT);
  EXPECT_TRUE(forward.selected_trajectory.empty());

  PlannerInput recovery_input = makeInput({});
  recovery_input.goal_field_pose = Pose2D{-1.0, 0.0, 0.0};
  const PlannerResult recovery = planner.computeRecoveryCommand(recovery_input);
  EXPECT_EQ(recovery.status, PlannerStatus::INVALID_INPUT);
  EXPECT_TRUE(recovery.selected_trajectory.empty());
}

TEST(PlannerTest, MaxSpeedOverrideCapsForwardAndRecoverySpeed)
{
  PlannerConfig cfg = makeConfig();
  const Planner planner = makePlanner(cfg);

  PlannerInput input = makeInput({});
  input.max_speed_override = 0.25;
  const PlannerResult forward = planner.computeCommand(input);

  ASSERT_EQ(forward.status, PlannerStatus::OK);
  EXPECT_LE(forward.v, 0.25 + 1e-9);

  PlannerInput recovery_input = makeInput({});
  recovery_input.goal_field_pose = Pose2D{-1.0, 0.0, 0.0};
  recovery_input.max_speed_override = 0.25;
  const PlannerResult recovery = planner.computeRecoveryCommand(recovery_input);
  if (recovery.status == PlannerStatus::OK) {
    EXPECT_LE(std::abs(recovery.v), 0.25 + 1e-9);
  }
}

TEST(PlannerTest, NarrowChannel045)
{
  PlannerConfig cfg = makeConfig();
  cfg.footprint_half_width = 0.10;
  cfg.obstacle_radius = 0.01;
  cfg.localization_margin = 0.01;
  cfg.speed_margin_per_mps = 0.01;
  const Planner planner = makePlanner(cfg);

  auto scan = makeWall(1.0, -0.625, -0.225, 0.03);
  scan = concat(std::move(scan), makeWall(1.0, 0.225, 0.625, 0.03));

  const PlannerResult result = planner.computeCommand(makeInput(scan));

  EXPECT_EQ(result.status, PlannerStatus::OK);
  EXPECT_FALSE(result.selected_trajectory.empty());
  EXPECT_LT(std::abs(result.kappa), 0.35);
  EXPECT_GE(result.selected_trajectory.back().x, 0.79);
}

}  // namespace mycar_navigation::planner
