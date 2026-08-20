#ifndef MYCAR_NAVIGATION_PLANNER_PLANNER_TYPES_HPP_
#define MYCAR_NAVIGATION_PLANNER_PLANNER_TYPES_HPP_

#include <cstdint>
#include <vector>

#include "mycar_navigation/nav_core/types.hpp"

namespace mycar_navigation::planner
{

using nav_core::GridIndex;
using nav_core::MapClass;
using nav_core::Point2D;
using nav_core::Pose2D;

struct PlannerConfig
{
  double grid_resolution = 0.05;
  double grid_half_extent = 3.0;

  double footprint_half_length = 0.22;
  double footprint_half_width = 0.14;
  double min_turning_radius = 0.35;

  double obstacle_radius = 0.05;
  double localization_margin = 0.05;
  double speed_margin_per_mps = 0.05;

  double max_speed = 1.0;
  double min_speed = 0.05;
  double a_lat_max = 2.0;
  double a_brake = 1.0;
  double a_accel = 1.0;
  double t_latency = 0.2;
  double brake_margin = 0.10;

  int n_curvatures = 31;
  double dt = 0.1;
  double min_horizon = 0.5;

  double goal_approach_distance = 0.5;

  bool treat_unknown_as_forbidden = true;

  double w_goal = 1.0;
  double w_clear = 0.3;
  double w_smooth = 0.2;
  double w_soft_cost = 0.4;

  double min_feasible_length = 0.10;
};

enum class PlannerStatus : uint8_t
{
  OK,
  NO_FORWARD_TRAJECTORY,
  NO_RECOVERY_TRAJECTORY,
  INVALID_INPUT
};

struct PlannerInput
{
  Pose2D robot_field_pose;
  Pose2D goal_field_pose;
  std::vector<Point2D> scan_points_base;
  double prev_v = 0.0;
  double prev_kappa = 0.0;
  double max_speed_override = 0.0;
};

struct PlannerResult
{
  double v = 0.0;
  double kappa = 0.0;
  PlannerStatus status = PlannerStatus::NO_FORWARD_TRAJECTORY;
  double distance_to_goal = 0.0;
  double best_score = 0.0;
  std::vector<Pose2D> selected_trajectory;

  // Diagnostics (additive, default 0) — used by NavigatorCore for MAP_CONFLICT
  // detection and markers. arcs_blocked_by_* count the first-collision layer of
  // each infeasible candidate arc.
  int arcs_total = 0;
  int arcs_feasible = 0;
  int arcs_blocked_by_scan = 0;
  int arcs_blocked_by_map = 0;
  double soft_cost_sum = 0.0;
};

}  // namespace mycar_navigation::planner

#endif  // MYCAR_NAVIGATION_PLANNER_PLANNER_TYPES_HPP_
