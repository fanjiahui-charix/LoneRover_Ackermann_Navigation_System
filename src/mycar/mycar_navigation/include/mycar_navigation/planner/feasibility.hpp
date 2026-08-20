#ifndef MYCAR_NAVIGATION_PLANNER_FEASIBILITY_HPP_
#define MYCAR_NAVIGATION_PLANNER_FEASIBILITY_HPP_

#include <vector>

#include "mycar_navigation/nav_core/footprint.hpp"
#include "mycar_navigation/nav_core/types.hpp"
#include "mycar_navigation/planner/local_grid.hpp"

namespace mycar_navigation::planner
{

using nav_core::GridIndex;
using nav_core::MapClass;
using nav_core::Point2D;
using nav_core::Pose2D;

struct ArcEvaluation
{
  bool feasible = false;
  double free_length = 0.0;
  bool blocked_by_scan = false;
  bool blocked_by_map = false;
  double soft_cost_sum = 0.0;
};

ArcEvaluation evaluateArc(
  const std::vector<Pose2D> & rollout_poses, double step_length, double min_feasible_length,
  const nav_core::Footprint & fp, const LocalGrid & grid, double resolution);

}  // namespace mycar_navigation::planner

#endif  // MYCAR_NAVIGATION_PLANNER_FEASIBILITY_HPP_
