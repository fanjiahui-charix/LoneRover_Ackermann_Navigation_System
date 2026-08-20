#ifndef MYCAR_NAVIGATION_PLANNER_SCORER_HPP_
#define MYCAR_NAVIGATION_PLANNER_SCORER_HPP_

#include <vector>

#include "mycar_navigation/nav_core/types.hpp"
#include "mycar_navigation/planner/planner_types.hpp"

namespace mycar_navigation::planner
{

using nav_core::GridIndex;
using nav_core::MapClass;
using nav_core::Point2D;
using nav_core::Pose2D;

double scoreArc(
  double kappa, double kappa_prev, double free_length,
  double soft_cost_sum, const std::vector<Pose2D> & rollout_poses, const Point2D & goal_base,
  const PlannerConfig & cfg);

}  // namespace mycar_navigation::planner

#endif  // MYCAR_NAVIGATION_PLANNER_SCORER_HPP_
