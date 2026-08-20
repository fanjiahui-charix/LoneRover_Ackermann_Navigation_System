#include "mycar_navigation/planner/scorer.hpp"

#include <cmath>
#include <limits>

namespace mycar_navigation::planner
{

double scoreArc(
  double kappa, double kappa_prev, double free_length,
  double soft_cost_sum, const std::vector<Pose2D> & rollout_poses, const Point2D & goal_base,
  const PlannerConfig & cfg)
{
  if (rollout_poses.empty()) {
    return -std::numeric_limits<double>::max();
  }

  const Pose2D & endpoint = rollout_poses.back();
  const double dx = endpoint.x - goal_base.x;
  const double dy = endpoint.y - goal_base.y;
  const double distance_to_goal = std::hypot(dx, dy);
  const double smooth_penalty = std::abs(kappa - kappa_prev);

  return cfg.w_goal * (-distance_to_goal) +
         cfg.w_clear * free_length -
         cfg.w_smooth * smooth_penalty -
         cfg.w_soft_cost * soft_cost_sum;
}

}  // namespace mycar_navigation::planner
