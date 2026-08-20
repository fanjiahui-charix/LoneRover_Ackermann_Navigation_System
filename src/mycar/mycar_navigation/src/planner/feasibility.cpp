#include "mycar_navigation/planner/feasibility.hpp"

#include <stdexcept>

namespace mycar_navigation::planner
{

ArcEvaluation evaluateArc(
  const std::vector<Pose2D> & rollout_poses, double step_length, double min_feasible_length,
  const nav_core::Footprint & fp, const LocalGrid & grid, double resolution)
{
  if (step_length < 0.0) {
    throw std::invalid_argument("step_length must be non-negative");
  }
  if (min_feasible_length < 0.0) {
    throw std::invalid_argument("min_feasible_length must be non-negative");
  }
  if (!(resolution > 0.0)) {
    throw std::invalid_argument("resolution must be positive");
  }

  ArcEvaluation evaluation;
  if (rollout_poses.empty()) {
    return evaluation;
  }

  double last_free_length = 0.0;
  for (std::size_t pose_index = 0; pose_index < rollout_poses.size(); ++pose_index) {
    const auto covered_cells = fp.coveredCells(rollout_poses[pose_index], resolution);

    bool scan_collision = false;
    bool map_collision = false;
    for (const GridIndex & cell : covered_cells) {
      scan_collision = scan_collision || grid.isScanOccupied(cell);
      map_collision = map_collision || grid.isMapForbidden(cell);
      if (!scan_collision && !map_collision && grid.isMapSoftCost(cell)) {
        evaluation.soft_cost_sum += 1.0;
      }
      if (scan_collision || map_collision) {
        break;
      }
    }

    if (scan_collision || map_collision) {
      evaluation.free_length = last_free_length;
      evaluation.blocked_by_scan = scan_collision;
      evaluation.blocked_by_map = map_collision;
      evaluation.feasible = evaluation.free_length >= min_feasible_length;
      return evaluation;
    }

    last_free_length = static_cast<double>(pose_index) * step_length;
  }

  evaluation.free_length = last_free_length;
  evaluation.feasible = evaluation.free_length >= min_feasible_length;
  return evaluation;
}

}  // namespace mycar_navigation::planner
