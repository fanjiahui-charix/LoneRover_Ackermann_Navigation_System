#include "mycar_navigation/planner/planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

#include "mycar_navigation/planner/feasibility.hpp"
#include "mycar_navigation/planner/local_grid.hpp"
#include "mycar_navigation/planner/scorer.hpp"
#include "mycar_navigation/planner/speed_limiter.hpp"

namespace mycar_navigation::planner
{
namespace
{

double clampHorizon(double min_horizon, double requested_horizon, double grid_half_extent)
{
  const double required_horizon = std::max(min_horizon, requested_horizon);
  if (grid_half_extent + 1e-9 < required_horizon) {
    return 0.0;
  }
  return required_horizon;
}

bool isFinitePose(const Pose2D & pose)
{
  return std::isfinite(pose.x) && std::isfinite(pose.y) && std::isfinite(pose.yaw);
}

bool isFinitePoint(const Point2D & point)
{
  return std::isfinite(point.x) && std::isfinite(point.y);
}

struct Candidate
{
  double score{-std::numeric_limits<double>::infinity()};
  double v{0.0};
  double kappa{0.0};
  std::vector<Pose2D> rollout;
};

}  // namespace

Planner::Planner(
  const PlannerConfig & cfg, const nav_core::Footprint & fp,
  const nav_core::MapMask & map, const nav_core::FieldOdomTransform & tf)
: cfg_(cfg),
  fp_(fp),
  map_(map),
  tf_(tf),
  integrator_(),
  sampler_(cfg.min_turning_radius)
{
  if (!(cfg_.grid_resolution > 0.0)) {
    throw std::invalid_argument("Planner grid_resolution must be positive");
  }
  if (!(cfg_.grid_half_extent > 0.0)) {
    throw std::invalid_argument("Planner grid_half_extent must be positive");
  }
  if (!(cfg_.dt > 0.0)) {
    throw std::invalid_argument("Planner dt must be positive");
  }
  if (!(cfg_.min_horizon > 0.0)) {
    throw std::invalid_argument("Planner min_horizon must be positive");
  }
  if (cfg_.n_curvatures <= 0) {
    throw std::invalid_argument("Planner n_curvatures must be positive");
  }
}

PlannerResult Planner::computeCommand(const PlannerInput & in) const
{
  PlannerResult result;
  result.status = PlannerStatus::INVALID_INPUT;

  PlannerConfig cfg = cfg_;
  if (in.max_speed_override > 0.0) {
    cfg.max_speed = std::min(cfg.max_speed, in.max_speed_override);
  }

  if (!isFinitePose(in.robot_field_pose) || !isFinitePose(in.goal_field_pose) ||
    !std::isfinite(in.prev_v) || !std::isfinite(in.prev_kappa) ||
    !std::isfinite(in.max_speed_override))
  {
    return result;
  }
  for (const Point2D & point : in.scan_points_base) {
    if (!isFinitePoint(point)) {
      return result;
    }
  }

  LocalGrid grid(cfg.grid_resolution, cfg.grid_half_extent);
  grid.clear();
  const double inflation_radius =
    cfg.obstacle_radius + cfg.localization_margin + cfg.speed_margin_per_mps * cfg.max_speed;
  grid.addScanPoints(in.scan_points_base, inflation_radius);
  grid.addMapLayers(map_, tf_, in.robot_field_pose, cfg.treat_unknown_as_forbidden);

  const Point2D goal_base = tf_.fieldToBase(
    Point2D{in.goal_field_pose.x, in.goal_field_pose.y}, in.robot_field_pose);
  result.distance_to_goal = std::hypot(goal_base.x, goal_base.y);

  const double requested_horizon =
    stopDistance(cfg.max_speed, cfg.a_brake, cfg.t_latency, cfg.brake_margin) +
    cfg.brake_margin;
  const double horizon = clampHorizon(cfg.min_horizon, requested_horizon, cfg.grid_half_extent);
  if (horizon <= 0.0) {
    result.status = PlannerStatus::INVALID_INPUT;
    return result;
  }

  Candidate best_candidate;
  const auto coarse_samples = sampler_.sample(cfg.n_curvatures);
  const auto evaluate_samples = [&](const std::vector<double> & samples, bool count_diag) {
      for (const double kappa : samples) {
        auto rollout = integrator_.integrate(Pose2D{}, 1.0, kappa, horizon, cfg.dt);
        const ArcEvaluation evaluation = evaluateArc(
          rollout, cfg.dt, cfg.min_feasible_length, fp_, grid, cfg.grid_resolution);
        if (count_diag) {
          ++result.arcs_total;
          if (evaluation.feasible) {
            ++result.arcs_feasible;
          } else if (evaluation.blocked_by_scan) {
            ++result.arcs_blocked_by_scan;
          } else if (evaluation.blocked_by_map) {
            ++result.arcs_blocked_by_map;
          }
        }
        if (!evaluation.feasible) {
          continue;
        }

        const double candidate_v = arcSpeed(
          kappa, evaluation.free_length, result.distance_to_goal, in.prev_v, cfg);
        if (candidate_v < cfg.min_speed) {
          continue;
        }

        const double candidate_score = scoreArc(
          kappa, in.prev_kappa, evaluation.free_length, evaluation.soft_cost_sum, rollout, goal_base, cfg);
        const double centered_penalty =
          (std::abs(goal_base.y) < 0.15 && result.distance_to_goal > 0.8) ? 0.05 * std::abs(kappa) : 0.0;
        const double adjusted_score = candidate_score + centered_penalty;
        if (adjusted_score > best_candidate.score) {
          best_candidate.score = adjusted_score;
          best_candidate.v = candidate_v;
          best_candidate.kappa = kappa;
          result.soft_cost_sum = evaluation.soft_cost_sum;
          best_candidate.rollout = std::move(rollout);
        }
      }
    };

  evaluate_samples(coarse_samples, true);

  if (std::isfinite(best_candidate.score) && !best_candidate.rollout.empty()) {
    const double coarse_spacing = sampler_.maxCurvature() * 2.0 /
      static_cast<double>(std::max(1, cfg.n_curvatures - 1));
    const double refine_half_width = std::max(0.05, coarse_spacing);
    const auto refined_samples = sampler_.refineAround(best_candidate.kappa, refine_half_width, 7);
    evaluate_samples(refined_samples, false);
  }

  if (!std::isfinite(best_candidate.score) || best_candidate.rollout.empty()) {
    result.status = PlannerStatus::NO_FORWARD_TRAJECTORY;
    return result;
  }

  result.v = best_candidate.v;
  result.kappa = best_candidate.kappa;
  result.status = PlannerStatus::OK;
  result.best_score = best_candidate.score;
  result.selected_trajectory = std::move(best_candidate.rollout);
  return result;
}

PlannerResult Planner::computeRecoveryCommand(const PlannerInput & in) const
{
  PlannerResult result;
  result.status = PlannerStatus::INVALID_INPUT;

  PlannerConfig cfg = cfg_;
  if (in.max_speed_override > 0.0) {
    cfg.max_speed = std::min(cfg.max_speed, in.max_speed_override);
  }

  if (!isFinitePose(in.robot_field_pose) || !isFinitePose(in.goal_field_pose) ||
    !std::isfinite(in.prev_v) || !std::isfinite(in.prev_kappa) ||
    !std::isfinite(in.max_speed_override))
  {
    return result;
  }
  for (const Point2D & point : in.scan_points_base) {
    if (!isFinitePoint(point)) {
      return result;
    }
  }

  LocalGrid grid(cfg.grid_resolution, cfg.grid_half_extent);
  grid.clear();
  const double inflation_radius =
    cfg.obstacle_radius + cfg.localization_margin + cfg.speed_margin_per_mps * cfg.max_speed;
  grid.addScanPoints(in.scan_points_base, inflation_radius);
  grid.addMapLayers(map_, tf_, in.robot_field_pose, cfg.treat_unknown_as_forbidden);

  const Point2D goal_base = tf_.fieldToBase(
    Point2D{in.goal_field_pose.x, in.goal_field_pose.y}, in.robot_field_pose);
  result.distance_to_goal = std::hypot(goal_base.x, goal_base.y);

  const double requested_horizon =
    stopDistance(cfg.max_speed, cfg.a_brake, cfg.t_latency, cfg.brake_margin) +
    cfg.brake_margin;
  const double horizon = clampHorizon(cfg.min_horizon, requested_horizon, cfg.grid_half_extent);
  if (horizon <= 0.0) {
    result.status = PlannerStatus::INVALID_INPUT;
    return result;
  }

  Candidate best_candidate;
  for (const double kappa : sampler_.sample(cfg.n_curvatures)) {
    auto rollout = integrator_.integrate(Pose2D{}, -1.0, kappa, horizon, cfg.dt);
    const ArcEvaluation evaluation = evaluateArc(
      rollout, cfg.dt, cfg.min_feasible_length, fp_, grid, cfg.grid_resolution);
    if (!evaluation.feasible) {
      continue;
    }

    const double magnitude = arcSpeed(
      kappa, evaluation.free_length, result.distance_to_goal, std::abs(in.prev_v), cfg);
    const double reverse_limit = brakeLimitedSpeed(
      evaluation.free_length, cfg.a_brake, cfg.t_latency, cfg.brake_margin, cfg.max_speed);
    const double reverse_speed_magnitude = std::min({magnitude, reverse_limit, 0.5 * cfg.max_speed});
    if (reverse_speed_magnitude <= 0.0) {
      continue;
    }
    const double reverse_speed = -reverse_speed_magnitude;
    const double candidate_score = evaluation.free_length;
    if (candidate_score > best_candidate.score) {
      best_candidate.score = candidate_score;
      best_candidate.v = reverse_speed;
      best_candidate.kappa = kappa;
      best_candidate.rollout = std::move(rollout);
    }
  }

  if (!std::isfinite(best_candidate.score) || best_candidate.rollout.empty()) {
    result.status = PlannerStatus::NO_RECOVERY_TRAJECTORY;
    return result;
  }

  result.v = best_candidate.v;
  result.kappa = best_candidate.kappa;
  result.status = PlannerStatus::OK;
  result.best_score = best_candidate.score;
  result.selected_trajectory = std::move(best_candidate.rollout);
  return result;
}

}  // namespace mycar_navigation::planner
