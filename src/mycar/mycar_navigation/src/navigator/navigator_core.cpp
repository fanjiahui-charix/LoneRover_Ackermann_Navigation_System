#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "mycar_navigation/navigator/navigator_core.hpp"

namespace mycar_navigation::navigator
{
namespace
{
constexpr double kEps = 1e-9;

bool isFinitePose(const Pose2D & pose)
{
  return std::isfinite(pose.x) && std::isfinite(pose.y) && std::isfinite(pose.yaw);
}

bool isTransitGoal(uint8_t goal_type)
{
  return goal_type == 0U;
}

std::string formatBlockedReason(planner::PlannerStatus status)
{
  switch (status) {
    case planner::PlannerStatus::NO_FORWARD_TRAJECTORY:
      return "no_forward_trajectory";
    case planner::PlannerStatus::NO_RECOVERY_TRAJECTORY:
      return "no_recovery_trajectory";
    case planner::PlannerStatus::INVALID_INPUT:
      return "planner_invalid_input";
    case planner::PlannerStatus::OK:
      break;
  }
  return "planner_unknown";
}
}  // namespace

NavigatorCore::NavigatorCore(
  const NavCoreConfig & cfg, const nav_core::MapMask & map,
  const nav_core::FieldOdomTransform & tf)
: cfg_(cfg),
  planner_(
    [&cfg]() {
      planner::PlannerConfig planner_cfg = cfg.planner;
      return planner_cfg;
    }(),
    nav_core::Footprint(cfg.planner.footprint_half_length, cfg.planner.footprint_half_width),
    map, tf),
  footprint_(cfg.planner.footprint_half_length, cfg.planner.footprint_half_width),
  map_(map),
  tf_(tf)
{
}

NavOutputs NavigatorCore::makeStatus(
  NavState state, const std::string & reason, double cmd_v, double cmd_kappa)
{
  NavOutputs out;
  out.publish_cmd = true;
  out.cmd_v = cmd_v;
  out.cmd_kappa = cmd_kappa;
  out.state = state;
  out.reason = reason;
  out.publish_status = has_active_goal_ && state != NavState::IDLE;
  out.status_goal_id = current_goal_id_;
  return out;
}

void NavigatorCore::resetForNewGoal(const std::string & goal_id)
{
  current_goal_id_ = goal_id;
  has_active_goal_ = true;
  goal_reached_ = false;
  recovery_attempts_ = 0;
  blocked_latched_ = false;
  map_conflict_count_ = 0;
  progress_history_.clear();
  prev_v_ = 0.0;
  prev_kappa_ = 0.0;
}

NavOutputs NavigatorCore::cancelCurrentGoal(const std::string & goal_id)
{
  NavOutputs out;
  if (!goal_id.empty() && has_active_goal_ && goal_id != current_goal_id_) {
    return out;
  }
  current_goal_id_.clear();
  has_active_goal_ = false;
  goal_reached_ = false;
  recovery_attempts_ = 0;
  blocked_latched_ = false;
  map_conflict_count_ = 0;
  progress_history_.clear();
  prev_v_ = 0.0;
  prev_kappa_ = 0.0;
  state_ = NavState::IDLE;
  out.publish_cmd = true;
  out.cmd_v = 0.0;
  out.cmd_kappa = 0.0;
  out.state = NavState::IDLE;
  return out;
}

bool NavigatorCore::checkOdomFault(const NavInputs & in, std::string & reason) const
{
  if (!in.have_odom) {
    reason = "odom_missing";
    return true;
  }
  if (in.now_sec - in.odom_stamp_sec > cfg_.odom_timeout_sec) {
    reason = "odom_timeout";
    return true;
  }
  if (!isFinitePose(in.odom_pose)) {
    reason = "odom_nonfinite";
    return true;
  }
  if (have_prev_odom_) {
    if (in.odom_stamp_sec + kEps < prev_odom_stamp_) {
      reason = "odom_time_reversal";
      return true;
    }
    const double dx = in.odom_pose.x - prev_odom_pose_.x;
    const double dy = in.odom_pose.y - prev_odom_pose_.y;
    if (std::hypot(dx, dy) > cfg_.odom_jump_dist) {
      reason = "odom_position_jump";
      return true;
    }
    if (std::abs(in.odom_pose.yaw - prev_odom_pose_.yaw) > cfg_.odom_jump_yaw) {
      reason = "odom_yaw_jump";
      return true;
    }
  }
  return false;
}

bool NavigatorCore::footprintOverlapsHardMap(const Pose2D & robot_field_pose) const
{
  const auto covered_cells = footprint_.coveredCells(robot_field_pose, map_.resolution());
  for (const nav_core::GridIndex & cell : covered_cells) {
    const nav_core::Point2D world{
      map_.origin().x + (static_cast<double>(cell.i) + 0.5) * map_.resolution(),
      map_.origin().y + (static_cast<double>(cell.j) + 0.5) * map_.resolution()};
    if (map_.classifyWorld(world) == nav_core::MapClass::HARD_FORBIDDEN) {
      return true;
    }
  }
  return false;
}

NavOutputs NavigatorCore::update(const NavInputs & in)
{
  NavOutputs out;
  out.state = state_;

  if (in.goal_is_new) {
    if (in.goal_frame != cfg_.goal_frame) {
      state_ = NavState::INVALID_GOAL;
      has_active_goal_ = false;
      out = makeStatus(NavState::INVALID_GOAL, "invalid_goal_frame", 0.0, 0.0);
      out.publish_status = true;
      out.status_goal_id = in.goal_id;
      return out;
    }
    resetForNewGoal(in.goal_id);
    state_ = NavState::ACCEPTED;
    out = makeStatus(NavState::ACCEPTED, "accepted", 0.0, 0.0);
    out.publish_status = true;
    return out;
  }

  if (!in.have_goal || !has_active_goal_) {
    state_ = NavState::IDLE;
    out.publish_cmd = true;
    out.cmd_v = 0.0;
    out.cmd_kappa = 0.0;
    out.state = NavState::IDLE;
    out.publish_status = false;
    return out;
  }

  std::string odom_fault_reason;
  if (checkOdomFault(in, odom_fault_reason)) {
    state_ = NavState::ODOM_FAULT;
    out = makeStatus(NavState::ODOM_FAULT, odom_fault_reason, 0.0, 0.0);
    return out;
  }

  const bool have_hard_scan_timeout =
    !in.have_scan || (in.now_sec - in.scan_stamp_sec > cfg_.scan_hard_timeout_sec);
  if (have_hard_scan_timeout) {
    state_ = NavState::SENSOR_TIMEOUT;
    out = makeStatus(NavState::SENSOR_TIMEOUT, "scan_hard_timeout", 0.0, 0.0);
    prev_odom_pose_ = in.odom_pose;
    prev_odom_stamp_ = in.odom_stamp_sec;
    have_prev_odom_ = true;
    return out;
  }

  const bool have_soft_scan_timeout =
    (in.now_sec - in.scan_stamp_sec > cfg_.scan_soft_timeout_sec) ||
    (std::abs(in.scan_stamp_sec - in.odom_stamp_sec) > cfg_.scan_odom_max_dt);

  const Pose2D robot_field_pose = tf_.odomToField(in.odom_pose);
  planner::PlannerInput planner_in;
  planner_in.robot_field_pose = robot_field_pose;
  planner_in.goal_field_pose = in.goal_field_pose;
  planner_in.scan_points_base = in.scan_points_base;
  planner_in.prev_v = prev_v_;
  planner_in.prev_kappa = prev_kappa_;
  planner_in.max_speed_override = in.goal_max_speed;

  const double effective_goal_max_speed =
    in.goal_max_speed > 0.0 ? std::min(cfg_.planner.max_speed, in.goal_max_speed) : cfg_.planner.max_speed;

  const bool footprint_map_conflict = footprintOverlapsHardMap(robot_field_pose);
  if (footprint_map_conflict) {
    ++map_conflict_count_;
    if (map_conflict_count_ >= cfg_.map_conflict_cycles) {
      state_ = NavState::MAP_CONFLICT;
      out = makeStatus(NavState::MAP_CONFLICT, "footprint_in_hard_map", 0.0, 0.0);
      prev_odom_pose_ = in.odom_pose;
      prev_odom_stamp_ = in.odom_stamp_sec;
      have_prev_odom_ = true;
      return out;
    }
  }

  if (blocked_latched_) {
    const planner::PlannerResult probe = planner_.computeCommand(planner_in);
    if (probe.status != planner::PlannerStatus::OK) {
      state_ = NavState::BLOCKED_NO_TRAJECTORY;
      out = makeStatus(NavState::BLOCKED_NO_TRAJECTORY, "blocked_latched", 0.0, 0.0);
      out.arcs_total = probe.arcs_total;
      out.arcs_feasible = probe.arcs_feasible;
      out.arcs_blocked_by_scan = probe.arcs_blocked_by_scan;
      out.arcs_blocked_by_map = probe.arcs_blocked_by_map;
      prev_odom_pose_ = in.odom_pose;
      prev_odom_stamp_ = in.odom_stamp_sec;
      have_prev_odom_ = true;
      return out;
    }
    blocked_latched_ = false;
    recovery_attempts_ = 0;
  }

  const planner::PlannerResult result = planner_.computeCommand(planner_in);
  out.arcs_total = result.arcs_total;
  out.arcs_feasible = result.arcs_feasible;
  out.arcs_blocked_by_scan = result.arcs_blocked_by_scan;
  out.arcs_blocked_by_map = result.arcs_blocked_by_map;

  if (result.status == planner::PlannerStatus::INVALID_INPUT) {
    state_ = NavState::ODOM_FAULT;
    out = makeStatus(NavState::ODOM_FAULT, "planner_invalid_input", 0.0, 0.0);
    out.arcs_total = result.arcs_total;
    out.arcs_feasible = result.arcs_feasible;
    out.arcs_blocked_by_scan = result.arcs_blocked_by_scan;
    out.arcs_blocked_by_map = result.arcs_blocked_by_map;
  } else {
    const double default_tolerance = isTransitGoal(in.goal_type) ? cfg_.transit_tolerance : cfg_.stop_tolerance;
    const double tolerance = in.goal_tolerance > 0.0 ? in.goal_tolerance : default_tolerance;
    const double distance = result.distance_to_goal;

    if (distance <= tolerance) {
      state_ = NavState::REACHED;
      goal_reached_ = true;
      recovery_attempts_ = 0;
      blocked_latched_ = false;
      progress_history_.clear();
      out = makeStatus(NavState::REACHED, "goal_reached", isTransitGoal(in.goal_type) ? 0.0 : 0.0, result.kappa);
      out.selected_trajectory = result.selected_trajectory;
    } else if (result.status == planner::PlannerStatus::OK) {
      progress_history_.emplace_back(in.now_sec, distance);
      while (!progress_history_.empty() &&
        in.now_sec - progress_history_.front().first > cfg_.progress_window_sec)
      {
        progress_history_.pop_front();
      }

      if (!footprint_map_conflict) {
        map_conflict_count_ = 0;
      }

      if (map_conflict_count_ >= cfg_.map_conflict_cycles) {
        state_ = NavState::MAP_CONFLICT;
        out = makeStatus(NavState::MAP_CONFLICT, "map_conflict", 0.0, 0.0);
        out.selected_trajectory = result.selected_trajectory;
      } else if (!progress_history_.empty() &&
        in.now_sec - progress_history_.front().first >= cfg_.progress_window_sec - kEps &&
        progress_history_.front().second - distance < cfg_.progress_min_dist)
      {
        state_ = NavState::BLOCKED_NO_PROGRESS;
        out = makeStatus(NavState::BLOCKED_NO_PROGRESS, "no_progress", 0.0, 0.0);
        out.selected_trajectory = result.selected_trajectory;
      } else {
        state_ = NavState::ACTIVE;
        out.publish_cmd = true;
        out.cmd_v = result.v;
        if (have_soft_scan_timeout) {
          const double speed_cap = effective_goal_max_speed * cfg_.soft_timeout_speed_factor;
          out.cmd_v = std::clamp(out.cmd_v, -speed_cap, speed_cap);
        }
        out.cmd_kappa = result.kappa;
        out.state = NavState::ACTIVE;
        out.publish_status = false;
        out.status_goal_id = current_goal_id_;
        out.reason = have_soft_scan_timeout ? "scan_soft_timeout" : "active";
        out.selected_trajectory = result.selected_trajectory;
        prev_v_ = out.cmd_v;
        prev_kappa_ = out.cmd_kappa;
        recovery_attempts_ = 0;
      }
    } else {
      if (result.arcs_total > 0 &&
        result.arcs_blocked_by_map > 0.6 * static_cast<double>(result.arcs_total) &&
        result.arcs_blocked_by_map > result.arcs_blocked_by_scan)
      {
        ++map_conflict_count_;
        if (map_conflict_count_ >= cfg_.map_conflict_cycles) {
          state_ = NavState::MAP_CONFLICT;
          out = makeStatus(NavState::MAP_CONFLICT, "map_dominant_blocking", 0.0, 0.0);
          out.arcs_total = result.arcs_total;
          out.arcs_feasible = result.arcs_feasible;
          out.arcs_blocked_by_scan = result.arcs_blocked_by_scan;
          out.arcs_blocked_by_map = result.arcs_blocked_by_map;
          return out;
        }
      } else {
        map_conflict_count_ = 0;
      }

      const planner::PlannerResult recovery = planner_.computeRecoveryCommand(planner_in);
      out.arcs_total = result.arcs_total;
      out.arcs_feasible = result.arcs_feasible;
      out.arcs_blocked_by_scan = result.arcs_blocked_by_scan;
      out.arcs_blocked_by_map = result.arcs_blocked_by_map;

      if (recovery.status == planner::PlannerStatus::OK && recovery_attempts_ < cfg_.max_recovery_attempts) {
        ++recovery_attempts_;
        state_ = NavState::RECOVERING;
        out.publish_cmd = true;
        out.cmd_v = recovery.v;
        out.cmd_kappa = recovery.kappa;
        out.state = NavState::RECOVERING;
        out.publish_status = true;
        out.status_goal_id = current_goal_id_;
        out.reason = "recovering";
        out.selected_trajectory = recovery.selected_trajectory;
        prev_v_ = out.cmd_v;
        prev_kappa_ = out.cmd_kappa;
      } else {
        blocked_latched_ = true;
        state_ = NavState::BLOCKED_NO_TRAJECTORY;
        out = makeStatus(
          NavState::BLOCKED_NO_TRAJECTORY,
          formatBlockedReason(recovery.status == planner::PlannerStatus::OK ? result.status : recovery.status),
          0.0, 0.0);
      }
    }
  }

  out.publish_status = out.publish_status || (out.state != NavState::ACTIVE && out.state != NavState::IDLE);
  out.status_goal_id = current_goal_id_;
  prev_odom_pose_ = in.odom_pose;
  prev_odom_stamp_ = in.odom_stamp_sec;
  have_prev_odom_ = true;
  return out;
}

NavState NavigatorCore::state() const
{
  return state_;
}

}  // namespace mycar_navigation::navigator
