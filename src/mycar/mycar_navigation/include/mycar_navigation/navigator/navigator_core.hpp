#ifndef MYCAR_NAVIGATION_NAVIGATOR_NAVIGATOR_CORE_HPP_
#define MYCAR_NAVIGATION_NAVIGATOR_NAVIGATOR_CORE_HPP_

#include <cstdint>
#include <deque>
#include <string>
#include <utility>
#include <vector>

#include "mycar_navigation/nav_core/field_odom_transform.hpp"
#include "mycar_navigation/nav_core/footprint.hpp"
#include "mycar_navigation/nav_core/map_mask.hpp"
#include "mycar_navigation/nav_core/types.hpp"
#include "mycar_navigation/planner/planner.hpp"
#include "mycar_navigation/planner/planner_types.hpp"

namespace mycar_navigation::navigator
{

using nav_core::Point2D;
using nav_core::Pose2D;

// All stateful navigator decisions (watchdogs, goal handshake, recovery
// latching, arrival, progress, MAP_CONFLICT) live here, ROS-free, so they are
// host-testable. The ROS node is a thin I/O shell around this class.
struct NavCoreConfig
{
  planner::PlannerConfig planner;       // pass-through (footprint/Rmin/speed/brake/sampling/map...)

  double scan_soft_timeout_sec = 0.3;   // soft: limit speed conservatively
  double scan_hard_timeout_sec = 0.6;   // hard: zero speed + SENSOR_TIMEOUT
  double odom_timeout_sec = 0.3;        // odom stale -> ODOM_FAULT
  double odom_jump_dist = 0.5;          // adjacent odom position jump (m)
  double odom_jump_yaw = 1.0;           // yaw jump (rad)
  double scan_odom_max_dt = 0.2;        // max |scan_stamp - odom_stamp|
  double soft_timeout_speed_factor = 0.5;  // speed cap factor under soft timeout

  double transit_tolerance = 0.30;      // transit arrival tolerance (loose, no stop)
  double stop_tolerance = 0.12;         // stop/terminal arrival tolerance (tight, stop)

  int max_recovery_attempts = 5;        // reverse recovery cap
  double progress_min_dist = 0.05;      // minimum progress within window
  double progress_window_sec = 3.0;     // progress monitoring window
  int map_conflict_cycles = 10;         // consecutive map-dominant-blocking -> MAP_CONFLICT

  std::string goal_frame = "field";     // accepted goal frame_id
};

enum class NavState : uint8_t
{
  // First ten align 1:1 with GoalStatus.msg STATE_* values (0..9) for a direct
  // node-side switch mapping.
  ACCEPTED,
  ACTIVE,
  RECOVERING,
  REACHED,
  BLOCKED_NO_TRAJECTORY,
  BLOCKED_NO_PROGRESS,
  SENSOR_TIMEOUT,
  INVALID_GOAL,
  MAP_CONFLICT,
  ODOM_FAULT,
  IDLE  // internal-only (no active goal); node does not publish status for it
};

// The node feeds one snapshot of currently-known inputs per cycle (each with a
// stamp). The core holds the history/latches internally.
struct NavInputs
{
  double now_sec = 0.0;

  bool have_odom = false;
  Pose2D odom_pose;                     // odom frame; core applies tf -> field
  double odom_stamp_sec = 0.0;

  bool have_scan = false;
  std::vector<Point2D> scan_points_base;
  double scan_stamp_sec = 0.0;

  bool have_goal = false;               // is there an active goal
  bool goal_is_new = false;             // new goal arrived this cycle (triggers handshake)
  std::string goal_id;
  std::string goal_frame;
  Pose2D goal_field_pose;
  uint8_t goal_type = 0;
  double goal_max_speed = 0.0;
  double goal_tolerance = 0.0;
};

struct NavOutputs
{
  bool publish_cmd = false;
  double cmd_v = 0.0;
  double cmd_kappa = 0.0;               // node computes omega = v * kappa

  bool publish_status = false;
  std::string status_goal_id;
  NavState state = NavState::IDLE;
  std::string reason;

  std::vector<Pose2D> selected_trajectory;  // base frame, for markers

  int arcs_total = 0;
  int arcs_feasible = 0;
  int arcs_blocked_by_scan = 0;
  int arcs_blocked_by_map = 0;
};

class NavigatorCore
{
public:
  NavigatorCore(
    const NavCoreConfig & cfg, const nav_core::MapMask & map,
    const nav_core::FieldOdomTransform & tf);

  NavOutputs update(const NavInputs & in);
  NavOutputs cancelCurrentGoal(const std::string & goal_id = "");
  NavState state() const;

private:
  NavCoreConfig cfg_;
  planner::Planner planner_;
  nav_core::Footprint footprint_;
  nav_core::MapMask map_;
  nav_core::FieldOdomTransform tf_;

  // ---- persistent state ----
  NavState state_ = NavState::IDLE;
  std::string current_goal_id_;
  bool has_active_goal_ = false;
  bool goal_reached_ = false;           // current goal already satisfied

  bool have_prev_odom_ = false;
  Pose2D prev_odom_pose_;
  double prev_odom_stamp_ = 0.0;

  double prev_v_ = 0.0;
  double prev_kappa_ = 0.0;

  int recovery_attempts_ = 0;
  bool blocked_latched_ = false;        // BLOCKED_NO_TRAJECTORY latch

  int map_conflict_count_ = 0;

  std::deque<std::pair<double, double>> progress_history_;  // (now_sec, distance_to_goal)

  // Helper: build a status-only output for fault/terminal states.
  NavOutputs makeStatus(NavState state, const std::string & reason, double cmd_v, double cmd_kappa);
  void resetForNewGoal(const std::string & goal_id);
  bool checkOdomFault(const NavInputs & in, std::string & reason) const;
  bool footprintOverlapsHardMap(const Pose2D & robot_field_pose) const;
};

}  // namespace mycar_navigation::navigator

#endif  // MYCAR_NAVIGATION_NAVIGATOR_NAVIGATOR_CORE_HPP_
