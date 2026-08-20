#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "mycar_navigation/navigator/navigator_core.hpp"

namespace mycar_navigation::navigator
{
namespace
{
using mycar_navigation::nav_core::FieldOdomTransform;
using mycar_navigation::nav_core::MapClass;
using mycar_navigation::nav_core::MapMask;

constexpr double kPi = 3.14159265358979323846;

MapMask buildOpenMap()
{
  return MapMask::fromCells(
    20, 20, 0.1, -1.0, -1.0,
    std::vector<MapClass>(400, MapClass::DRIVABLE));
}

NavCoreConfig makeConfig()
{
  NavCoreConfig cfg;
  cfg.planner.dt = 0.1;
  cfg.planner.max_speed = 1.0;
  cfg.planner.min_speed = 0.05;
  cfg.planner.min_turning_radius = 0.5;
  cfg.planner.n_curvatures = 9;
  cfg.planner.a_lat_max = 2.0;
  cfg.planner.a_brake = 1.0;
  cfg.planner.a_accel = 1.0;
  cfg.planner.t_latency = 0.1;
  cfg.planner.brake_margin = 0.1;
  cfg.planner.footprint_half_length = 0.10;
  cfg.planner.footprint_half_width = 0.08;
  cfg.transit_tolerance = 0.15;
  cfg.stop_tolerance = 0.08;
  cfg.progress_window_sec = 1.0;
  cfg.progress_min_dist = 0.05;
  cfg.map_conflict_cycles = 2;
  cfg.max_recovery_attempts = 2;
  cfg.scan_soft_timeout_sec = 0.3;
  cfg.scan_hard_timeout_sec = 0.6;
  cfg.scan_odom_max_dt = 0.2;
  cfg.soft_timeout_speed_factor = 0.25;
  cfg.odom_timeout_sec = 0.3;
  cfg.odom_jump_dist = 0.5;
  cfg.odom_jump_yaw = 1.0;
  cfg.goal_frame = "field";
  return cfg;
}

NavInputs baseInputs()
{
  NavInputs in;
  in.now_sec = 1.0;
  in.have_odom = true;
  in.odom_pose = Pose2D{0.0, 0.0, 0.0};
  in.odom_stamp_sec = 1.0;
  in.have_scan = true;
  in.scan_stamp_sec = 1.0;
  in.have_goal = true;
  in.goal_id = "g1";
  in.goal_frame = "field";
  in.goal_field_pose = Pose2D{1.0, 0.0, 0.0};
  in.goal_type = 0U;
  in.goal_max_speed = 0.8;
  return in;
}

TEST(NavigatorCoreTest, NewGoalPublishesAcceptedStatus)
{
  NavigatorCore core(makeConfig(), buildOpenMap(), FieldOdomTransform(Pose2D{0.0, 0.0, 0.0}));
  auto in = baseInputs();
  in.goal_is_new = true;

  const NavOutputs out = core.update(in);

  EXPECT_EQ(out.state, NavState::ACCEPTED);
  EXPECT_TRUE(out.publish_status);
  EXPECT_EQ(out.reason, "accepted");
  EXPECT_EQ(out.status_goal_id, "g1");
  EXPECT_DOUBLE_EQ(out.cmd_v, 0.0);
}

TEST(NavigatorCoreTest, InvalidGoalFrameIsRejected)
{
  NavigatorCore core(makeConfig(), buildOpenMap(), FieldOdomTransform(Pose2D{0.0, 0.0, 0.0}));
  auto in = baseInputs();
  in.goal_is_new = true;
  in.goal_frame = "map";

  const NavOutputs out = core.update(in);

  EXPECT_EQ(out.state, NavState::INVALID_GOAL);
  EXPECT_TRUE(out.publish_status);
  EXPECT_EQ(out.reason, "invalid_goal_frame");
  EXPECT_EQ(out.status_goal_id, "g1");
}

TEST(NavigatorCoreTest, MissingGoalKeepsNodeIdle)
{
  NavigatorCore core(makeConfig(), buildOpenMap(), FieldOdomTransform(Pose2D{0.0, 0.0, 0.0}));
  auto in = baseInputs();
  in.have_goal = false;

  const NavOutputs out = core.update(in);

  EXPECT_EQ(out.state, NavState::IDLE);
  EXPECT_TRUE(out.publish_cmd);
  EXPECT_FALSE(out.publish_status);
  EXPECT_DOUBLE_EQ(out.cmd_v, 0.0);
}

TEST(NavigatorCoreTest, OdomTimeoutTriggersFault)
{
  NavigatorCore core(makeConfig(), buildOpenMap(), FieldOdomTransform(Pose2D{0.0, 0.0, 0.0}));
  auto accept = baseInputs();
  accept.goal_is_new = true;
  static_cast<void>(core.update(accept));

  auto in = baseInputs();
  in.now_sec = 2.0;
  in.odom_stamp_sec = 1.5;

  const NavOutputs out = core.update(in);

  EXPECT_EQ(out.state, NavState::ODOM_FAULT);
  EXPECT_TRUE(out.publish_status);
  EXPECT_EQ(out.reason, "odom_timeout");
}

TEST(NavigatorCoreTest, HardScanTimeoutStopsVehicle)
{
  NavigatorCore core(makeConfig(), buildOpenMap(), FieldOdomTransform(Pose2D{0.0, 0.0, 0.0}));
  auto accept = baseInputs();
  accept.goal_is_new = true;
  static_cast<void>(core.update(accept));

  auto in = baseInputs();
  in.now_sec = 1.2;
  in.scan_stamp_sec = 0.5;

  const NavOutputs out = core.update(in);

  EXPECT_EQ(out.state, NavState::SENSOR_TIMEOUT);
  EXPECT_EQ(out.reason, "scan_hard_timeout");
  EXPECT_DOUBLE_EQ(out.cmd_v, 0.0);
  EXPECT_TRUE(out.publish_status);
}

TEST(NavigatorCoreTest, SoftScanOdomSkewCapsSpeedButStaysActive)
{
  NavigatorCore core(makeConfig(), buildOpenMap(), FieldOdomTransform(Pose2D{0.0, 0.0, 0.0}));
  auto accept = baseInputs();
  accept.goal_is_new = true;
  static_cast<void>(core.update(accept));

  auto in = baseInputs();
  in.now_sec = 1.2;
  in.goal_field_pose = Pose2D{1.5, 0.0, 0.0};
  in.odom_stamp_sec = 1.0;
  in.scan_stamp_sec = 0.75;

  const NavOutputs out = core.update(in);

  EXPECT_EQ(out.state, NavState::ACTIVE);
  EXPECT_FALSE(out.publish_status);
  EXPECT_EQ(out.reason, "scan_soft_timeout");
  EXPECT_LE(std::abs(out.cmd_v), makeConfig().planner.max_speed * makeConfig().soft_timeout_speed_factor + 1e-9);
}

TEST(NavigatorCoreTest, TransitGoalWithinToleranceMarksReached)
{
  NavigatorCore core(makeConfig(), buildOpenMap(), FieldOdomTransform(Pose2D{0.0, 0.0, 0.0}));
  auto accept = baseInputs();
  accept.goal_is_new = true;
  accept.goal_field_pose = Pose2D{0.05, 0.0, 0.0};
  static_cast<void>(core.update(accept));

  auto in = baseInputs();
  in.goal_field_pose = Pose2D{0.05, 0.0, 0.0};

  const NavOutputs out = core.update(in);

  EXPECT_EQ(out.state, NavState::REACHED);
  EXPECT_TRUE(out.publish_status);
  EXPECT_EQ(out.reason, "goal_reached");
  EXPECT_DOUBLE_EQ(out.cmd_v, 0.0);
}

TEST(NavigatorCoreTest, GoalToleranceOverridesDefaultArrivalTolerance)
{
  NavigatorCore core(makeConfig(), buildOpenMap(), FieldOdomTransform(Pose2D{0.0, 0.0, 0.0}));
  auto accept = baseInputs();
  accept.goal_is_new = true;
  accept.goal_field_pose = Pose2D{0.20, 0.0, 0.0};
  accept.goal_tolerance = 0.25;
  static_cast<void>(core.update(accept));

  auto in = baseInputs();
  in.goal_field_pose = Pose2D{0.20, 0.0, 0.0};
  in.goal_tolerance = 0.25;

  const NavOutputs out = core.update(in);

  EXPECT_EQ(out.state, NavState::REACHED);
  EXPECT_EQ(out.reason, "goal_reached");
}

TEST(NavigatorCoreTest, CancelCurrentGoalClearsStateAndCommandsStop)
{
  NavigatorCore core(makeConfig(), buildOpenMap(), FieldOdomTransform(Pose2D{0.0, 0.0, 0.0}));
  auto accept = baseInputs();
  accept.goal_is_new = true;
  static_cast<void>(core.update(accept));

  const NavOutputs cancel = core.cancelCurrentGoal();

  EXPECT_EQ(cancel.state, NavState::IDLE);
  EXPECT_TRUE(cancel.publish_cmd);
  EXPECT_DOUBLE_EQ(cancel.cmd_v, 0.0);
  EXPECT_EQ(core.state(), NavState::IDLE);

  auto in = baseInputs();
  in.have_goal = false;
  const NavOutputs idle = core.update(in);
  EXPECT_EQ(idle.state, NavState::IDLE);
  EXPECT_FALSE(idle.publish_status);
}

TEST(NavigatorCoreTest, GoalMaxSpeedCapsSoftTimeoutSpeed)
{
  NavigatorCore core(makeConfig(), buildOpenMap(), FieldOdomTransform(Pose2D{0.0, 0.0, 0.0}));
  auto accept = baseInputs();
  accept.goal_is_new = true;
  accept.goal_max_speed = 0.2;
  static_cast<void>(core.update(accept));

  auto in = baseInputs();
  in.now_sec = 1.2;
  in.goal_field_pose = Pose2D{1.5, 0.0, 0.0};
  in.goal_max_speed = 0.2;
  in.odom_stamp_sec = 1.0;
  in.scan_stamp_sec = 0.75;

  const NavOutputs out = core.update(in);

  EXPECT_EQ(out.state, NavState::ACTIVE);
  EXPECT_LE(std::abs(out.cmd_v), 0.2 * makeConfig().soft_timeout_speed_factor + 1e-9);
}

TEST(NavigatorCoreTest, ActiveCommandExportsPlannerDiagnostics)
{
  NavigatorCore core(makeConfig(), buildOpenMap(), FieldOdomTransform(Pose2D{0.0, 0.0, 0.0}));
  auto accept = baseInputs();
  accept.goal_is_new = true;
  static_cast<void>(core.update(accept));

  auto in = baseInputs();
  in.now_sec = 1.1;
  in.odom_stamp_sec = 1.1;
  in.scan_stamp_sec = 1.1;
  in.scan_points_base = {Point2D{0.9, 0.1}, Point2D{1.0, -0.1}};

  const NavOutputs out = core.update(in);

  EXPECT_EQ(out.state, NavState::ACTIVE);
  EXPECT_GT(out.arcs_total, 0);
  EXPECT_GE(out.arcs_feasible, 0);
  EXPECT_GE(out.arcs_blocked_by_scan, 0);
  EXPECT_GE(out.arcs_blocked_by_map, 0);
}

TEST(NavigatorCoreTest, BlockedTrajectoryPublishesBlockedStatus)
{
  NavCoreConfig cfg = makeConfig();
  cfg.max_recovery_attempts = 0;
  NavigatorCore core(cfg, buildOpenMap(), FieldOdomTransform(Pose2D{0.0, 0.0, 0.0}));

  auto accept = baseInputs();
  accept.goal_is_new = true;
  static_cast<void>(core.update(accept));

  auto blocked = baseInputs();
  blocked.scan_points_base = {
    Point2D{0.10, 0.00}, Point2D{0.10, 0.05}, Point2D{0.10, -0.05},
    Point2D{0.15, 0.00}, Point2D{0.15, 0.05}, Point2D{0.15, -0.05},
    Point2D{0.20, 0.00}, Point2D{0.20, 0.06}, Point2D{0.20, -0.06},
    Point2D{0.25, 0.00}, Point2D{0.25, 0.06}, Point2D{0.25, -0.06}};

  const NavOutputs out = core.update(blocked);

  EXPECT_EQ(out.state, NavState::BLOCKED_NO_TRAJECTORY);
  EXPECT_TRUE(out.publish_status);
  EXPECT_FALSE(out.reason.empty());
  EXPECT_DOUBLE_EQ(out.cmd_v, 0.0);
}

TEST(NavigatorCoreTest, RepeatedRecoveryFailureLatchesBlocked)
{
  NavCoreConfig cfg = makeConfig();
  cfg.max_recovery_attempts = 0;
  NavigatorCore core(cfg, buildOpenMap(), FieldOdomTransform(Pose2D{0.0, 0.0, 0.0}));

  auto accept = baseInputs();
  accept.goal_is_new = true;
  static_cast<void>(core.update(accept));

  auto in = baseInputs();
  in.scan_points_base = {
    Point2D{0.10, 0.00}, Point2D{0.10, 0.05}, Point2D{0.10, -0.05},
    Point2D{0.15, 0.00}, Point2D{0.15, 0.05}, Point2D{0.15, -0.05},
    Point2D{0.20, 0.00}, Point2D{0.20, 0.06}, Point2D{0.20, -0.06},
    Point2D{0.25, 0.00}, Point2D{0.25, 0.06}, Point2D{0.25, -0.06}};

  const NavOutputs out = core.update(in);

  EXPECT_EQ(out.state, NavState::BLOCKED_NO_TRAJECTORY);
  EXPECT_TRUE(out.publish_status);
  EXPECT_FALSE(out.reason.empty());
  EXPECT_DOUBLE_EQ(out.cmd_v, 0.0);
}

TEST(NavigatorCoreTest, OdomJumpAfterMotionTriggersFault)
{
  NavigatorCore core(makeConfig(), buildOpenMap(), FieldOdomTransform(Pose2D{0.0, 0.0, 0.0}));
  auto accept = baseInputs();
  accept.goal_is_new = true;
  static_cast<void>(core.update(accept));

  auto nominal = baseInputs();
  nominal.now_sec = 1.1;
  nominal.odom_stamp_sec = 1.1;
  nominal.scan_stamp_sec = 1.1;
  nominal.goal_field_pose = Pose2D{1.0, 0.5, 0.0};
  const NavOutputs nominal_out = core.update(nominal);
  ASSERT_EQ(nominal_out.state, NavState::ACTIVE);

  auto in = baseInputs();
  in.now_sec = 1.2;
  in.odom_stamp_sec = 1.2;
  in.scan_stamp_sec = 1.2;
  in.odom_pose = Pose2D{1.0, 0.0, 0.0};
  in.goal_field_pose = Pose2D{1.0, 0.5, 0.0};

  const NavOutputs out = core.update(in);

  EXPECT_EQ(out.state, NavState::ODOM_FAULT);
  EXPECT_EQ(out.reason, "odom_position_jump");
}

}  // namespace
}  // namespace mycar_navigation::navigator
