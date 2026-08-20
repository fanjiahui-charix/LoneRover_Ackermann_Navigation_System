#include <gtest/gtest.h>

#include <cmath>

#include "mycar_navigation/planner/speed_limiter.hpp"

namespace mycar_navigation::planner
{
namespace
{
PlannerConfig makeConfig()
{
  PlannerConfig cfg;
  cfg.max_speed = 1.0;
  cfg.min_speed = 0.05;
  cfg.a_lat_max = 2.0;
  cfg.a_brake = 1.0;
  cfg.a_accel = 2.0;
  cfg.t_latency = 0.2;
  cfg.brake_margin = 0.1;
  cfg.dt = 0.1;
  cfg.goal_approach_distance = 0.5;
  return cfg;
}
}  // namespace

TEST(SpeedLimiterTest, CurvatureLimitReturnsMaxForStraight)
{
  EXPECT_DOUBLE_EQ(curvatureSpeedLimit(0.0, 2.0, 1.5), 1.5);
}

TEST(SpeedLimiterTest, StopDistanceMatchesKinematics)
{
  EXPECT_NEAR(stopDistance(1.0, 2.0, 0.2, 0.1), 0.55, 1e-9);
}

TEST(SpeedLimiterTest, BrakeLimitedSpeedShrinksWithFreeLength)
{
  const double far = brakeLimitedSpeed(2.0, 1.0, 0.2, 0.1, 1.0);
  const double near = brakeLimitedSpeed(0.5, 1.0, 0.2, 0.1, 1.0);
  EXPECT_GT(far, near);
}

TEST(SpeedLimiterTest, ApproachSpeedSlowsNearGoal)
{
  EXPECT_LT(approachSpeed(0.1, 0.5, 1.0, 0.05), 1.0);
}

TEST(SpeedLimiterTest, AccelClampLimitsDelta)
{
  EXPECT_NEAR(accelClamp(1.0, 0.0, 2.0, 0.1), 0.2, 1e-9);
}

TEST(SpeedLimiterTest, ArcSpeedCombinesLimits)
{
  const PlannerConfig cfg = makeConfig();
  const double open = arcSpeed(0.0, 3.0, 3.0, 0.8, cfg);
  const double blocked = arcSpeed(0.0, 0.4, 3.0, 0.8, cfg);
  EXPECT_GT(open, blocked);
}

TEST(SpeedLimiterTest, BrakeLimitedSpeedCanFallBelowMinSpeed)
{
  const PlannerConfig cfg = makeConfig();
  const double limited = brakeLimitedSpeed(0.102, cfg.a_brake, cfg.t_latency, cfg.brake_margin, cfg.max_speed);
  EXPECT_GT(limited, 0.0);
  EXPECT_LT(limited, cfg.min_speed);
  EXPECT_LE(stopDistance(limited, cfg.a_brake, cfg.t_latency, cfg.brake_margin), 0.102 + 1e-9);
}

}  // namespace mycar_navigation::planner
