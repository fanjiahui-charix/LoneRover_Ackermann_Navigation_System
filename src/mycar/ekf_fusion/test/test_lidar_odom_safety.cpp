#include <gtest/gtest.h>

#include "robot_localization/lidar_odom_safety.hpp"
#include "robot_localization/real_vehicle_events.hpp"

namespace
{
using robot_localization::LidarSafetyConfig;
using robot_localization::LidarSafetyInput;
using robot_localization::LidarSafetyReason;

TEST(LidarOdomSafety, RejectsDisabledStaleRegressionAndCooldown)
{
  LidarSafetyConfig config;
  LidarSafetyInput input;

  config.enabled = false;
  EXPECT_EQ(robot_localization::evaluateLidarSafety(config, input).reason,
    LidarSafetyReason::Disabled);
  config.enabled = true;
  EXPECT_EQ(robot_localization::evaluateLidarSafety(config, input).reason,
    LidarSafetyReason::ZeroTimestamp);

  input.have_last_received_stamp = true;
  input.stamp_sec = 1.0;
  input.last_received_stamp_sec = 1.0;
  EXPECT_EQ(robot_localization::evaluateLidarSafety(config, input).reason,
    LidarSafetyReason::TimestampRegression);
  input.have_last_received_stamp = false;

  input.stamp_sec = 1.0;
  input.now_sec = 1.6;
  EXPECT_EQ(robot_localization::evaluateLidarSafety(config, input).reason,
    LidarSafetyReason::Stale);
  input.now_sec = 1.0;

  input.have_last_accepted_stamp = true;
  input.last_accepted_stamp_sec = 0.99;
  EXPECT_EQ(robot_localization::evaluateLidarSafety(config, input).reason,
    LidarSafetyReason::Cooldown);
  input.have_last_received_stamp = true;
  input.last_received_stamp_sec = 0.99;
  input.last_accepted_stamp_sec = 0.5;
  EXPECT_TRUE(robot_localization::evaluateLidarSafety(config, input).accepted)
    << "A received-but-rejected frame must not consume cooldown";
  input.have_last_received_stamp = false;
  input.have_last_accepted_stamp = false;

  EXPECT_FALSE(robot_localization::positionInnovationAccepted(1.0, 0.5));
  EXPECT_TRUE(robot_localization::positionInnovationAccepted(0.4, 0.5));
}

TEST(LidarOdomSafety, MahalanobisAndSingleCorrectionLimitsAreEnforced)
{
  EXPECT_FALSE(robot_localization::mahalanobisAccepted(16.0, 3.0));
  EXPECT_TRUE(robot_localization::mahalanobisAccepted(4.0, 3.0));

  const auto clamp = robot_localization::computePositionClamp(0.20, 0.10, 0.030649);
  EXPECT_TRUE(clamp.clamped);
  EXPECT_LE(clamp.applied_norm, 0.030649 + 1.0e-12);

  const auto yaw_clamp = robot_localization::computeYawClamp(0.8, 0.35);
  EXPECT_TRUE(yaw_clamp.clamped);
  EXPECT_LE(yaw_clamp.applied_abs, 0.35 + 1.0e-12);
}

TEST(RealVehicleEvents, VerifiedTriggersScaleProcessNoiseAndNormalRestoresBase)
{
  robot_localization::RealVehicleEventConfig config;
  robot_localization::RealVehicleEventInput input;
  input.imu_fresh = true;
  input.wheel_fresh = true;
  EXPECT_DOUBLE_EQ(robot_localization::evaluateRealVehicleEvents(
    input, config).process_noise_scale, 1.0);

  input.imu_ax = config.planar_accel_threshold + 0.1;
  EXPECT_TRUE(robot_localization::evaluateRealVehicleEvents(input, config).planar_accel);
  input.imu_ax = 0.0;
  input.wheel_acceleration = config.wheel_accel_threshold + 0.1;
  EXPECT_TRUE(robot_localization::evaluateRealVehicleEvents(input, config).wheel_accel);
  input.wheel_acceleration = 0.0;
  input.wheel_yaw_rate = config.yaw_rate_disagreement_threshold + 0.1;
  EXPECT_TRUE(robot_localization::evaluateRealVehicleEvents(
    input, config).yaw_rate_disagreement);

  input.imu_fresh = false;
  input.wheel_fresh = false;
  EXPECT_DOUBLE_EQ(robot_localization::evaluateRealVehicleEvents(
    input, config).process_noise_scale, 1.0);
}
}  // namespace
