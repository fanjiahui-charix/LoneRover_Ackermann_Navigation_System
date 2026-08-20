#include <gtest/gtest.h>

#include "simple_lidar_odom/lidar_odom_policy.hpp"

namespace
{
using simple_lidar_odom::PublishPolicyConfig;
using simple_lidar_odom::PublishPolicyState;
using simple_lidar_odom::QualityLevel;

TEST(LidarOdomPolicy, QualityClassifierProducesLowMediumAndHigh)
{
  simple_lidar_odom::QualityThresholds thresholds;
  simple_lidar_odom::QualityMetrics metrics;
  EXPECT_EQ(simple_lidar_odom::classifyQuality(metrics, thresholds), QualityLevel::Low);

  metrics.valid = true;
  metrics.mean_residual = thresholds.max_mean_residual_m + 0.01;
  metrics.hessian_condition = thresholds.max_hessian_condition;
  metrics.correction_norm = 0.0;
  EXPECT_EQ(simple_lidar_odom::classifyQuality(metrics, thresholds), QualityLevel::Medium);

  metrics.cone_correspondences = thresholds.min_cone_correspondences;
  metrics.cone_baseline = thresholds.min_cone_baseline_m;
  metrics.cone_angle_span = thresholds.min_cone_angle_span_rad;
  metrics.mean_residual = thresholds.max_mean_residual_m;
  metrics.min_hessian_eigenvalue = thresholds.min_hessian_eigenvalue;
  metrics.hessian_condition = thresholds.max_hessian_condition;
  metrics.correction_norm = thresholds.max_correction_norm_m;
  EXPECT_EQ(simple_lidar_odom::classifyQuality(metrics, thresholds), QualityLevel::High);
}

TEST(LidarOdomPolicy, LowAndMediumDoNotPublishInMainMode)
{
  PublishPolicyConfig config;
  config.high_only = true;
  config.invalid_publish_drop = true;
  PublishPolicyState state;
  state.consecutive_high_frames = 5;
  state.frames_since_publish = 20;
  state.have_last_publish_stamp = true;
  state.last_publish_stamp_s = 0.0;

  EXPECT_FALSE(simple_lidar_odom::evaluatePublishPolicy(
    QualityLevel::Low, false, 10.0, config, state).publish);
  EXPECT_EQ(simple_lidar_odom::evaluatePublishPolicy(
    QualityLevel::Low, false, 10.0, config, state).reason, "invalid_drop");
  EXPECT_FALSE(simple_lidar_odom::evaluatePublishPolicy(
    QualityLevel::Medium, true, 10.0, config, state).publish);
}

TEST(LidarOdomPolicy, CooldownAndIntervalAreEnforced)
{
  PublishPolicyConfig config;
  config.cooldown_frames = 3;
  config.min_publish_interval_s = 0.15;
  PublishPolicyState state;
  state.consecutive_high_frames = 1;
  state.frames_since_publish = 2;
  state.have_last_publish_stamp = true;
  state.last_publish_stamp_s = 0.0;
  EXPECT_EQ(simple_lidar_odom::evaluatePublishPolicy(
    QualityLevel::High, true, 1.0, config, state).reason, "cooldown");

  state.frames_since_publish = 3;
  state.last_publish_stamp_s = 0.90;
  EXPECT_EQ(simple_lidar_odom::evaluatePublishPolicy(
    QualityLevel::High, true, 1.0, config, state).reason, "min_publish_interval");
}

TEST(LidarOdomPolicy, HighPublishesWhenAllVerifiedGatesPass)
{
  PublishPolicyConfig config;
  PublishPolicyState state;
  state.consecutive_high_frames = 1;
  state.frames_since_publish = 3;
  state.have_last_publish_stamp = true;
  state.last_publish_stamp_s = 0.0;
  EXPECT_TRUE(simple_lidar_odom::evaluatePublishPolicy(
    QualityLevel::High, true, 1.0, config, state).publish);
}
}  // namespace
