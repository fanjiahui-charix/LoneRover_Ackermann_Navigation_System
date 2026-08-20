#include "simple_lidar_odom/lidar_odom_policy.hpp"

#include <algorithm>
#include <cmath>

namespace simple_lidar_odom
{

const char * qualityLevelName(QualityLevel level)
{
  switch (level) {
    case QualityLevel::High:
      return "HIGH";
    case QualityLevel::Medium:
      return "MEDIUM";
    default:
      return "LOW";
  }
}

QualityLevel classifyQuality(
  const QualityMetrics & metrics,
  const QualityThresholds & thresholds)
{
  if (!metrics.valid || !std::isfinite(metrics.mean_residual) ||
    !std::isfinite(metrics.hessian_condition) ||
    !std::isfinite(metrics.correction_norm))
  {
    return QualityLevel::Low;
  }

  const bool high =
    metrics.cone_correspondences >= thresholds.min_cone_correspondences &&
    metrics.cone_baseline >= thresholds.min_cone_baseline_m &&
    metrics.cone_angle_span >= thresholds.min_cone_angle_span_rad &&
    metrics.mean_residual <= thresholds.max_mean_residual_m &&
    metrics.min_hessian_eigenvalue >= thresholds.min_hessian_eigenvalue &&
    metrics.hessian_condition <= thresholds.max_hessian_condition &&
    metrics.correction_norm <= thresholds.max_correction_norm_m;
  return high ? QualityLevel::High : QualityLevel::Medium;
}

PublishDecision evaluatePublishPolicy(
  QualityLevel quality,
  bool measurement_valid,
  double stamp_s,
  const PublishPolicyConfig & config,
  const PublishPolicyState & state)
{
  if (!measurement_valid) {
    return {false, config.invalid_publish_drop ? "invalid_drop" : "invalid_policy_unsupported"};
  }
  if (config.high_only && quality != QualityLevel::High) {
    return {false, quality == QualityLevel::Medium ? "quality_medium" : "quality_low"};
  }
  if (quality == QualityLevel::Low) {
    return {false, "quality_low"};
  }
  if (quality == QualityLevel::High &&
    state.consecutive_high_frames < std::max(1, config.extra_high_streak))
  {
    return {false, "high_streak"};
  }
  if (state.frames_since_publish < std::max(0, config.cooldown_frames)) {
    return {false, "cooldown"};
  }
  if (state.have_last_publish_stamp &&
    stamp_s - state.last_publish_stamp_s < config.min_publish_interval_s)
  {
    return {false, "min_publish_interval"};
  }
  return {true, "accepted"};
}

}  // namespace simple_lidar_odom
