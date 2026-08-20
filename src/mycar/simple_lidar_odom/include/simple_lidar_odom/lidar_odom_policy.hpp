#ifndef SIMPLE_LIDAR_ODOM__LIDAR_ODOM_POLICY_HPP_
#define SIMPLE_LIDAR_ODOM__LIDAR_ODOM_POLICY_HPP_

#include <string>

namespace simple_lidar_odom
{

enum class QualityLevel { Low, Medium, High };

const char * qualityLevelName(QualityLevel level);

struct QualityMetrics
{
  bool valid{false};
  int cone_correspondences{0};
  double cone_baseline{0.0};
  double cone_angle_span{0.0};
  double mean_residual{0.0};
  double min_hessian_eigenvalue{0.0};
  double hessian_condition{0.0};
  double correction_norm{0.0};
};

struct QualityThresholds
{
  int min_cone_correspondences{4};
  double min_cone_baseline_m{1.412672112};
  double min_cone_angle_span_rad{1.583992435};
  double max_mean_residual_m{0.02682085338};
  double min_hessian_eigenvalue{0.3773939394};
  double max_hessian_condition{1531.869725};
  double max_correction_norm_m{0.4973780206};
};

QualityLevel classifyQuality(
  const QualityMetrics & metrics,
  const QualityThresholds & thresholds);

struct PublishPolicyConfig
{
  bool high_only{true};
  bool invalid_publish_drop{true};
  int extra_high_streak{1};
  int cooldown_frames{3};
  double min_publish_interval_s{0.1389967094};
};

struct PublishPolicyState
{
  int consecutive_high_frames{0};
  int frames_since_publish{0};
  bool have_last_publish_stamp{false};
  double last_publish_stamp_s{0.0};
};

struct PublishDecision
{
  bool publish{false};
  std::string reason{"quality_low"};
};

PublishDecision evaluatePublishPolicy(
  QualityLevel quality,
  bool measurement_valid,
  double stamp_s,
  const PublishPolicyConfig & config,
  const PublishPolicyState & state);

}  // namespace simple_lidar_odom

#endif  // SIMPLE_LIDAR_ODOM__LIDAR_ODOM_POLICY_HPP_
