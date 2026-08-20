#ifndef ROBOT_LOCALIZATION__LIDAR_ODOM_SAFETY_HPP_
#define ROBOT_LOCALIZATION__LIDAR_ODOM_SAFETY_HPP_

#include <string>

namespace robot_localization
{

enum class LidarSafetyReason
{
  Accepted,
  Disabled,
  ZeroTimestamp,
  TimestampRegression,
  Stale,
  Future,
  Cooldown
};

const char * lidarSafetyReasonName(LidarSafetyReason reason);

struct LidarSafetyConfig
{
  bool enabled{true};
  double stale_timeout_sec{0.25};
  double max_future_sec{0.05};
  double min_update_interval_sec{0.1389967094};
};

struct LidarSafetyInput
{
  double stamp_sec{0.0};
  double now_sec{0.0};
  bool have_last_received_stamp{false};
  double last_received_stamp_sec{0.0};
  bool have_last_accepted_stamp{false};
  double last_accepted_stamp_sec{0.0};
};

struct LidarSafetyDecision
{
  bool accepted{false};
  LidarSafetyReason reason{LidarSafetyReason::Disabled};
};

LidarSafetyDecision evaluateLidarSafety(
  const LidarSafetyConfig & config,
  const LidarSafetyInput & input);

bool mahalanobisAccepted(double nis, double sigma_threshold);

bool positionInnovationAccepted(double innovation_m, double max_innovation_m);

struct PositionClampResult
{
  double scale{1.0};
  double requested_norm{0.0};
  double applied_norm{0.0};
  bool clamped{false};
};

PositionClampResult computePositionClamp(
  double update_x,
  double update_y,
  double max_update_m);

struct YawClampResult
{
  double scale{1.0};
  double requested_abs{0.0};
  double applied_abs{0.0};
  bool clamped{false};
};

YawClampResult computeYawClamp(double yaw_update, double max_update_rad);

}  // namespace robot_localization

#endif  // ROBOT_LOCALIZATION__LIDAR_ODOM_SAFETY_HPP_
