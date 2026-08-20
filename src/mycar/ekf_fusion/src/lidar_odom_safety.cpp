#include "robot_localization/lidar_odom_safety.hpp"

#include <cmath>
#include <limits>

namespace robot_localization
{

const char * lidarSafetyReasonName(LidarSafetyReason reason)
{
  switch (reason) {
    case LidarSafetyReason::Accepted: return "accepted";
    case LidarSafetyReason::Disabled: return "disabled";
    case LidarSafetyReason::ZeroTimestamp: return "zero_timestamp";
    case LidarSafetyReason::TimestampRegression: return "timestamp_regression";
    case LidarSafetyReason::Stale: return "stale";
    case LidarSafetyReason::Future: return "future_timestamp";
    case LidarSafetyReason::Cooldown: return "cooldown";
    default: return "unknown";
  }
}

LidarSafetyDecision evaluateLidarSafety(
  const LidarSafetyConfig & config,
  const LidarSafetyInput & input)
{
  if (!config.enabled) {
    return {false, LidarSafetyReason::Disabled};
  }
  if (!std::isfinite(input.stamp_sec) || input.stamp_sec <= 0.0) {
    return {false, LidarSafetyReason::ZeroTimestamp};
  }
  if (input.have_last_received_stamp &&
    input.stamp_sec <= input.last_received_stamp_sec)
  {
    return {false, LidarSafetyReason::TimestampRegression};
  }
  const double age = input.now_sec - input.stamp_sec;
  if (age > config.stale_timeout_sec) {
    return {false, LidarSafetyReason::Stale};
  }
  if (age < -config.max_future_sec) {
    return {false, LidarSafetyReason::Future};
  }
  if (input.have_last_accepted_stamp && config.min_update_interval_sec > 0.0 &&
    input.stamp_sec - input.last_accepted_stamp_sec < config.min_update_interval_sec)
  {
    return {false, LidarSafetyReason::Cooldown};
  }
  return {true, LidarSafetyReason::Accepted};
}

bool mahalanobisAccepted(double nis, double sigma_threshold)
{
  return std::isfinite(nis) && nis < sigma_threshold * sigma_threshold;
}

bool positionInnovationAccepted(double innovation_m, double max_innovation_m)
{
  return std::isfinite(innovation_m) &&
         (!std::isfinite(max_innovation_m) || max_innovation_m <= 0.0 ||
          innovation_m <= max_innovation_m);
}

PositionClampResult computePositionClamp(
  double update_x,
  double update_y,
  double max_update_m)
{
  PositionClampResult result;
  result.requested_norm = std::hypot(update_x, update_y);
  result.applied_norm = result.requested_norm;
  if (std::isfinite(max_update_m) && max_update_m > 0.0 &&
    result.requested_norm > max_update_m)
  {
    result.scale = max_update_m / result.requested_norm;
    result.applied_norm = max_update_m;
    result.clamped = true;
  }
  return result;
}

YawClampResult computeYawClamp(double yaw_update, double max_update_rad)
{
  YawClampResult result;
  result.requested_abs = std::abs(yaw_update);
  result.applied_abs = result.requested_abs;
  if (std::isfinite(max_update_rad) && max_update_rad > 0.0 &&
    result.requested_abs > max_update_rad)
  {
    result.scale = max_update_rad / result.requested_abs;
    result.applied_abs = max_update_rad;
    result.clamped = true;
  }
  return result;
}

}  // namespace robot_localization
