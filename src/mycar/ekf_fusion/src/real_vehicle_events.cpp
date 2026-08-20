#include "robot_localization/real_vehicle_events.hpp"

#include <cmath>

namespace robot_localization
{

RealVehicleEventDecision evaluateRealVehicleEvents(
  const RealVehicleEventInput & input,
  const RealVehicleEventConfig & config)
{
  RealVehicleEventDecision result;
  result.planar_accel = input.imu_fresh &&
    std::hypot(input.imu_ax, input.imu_ay) > config.planar_accel_threshold;
  result.wheel_accel = input.wheel_fresh &&
    std::abs(input.wheel_acceleration) > config.wheel_accel_threshold;
  result.yaw_rate_disagreement = input.imu_fresh && input.wheel_fresh &&
    std::abs(input.wheel_yaw_rate - input.imu_yaw_rate) >
    config.yaw_rate_disagreement_threshold;
  if (result.planar_accel || result.wheel_accel) {
    result.process_noise_scale += config.process_accel_multiplier;
  }
  if (result.yaw_rate_disagreement) {
    result.process_noise_scale += config.process_disagreement_multiplier;
  }
  return result;
}

}  // namespace robot_localization
