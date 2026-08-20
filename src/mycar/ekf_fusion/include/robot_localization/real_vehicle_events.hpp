#ifndef ROBOT_LOCALIZATION__REAL_VEHICLE_EVENTS_HPP_
#define ROBOT_LOCALIZATION__REAL_VEHICLE_EVENTS_HPP_

namespace robot_localization
{

struct RealVehicleEventInput
{
  bool imu_fresh{false};
  bool wheel_fresh{false};
  double imu_ax{0.0};
  double imu_ay{0.0};
  double wheel_acceleration{0.0};
  double wheel_yaw_rate{0.0};
  double imu_yaw_rate{0.0};
};

struct RealVehicleEventConfig
{
  double planar_accel_threshold{3.859514307};
  double wheel_accel_threshold{2.621150216};
  double yaw_rate_disagreement_threshold{0.8143556116};
  double process_accel_multiplier{1.251034404};
  double process_disagreement_multiplier{0.1906219574};
};

struct RealVehicleEventDecision
{
  bool planar_accel{false};
  bool wheel_accel{false};
  bool yaw_rate_disagreement{false};
  double process_noise_scale{1.0};
};

RealVehicleEventDecision evaluateRealVehicleEvents(
  const RealVehicleEventInput & input,
  const RealVehicleEventConfig & config);

}  // namespace robot_localization

#endif  // ROBOT_LOCALIZATION__REAL_VEHICLE_EVENTS_HPP_
