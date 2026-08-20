#ifndef ORIGINCARPRO_BASE_IMU_FILTER_HPP_
#define ORIGINCARPRO_BASE_IMU_FILTER_HPP_

#include <string>

#include <geometry_msgs/msg/quaternion.hpp>

namespace origincarpro_base
{

struct ImuSample
{
  double ax = 0.0;
  double ay = 0.0;
  double az = 0.0;

  double gx = 0.0;
  double gy = 0.0;
  double gz = 0.0;

  double mx = 0.0;
  double my = 0.0;
  double mz = 0.0;
};

class ImuFilter
{
public:
  ImuFilter();

  void configure(
    const std::string & mode,
    double roll_pitch_alpha,
    double yaw_gyro_alpha,
    double mag_declination_rad,
    bool relative_orientation);

  void reset();
  void setRelativeZero();
  void update(const ImuSample & sample, double dt);

  geometry_msgs::msg::Quaternion orientation() const;

private:
  double normalizeAngle(double angle) const;

  std::string mode_;
  double roll_pitch_alpha_;
  double yaw_gyro_alpha_;
  double mag_declination_rad_;
  bool relative_orientation_;

  bool initialized_;
  bool relative_initialized_;

  double roll_;
  double pitch_;
  double yaw_;

  double roll_zero_;
  double pitch_zero_;
  double yaw_zero_;
};

geometry_msgs::msg::Quaternion quaternionFromRPY(double roll, double pitch, double yaw);

}  // namespace origincarpro_base

#endif  // ORIGINCARPRO_BASE_IMU_FILTER_HPP_
