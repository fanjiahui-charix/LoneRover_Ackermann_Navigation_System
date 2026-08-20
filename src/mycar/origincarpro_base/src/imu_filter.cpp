#include "origincarpro_base/imu_filter.hpp"

#include <cmath>

#include <tf2/LinearMath/Quaternion.h>

namespace origincarpro_base
{

namespace
{
constexpr double kPi = 3.14159265358979323846;
}

geometry_msgs::msg::Quaternion quaternionFromRPY(double roll, double pitch, double yaw)
{
  tf2::Quaternion q;
  q.setRPY(roll, pitch, yaw);
  q.normalize();

  geometry_msgs::msg::Quaternion msg;
  msg.x = q.x();
  msg.y = q.y();
  msg.z = q.z();
  msg.w = q.w();
  return msg;
}

ImuFilter::ImuFilter()
: mode_("six_axis"),
  roll_pitch_alpha_(0.98),
  yaw_gyro_alpha_(0.98),
  mag_declination_rad_(0.0),
  relative_orientation_(true),
  initialized_(false),
  relative_initialized_(false),
  roll_(0.0),
  pitch_(0.0),
  yaw_(0.0),
  roll_zero_(0.0),
  pitch_zero_(0.0),
  yaw_zero_(0.0)
{
}

void ImuFilter::configure(
  const std::string & mode,
  double roll_pitch_alpha,
  double yaw_gyro_alpha,
  double mag_declination_rad,
  bool relative_orientation)
{
  mode_ = mode;
  roll_pitch_alpha_ = roll_pitch_alpha;
  yaw_gyro_alpha_ = yaw_gyro_alpha;
  mag_declination_rad_ = mag_declination_rad;
  relative_orientation_ = relative_orientation;
}

void ImuFilter::reset()
{
  initialized_ = false;
  relative_initialized_ = false;
  roll_ = 0.0;
  pitch_ = 0.0;
  yaw_ = 0.0;
  roll_zero_ = 0.0;
  pitch_zero_ = 0.0;
  yaw_zero_ = 0.0;
}

void ImuFilter::setRelativeZero()
{
  roll_zero_ = roll_;
  pitch_zero_ = pitch_;
  yaw_zero_ = yaw_;
  relative_initialized_ = true;
}

void ImuFilter::update(const ImuSample & sample, double dt)
{
  if (dt <= 0.0 || dt > 1.0) {
    dt = 0.02;
  }

  const double acc_roll = std::atan2(sample.ay, sample.az);
  const double acc_pitch = std::atan2(-sample.ax, std::sqrt(sample.ay * sample.ay + sample.az * sample.az));

  double mag_yaw = yaw_;
  if (mode_ == "nine_axis") {
    const double mx_h = sample.mx * std::cos(pitch_) +
                        sample.my * std::sin(roll_) * std::sin(pitch_) +
                        sample.mz * std::cos(roll_) * std::sin(pitch_);

    const double my_h = sample.my * std::cos(roll_) -
                        sample.mz * std::sin(roll_);

    mag_yaw = normalizeAngle(std::atan2(-my_h, mx_h) + mag_declination_rad_);
  }

  if (!initialized_) {
    roll_ = acc_roll;
    pitch_ = acc_pitch;
    yaw_ = (mode_ == "nine_axis") ? mag_yaw : 0.0;
    initialized_ = true;
    return;
  }

  const double gyro_roll = roll_ + sample.gx * dt;
  const double gyro_pitch = pitch_ + sample.gy * dt;
  const double gyro_yaw = normalizeAngle(yaw_ + sample.gz * dt);

  roll_ = roll_pitch_alpha_ * gyro_roll + (1.0 - roll_pitch_alpha_) * acc_roll;
  pitch_ = roll_pitch_alpha_ * gyro_pitch + (1.0 - roll_pitch_alpha_) * acc_pitch;

  if (mode_ == "nine_axis") {
    const double yaw_error = normalizeAngle(mag_yaw - gyro_yaw);
    yaw_ = normalizeAngle(gyro_yaw + (1.0 - yaw_gyro_alpha_) * yaw_error);
  } else {
    yaw_ = gyro_yaw;
  }
}

geometry_msgs::msg::Quaternion ImuFilter::orientation() const
{
  double roll = roll_;
  double pitch = pitch_;
  double yaw = yaw_;

  if (relative_orientation_ && relative_initialized_) {
    roll = normalizeAngle(roll - roll_zero_);
    pitch = normalizeAngle(pitch - pitch_zero_);
    yaw = normalizeAngle(yaw - yaw_zero_);
  }

  return quaternionFromRPY(roll, pitch, yaw);
}

double ImuFilter::normalizeAngle(double angle) const
{
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }

  while (angle < -kPi) {
    angle += 2.0 * kPi;
  }

  return angle;
}

}  // namespace origincarpro_base
