#ifndef ORIGINCARPRO_BASE_ORIGINCARPRO_BASE_HPP_
#define ORIGINCARPRO_BASE_ORIGINCARPRO_BASE_HPP_

#include <array>
#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

#include "origincarpro_base/imu_filter.hpp"
#include "origincarpro_base/protocol.hpp"
#include "origincarpro_base/serial_port.hpp"

namespace origincarpro_base
{

class OriginCarProBase : public rclcpp::Node
{
public:
  explicit OriginCarProBase(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~OriginCarProBase() override;

private:
  void declareAndLoadParameters();
  void openSerial();

  void serialTimerCallback();
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void enforceCmdVelTimeout();
  void sendStopFrame();

  void processIncomingBytes(const uint8_t * data, std::size_t len);
  void handleFrame(const RobotFrame & frame, const rclcpp::Time & stamp);
  rclcpp::Time estimateFrameStamp(
    const RobotFrame & frame,
    const rclcpp::Time & fallback_stamp);
  double clampSensorDt(double dt, const char * label);

  bool updateZeroCalibration(
    const RobotFrame & frame,
    const ImuSample & uncorrected_sample,
    const rclcpp::Time & stamp);
  ImuSample convertImuSample(const RobotFrame & frame, bool apply_runtime_bias) const;
  bool isForcedGyroZero(const RobotFrame & frame) const;

  void publishOdom(const RobotFrame & frame, const rclcpp::Time & stamp, double dt);
  void publishImu(const ImuSample & sample, const rclcpp::Time & stamp, double dt);
  void publishStaticTf();

  std::array<double, 36> getDoubleArray36(const std::string & name);
  std::array<double, 9> getDoubleArray9(const std::string & name);
  std::array<double, 3> getDoubleArray3(const std::string & name);
  std::vector<double> getDoubleVector(const std::string & name, std::size_t expected_size);

  SerialPort serial_;
  std::vector<uint8_t> rx_buffer_;

  rclcpp::TimerBase::SharedPtr serial_timer_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_raw_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_fused_pub_;

  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;

  std::string serial_port_;
  int baud_rate_;

  std::string odom_topic_;
  std::string imu_raw_topic_;
  std::string imu_fused_topic_;
  std::string cmd_vel_topic_;
  double cmd_vel_timeout_sec_;
  std::chrono::steady_clock::time_point last_cmd_vel_time_;
  bool have_cmd_vel_;
  bool stale_stop_sent_;
  bool last_cmd_was_nonzero_;

  std::string odom_frame_id_;
  std::string base_frame_id_;
  std::string base_link_reference_;
  std::string imu_frame_id_;

  std::string imu_fusion_mode_;
  std::string imu_model_;
  double imu_zero_calibration_seconds_;
  bool nine_axis_relative_orientation_;
  bool imu_data_already_in_base_frame_;

  double accel_scale_;
  double gyro_scale_;
  double odom_linear_scale_;
  double odom_angular_scale_;
  double gyro_z_bias_correction_;
  double startup_bias_blend_;
  double odom_linear_deadzone_;
  double imu_yaw_rate_deadzone_;
  int imu_zero_calibration_min_samples_;
  int imu_zero_calibration_min_z_samples_;
  bool ignore_forced_zero_gyro_z_;
  double accel_lsb_per_g_;
  double gyro_lsb_per_dps_;
  double mag_ut_per_lsb_;
  bool use_offline_imu_calibration_;
  std::array<double, 9> acc_ta_;
  std::array<double, 3> acc_ba_;
  std::array<double, 3> offline_gyro_bias_;

  double roll_pitch_alpha_;
  double yaw_gyro_alpha_;
  double mag_declination_rad_;

  std::array<double, 36> odom_pose_covariance_;
  std::array<double, 36> odom_twist_covariance_;
  std::array<double, 9> imu_orientation_covariance_;
  std::array<double, 9> imu_angular_velocity_covariance_;
  std::array<double, 9> imu_linear_acceleration_covariance_;

  double x_;
  double y_;
  double yaw_;

  rclcpp::Time last_odom_time_;
  rclcpp::Time last_imu_time_;
  rclcpp::Time last_serial_frame_stamp_;
  bool have_last_odom_;
  bool have_last_imu_;
  bool have_last_serial_frame_stamp_;
  uint16_t last_serial_seq_;
  uint32_t last_serial_tick_ms_;
  bool have_last_serial_seq_;
  bool have_last_serial_tick_;
  double last_stable_serial_dt_;

  std::array<double, 3> startup_gyro_sum_;
  std::array<double, 3> startup_gyro_bias_;
  int startup_gyro_xy_count_;
  int startup_gyro_z_count_;
  int startup_forced_zero_z_count_;

  rclcpp::Time start_time_;
  rclcpp::Time calibration_start_stamp_;
  bool have_calibration_start_stamp_;
  bool calibration_done_;
  bool imu_relative_zero_set_;

  ImuFilter imu_filter_;
};

}  // namespace origincarpro_base

#endif  // ORIGINCARPRO_BASE_ORIGINCARPRO_BASE_HPP_
