#include "origincarpro_base/origincarpro_base.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <geometry_msgs/msg/transform_stamped.hpp>

namespace origincarpro_base
{

namespace
{
constexpr double kGravity = 9.80665;
constexpr double kDegToRad = 0.017453292519943295;
constexpr double kDefaultAccelLsbPerG = 16384.0;
constexpr double kDefaultGyroLsbPerDps = 65.5;
constexpr double kDefaultAccelScale = 1670.65;
constexpr double kDefaultGyroScale = 0.00026644;
constexpr double kNominalSensorDt = 0.02;
constexpr double kMinSensorDt = 0.005;
constexpr double kMaxSensorDt = 0.05;
constexpr double kMaxStampHostSkew = 0.50;
constexpr double kPi = 3.14159265358979323846;
}

OriginCarProBase::OriginCarProBase(const rclcpp::NodeOptions & options)
: Node("origincarpro_base_node", options),
  baud_rate_(230400),
  cmd_vel_timeout_sec_(0.25),
  last_cmd_vel_time_(std::chrono::steady_clock::now()),
  have_cmd_vel_(false),
  stale_stop_sent_(false),
  last_cmd_was_nonzero_(false),
  x_(0.0),
  y_(0.0),
  yaw_(0.0),
  have_last_odom_(false),
  have_last_imu_(false),
  have_last_serial_frame_stamp_(false),
  last_serial_seq_(0),
  last_serial_tick_ms_(0),
  have_last_serial_seq_(false),
  have_last_serial_tick_(false),
  last_stable_serial_dt_(kNominalSensorDt),
  startup_gyro_sum_{0.0, 0.0, 0.0},
  startup_gyro_bias_{0.0, 0.0, 0.0},
  startup_gyro_xy_count_(0),
  startup_gyro_z_count_(0),
  startup_forced_zero_z_count_(0),
  calibration_start_stamp_(0, 0, RCL_ROS_TIME),
  have_calibration_start_stamp_(false),
  calibration_done_(false),
  imu_relative_zero_set_(false)
{
  declareAndLoadParameters();

  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 10);
  imu_raw_pub_ = create_publisher<sensor_msgs::msg::Imu>(imu_raw_topic_, 10);
  imu_fused_pub_ = create_publisher<sensor_msgs::msg::Imu>(imu_fused_topic_, 10);

  cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    cmd_vel_topic_,
    10,
    std::bind(&OriginCarProBase::cmdVelCallback, this, std::placeholders::_1));

  static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

  imu_filter_.configure(
    imu_fusion_mode_,
    roll_pitch_alpha_,
    yaw_gyro_alpha_,
    mag_declination_rad_,
    nine_axis_relative_orientation_);

  start_time_ = now();

  openSerial();
  // Clear any command retained by the controller before this ROS process started.
  sendStopFrame();
  publishStaticTf();

  serial_timer_ = create_wall_timer(
    std::chrono::milliseconds(5),
    std::bind(&OriginCarProBase::serialTimerCallback, this));
}

OriginCarProBase::~OriginCarProBase()
{
  sendStopFrame();
  serial_.closePort();
}

void OriginCarProBase::declareAndLoadParameters()
{
  declare_parameter<std::string>("serial_port", "/dev/ttyACM0");
  declare_parameter<int>("baud_rate", 230400);

  declare_parameter<std::string>("odom_topic", "/odom/data_raw");
  declare_parameter<std::string>("imu_raw_topic", "/imu/data_raw");
  declare_parameter<std::string>("imu_fused_topic", "/imu/fused/data_raw");
  declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
  declare_parameter<double>("cmd_vel_timeout_sec", 0.25);

  declare_parameter<std::string>("odom_frame_id", "odom");
  declare_parameter<std::string>("base_frame_id", "base_link");
  declare_parameter<std::string>(
    "base_link_reference", "rear_axle_center_ground_projection");
  declare_parameter<std::string>("imu_frame_id", "imu_link");

  declare_parameter<std::string>("imu_fusion_mode", "six_axis");
  declare_parameter<std::string>("imu_model", "ICM-20948");
  declare_parameter<double>("imu_zero_calibration_seconds", 10.0);
  declare_parameter<bool>("nine_axis_relative_orientation", true);
  declare_parameter<bool>("imu_data_already_in_base_frame", true);

  declare_parameter<double>("accel_scale", kDefaultAccelScale);
  declare_parameter<double>("gyro_scale", kDefaultGyroScale);
  declare_parameter<double>("odom_linear_scale", 1.0);
  declare_parameter<double>("odom_angular_scale", 1.0);
  declare_parameter<double>("gyro_z_bias_correction", 0.0);
  declare_parameter<double>("startup_bias_blend", 1.0);
  declare_parameter<double>("odom_linear_deadzone", 0.0);
  declare_parameter<double>("imu_yaw_rate_deadzone", 0.0);
  declare_parameter<int>("imu_zero_calibration_min_samples", 100);
  declare_parameter<int>("imu_zero_calibration_min_z_samples", 25);
  declare_parameter<bool>("ignore_forced_zero_gyro_z", true);
  declare_parameter<double>("accel_lsb_per_g", kDefaultAccelLsbPerG);
  declare_parameter<double>("gyro_lsb_per_dps", kDefaultGyroLsbPerDps);
  declare_parameter<double>("mag_ut_per_lsb", 0.15);
  declare_parameter<bool>("use_offline_imu_calibration", true);
  declare_parameter<std::vector<double>>(
    "acc_ta",
    std::vector<double>{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0});
  declare_parameter<std::vector<double>>("acc_ba", std::vector<double>{0.0, 0.0, 0.0});
  declare_parameter<std::vector<double>>("gyro_bias", std::vector<double>{0.0, 0.0, 0.0});

  declare_parameter<double>("roll_pitch_alpha", 0.98);
  declare_parameter<double>("yaw_gyro_alpha", 0.98);
  declare_parameter<double>("mag_declination_rad", 0.0);

  declare_parameter<std::vector<double>>("odom_pose_covariance", std::vector<double>(36, 0.0));
  declare_parameter<std::vector<double>>("odom_twist_covariance", std::vector<double>(36, 0.0));
  declare_parameter<std::vector<double>>("imu_orientation_covariance", std::vector<double>(9, 0.0));
  declare_parameter<std::vector<double>>("imu_angular_velocity_covariance", std::vector<double>(9, 0.0));
  declare_parameter<std::vector<double>>("imu_linear_acceleration_covariance", std::vector<double>(9, 0.0));

  declare_parameter<std::vector<double>>("laser_xyz", std::vector<double>{0.067, 0.0, 0.146});
  declare_parameter<std::vector<double>>("laser_rpy", std::vector<double>{0.0, 0.0, 0.0});
  declare_parameter<std::vector<double>>("imu_xyz", std::vector<double>{0.019, -0.036, 0.048});
  declare_parameter<std::vector<double>>("imu_rpy", std::vector<double>{0.0, 0.0, 0.0});

  serial_port_ = get_parameter("serial_port").as_string();
  baud_rate_ = get_parameter("baud_rate").as_int();

  odom_topic_ = get_parameter("odom_topic").as_string();
  imu_raw_topic_ = get_parameter("imu_raw_topic").as_string();
  imu_fused_topic_ = get_parameter("imu_fused_topic").as_string();
  cmd_vel_topic_ = get_parameter("cmd_vel_topic").as_string();
  cmd_vel_timeout_sec_ = get_parameter("cmd_vel_timeout_sec").as_double();
  if (!std::isfinite(cmd_vel_timeout_sec_) || cmd_vel_timeout_sec_ < 0.05) {
    RCLCPP_WARN(
      get_logger(),
      "cmd_vel_timeout_sec must be finite and >= 0.05; using 0.25 s");
    cmd_vel_timeout_sec_ = 0.25;
  }

  odom_frame_id_ = get_parameter("odom_frame_id").as_string();
  base_frame_id_ = get_parameter("base_frame_id").as_string();
  base_link_reference_ = get_parameter("base_link_reference").as_string();
  imu_frame_id_ = get_parameter("imu_frame_id").as_string();
  if (base_frame_id_ != "base_link" ||
    base_link_reference_ != "rear_axle_center_ground_projection")
  {
    throw std::runtime_error(
      "navigation frame contract requires base_link at rear axle center ground projection");
  }

  imu_fusion_mode_ = get_parameter("imu_fusion_mode").as_string();
  imu_model_ = get_parameter("imu_model").as_string();
  imu_zero_calibration_seconds_ = get_parameter("imu_zero_calibration_seconds").as_double();
  nine_axis_relative_orientation_ = get_parameter("nine_axis_relative_orientation").as_bool();
  imu_data_already_in_base_frame_ = get_parameter("imu_data_already_in_base_frame").as_bool();

  accel_scale_ = get_parameter("accel_scale").as_double();
  gyro_scale_ = get_parameter("gyro_scale").as_double();
  odom_linear_scale_ = get_parameter("odom_linear_scale").as_double();
  odom_angular_scale_ = get_parameter("odom_angular_scale").as_double();
  gyro_z_bias_correction_ = get_parameter("gyro_z_bias_correction").as_double();
  startup_bias_blend_ = std::clamp(
    get_parameter("startup_bias_blend").as_double(), 0.0, 1.0);
  odom_linear_deadzone_ = std::max(
    0.0, get_parameter("odom_linear_deadzone").as_double());
  imu_yaw_rate_deadzone_ = std::max(
    0.0, get_parameter("imu_yaw_rate_deadzone").as_double());
  imu_zero_calibration_min_samples_ = std::max(
    1, static_cast<int>(get_parameter("imu_zero_calibration_min_samples").as_int()));
  imu_zero_calibration_min_z_samples_ = std::max(
    1, static_cast<int>(get_parameter("imu_zero_calibration_min_z_samples").as_int()));
  ignore_forced_zero_gyro_z_ = get_parameter("ignore_forced_zero_gyro_z").as_bool();
  accel_lsb_per_g_ = get_parameter("accel_lsb_per_g").as_double();
  gyro_lsb_per_dps_ = get_parameter("gyro_lsb_per_dps").as_double();
  mag_ut_per_lsb_ = get_parameter("mag_ut_per_lsb").as_double();
  use_offline_imu_calibration_ = get_parameter("use_offline_imu_calibration").as_bool();
  acc_ta_ = getDoubleArray9("acc_ta");
  acc_ba_ = getDoubleArray3("acc_ba");
  offline_gyro_bias_ = getDoubleArray3("gyro_bias");

  if (accel_scale_ <= 0.0) {
    accel_scale_ = accel_lsb_per_g_ / kGravity;
    RCLCPP_WARN(
      get_logger(),
      "accel_scale 非法，已按 accel_lsb_per_g=%.3f 换算为 %.6f",
      accel_lsb_per_g_,
      accel_scale_);
  }

  if (gyro_scale_ <= 0.0) {
    gyro_scale_ = kDegToRad / gyro_lsb_per_dps_;
    RCLCPP_WARN(
      get_logger(),
      "gyro_scale 非法，已按 gyro_lsb_per_dps=%.3f 换算为 %.8f",
      gyro_lsb_per_dps_,
      gyro_scale_);
  }

  RCLCPP_INFO(
    get_logger(),
    "IMU six-axis scale: model=%s accel_scale=%.6f gyro_scale=%.8f offline_calib=%s",
    imu_model_.c_str(),
    accel_scale_,
    gyro_scale_,
    use_offline_imu_calibration_ ? "true" : "false");
  RCLCPP_INFO(
    get_logger(),
    "IMU startup bias: duration=%.2fs min_xy=%d min_z=%d blend=%.6f "
    "residual_z=%.9f ignore_forced_zero_z=%s",
    imu_zero_calibration_seconds_,
    imu_zero_calibration_min_samples_,
    imu_zero_calibration_min_z_samples_,
    startup_bias_blend_,
    gyro_z_bias_correction_,
    ignore_forced_zero_gyro_z_ ? "true" : "false");

  roll_pitch_alpha_ = get_parameter("roll_pitch_alpha").as_double();
  yaw_gyro_alpha_ = get_parameter("yaw_gyro_alpha").as_double();
  mag_declination_rad_ = get_parameter("mag_declination_rad").as_double();

  odom_pose_covariance_ = getDoubleArray36("odom_pose_covariance");
  odom_twist_covariance_ = getDoubleArray36("odom_twist_covariance");
  imu_orientation_covariance_ = getDoubleArray9("imu_orientation_covariance");
  imu_angular_velocity_covariance_ = getDoubleArray9("imu_angular_velocity_covariance");
  imu_linear_acceleration_covariance_ = getDoubleArray9("imu_linear_acceleration_covariance");
}

void OriginCarProBase::openSerial()
{
  if (!serial_.openPort(serial_port_, baud_rate_)) {
    RCLCPP_ERROR(get_logger(), "串口打开失败：%s @ %d", serial_port_.c_str(), baud_rate_);
    return;
  }

  RCLCPP_INFO(get_logger(), "串口已打开：%s @ %d", serial_port_.c_str(), baud_rate_);
}

void OriginCarProBase::serialTimerCallback()
{
  enforceCmdVelTimeout();
  if (!serial_.isOpen()) {
    return;
  }

  uint8_t buffer[512];
  const int n = serial_.readBytes(buffer, sizeof(buffer));

  if (n > 0) {
    processIncomingBytes(buffer, static_cast<std::size_t>(n));
  } else if (n < 0) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "串口读取异常");
  }
}

void OriginCarProBase::processIncomingBytes(const uint8_t * data, std::size_t len)
{
  rx_buffer_.insert(rx_buffer_.end(), data, data + len);

  std::vector<RobotFrame> parsed_frames;

  while (rx_buffer_.size() >= ROBOT_RX_LEGACY_LEN) {
    auto header_it = std::find(rx_buffer_.begin(), rx_buffer_.end(), FRAME_HEADER);

    if (header_it == rx_buffer_.end()) {
      rx_buffer_.clear();
      break;
    }

    rx_buffer_.erase(rx_buffer_.begin(), header_it);

    if (rx_buffer_.size() < ROBOT_RX_LEGACY_LEN) {
      break;
    }

    RobotFrame frame;

    if (rx_buffer_[1] == ROBOT_RX_PROTOCOL_V2) {
      if (rx_buffer_.size() < ROBOT_RX_V2_LEN) {
        break;
      }

      std::array<uint8_t, ROBOT_RX_V2_LEN> frame_buf {};
      std::copy_n(rx_buffer_.begin(), ROBOT_RX_V2_LEN, frame_buf.begin());

      if (parseRobotFrameV2(frame_buf, frame)) {
        rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + ROBOT_RX_V2_LEN);
        parsed_frames.push_back(frame);
      } else {
        RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          2000,
          "dropping invalid STM32 v2 feedback frame (type/len/tail/CRC check failed)");
        rx_buffer_.erase(rx_buffer_.begin());
      }

      continue;
    }

    std::array<uint8_t, ROBOT_RX_LEGACY_LEN> frame_buf {};
    std::copy_n(rx_buffer_.begin(), ROBOT_RX_LEGACY_LEN, frame_buf.begin());

    if (parseRobotFrameLegacy(frame_buf, frame)) {
      rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + ROBOT_RX_LEGACY_LEN);
      parsed_frames.push_back(frame);
    } else {
      rx_buffer_.erase(rx_buffer_.begin());
    }
  }

  const std::size_t frame_count = parsed_frames.size();
  if (frame_count == 0) {
    return;
  }

  const rclcpp::Time now_time = now();
  double frame_spacing = last_stable_serial_dt_;

  if (have_last_serial_frame_stamp_) {
    const double span = (now_time - last_serial_frame_stamp_).seconds();
    const double estimated_spacing = span / static_cast<double>(frame_count);
    if (estimated_spacing >= kMinSensorDt && estimated_spacing <= kMaxSensorDt) {
      frame_spacing = estimated_spacing;
      last_stable_serial_dt_ = estimated_spacing;
    } else {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        10000,
        "serial burst spacing estimate %.6f s from span %.6f s over %zu frames is outside "
        "[%.3f, %.3f], using nominal %.3f s",
        estimated_spacing,
        span,
        frame_count,
        kMinSensorDt,
        kMaxSensorDt,
        kNominalSensorDt);
    }
  }

  for (std::size_t i = 0; i < frame_count; ++i) {
    const double age = static_cast<double>(frame_count - 1U - i) * frame_spacing;
    const rclcpp::Time fallback_stamp = now_time - rclcpp::Duration::from_seconds(age);
    const rclcpp::Time stamp = estimateFrameStamp(parsed_frames[i], fallback_stamp);
    if (stamp.nanoseconds() == 0) {
      continue;
    }

    handleFrame(parsed_frames[i], stamp);
    last_serial_frame_stamp_ = stamp;
    have_last_serial_frame_stamp_ = true;
  }
}

rclcpp::Time OriginCarProBase::estimateFrameStamp(
  const RobotFrame & frame,
  const rclcpp::Time & fallback_stamp)
{
  if (!frame.has_protocol_timing) {
    return fallback_stamp;
  }

  if (have_last_serial_seq_) {
    const uint16_t expected_seq = static_cast<uint16_t>(last_serial_seq_ + 1U);
    if (frame.seq == last_serial_seq_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "dropping duplicate STM32 feedback frame seq=%u",
        static_cast<unsigned>(frame.seq));
      return rclcpp::Time(0, 0, get_clock()->get_clock_type());
    }

    if (frame.seq != expected_seq) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "STM32 feedback seq jump: last=%u current=%u expected=%u",
        static_cast<unsigned>(last_serial_seq_),
        static_cast<unsigned>(frame.seq),
        static_cast<unsigned>(expected_seq));
    }
  }

  rclcpp::Time stamp = fallback_stamp;
  bool tick_stamp_valid = false;

  if (have_last_serial_tick_ && have_last_serial_frame_stamp_) {
    const uint32_t tick_delta_ms = frame.tick_ms - last_serial_tick_ms_;
    const double tick_dt = static_cast<double>(tick_delta_ms) / 1000.0;

    if (tick_dt >= kMinSensorDt && tick_dt <= kMaxSensorDt) {
      stamp = last_serial_frame_stamp_ + rclcpp::Duration::from_seconds(tick_dt);
      tick_stamp_valid = true;
      last_stable_serial_dt_ = tick_dt;
    } else {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "STM32 feedback tick delta %.6f s from tick %u -> %u outside [%.3f, %.3f], "
        "using host/burst fallback",
        tick_dt,
        static_cast<unsigned>(last_serial_tick_ms_),
        static_cast<unsigned>(frame.tick_ms),
        kMinSensorDt,
        kMaxSensorDt);
    }
  }

  if (have_last_serial_frame_stamp_ && stamp <= last_serial_frame_stamp_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "STM32 feedback stamp is non-monotonic; using %.3f s fallback step",
      last_stable_serial_dt_);
    stamp = last_serial_frame_stamp_ + rclcpp::Duration::from_seconds(last_stable_serial_dt_);
    tick_stamp_valid = false;
  }

  if (
    tick_stamp_valid &&
    std::abs((stamp - fallback_stamp).seconds()) > kMaxStampHostSkew)
  {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "STM32 tick-derived stamp differs from host time by more than %.2f s; "
      "re-anchoring to host/burst time",
      kMaxStampHostSkew);
    stamp = fallback_stamp;
  }

  last_serial_seq_ = frame.seq;
  last_serial_tick_ms_ = frame.tick_ms;
  have_last_serial_seq_ = true;
  have_last_serial_tick_ = true;

  return stamp;
}

void OriginCarProBase::handleFrame(const RobotFrame & frame, const rclcpp::Time & stamp)
{
  const ImuSample uncorrected_sample = convertImuSample(frame, false);

  if (!updateZeroCalibration(frame, uncorrected_sample, stamp)) {
    return;
  }

  const ImuSample sample = convertImuSample(frame, true);

  double dt_odom = 0.02;
  if (have_last_odom_) {
    dt_odom = (stamp - last_odom_time_).seconds();
    dt_odom = clampSensorDt(dt_odom, "odom");
  }
  last_odom_time_ = stamp;
  have_last_odom_ = true;

  double dt_imu = 0.02;
  if (have_last_imu_) {
    dt_imu = (stamp - last_imu_time_).seconds();
    dt_imu = clampSensorDt(dt_imu, "imu");
  }

  publishOdom(frame, stamp, dt_odom);
  publishImu(sample, stamp, dt_imu);

  last_imu_time_ = stamp;
  have_last_imu_ = true;
}

double OriginCarProBase::clampSensorDt(double dt, const char * label)
{
  if (dt < kMinSensorDt || dt > kMaxSensorDt) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      10000,
      "%s dt %.6f s outside [%.3f, %.3f], using %.3f s for 50 Hz base",
      label,
      dt,
      kMinSensorDt,
      kMaxSensorDt,
      kNominalSensorDt);
    return kNominalSensorDt;
  }

  return dt;
}

bool OriginCarProBase::updateZeroCalibration(
  const RobotFrame & frame,
  const ImuSample & uncorrected_sample,
  const rclcpp::Time & stamp)
{
  if (calibration_done_) {
    return true;
  }

  if (!have_calibration_start_stamp_) {
    calibration_start_stamp_ = stamp;
    have_calibration_start_stamp_ = true;
  }

  const double elapsed = (stamp - calibration_start_stamp_).seconds();

  if (elapsed < imu_zero_calibration_seconds_) {
    startup_gyro_sum_[0] += uncorrected_sample.gx;
    startup_gyro_sum_[1] += uncorrected_sample.gy;
    ++startup_gyro_xy_count_;

    if (isForcedGyroZero(frame)) {
      ++startup_forced_zero_z_count_;
    } else {
      startup_gyro_sum_[2] += uncorrected_sample.gz;
      ++startup_gyro_z_count_;
    }

    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      1000,
      "IMU startup bias collecting: %.1f/%.1fs xy_samples=%d z_samples=%d "
      "forced_zero_z=%d",
      elapsed,
      imu_zero_calibration_seconds_,
      startup_gyro_xy_count_,
      startup_gyro_z_count_,
      startup_forced_zero_z_count_);

    return false;
  }

  if (startup_gyro_xy_count_ > 0) {
    startup_gyro_bias_[0] =
      startup_gyro_sum_[0] / static_cast<double>(startup_gyro_xy_count_);
    startup_gyro_bias_[1] =
      startup_gyro_sum_[1] / static_cast<double>(startup_gyro_xy_count_);
  }
  if (startup_gyro_z_count_ >= imu_zero_calibration_min_z_samples_) {
    startup_gyro_bias_[2] =
      startup_gyro_sum_[2] / static_cast<double>(startup_gyro_z_count_);
  } else {
    startup_gyro_bias_[2] = 0.0;
    RCLCPP_WARN(
      get_logger(),
      "IMU startup gyro.z bias unavailable: valid_z_samples=%d < min_z_samples=%d; "
      "STM32 forced-zero samples=%d. Using startup_z=0 and residual_z=%.9f only.",
      startup_gyro_z_count_,
      imu_zero_calibration_min_z_samples_,
      startup_forced_zero_z_count_,
      gyro_z_bias_correction_);
  }
  if (startup_gyro_xy_count_ < imu_zero_calibration_min_samples_) {
    RCLCPP_WARN(
      get_logger(),
      "IMU startup calibration collected only %d xy samples (configured minimum %d)",
      startup_gyro_xy_count_,
      imu_zero_calibration_min_samples_);
  }

  calibration_done_ = true;
  imu_relative_zero_set_ = false;
  have_last_odom_ = false;
  have_last_imu_ = false;
  x_ = 0.0;
  y_ = 0.0;
  yaw_ = 0.0;

  imu_filter_.reset();

  RCLCPP_INFO(
    get_logger(),
    "IMU startup bias frozen: duration=%.3fs xy_samples=%d z_samples=%d forced_zero_z=%d "
    "startup=(%.9f, %.9f, %.9f) blend=%.6f residual_z=%.9f final_z=%.9f",
    elapsed,
    startup_gyro_xy_count_,
    startup_gyro_z_count_,
    startup_forced_zero_z_count_,
    startup_gyro_bias_[0],
    startup_gyro_bias_[1],
    startup_gyro_bias_[2],
    startup_bias_blend_,
    gyro_z_bias_correction_,
    startup_bias_blend_ * startup_gyro_bias_[2] + gyro_z_bias_correction_);

  return true;
}

bool OriginCarProBase::isForcedGyroZero(const RobotFrame & frame) const
{
  return ignore_forced_zero_gyro_z_ && frame.flag_stop != 0U && frame.gyro_z_raw == 0;
}

ImuSample OriginCarProBase::convertImuSample(
  const RobotFrame & frame,
  bool apply_runtime_bias) const
{
  ImuSample s;

  double ax = static_cast<double>(frame.acc_x_raw) / accel_scale_;
  double ay = static_cast<double>(frame.acc_y_raw) / accel_scale_;
  double az = static_cast<double>(frame.acc_z_raw) / accel_scale_;

  double gx = static_cast<double>(frame.gyro_x_raw) * gyro_scale_;
  double gy = static_cast<double>(frame.gyro_y_raw) * gyro_scale_;
  double gz = static_cast<double>(frame.gyro_z_raw) * gyro_scale_;

  double mx = 0.0;
  double my = 0.0;
  double mz = 0.0;

  if (!imu_data_already_in_base_frame_) {
    const double ax_raw = ax;
    const double ay_raw = ay;
    const double gx_raw = gx;
    const double gy_raw = gy;
    const double mx_raw = mx;
    const double my_raw = my;

    ax = ay_raw;
    ay = -ax_raw;

    gx = gy_raw;
    gy = -gx_raw;

    mx = my_raw;
    my = -mx_raw;
  }

  if (use_offline_imu_calibration_) {
    const double raw_ax = ax - acc_ba_[0];
    const double raw_ay = ay - acc_ba_[1];
    const double raw_az = az - acc_ba_[2];

    ax = acc_ta_[0] * raw_ax + acc_ta_[1] * raw_ay + acc_ta_[2] * raw_az;
    ay = acc_ta_[3] * raw_ax + acc_ta_[4] * raw_ay + acc_ta_[5] * raw_az;
    az = acc_ta_[6] * raw_ax + acc_ta_[7] * raw_ay + acc_ta_[8] * raw_az;

    gx -= offline_gyro_bias_[0];
    gy -= offline_gyro_bias_[1];
    gz -= offline_gyro_bias_[2];
  }

  s.ax = ax;
  s.ay = ay;
  s.az = az;

  if (apply_runtime_bias) {
    s.gx = gx - startup_bias_blend_ * startup_gyro_bias_[0];
    s.gy = gy - startup_bias_blend_ * startup_gyro_bias_[1];
    if (isForcedGyroZero(frame)) {
      // The lower computer replaced the physical gyro.z with an artificial zero.
      // Keep that semantic zero instead of subtracting biases and creating a fake
      // negative yaw rate.
      s.gz = 0.0;
    } else {
      s.gz = gz - startup_bias_blend_ * startup_gyro_bias_[2] - gyro_z_bias_correction_;
      if (std::abs(s.gz) < imu_yaw_rate_deadzone_) {
        s.gz = 0.0;
      }
    }
  } else {
    // Startup calibration consumes this uncorrected (post fixed factory/offline
    // calibration) sample. It must never contain the mutable startup estimate or
    // the runtime residual correction.
    s.gx = gx;
    s.gy = gy;
    s.gz = gz;
  }

  s.mx = mx;
  s.my = my;
  s.mz = mz;

  return s;
}

void OriginCarProBase::publishOdom(const RobotFrame & frame, const rclcpp::Time & stamp, double dt)
{
  double vx = frame.vx_mps * odom_linear_scale_;
  double vy = frame.vy_mps * odom_linear_scale_;
  const double wz = frame.wz_radps * odom_angular_scale_;

  if (std::abs(vx) < odom_linear_deadzone_) {
    vx = 0.0;
  }
  if (std::abs(vy) < odom_linear_deadzone_) {
    vy = 0.0;
  }

  x_ += (vx * std::cos(yaw_) - vy * std::sin(yaw_)) * dt;
  y_ += (vx * std::sin(yaw_) + vy * std::cos(yaw_)) * dt;
  yaw_ += wz * dt;

  nav_msgs::msg::Odometry msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = odom_frame_id_;
  msg.child_frame_id = base_frame_id_;

  msg.pose.pose.position.x = x_;
  msg.pose.pose.position.y = y_;
  msg.pose.pose.position.z = 0.0;
  msg.pose.pose.orientation = quaternionFromRPY(0.0, 0.0, yaw_);

  msg.twist.twist.linear.x = vx;
  msg.twist.twist.linear.y = vy;
  msg.twist.twist.angular.z = wz;

  msg.pose.covariance = odom_pose_covariance_;
  msg.twist.covariance = odom_twist_covariance_;

  odom_pub_->publish(msg);
}

void OriginCarProBase::publishImu(
  const ImuSample & sample,
  const rclcpp::Time & stamp,
  double dt)
{
  sensor_msgs::msg::Imu raw_msg;
  raw_msg.header.stamp = stamp;
  raw_msg.header.frame_id = imu_frame_id_;

  raw_msg.orientation_covariance[0] = -1.0;

  raw_msg.angular_velocity.x = sample.gx;
  raw_msg.angular_velocity.y = sample.gy;
  raw_msg.angular_velocity.z = sample.gz;

  raw_msg.linear_acceleration.x = sample.ax;
  raw_msg.linear_acceleration.y = sample.ay;
  raw_msg.linear_acceleration.z = sample.az;

  raw_msg.angular_velocity_covariance = imu_angular_velocity_covariance_;
  raw_msg.linear_acceleration_covariance = imu_linear_acceleration_covariance_;

  imu_raw_pub_->publish(raw_msg);

  imu_filter_.update(sample, dt);

  if (nine_axis_relative_orientation_ && !imu_relative_zero_set_) {
    imu_filter_.setRelativeZero();
    imu_relative_zero_set_ = true;
  }

  sensor_msgs::msg::Imu fused_msg = raw_msg;
  fused_msg.orientation = imu_filter_.orientation();
  fused_msg.orientation_covariance = imu_orientation_covariance_;

  imu_fused_pub_->publish(fused_msg);
}

void OriginCarProBase::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  if (!calibration_done_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "IMU 零点统计未完成，忽略 cmd_vel");
    return;
  }

  if (!serial_.isOpen()) {
    return;
  }

  const auto frame = buildCmdVelFrame(
    msg->linear.x,
    msg->linear.y,
    msg->angular.z);

  if (serial_.writeBytes(frame.data(), frame.size())) {
    last_cmd_vel_time_ = std::chrono::steady_clock::now();
    have_cmd_vel_ = true;
    stale_stop_sent_ = false;
    last_cmd_was_nonzero_ =
      std::abs(msg->linear.x) > 1e-6 ||
      std::abs(msg->linear.y) > 1e-6 ||
      std::abs(msg->angular.z) > 1e-6;
  }
}

void OriginCarProBase::enforceCmdVelTimeout()
{
  if (!have_cmd_vel_ || stale_stop_sent_ || !serial_.isOpen()) {
    return;
  }
  const double age_sec = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - last_cmd_vel_time_).count();
  if (age_sec <= cmd_vel_timeout_sec_) {
    return;
  }

  sendStopFrame();
  stale_stop_sent_ = true;
  if (last_cmd_was_nonzero_) {
    RCLCPP_ERROR(
      get_logger(),
      "cmd_vel stale for %.3f s (limit %.3f s); sent zero command to chassis",
      age_sec, cmd_vel_timeout_sec_);
  }
  last_cmd_was_nonzero_ = false;
}

void OriginCarProBase::sendStopFrame()
{
  if (!serial_.isOpen()) {
    return;
  }
  const auto stop_frame = buildCmdVelFrame(0.0, 0.0, 0.0);
  serial_.writeBytes(stop_frame.data(), stop_frame.size());
}

void OriginCarProBase::publishStaticTf()
{
  const auto make_tf =
    [this](const std::string & child, const std::vector<double> & xyz, const std::vector<double> & rpy)
    {
      geometry_msgs::msg::TransformStamped tf;
      tf.header.stamp = now();
      tf.header.frame_id = base_frame_id_;
      tf.child_frame_id = child;

      tf.transform.translation.x = xyz[0];
      tf.transform.translation.y = xyz[1];
      tf.transform.translation.z = xyz[2];

      tf.transform.rotation = quaternionFromRPY(rpy[0], rpy[1], rpy[2]);

      return tf;
    };
  std::vector<geometry_msgs::msg::TransformStamped> tfs;
  tfs.push_back(make_tf("laser_link", getDoubleVector("laser_xyz", 3), getDoubleVector("laser_rpy", 3)));
  tfs.push_back(make_tf("imu_link", getDoubleVector("imu_xyz", 3), getDoubleVector("imu_rpy", 3)));

  static_tf_broadcaster_->sendTransform(tfs);
}

std::array<double, 36> OriginCarProBase::getDoubleArray36(const std::string & name)
{
  const auto vec = get_parameter(name).as_double_array();
  std::array<double, 36> arr {};
  for (std::size_t i = 0; i < std::min<std::size_t>(36, vec.size()); ++i) {
    arr[i] = vec[i];
  }
  return arr;
}

std::array<double, 9> OriginCarProBase::getDoubleArray9(const std::string & name)
{
  const auto vec = get_parameter(name).as_double_array();
  std::array<double, 9> arr {};
  for (std::size_t i = 0; i < std::min<std::size_t>(9, vec.size()); ++i) {
    arr[i] = vec[i];
  }
  return arr;
}

std::array<double, 3> OriginCarProBase::getDoubleArray3(const std::string & name)
{
  const auto vec = get_parameter(name).as_double_array();
  std::array<double, 3> arr {};
  for (std::size_t i = 0; i < std::min<std::size_t>(3, vec.size()); ++i) {
    arr[i] = vec[i];
  }
  return arr;
}

std::vector<double> OriginCarProBase::getDoubleVector(const std::string & name, std::size_t expected_size)
{
  auto vec = get_parameter(name).as_double_array();

  if (vec.size() != expected_size) {
    RCLCPP_WARN(get_logger(), "参数 %s 长度异常，使用零值", name.c_str());
    vec.assign(expected_size, 0.0);
  }

  return vec;
}

}  // namespace origincarpro_base
