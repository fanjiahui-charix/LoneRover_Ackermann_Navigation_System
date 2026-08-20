#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Dense>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_srvs/srv/empty.hpp"
#include "tf2/exceptions.h"
#include "tf2/time.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "simple_lidar_odom/landmark_extractor.hpp"
#include "simple_lidar_odom/lidar_odom_policy.hpp"
#include "simple_lidar_odom/local_landmark_map.hpp"
#include "simple_lidar_odom/persistent_cone_map.hpp"
#include "simple_lidar_odom/pose_covariance.hpp"
#include "simple_lidar_odom/unique_assignment.hpp"

namespace simple_lidar_odom
{

struct Pose2D
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct TimedPose
{
  rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
  Pose2D pose;
  // Body-frame twist (base_link) carried alongside the pose, used for scan deskew.
  double vx{0.0};
  double vy{0.0};
  double wz{0.0};
};

struct TimedTwist
{
  rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
  double vx{0.0};
  double vy{0.0};
  double wz{0.0};
};

struct MatchStats
{
  Eigen::Matrix3d H{Eigen::Matrix3d::Zero()};
  double weighted_ssr{0.0};
  double total_error{0.0};
  int correspondences{0};
};

struct IcpResult
{
  Pose2D pose;
  bool valid{false};
  bool converged{false};
  int correspondences{0};
  int cone_correspondences{0};
  double mean_error{std::numeric_limits<double>::infinity()};
  double min_hessian_eigenvalue{0.0};
  Eigen::Matrix3d final_hessian{Eigen::Matrix3d::Zero()};
  double weighted_ssr{0.0};
  double cone_baseline{0.0};
  double cone_angle_span{0.0};
  double hessian_condition{std::numeric_limits<double>::infinity()};
  double correction_norm{std::numeric_limits<double>::infinity()};
  double applied_correction_norm{0.0};
  int current_cones{0};
  int fence_support_points{0};
};

static double normalizeAngle(double a)
{
  while (a > M_PI) {
    a -= 2.0 * M_PI;
  }
  while (a < -M_PI) {
    a += 2.0 * M_PI;
  }
  return a;
}

static double angleDiff(double a, double b)
{
  return normalizeAngle(a - b);
}

static Pose2D composePose(const Pose2D & a, const Pose2D & b)
{
  const double c = std::cos(a.yaw);
  const double s = std::sin(a.yaw);
  Pose2D out;
  out.x = a.x + c * b.x - s * b.y;
  out.y = a.y + s * b.x + c * b.y;
  out.yaw = normalizeAngle(a.yaw + b.yaw);
  return out;
}

static Pose2D inversePose(const Pose2D & p)
{
  const double c = std::cos(p.yaw);
  const double s = std::sin(p.yaw);
  Pose2D out;
  out.x = -c * p.x - s * p.y;
  out.y = s * p.x - c * p.y;
  out.yaw = normalizeAngle(-p.yaw);
  return out;
}

// Pose of the body at time t0+tau expressed in the body frame at t0, under a
// constant body twist (vx, vy, wz). Exact SE(2) exponential, with a small-angle
// branch for wz ~ 0. Used to deskew a laser scan over its finite sweep time.
static Pose2D integrateTwist(double vx, double vy, double wz, double tau)
{
  Pose2D d;
  const double dtheta = wz * tau;
  if (std::abs(wz) < 1.0e-6) {
    d.x = vx * tau;
    d.y = vy * tau;
    d.yaw = dtheta;
    return d;
  }
  const double s = std::sin(dtheta);
  const double c = std::cos(dtheta);
  d.x = (vx * s - vy * (1.0 - c)) / wz;
  d.y = (vx * (1.0 - c) + vy * s) / wz;
  d.yaw = normalizeAngle(dtheta);
  return d;
}

class LidarOdomNode : public rclcpp::Node
{
public:
  using Point = Point2;

  LidarOdomNode()
  : Node("simple_lidar_odom"), local_landmark_map_(30)
  {
    scan_topic_ = declare_parameter<std::string>("scan_topic", "/scan");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/lidar_odom");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    publish_tf_ = declare_parameter<bool>("publish_tf", false);

    min_range_ = declare_parameter<double>("min_range", 0.15);
    max_range_ = declare_parameter<double>("max_range", 2.2);
    downsample_step_ = declare_parameter<int>("downsample_step", 1);
    min_scan_points_ = declare_parameter<int>("min_scan_points", 3);

    local_map_frames_ = declare_parameter<int>("local_map_frames", 30);
    local_landmark_map_.setMaxAge(local_map_frames_);

    max_correspondence_dist_ = declare_parameter<double>("max_correspondence_dist", 0.20);
    max_mean_error_ = declare_parameter<double>("max_mean_error", 0.05);
    min_correspondences_ = declare_parameter<int>("min_correspondences", 3);

    icp_max_iterations_ = declare_parameter<int>("icp_max_iterations", 15);
    icp_converge_translation_ = declare_parameter<double>("icp_converge_translation", 0.001);
    icp_converge_rotation_ = declare_parameter<double>("icp_converge_rotation", 0.0005);
    robust_kernel_delta_ = declare_parameter<double>("robust_kernel_delta", 0.08);
    max_icp_step_translation_ = declare_parameter<double>("max_icp_step_translation", 0.10);
    max_icp_step_rotation_ = declare_parameter<double>("max_icp_step_rotation", 0.15);
    lambda_floor_ = declare_parameter<double>("lambda_floor", 1.0e-6);
    meas_noise_std_ = declare_parameter<double>("meas_noise_std", 0.05);
    init_pose_std_ = declare_parameter<double>("init_pose_std", 1.0e3);

    invalid_publish_policy_ = declare_parameter<std::string>("invalid_publish_policy", "drop");
    publish_pose_mode_ = declare_parameter<std::string>("publish_pose_mode", "absolute");
    publish_yaw_ = declare_parameter<bool>("publish_yaw", true);
    high_only_ = declare_parameter<bool>("high_only", true);
    quality_diagnostics_enable_ = declare_parameter<bool>("quality_diagnostics_enable", true);
    if (invalid_publish_policy_ != "drop") {
      RCLCPP_WARN(get_logger(), "Only invalid_publish_policy=drop is supported; forcing drop");
      invalid_publish_policy_ = "drop";
    }
    if (publish_pose_mode_ != "absolute") {
      RCLCPP_WARN(get_logger(), "Only publish_pose_mode=absolute is supported; forcing absolute");
      publish_pose_mode_ = "absolute";
    }
    min_cone_correspondences_ = declare_parameter<int>("min_cone_correspondences", 4);
    min_cone_baseline_m_ = declare_parameter<double>("min_cone_baseline_m", 1.412672112);
    min_cone_angle_span_rad_ = declare_parameter<double>("min_cone_angle_span_rad", 1.583992435);
    max_mean_residual_m_ = declare_parameter<double>("max_mean_residual_m", 0.02682085338);
    min_hessian_eigenvalue_quality_ =
      declare_parameter<double>("min_hessian_eigenvalue", 0.3773939394);
    max_hessian_condition_ = declare_parameter<double>("max_hessian_condition", 1531.869725);
    max_correction_norm_m_ = declare_parameter<double>("max_correction_norm_m", 0.4973780206);
    high_xy_gain_ = declare_parameter<double>("high_xy_gain", 0.9488666884);
    high_yaw_gain_ = declare_parameter<double>("high_yaw_gain", 0.05);
    bootstrap_frames_ = declare_parameter<int>("bootstrap_frames", 4);
    extra_high_streak_ = declare_parameter<int>("extra_high_streak", 1);
    cooldown_frames_ = declare_parameter<int>("cooldown_frames", 3);
    min_publish_interval_s_ = declare_parameter<double>("min_publish_interval_s", 0.1389967094);
    xy_covariance_base_ = declare_parameter<double>("xy_covariance_base", 0.008543882646);
    covariance_residual_scale_ = declare_parameter<double>("covariance_residual_scale", 1.035691223);
    covariance_eigenvalue_scale_ = declare_parameter<double>("covariance_eigenvalue_scale", 0.3400792863);
    covariance_condition_scale_ = declare_parameter<double>("covariance_condition_scale", 0.3368723875);
    covariance_correction_scale_ = declare_parameter<double>("covariance_correction_scale", 0.9162886246);
    yaw_covariance_ = declare_parameter<double>("yaw_covariance", 0.3536275267);

    // Persistent confirmed-cone map: cones denoise via running mean while
    // tentative, then freeze on confirmation as rigid anchors (implicit loop
    // closure on a small fully-visible field). Replaces the keyframe-frozen
    // age-evicted sliding window.
    cone_confirm_hits_ = declare_parameter<int>("cone_confirm_hits", 5);
    cone_tentative_prune_misses_ = declare_parameter<int>("cone_tentative_prune_misses", 10);
    cone_confirm_max_dist_ = declare_parameter<double>("cone_confirm_max_dist", 0.16);
    confirmed_min_reobserve_hits_ = declare_parameter<int>("confirmed_min_reobserve_hits", 2);
    confirmed_prune_misses_ = declare_parameter<int>("confirmed_prune_misses", 0);
    max_persistent_cones_ = declare_parameter<int>("max_persistent_cones", 30);
    reset_pose_on_map_reset_ = declare_parameter<bool>("reset_pose_on_map_reset", true);

    // Prior used when no fused odom is available. Default is a zero-velocity (last
    // pose) prior; constant-velocity extrapolation kept injecting motion while
    // stationary, which seeded the drift direction.
    use_cv_prior_ = declare_parameter<bool>("use_cv_prior", false);

    cone_radius_ = declare_parameter<double>("cone_radius", 0.04);
    cone_radius_min_ = declare_parameter<double>("cone_radius_min", 0.0359875192);
    cone_radius_max_ = declare_parameter<double>("cone_radius_max", 0.0529781543);
    cone_min_pts_ = declare_parameter<int>("cone_min_pts", 5);
    cone_max_pts_ = declare_parameter<int>("cone_max_pts", 21);
    cone_max_span_ = declare_parameter<double>("cone_max_span", 0.2035249622);
    cone_fit_max_rms_ = declare_parameter<double>("cone_fit_max_rms", 0.0062363306);
    fence_min_pts_ = declare_parameter<int>("fence_min_pts", 15);
    fence_split_residual_ = declare_parameter<double>("fence_split_residual", 0.0);
    fence_split_max_depth_ = declare_parameter<int>("fence_split_max_depth", 0);
    fence_aspect_ratio_ = declare_parameter<double>("fence_aspect_ratio", 4.0);
    fence_merge_angle_tol_ = declare_parameter<double>("fence_merge_angle_tol", 0.0756576558);
    fence_merge_dist_tol_ = declare_parameter<double>("fence_merge_dist_tol", 0.0945249897);
    fence_assoc_angle_tol_ = declare_parameter<double>("fence_assoc_angle_tol", 0.10);
    fence_max_residual_pts_ = declare_parameter<int>("fence_max_residual_pts", 30);
    // Fence point-to-line residuals are disabled by default for Phase 1 to keep the
    // validation surface small. NOTE: fences are NOT the stationary-drift cause (an
    // earlier conclusion to that effect came from an offline replay later proven
    // unfaithful to this node; a non-accumulating single-step test showed fences
    // slightly REDUCE per-step bias). The drift is the keyframe-frozen-map fix above.
    // Re-enable fences in Phase 2 once a persistent confirmed-cone world map exists.
    use_fence_residuals_ = declare_parameter<bool>("use_fence_residuals", false);
    seg_break_dist_ = declare_parameter<double>("seg_break_dist", 0.0965708121);

    deskew_source_ = declare_parameter<std::string>("deskew_source", "ekf_twist_lagged");
    ekf_odom_topic_ = declare_parameter<std::string>("ekf_odom_topic", "/odom");
    raw_odom_topic_ = declare_parameter<std::string>("raw_odom_topic", "/odom/data_raw");
    imu_topic_ = declare_parameter<std::string>("imu_topic", "/imu/data_raw");
    deskew_odom_timeout_ = declare_parameter<double>("deskew_odom_timeout", 0.20);
    deskew_imu_timeout_ = declare_parameter<double>("deskew_imu_timeout", 0.20);
    deskew_require_stamp_not_newer_than_scan_ =
      declare_parameter<bool>("deskew_require_stamp_not_newer_than_scan", true);
    deskew_min_lag_sec_ = declare_parameter<double>("deskew_min_lag_sec", 0.0);
    deskew_use_vx_ = declare_parameter<bool>("deskew_use_vx", true);
    deskew_use_vy_ = declare_parameter<bool>("deskew_use_vy", true);
    deskew_use_wz_ = declare_parameter<bool>("deskew_use_wz", true);
    deskew_twist_in_base_frame_ = declare_parameter<bool>("deskew_twist_in_base_frame", true);
    deskew_max_abs_vx_ = declare_parameter<double>("deskew_max_abs_vx", 2.0);
    deskew_max_abs_vy_ = declare_parameter<double>("deskew_max_abs_vy", 0.5);
    deskew_max_abs_wz_ = declare_parameter<double>("deskew_max_abs_wz", 4.0);
    fallback_deskew_source_ = declare_parameter<std::string>("fallback_deskew_source", "raw_odom_imu");

    // Scan motion deskew: undistort each ray by its acquisition time using the
    // fused-odom body twist (scan_time ~99 ms ~= the 10 Hz frame period, so the
    // vehicle keeps turning across one sweep). Requires scan.time_increment > 0.
    use_scan_deskew_ = declare_parameter<bool>("use_scan_deskew", true);

    covariance_min_x_ = declare_parameter<double>("covariance_min_x", 0.0025);
    covariance_min_y_ = declare_parameter<double>("covariance_min_y", 0.0025);
    covariance_min_yaw_ = declare_parameter<double>("covariance_min_yaw", 0.0225);
    covariance_fallback_x_ = declare_parameter<double>("covariance_fallback_x", 0.01);
    covariance_fallback_y_ = declare_parameter<double>("covariance_fallback_y", 0.01);
    covariance_fallback_yaw_ = declare_parameter<double>("covariance_fallback_yaw", 0.09);

    laser_extrinsics_source_ =
      declare_parameter<std::string>("laser_extrinsics_source", "tf");
    if (laser_extrinsics_source_ != "tf" && laser_extrinsics_source_ != "parameters") {
      RCLCPP_WARN(
        get_logger(), "Unsupported laser_extrinsics_source=%s; using tf",
        laser_extrinsics_source_.c_str());
      laser_extrinsics_source_ = "tf";
    }
    // Parameter values are a startup fallback. The main runtime source is the
    // base_link -> LaserScan frame TF published by the base package.
    laser_in_base_.x = declare_parameter<double>("laser_x_in_base", 0.067);
    laser_in_base_.y = declare_parameter<double>("laser_y_in_base", 0.0);
    laser_in_base_.yaw = declare_parameter<double>("laser_yaw_in_base", 0.0);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    persistent_cone_map_.configure(
      cone_confirm_max_dist_,
      cone_confirm_hits_,
      cone_tentative_prune_misses_,
      confirmed_min_reobserve_hits_,
      confirmed_prune_misses_,
      static_cast<std::size_t>(std::max(1, max_persistent_cones_)));

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 10);
    diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/lidar_odom/diagnostics", 10);
    current_cones_marker_pub_ =
      create_publisher<visualization_msgs::msg::MarkerArray>("/lidar_odom/debug/current_cones", 10);
    map_cones_marker_pub_ =
      create_publisher<visualization_msgs::msg::MarkerArray>("/lidar_odom/debug/map_cones", 10);
    rejected_clusters_marker_pub_ =
      create_publisher<visualization_msgs::msg::MarkerArray>("/lidar_odom/debug/rejected_clusters", 10);
    debug_marker_pub_ =
      create_publisher<visualization_msgs::msg::MarkerArray>("/lidar_odom/debug/markers", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, rclcpp::SensorDataQoS(),
      std::bind(&LidarOdomNode::scanCallback, this, std::placeholders::_1));
    ekf_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      ekf_odom_topic_, 50,
      std::bind(&LidarOdomNode::ekfOdomCallback, this, std::placeholders::_1));
    raw_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      raw_odom_topic_, 50,
      std::bind(&LidarOdomNode::rawOdomCallback, this, std::placeholders::_1));
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, 50,
      std::bind(&LidarOdomNode::imuCallback, this, std::placeholders::_1));
    reset_srv_ = create_service<std_srvs::srv::Empty>(
      "reset_lidar_odom_map",
      std::bind(&LidarOdomNode::resetMapCallback, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(
      get_logger(),
      "simple_lidar_odom started. scan_topic=%s, odom_topic=%s, ekf_odom_topic=%s, publish_tf=%s",
      scan_topic_.c_str(), odom_topic_.c_str(), ekf_odom_topic_.c_str(), publish_tf_ ? "true" : "false");
  }

private:
  struct AssociatedCone
  {
    Point scan_center_laser;
    Point map_center_odom;
  };

  struct AssociatedFence
  {
    PointList scan_points_laser;
    Point map_normal;
    double map_d{0.0};
  };

  void updateLaserExtrinsics(const std::string & laser_frame)
  {
    if (laser_extrinsics_source_ != "tf" || laser_frame.empty() ||
      (laser_extrinsics_from_tf_ && laser_frame == resolved_laser_frame_))
    {
      return;
    }

    try {
      const auto transform = tf_buffer_->lookupTransform(
        base_frame_, laser_frame, tf2::TimePointZero);
      const auto & translation = transform.transform.translation;
      const auto & rotation = transform.transform.rotation;
      laser_in_base_.x = translation.x;
      laser_in_base_.y = translation.y;
      laser_in_base_.yaw = std::atan2(
        2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
        1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z));
      laser_extrinsics_from_tf_ = true;
      resolved_laser_frame_ = laser_frame;
      RCLCPP_INFO(
        get_logger(), "Using TF %s -> %s laser extrinsics: x=%.6f y=%.6f yaw=%.6f",
        base_frame_.c_str(), laser_frame.c_str(), laser_in_base_.x,
        laser_in_base_.y, laser_in_base_.yaw);
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Waiting for TF %s -> %s; using configured fallback extrinsics: %s",
        base_frame_.c_str(), laser_frame.c_str(), error.what());
    }
  }

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    ++received_scan_count_;
    const bool stamp_is_zero = (msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0);
    if (stamp_is_zero) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "LaserScan header.stamp is zero; dropping scan");
      publishQualityDiagnostic(
        this->now(), QualityLevel::Low, false, "zero_scan_stamp", IcpResult{});
      return;
    }
    const rclcpp::Time stamp(msg->header.stamp);
    updateLaserExtrinsics(msg->header.frame_id);

    // Deskew the scan over its finite sweep before landmark extraction. Falls
    // back to raw polar points (have_deskew=false) when no fused twist is yet
    // available (e.g. first frames) or the vehicle is essentially still.
    PointList deskewed;
    const bool have_deskew = computeDeskewedPoints(*msg, stamp, deskewed);

    const ExtractedLandmarks landmarks = extractLandmarks(
      *msg,
      min_range_,
      max_range_,
      seg_break_dist_,
      cone_min_pts_,
      cone_max_pts_,
      cone_max_span_,
      cone_radius_,
      cone_radius_min_,
      cone_radius_max_,
      cone_fit_max_rms_,
      fence_min_pts_,
      fence_split_residual_,
      fence_split_max_depth_,
      fence_aspect_ratio_,
      fence_merge_angle_tol_,
      fence_merge_dist_tol_,
      have_deskew ? &deskewed : nullptr);

    const int extracted_count = static_cast<int>(landmarks.cone_centers.size());
    int fence_support_count = 0;
    for (const auto & fence : landmarks.fences) {
      fence_support_count += static_cast<int>(fence.support_points.size());
    }

    ++processed_frame_idx_;
    ++frames_since_high_publish_;

    if (!initialized_) {
      if (extracted_count + fence_support_count < min_scan_points_) {
        IcpResult diagnostic_result;
        diagnostic_result.current_cones = extracted_count;
        diagnostic_result.fence_support_points = fence_support_count;
        publishQualityDiagnostic(
          stamp, QualityLevel::Low, false, "insufficient_landmarks", diagnostic_result);
        publishDebugMarkers(stamp, landmarks);
        return;
      }
      pose_laser_ = laser_in_base_;
      updateConeMap(landmarks, pose_laser_, accepted_frame_idx_);  // bootstrap frame 0
      initialized_ = true;
      last_base_pose_ = getBasePoseFromLaserPose(pose_laser_);
      last_prediction_stamp_ = stamp;
      last_icp_result_ = IcpResult{};
      IcpResult diagnostic_result;
      diagnostic_result.current_cones = extracted_count;
      diagnostic_result.fence_support_points = fence_support_count;
      publishQualityDiagnostic(
        stamp, QualityLevel::Low, false, "bootstrap", diagnostic_result);
      publishDebugMarkers(stamp, landmarks);
      RCLCPP_INFO(get_logger(), "LiDAR odom bootstrap started; odom1 publication is suppressed");
      return;
    }

    // Prediction is advanced for every scan before any quality decision. An
    // ICP failure therefore cannot freeze the internal trajectory.
    const Pose2D predicted_pose = computeInitialGuess(stamp);
    pose_laser_ = predicted_pose;
    last_prediction_stamp_ = stamp;

    if (extracted_count + fence_support_count < min_scan_points_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Too few extracted landmark constraints: cones=%zu fence_pts=%d",
        landmarks.cone_centers.size(), fence_support_count);
      IcpResult diagnostic_result;
      diagnostic_result.current_cones = extracted_count;
      diagnostic_result.fence_support_points = fence_support_count;
      publishQualityDiagnostic(
        stamp, QualityLevel::Low, false, "insufficient_landmarks", diagnostic_result);
      publishDebugMarkers(stamp, landmarks);
      consecutive_high_frames_ = 0;
      return;
    }

    if (processed_frame_idx_ <= bootstrap_frames_ || map_reseed_pending_ ||
      persistent_cone_map_.cones().empty())
    {
      ++accepted_frame_idx_;
      updateConeMap(landmarks, pose_laser_, accepted_frame_idx_);
      map_reseed_pending_ = false;
      last_icp_delta_ = Pose2D{};
      IcpResult diagnostic_result;
      diagnostic_result.current_cones = extracted_count;
      diagnostic_result.fence_support_points = fence_support_count;
      publishQualityDiagnostic(
        stamp, QualityLevel::Low, false, "bootstrap", diagnostic_result);
      publishDebugMarkers(stamp, landmarks);
      return;
    }

    IcpResult result = alignLandmarksToLocalMap(landmarks, predicted_pose);
    result.current_cones = extracted_count;
    result.fence_support_points = fence_support_count;
    const QualityLevel quality = classifyQuality(toQualityMetrics(result), qualityThresholds());
    const bool high = quality == QualityLevel::High;
    consecutive_high_frames_ = high ? consecutive_high_frames_ + 1 : 0;

    if (high) {
      Pose2D corrected = predicted_pose;
      corrected.x += high_xy_gain_ * (result.pose.x - predicted_pose.x);
      corrected.y += high_xy_gain_ * (result.pose.y - predicted_pose.y);
      corrected.yaw = normalizeAngle(
        predicted_pose.yaw + high_yaw_gain_ * angleDiff(result.pose.yaw, predicted_pose.yaw));
      result.applied_correction_norm = std::hypot(
        corrected.x - predicted_pose.x, corrected.y - predicted_pose.y);
      last_icp_delta_ = composePose(inversePose(predicted_pose), corrected);
      pose_laser_ = corrected;
      result.pose = corrected;

      ++accepted_frame_idx_;
      updateConeMap(landmarks, pose_laser_, accepted_frame_idx_);

      PublishPolicyConfig config;
      config.high_only = high_only_;
      config.invalid_publish_drop = invalid_publish_policy_ == "drop";
      config.extra_high_streak = extra_high_streak_;
      config.cooldown_frames = cooldown_frames_;
      config.min_publish_interval_s = min_publish_interval_s_;
      PublishPolicyState state;
      state.consecutive_high_frames = consecutive_high_frames_;
      state.frames_since_publish = frames_since_high_publish_;
      state.have_last_publish_stamp = last_high_publish_stamp_.nanoseconds() != 0;
      state.last_publish_stamp_s = last_high_publish_stamp_.seconds();
      const PublishDecision decision = evaluatePublishPolicy(
        quality, result.valid, stamp.seconds(), config, state);
      if (decision.publish) {
        publishOdom(stamp, result, false);
        last_high_publish_stamp_ = stamp;
        frames_since_high_publish_ = 0;
        ++published_odom_count_;
      }
      publishQualityDiagnostic(stamp, quality, decision.publish, decision.reason, result);
    } else {
      result.pose = predicted_pose;
      result.valid = false;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "LiDAR odom dropped (not HIGH): corr=%d baseline=%.3f span=%.3f residual=%.4f "
        "min_eig=%.3e cond=%.1f correction=%.3f",
        result.correspondences, result.cone_baseline, result.cone_angle_span,
        result.mean_error, result.min_hessian_eigenvalue,
        result.hessian_condition, result.correction_norm);
      publishQualityDiagnostic(stamp, quality, false, "quality_low_or_medium", result);
    }

    last_icp_result_ = result;
    publishDebugMarkers(stamp, landmarks);
  }

  QualityMetrics toQualityMetrics(const IcpResult & result) const
  {
    return QualityMetrics{
      result.valid, result.cone_correspondences, result.cone_baseline,
      result.cone_angle_span, result.mean_error, result.min_hessian_eigenvalue,
      result.hessian_condition, result.correction_norm};
  }

  QualityThresholds qualityThresholds() const
  {
    return QualityThresholds{
      min_cone_correspondences_, min_cone_baseline_m_, min_cone_angle_span_rad_,
      max_mean_residual_m_, min_hessian_eigenvalue_quality_, max_hessian_condition_,
      max_correction_norm_m_};
  }

  double qualityCovarianceScale(const IcpResult & result) const
  {
    if (!result.valid) {
      return std::numeric_limits<double>::infinity();
    }
    return 1.0 +
      covariance_residual_scale_ *
      std::min(1.0, result.mean_error / std::max(max_mean_residual_m_, 1.0e-9)) +
      covariance_eigenvalue_scale_ *
      std::min(
      1.0,
      min_hessian_eigenvalue_quality_ /
      std::max(result.min_hessian_eigenvalue, 1.0e-9)) +
      covariance_condition_scale_ *
      std::min(1.0, result.hessian_condition / std::max(max_hessian_condition_, 1.0)) +
      covariance_correction_scale_ *
      std::min(1.0, result.correction_norm / std::max(max_correction_norm_m_, 1.0e-9));
  }

  void publishQualityDiagnostic(
    const rclcpp::Time & stamp, QualityLevel quality, bool accepted,
    const std::string & reason, const IcpResult & result)
  {
    if (!quality_diagnostics_enable_) {
      return;
    }
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = stamp;
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "lidar_odom/quality";
    status.hardware_id = "simple_lidar_odom";
    status.level = accepted ? diagnostic_msgs::msg::DiagnosticStatus::OK :
      diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = reason;
    auto add = [&status](const std::string & key, const std::string & value) {
        diagnostic_msgs::msg::KeyValue item;
        item.key = key;
        item.value = value;
        status.values.push_back(item);
      };
    add("quality", qualityLevelName(quality));
    add("accepted", accepted ? "true" : "false");
    add("reject_reason", accepted ? "none" : reason);
    add("correspondences", std::to_string(result.cone_correspondences));
    add("current_cones", std::to_string(result.current_cones));
    add("fence_support_points", std::to_string(result.fence_support_points));
    add("residual_m", std::to_string(result.mean_error));
    add("cone_baseline_m", std::to_string(result.cone_baseline));
    add("cone_angle_span_rad", std::to_string(result.cone_angle_span));
    add("min_hessian_eigenvalue", std::to_string(result.min_hessian_eigenvalue));
    add("hessian_condition", std::to_string(result.hessian_condition));
    add("requested_correction_m", std::to_string(result.correction_norm));
    add("applied_correction_m", std::to_string(result.applied_correction_norm));
    add("correction_gate_exceeded",
      result.correction_norm > max_correction_norm_m_ ? "true" : "false");
    add("consecutive_high", std::to_string(consecutive_high_frames_));
    add("frames_since_publish", std::to_string(frames_since_high_publish_));
    const double covariance_scale = qualityCovarianceScale(result);
    add("covariance_x", std::to_string(xy_covariance_base_ * covariance_scale));
    add("covariance_y", std::to_string(xy_covariance_base_ * covariance_scale));
    add("covariance_yaw", std::to_string(result.valid ? yaw_covariance_ :
      std::numeric_limits<double>::infinity()));
    int tentative_cones = 0;
    int confirmed_cones = 0;
    for (const auto & cone : persistent_cone_map_.cones()) {
      if (cone.state == ConeState::Confirmed) {
        ++confirmed_cones;
      } else {
        ++tentative_cones;
      }
    }
    add("map_tentative_cones", std::to_string(tentative_cones));
    add("map_confirmed_cones", std::to_string(confirmed_cones));
    add("laser_extrinsics_source", laser_extrinsics_from_tf_ ? "tf" : "parameter_fallback");
    add("publish_count", std::to_string(published_odom_count_));
    add("scan_count", std::to_string(received_scan_count_));
    const double rate = received_scan_count_ == 0 ? 0.0 :
      static_cast<double>(published_odom_count_) / static_cast<double>(received_scan_count_);
    add("publish_rate", std::to_string(rate));
    add("last_accepted_stamp", last_high_publish_stamp_.nanoseconds() == 0 ?
      "none" : std::to_string(last_high_publish_stamp_.seconds()));
    array.status.push_back(status);
    diagnostics_pub_->publish(array);
  }

  void ekfOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    pushTwist(ekf_twist_history_, rclcpp::Time(msg->header.stamp), msg->twist.twist);
  }

  void rawOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    pushTwist(raw_twist_history_, rclcpp::Time(msg->header.stamp), msg->twist.twist);
  }

  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    TimedTwist twist;
    twist.stamp = rclcpp::Time(msg->header.stamp);
    twist.wz = msg->angular_velocity.z;
    imu_wz_history_.push_back(twist);
    pruneTwistHistory(imu_wz_history_);
  }

  void resetMapCallback(
    const std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>)
  {
    persistent_cone_map_.clear();
    local_landmark_map_.clear();
    last_icp_delta_ = Pose2D{};
    accepted_frame_idx_ = 0;
    processed_frame_idx_ = 0;
    consecutive_high_frames_ = 0;
    frames_since_high_publish_ = 1000000;
    last_icp_result_ = IcpResult{};
    map_reseed_pending_ = false;

    if (reset_pose_on_map_reset_) {
      pose_laser_ = Pose2D{};
      last_base_pose_ = Pose2D{};
      last_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
      last_prediction_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
      last_high_publish_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
      initialized_ = false;
      RCLCPP_INFO(get_logger(), "reset_lidar_odom_map: cleared map and reset lidar odom pose");
      return;
    }

    map_reseed_pending_ = initialized_;
    RCLCPP_INFO(
      get_logger(),
      "reset_lidar_odom_map: cleared map only; lidar odom pose preserved for Stage-B safety");
  }

  void pushTwist(
    std::deque<TimedTwist> & history,
    const rclcpp::Time & stamp,
    const geometry_msgs::msg::Twist & twist)
  {
    TimedTwist timed;
    timed.stamp = stamp;
    timed.vx = twist.linear.x;
    timed.vy = twist.linear.y;
    timed.wz = twist.angular.z;
    history.push_back(timed);
    pruneTwistHistory(history);
  }

  void pruneTwistHistory(std::deque<TimedTwist> & history) const
  {
    while (history.size() > 200) {
      history.pop_front();
    }
    if (history.empty()) {
      return;
    }
    const rclcpp::Time newest = history.back().stamp;
    while (history.size() > 2 && (newest - history.front().stamp).seconds() > 2.0) {
      history.pop_front();
    }
  }

  // Build motion-deskewed laser-frame points for the scan. Each ray i is acquired
  // at header.stamp + i*time_increment; over the ~99 ms sweep the vehicle keeps
  // moving (up to 11 deg of rotation at 2 rad/s), so treating the frame as one
  // instantaneous snapshot smears the landmarks and the matcher under-rotates in
  // turns. Here each ray is transformed back into the laser frame at the scan
  // reference time (ray 0 == header.stamp) using the fused-odom body twist:
  //   T_{laser(ref)<-laser(t_i)} = L^-1 . integrateTwist(twist, i*dt) . L
  // (L = laser_in_base_; the conjugation maps the base-frame motion through the
  // sensor lever arm). Returns false (caller uses raw polar points) when deskew
  // is disabled, the scan lacks per-ray timing, no fused twist is available, or
  // the motion over the sweep is negligible.
  bool computeDeskewedPoints(
    const sensor_msgs::msg::LaserScan & scan,
    const rclcpp::Time & stamp,
    PointList & out)
  {
    if (!use_scan_deskew_) {
      return false;
    }
    double time_increment = static_cast<double>(scan.time_increment);
    if (time_increment <= 0.0 && scan.scan_time > 0.0 && scan.ranges.size() > 1) {
      time_increment = static_cast<double>(scan.scan_time) /
        static_cast<double>(scan.ranges.size() - 1);
    }
    if (time_increment <= 0.0) {
      return false;
    }
    TimedTwist twist;
    if (!selectDeskewTwist(stamp, twist)) {
      return false;
    }
    if (std::abs(twist.vx) < 1.0e-3 &&
      std::abs(twist.vy) < 1.0e-3 &&
      std::abs(twist.wz) < 1.0e-3)
    {
      return false;
    }

    const Pose2D laser_to_base = laser_in_base_;
    const Pose2D base_to_laser = inversePose(laser_in_base_);

    out.resize(scan.ranges.size());
    for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
      const float r = scan.ranges[i];
      if (!std::isfinite(r)) {
        out[i] = Point2::Zero();  // ignored by segmentScan (range invalid)
        continue;
      }
      const double angle = static_cast<double>(scan.angle_min) +
        static_cast<double>(i) * static_cast<double>(scan.angle_increment);
      const Point2 p_laser(
        static_cast<double>(r) * std::cos(angle),
        static_cast<double>(r) * std::sin(angle));

      const double tau = static_cast<double>(i) * time_increment;
      const Pose2D delta_base = integrateTwist(twist.vx, twist.vy, twist.wz, tau);
      const Pose2D t_i = composePose(composePose(base_to_laser, delta_base), laser_to_base);
      out[i] = transformPoint(p_laser, t_i);
    }
    return true;
  }

  Pose2D computeInitialGuess(const rclcpp::Time & stamp)
  {
    TimedTwist twist;
    const double dt = (stamp - last_prediction_stamp_).seconds();
    if (dt > 0.0 && dt < 1.0 && lookupRawOdomImuTwist(stamp, twist)) {
      const Pose2D delta = integrateTwist(twist.vx, twist.vy, twist.wz, dt);
      return composePose(pose_laser_, delta);
    }

    if (use_cv_prior_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Twist prior unavailable near scan stamp; falling back to constant-velocity extrapolation");
      return composePose(pose_laser_, last_icp_delta_);
    }

    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Twist prior unavailable near scan stamp; falling back to zero-velocity (last pose) prior");
    return pose_laser_;
  }

  bool selectDeskewTwist(const rclcpp::Time & stamp, TimedTwist & out)
  {
    if (deskew_source_ == "ekf_twist_lagged" &&
      lookupTwistBefore(ekf_twist_history_, stamp, deskew_odom_timeout_, out) &&
      twistWithinLimits(out))
    {
      maskDeskewTwist(out);
      return true;
    }

    if (deskew_source_ == "raw_odom" &&
      lookupTwistBefore(raw_twist_history_, stamp, deskew_odom_timeout_, out) &&
      twistWithinLimits(out))
    {
      maskDeskewTwist(out);
      return true;
    }

    if (fallback_deskew_source_ == "raw_odom_imu" && lookupRawOdomImuTwist(stamp, out)) {
      maskDeskewTwist(out);
      return true;
    }

    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "No valid %s deskew twist near scan stamp; deskew disabled for this frame",
      deskew_source_.c_str());
    return false;
  }

  bool lookupRawOdomImuTwist(const rclcpp::Time & stamp, TimedTwist & out)
  {
    TimedTwist raw;
    TimedTwist imu;
    if (!lookupTwistBefore(raw_twist_history_, stamp, deskew_odom_timeout_, raw)) {
      return false;
    }
    if (!lookupTwistBefore(imu_wz_history_, stamp, deskew_imu_timeout_, imu)) {
      return false;
    }
    out = raw;
    out.wz = imu.wz;
    return twistWithinLimits(out);
  }

  bool lookupTwistBefore(
    const std::deque<TimedTwist> & history,
    const rclcpp::Time & stamp,
    double timeout,
    TimedTwist & out) const
  {
    if (history.empty()) {
      return false;
    }

    const rclcpp::Time target = stamp -
      rclcpp::Duration::from_seconds(std::max(0.0, deskew_min_lag_sec_));

    for (auto it = history.rbegin(); it != history.rend(); ++it) {
      if (deskew_require_stamp_not_newer_than_scan_ && it->stamp > target) {
        continue;
      }
      const double age = (stamp - it->stamp).seconds();
      if (age < -1.0e-6 || age > timeout) {
        continue;
      }
      out = *it;
      return true;
    }
    return false;
  }

  bool twistWithinLimits(const TimedTwist & twist) const
  {
    return std::isfinite(twist.vx) && std::isfinite(twist.vy) && std::isfinite(twist.wz) &&
      std::abs(twist.vx) <= deskew_max_abs_vx_ &&
      std::abs(twist.vy) <= deskew_max_abs_vy_ &&
      std::abs(twist.wz) <= deskew_max_abs_wz_;
  }

  void maskDeskewTwist(TimedTwist & twist)
  {
    if (!deskew_use_vx_) {
      twist.vx = 0.0;
    }
    if (!deskew_use_vy_) {
      twist.vy = 0.0;
    }
    if (!deskew_use_wz_) {
      twist.wz = 0.0;
    }
    if (!deskew_twist_in_base_frame_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "deskew_twist_in_base_frame=false is unsupported; treating twist as base-frame");
    }
  }

  void publishDebugMarkers(
    const rclcpp::Time & stamp,
    const ExtractedLandmarks & landmarks)
  {
    visualization_msgs::msg::MarkerArray current;
    visualization_msgs::msg::MarkerArray map;
    visualization_msgs::msg::MarkerArray rejected;
    visualization_msgs::msg::MarkerArray all;

    appendDeleteAll(current, "current_cones", stamp);
    appendDeleteAll(map, "map_cones", stamp);
    appendDeleteAll(rejected, "rejected_clusters", stamp);
    appendDeleteAll(all, "lidar_odom_debug", stamp);

    int id = 0;
    for (const auto & cone_laser : landmarks.cone_centers) {
      const Point cone_odom = transformPoint(cone_laser, pose_laser_);
      auto marker = makeSphereMarker(
        "current_cones", id++, stamp, cone_odom, 0.08, 0.05F, 0.80F, 1.00F, 0.80F);
      current.markers.push_back(marker);
      marker.ns = "lidar_odom_current_cones";
      all.markers.push_back(marker);
    }

    id = 0;
    for (const auto & cone : persistent_cone_map_.cones()) {
      const bool confirmed = cone.state == ConeState::Confirmed;
      auto marker = makeSphereMarker(
        "map_cones", id++, stamp, cone.pos, confirmed ? 0.10 : 0.06,
        confirmed ? 0.05F : 1.00F,
        confirmed ? 0.95F : 0.70F,
        confirmed ? 0.20F : 0.05F,
        confirmed ? 0.90F : 0.55F);
      map.markers.push_back(marker);
      marker.ns = confirmed ? "lidar_odom_confirmed_map_cones" : "lidar_odom_tentative_map_cones";
      all.markers.push_back(marker);
    }

    id = 0;
    for (const auto & cluster : landmarks.rejected_clusters) {
      if (cluster.pts.empty()) {
        continue;
      }
      Point center = Point::Zero();
      for (const auto & p : cluster.pts) {
        center += p;
      }
      center /= static_cast<double>(cluster.pts.size());
      center = transformPoint(center, pose_laser_);
      auto marker = makeSphereMarker(
        "rejected_clusters", id++, stamp, center, 0.05, 1.00F, 0.10F, 0.10F, 0.45F);
      rejected.markers.push_back(marker);
      marker.ns = "lidar_odom_rejected_clusters";
      all.markers.push_back(marker);
    }

    current_cones_marker_pub_->publish(current);
    map_cones_marker_pub_->publish(map);
    rejected_clusters_marker_pub_->publish(rejected);
    debug_marker_pub_->publish(all);
  }

  void appendDeleteAll(
    visualization_msgs::msg::MarkerArray & markers,
    const std::string & ns,
    const rclcpp::Time & stamp) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = stamp;
    marker.header.frame_id = odom_frame_;
    marker.ns = ns;
    marker.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(marker);
  }

  visualization_msgs::msg::Marker makeSphereMarker(
    const std::string & ns,
    int id,
    const rclcpp::Time & stamp,
    const Point & point,
    double scale,
    float r,
    float g,
    float b,
    float a) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = stamp;
    marker.header.frame_id = odom_frame_;
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = point.x();
    marker.pose.position.y = point.y();
    marker.pose.position.z = 0.06;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = scale;
    marker.scale.y = scale;
    marker.scale.z = 0.06;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = a;
    marker.lifetime.sec = 0;
    marker.lifetime.nanosec = 300000000;
    return marker;
  }

  Point transformPoint(const Point & p, const Pose2D & pose) const
  {
    const double c = std::cos(pose.yaw);
    const double s = std::sin(pose.yaw);
    return Point(c * p.x() - s * p.y() + pose.x, s * p.x() + c * p.y() + pose.y);
  }

  PointList transformPoints(const PointList & pts, const Pose2D & pose) const
  {
    PointList out;
    out.reserve(pts.size());
    for (const auto & p : pts) {
      out.push_back(transformPoint(p, pose));
    }
    return out;
  }

  LineLandmark transformLineLandmark(const LineLandmark & line, const Pose2D & pose) const
  {
    LineLandmark transformed = line;
    transformed.centroid = transformPoint(line.centroid, pose);
    transformed.support_points = transformPoints(line.support_points, pose);
    const double c = std::cos(pose.yaw);
    const double s = std::sin(pose.yaw);
    transformed.normal = Point(
      c * line.normal.x() - s * line.normal.y(),
      s * line.normal.x() + c * line.normal.y());
    transformed.d = transformed.normal.dot(transformed.centroid);
    return transformed;
  }

  Pose2D getBasePoseFromLaserPose(const Pose2D & laser_pose) const
  {
    return composePose(laser_pose, inversePose(laser_in_base_));
  }

  // Dormant in Phase 2: the cone path now uses persistent_cone_map_. Kept (with
  // transformLineLandmark) for the fence path that is re-enabled in a later phase.
  void seedLandmarkMap(const ExtractedLandmarks & landmarks, const Pose2D & pose)
  {
    PointList cones_odom = transformPoints(landmarks.cone_centers, pose);
    std::vector<LineLandmark> fences_odom;
    fences_odom.reserve(landmarks.fences.size());
    for (const auto & fence : landmarks.fences) {
      fences_odom.push_back(transformLineLandmark(fence, pose));
    }
    local_landmark_map_.addFrame(cones_odom, fences_odom);
  }

  // Transform this frame's cone centroids into odom using the accepted pose and
  // fold them into the persistent map (running-mean denoise while tentative,
  // freeze on confirmation). frame_idx is a monotonic accepted-frame counter.
  void updateConeMap(const ExtractedLandmarks & landmarks, const Pose2D & pose, int frame_idx)
  {
    PointList cones_odom = transformPoints(landmarks.cone_centers, pose);
    persistent_cone_map_.update(cones_odom, frame_idx);
  }

  double huberWeight(double abs_error) const
  {
    if (robust_kernel_delta_ <= 0.0 || abs_error <= robust_kernel_delta_) {
      return 1.0;
    }
    return robust_kernel_delta_ / std::max(abs_error, 1.0e-9);
  }

  void addPointToPointResidual(
    const Point & residual,
    const Point & dwdtheta,
    Eigen::Matrix3d & H,
    Eigen::Vector3d & b,
    double & total_error,
    double & weighted_ssr,
    int & correspondences) const
  {
    const double e = residual.norm();
    const double w = huberWeight(e);

    Eigen::RowVector3d jx;
    jx << 1.0, 0.0, dwdtheta.x();
    Eigen::RowVector3d jy;
    jy << 0.0, 1.0, dwdtheta.y();

    H += w * (jx.transpose() * jx + jy.transpose() * jy);
    b += w * (jx.transpose() * residual.x() + jy.transpose() * residual.y());
    weighted_ssr += w * residual.squaredNorm();
    total_error += e;
    correspondences += 1;
  }

  void addPointToLineResidual(
    double residual,
    const Point & normal,
    const Point & dwdtheta,
    Eigen::Matrix3d & H,
    Eigen::Vector3d & b,
    double & total_error,
    double & weighted_ssr,
    int & correspondences) const
  {
    const double abs_r = std::abs(residual);
    const double w = huberWeight(abs_r);

    Eigen::RowVector3d J;
    J << normal.x(), normal.y(), normal.dot(dwdtheta);

    H += w * J.transpose() * J;
    b += w * J.transpose() * residual;
    weighted_ssr += w * residual * residual;
    total_error += abs_r;
    correspondences += 1;
  }

  bool associateLandmarks(
    const ExtractedLandmarks & scan_landmarks,
    const Pose2D & pose,
    std::vector<AssociatedCone> & cone_matches,
    std::vector<AssociatedFence> & fence_matches) const
  {
    cone_matches.clear();
    fence_matches.clear();

    PointList scan_cones_world;
    scan_cones_world.reserve(scan_landmarks.cone_centers.size());
    for (const auto & cone_laser : scan_landmarks.cone_centers) {
      scan_cones_world.push_back(transformPoint(cone_laser, pose));
    }
    PointList map_cones;
    map_cones.reserve(persistent_cone_map_.cones().size());
    for (const auto & map_cone : persistent_cone_map_.cones()) {
      map_cones.push_back(map_cone.pos);
    }
    const auto unique_matches = greedyUniqueAssignment(
      scan_cones_world, map_cones, max_correspondence_dist_);
    cone_matches.reserve(unique_matches.size());
    for (const auto & match : unique_matches) {
      cone_matches.push_back(AssociatedCone{
        scan_landmarks.cone_centers[match.observation_index],
        map_cones[match.landmark_index]});
    }

    for (const auto & fence_scan : scan_landmarks.fences) {
      if (!use_fence_residuals_) {
        break;
      }
      const double c = std::cos(pose.yaw);
      const double s = std::sin(pose.yaw);
      Point world_normal(
        c * fence_scan.normal.x() - s * fence_scan.normal.y(),
        s * fence_scan.normal.x() + c * fence_scan.normal.y());
      Point world_centroid = transformPoint(fence_scan.centroid, pose);
      const double world_d = world_normal.dot(world_centroid);

      const FenceEntry * best_match = nullptr;
      double best_abs_d = max_correspondence_dist_;
      for (const auto & map_fence : local_landmark_map_.fences()) {
        if (world_normal.dot(map_fence.normal) <= 0.0) {
          continue;
        }
        const double sin_angle = std::abs(
          world_normal.x() * map_fence.normal.y() - world_normal.y() * map_fence.normal.x());
        if (sin_angle >= fence_assoc_angle_tol_) {
          continue;
        }
        const double d_diff = std::abs(world_d - map_fence.d);
        if (d_diff < best_abs_d) {
          best_abs_d = d_diff;
          best_match = &map_fence;
        }
      }

      if (best_match != nullptr) {
        AssociatedFence match;
        match.map_normal = best_match->normal;
        match.map_d = best_match->d;
        match.scan_points_laser = fence_scan.support_points;
        if (static_cast<int>(match.scan_points_laser.size()) > fence_max_residual_pts_) {
          const std::size_t step = std::max<std::size_t>(
            1, match.scan_points_laser.size() / static_cast<std::size_t>(fence_max_residual_pts_));
          PointList limited;
          for (std::size_t i = 0; i < match.scan_points_laser.size(); i += step) {
            limited.push_back(match.scan_points_laser[i]);
            if (static_cast<int>(limited.size()) >= fence_max_residual_pts_) {
              break;
            }
          }
          match.scan_points_laser = std::move(limited);
        }
        fence_matches.push_back(std::move(match));
      }
    }

    return static_cast<int>(cone_matches.size() + fence_matches.size()) > 0;
  }

  MatchStats accumulateResiduals(
    const std::vector<AssociatedCone> & cone_matches,
    const std::vector<AssociatedFence> & fence_matches,
    const Pose2D & pose,
    Eigen::Vector3d & b) const
  {
    MatchStats stats;
    const double c = std::cos(pose.yaw);
    const double s = std::sin(pose.yaw);

    for (const auto & match : cone_matches) {
      const Point p_world(
        c * match.scan_center_laser.x() - s * match.scan_center_laser.y() + pose.x,
        s * match.scan_center_laser.x() + c * match.scan_center_laser.y() + pose.y);
      const Point dwdtheta(
        -s * match.scan_center_laser.x() - c * match.scan_center_laser.y(),
         c * match.scan_center_laser.x() - s * match.scan_center_laser.y());
      addPointToPointResidual(
        p_world - match.map_center_odom,
        dwdtheta,
        stats.H,
        b,
        stats.total_error,
        stats.weighted_ssr,
        stats.correspondences);
    }

    for (const auto & match : fence_matches) {
      for (const auto & p_laser : match.scan_points_laser) {
        const Point p_world(
          c * p_laser.x() - s * p_laser.y() + pose.x,
          s * p_laser.x() + c * p_laser.y() + pose.y);
        const Point dwdtheta(
          -s * p_laser.x() - c * p_laser.y(),
           c * p_laser.x() - s * p_laser.y());
        const double residual = match.map_normal.dot(p_world) - match.map_d;
        addPointToLineResidual(
          residual,
          match.map_normal,
          dwdtheta,
          stats.H,
          b,
          stats.total_error,
          stats.weighted_ssr,
          stats.correspondences);
      }
    }

    return stats;
  }

  IcpResult alignLandmarksToLocalMap(
    const ExtractedLandmarks & scan_landmarks,
    const Pose2D & initial_guess) const
  {
    IcpResult result;
    result.pose = initial_guess;

    if (persistent_cone_map_.cones().empty() && local_landmark_map_.fences().empty()) {
      return result;
    }

    Pose2D pose = initial_guess;
    int last_corr = 0;
    double last_mean_error = std::numeric_limits<double>::infinity();
    double last_min_eig = 0.0;
    Eigen::Matrix3d last_H = Eigen::Matrix3d::Zero();
    double last_ssr = 0.0;
    bool converged = false;

    std::vector<AssociatedCone> cone_matches;
    std::vector<AssociatedFence> fence_matches;

    for (int iter = 0; iter < icp_max_iterations_; ++iter) {
      if (!associateLandmarks(scan_landmarks, pose, cone_matches, fence_matches)) {
        break;
      }

      Eigen::Vector3d b = Eigen::Vector3d::Zero();
      MatchStats stats = accumulateResiduals(cone_matches, fence_matches, pose, b);
      last_corr = stats.correspondences;
      last_mean_error = stats.correspondences > 0 ?
        stats.total_error / static_cast<double>(stats.correspondences) :
        std::numeric_limits<double>::infinity();
      last_H = stats.H;
      last_ssr = stats.weighted_ssr;

      if (stats.correspondences < min_correspondences_) {
        break;
      }

      const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig_solver(stats.H);
      if (eig_solver.info() == Eigen::Success) {
        last_min_eig = eig_solver.eigenvalues()(0);
      }

      Eigen::Matrix3d regularized_H = stats.H + Eigen::Matrix3d::Identity() * 1.0e-9;
      const Eigen::LDLT<Eigen::Matrix3d> ldlt(regularized_H);
      if (ldlt.info() != Eigen::Success) {
        break;
      }

      Eigen::Vector3d delta = -ldlt.solve(b);
      if (!delta.allFinite()) {
        break;
      }

      const double dt = std::hypot(delta.x(), delta.y());
      if (dt > max_icp_step_translation_) {
        const double scale = max_icp_step_translation_ / std::max(dt, 1.0e-9);
        delta.x() *= scale;
        delta.y() *= scale;
      }
      if (std::abs(delta.z()) > max_icp_step_rotation_) {
        delta.z() = std::copysign(max_icp_step_rotation_, delta.z());
      }

      pose.x += delta.x();
      pose.y += delta.y();
      pose.yaw = normalizeAngle(pose.yaw + delta.z());

      if (std::hypot(delta.x(), delta.y()) < icp_converge_translation_ &&
        std::abs(delta.z()) < icp_converge_rotation_)
      {
        converged = true;
        break;
      }
    }

    result.pose = pose;
    result.converged = converged;
    result.correspondences = last_corr;
    result.cone_correspondences = static_cast<int>(cone_matches.size());
    result.mean_error = last_mean_error;
    result.min_hessian_eigenvalue = last_min_eig;
    result.final_hessian = last_H;
    result.weighted_ssr = last_ssr;
    result.correction_norm = std::hypot(pose.x - initial_guess.x, pose.y - initial_guess.y);

    for (std::size_t i = 0; i < cone_matches.size(); ++i) {
      for (std::size_t j = i + 1; j < cone_matches.size(); ++j) {
        result.cone_baseline = std::max(
          result.cone_baseline,
          (cone_matches[i].map_center_odom - cone_matches[j].map_center_odom).norm());
      }
    }

    if (cone_matches.size() >= 2) {
      std::vector<double> bearings;
      bearings.reserve(cone_matches.size());
      for (const auto & match : cone_matches) {
        double bearing = std::atan2(match.scan_center_laser.y(), match.scan_center_laser.x());
        if (bearing < 0.0) {
          bearing += 2.0 * M_PI;
        }
        bearings.push_back(bearing);
      }
      std::sort(bearings.begin(), bearings.end());
      double largest_gap = 0.0;
      for (std::size_t i = 1; i < bearings.size(); ++i) {
        largest_gap = std::max(largest_gap, bearings[i] - bearings[i - 1]);
      }
      largest_gap = std::max(
        largest_gap, bearings.front() + 2.0 * M_PI - bearings.back());
      result.cone_angle_span = 2.0 * M_PI - largest_gap;
    }

    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> final_eig(last_H);
    if (final_eig.info() == Eigen::Success && final_eig.eigenvalues()(0) > 0.0) {
      result.hessian_condition =
        final_eig.eigenvalues()(2) / final_eig.eigenvalues()(0);
    }
    result.valid =
      last_corr >= min_correspondences_ &&
      std::isfinite(last_mean_error) &&
      last_mean_error <= max_mean_error_;
    return result;
  }

  Eigen::Matrix3d covarianceFromHessian(const IcpResult & result, bool first_frame) const
  {
    if (first_frame) {
      return Eigen::Matrix3d::Identity() * (init_pose_std_ * init_pose_std_);
    }
    if (!result.valid || result.correspondences <= 0) {
      return Eigen::Matrix3d::Identity() * 1.0e6;
    }

    const std::size_t dof = static_cast<std::size_t>(std::max(result.correspondences - 3, 1));
    const double sigma2 = sigma2FromResiduals(
      result.weighted_ssr,
      dof,
      meas_noise_std_ * meas_noise_std_);
    return poseCovarianceFromHessian(result.final_hessian, sigma2, lambda_floor_);
  }

  void publishOdom(const rclcpp::Time & stamp, const IcpResult & icp_result, bool first_frame)
  {
    const Pose2D base_pose = getBasePoseFromLaserPose(pose_laser_);

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;
    odom.pose.pose.position.x = base_pose.x;
    odom.pose.pose.position.y = base_pose.y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation.x = 0.0;
    odom.pose.pose.orientation.y = 0.0;
    const double published_yaw = publish_yaw_ ? base_pose.yaw : 0.0;
    odom.pose.pose.orientation.z = std::sin(published_yaw * 0.5);
    odom.pose.pose.orientation.w = std::cos(published_yaw * 0.5);

    const Eigen::Matrix3d pose_cov = covarianceFromHessian(icp_result, first_frame);
    odom.pose.covariance = mapPoseCovToOdom(pose_cov, 1.0e6);
    const bool bad_covariance =
      !std::isfinite(odom.pose.covariance[0]) ||
      !std::isfinite(odom.pose.covariance[7]) ||
      !std::isfinite(odom.pose.covariance[35]) ||
      odom.pose.covariance[0] < 0.0 ||
      odom.pose.covariance[7] < 0.0 ||
      odom.pose.covariance[35] < 0.0;
    if (bad_covariance) {
      odom.pose.covariance = mapPoseCovToOdom(Eigen::Matrix3d::Zero(), 1.0e6);
      odom.pose.covariance[0] = covariance_fallback_x_;
      odom.pose.covariance[7] = covariance_fallback_y_;
      odom.pose.covariance[35] = covariance_fallback_yaw_;
    } else {
      odom.pose.covariance[0] = std::max(odom.pose.covariance[0], covariance_min_x_);
      odom.pose.covariance[7] = std::max(odom.pose.covariance[7], covariance_min_y_);
      odom.pose.covariance[35] = std::max(odom.pose.covariance[35], covariance_min_yaw_);
    }
    if (icp_result.valid) {
      const double quality_scale = qualityCovarianceScale(icp_result);
      odom.pose.covariance[0] = xy_covariance_base_ * quality_scale;
      odom.pose.covariance[7] = xy_covariance_base_ * quality_scale;
      odom.pose.covariance[35] = yaw_covariance_;
      odom.pose.covariance[1] = 0.0;
      odom.pose.covariance[5] = 0.0;
      odom.pose.covariance[6] = 0.0;
      odom.pose.covariance[11] = 0.0;
      odom.pose.covariance[30] = 0.0;
      odom.pose.covariance[31] = 0.0;
    }
    if (!first_frame && !icp_result.valid) {
      odom.pose.covariance[0] = 1.0e6;
      odom.pose.covariance[7] = 1.0e6;
      odom.pose.covariance[35] = 1.0e6;
      odom.pose.covariance[1] = 0.0;
      odom.pose.covariance[5] = 0.0;
      odom.pose.covariance[6] = 0.0;
      odom.pose.covariance[11] = 0.0;
      odom.pose.covariance[30] = 0.0;
      odom.pose.covariance[31] = 0.0;
    }

    odom.twist.twist.linear.x = 0.0;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.angular.z = 0.0;
    if (last_stamp_.nanoseconds() > 0) {
      const double dt = (stamp - last_stamp_).seconds();
      if (dt > 1.0e-4 && dt < 1.0) {
        const double dx_w = base_pose.x - last_base_pose_.x;
        const double dy_w = base_pose.y - last_base_pose_.y;
        const double dyaw = angleDiff(base_pose.yaw, last_base_pose_.yaw);
        const double c = std::cos(base_pose.yaw);
        const double s = std::sin(base_pose.yaw);
        odom.twist.twist.linear.x = (c * dx_w + s * dy_w) / dt;
        odom.twist.twist.linear.y = (-s * dx_w + c * dy_w) / dt;
        odom.twist.twist.angular.z = dyaw / dt;
      }
    }

    odom.twist.covariance.fill(0.0);
    const double twist_diag = (!icp_result.valid) ? 1.0e6 : 0.25;
    odom.twist.covariance[0] = twist_diag;
    odom.twist.covariance[7] = twist_diag;
    odom.twist.covariance[14] = 1.0e6;
    odom.twist.covariance[21] = 1.0e6;
    odom.twist.covariance[28] = 1.0e6;
    odom.twist.covariance[35] = twist_diag;

    odom_pub_->publish(odom);

    if (publish_tf_) {
      geometry_msgs::msg::TransformStamped tf;
      tf.header.stamp = stamp;
      tf.header.frame_id = odom_frame_;
      tf.child_frame_id = base_frame_;
      tf.transform.translation.x = base_pose.x;
      tf.transform.translation.y = base_pose.y;
      tf.transform.translation.z = 0.0;
      tf.transform.rotation = odom.pose.pose.orientation;
      tf_broadcaster_->sendTransform(tf);
    }

    last_base_pose_ = base_pose;
    last_stamp_ = stamp;
  }

  std::string scan_topic_;
  std::string odom_topic_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string deskew_source_;
  std::string ekf_odom_topic_;
  std::string raw_odom_topic_;
  std::string imu_topic_;
  std::string fallback_deskew_source_;
  bool publish_tf_{false};

  double min_range_{0.15};
  double max_range_{2.2};
  int downsample_step_{1};
  int min_scan_points_{3};

  int local_map_frames_{30};

  double max_correspondence_dist_{0.20};
  double max_mean_error_{0.05};
  int min_correspondences_{3};

  int icp_max_iterations_{15};
  double icp_converge_translation_{0.001};
  double icp_converge_rotation_{0.0005};
  double robust_kernel_delta_{0.08};
  double max_icp_step_translation_{0.10};
  double max_icp_step_rotation_{0.15};
  double lambda_floor_{1.0e-6};
  double meas_noise_std_{0.05};
  double init_pose_std_{1.0e3};

  std::string invalid_publish_policy_{"drop"};
  std::string publish_pose_mode_{"absolute"};
  bool publish_yaw_{true};
  bool high_only_{true};
  bool quality_diagnostics_enable_{true};
  int min_cone_correspondences_{4};
  double min_cone_baseline_m_{1.412672112};
  double min_cone_angle_span_rad_{1.583992435};
  double max_mean_residual_m_{0.02682085338};
  double min_hessian_eigenvalue_quality_{0.3773939394};
  double max_hessian_condition_{1531.869725};
  double max_correction_norm_m_{0.4973780206};
  double high_xy_gain_{0.9488666884};
  double high_yaw_gain_{0.05};
  int bootstrap_frames_{4};
  int extra_high_streak_{1};
  int cooldown_frames_{3};
  double min_publish_interval_s_{0.1389967094};
  double xy_covariance_base_{0.008543882646};
  double covariance_residual_scale_{1.035691223};
  double covariance_eigenvalue_scale_{0.3400792863};
  double covariance_condition_scale_{0.3368723875};
  double covariance_correction_scale_{0.9162886246};
  double yaw_covariance_{0.3536275267};

  int cone_confirm_hits_{5};
  int cone_tentative_prune_misses_{10};
  double cone_confirm_max_dist_{0.16};
  int confirmed_min_reobserve_hits_{2};
  int confirmed_prune_misses_{0};
  int max_persistent_cones_{30};
  bool reset_pose_on_map_reset_{true};
  bool use_cv_prior_{false};

  double cone_radius_{0.04};
  double cone_radius_min_{0.0359875192};
  double cone_radius_max_{0.0529781543};
  int cone_min_pts_{5};
  int cone_max_pts_{21};
  double cone_max_span_{0.2035249622};
  double cone_fit_max_rms_{0.0062363306};
  int fence_min_pts_{15};
  double fence_split_residual_{0.0};
  int fence_split_max_depth_{0};
  double fence_aspect_ratio_{4.0};
  double fence_merge_angle_tol_{0.0756576558};
  double fence_merge_dist_tol_{0.0945249897};
  double fence_assoc_angle_tol_{0.10};
  int fence_max_residual_pts_{30};
  bool use_fence_residuals_{false};
  double seg_break_dist_{0.0965708121};
  double deskew_odom_timeout_{0.20};
  double deskew_imu_timeout_{0.20};
  bool deskew_require_stamp_not_newer_than_scan_{true};
  double deskew_min_lag_sec_{0.0};
  bool deskew_use_vx_{true};
  bool deskew_use_vy_{true};
  bool deskew_use_wz_{true};
  bool deskew_twist_in_base_frame_{true};
  double deskew_max_abs_vx_{2.0};
  double deskew_max_abs_vy_{0.5};
  double deskew_max_abs_wz_{4.0};
  bool use_scan_deskew_{true};
  double covariance_min_x_{0.0025};
  double covariance_min_y_{0.0025};
  double covariance_min_yaw_{0.0225};
  double covariance_fallback_x_{0.01};
  double covariance_fallback_y_{0.01};
  double covariance_fallback_yaw_{0.09};
  std::string laser_extrinsics_source_{"tf"};
  std::string resolved_laser_frame_;
  bool laser_extrinsics_from_tf_{false};

  Pose2D laser_in_base_;
  Pose2D pose_laser_;
  Pose2D last_base_pose_;
  Pose2D last_icp_delta_;
  rclcpp::Time last_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_prediction_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_high_publish_stamp_{0, 0, RCL_ROS_TIME};
  bool initialized_{false};
  bool map_reseed_pending_{false};
  int accepted_frame_idx_{0};
  int processed_frame_idx_{0};
  int consecutive_high_frames_{0};
  int frames_since_high_publish_{1000000};
  std::uint64_t received_scan_count_{0};
  std::uint64_t published_odom_count_{0};

  std::deque<TimedTwist> ekf_twist_history_;
  std::deque<TimedTwist> raw_twist_history_;
  std::deque<TimedTwist> imu_wz_history_;
  LocalLandmarkMap local_landmark_map_;
  PersistentConeMap persistent_cone_map_;
  IcpResult last_icp_result_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ekf_odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr raw_odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr current_cones_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr map_cones_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr rejected_clusters_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_marker_pub_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_srv_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace simple_lidar_odom

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<simple_lidar_odom::LidarOdomNode>());
  rclcpp::shutdown();
  return 0;
}
