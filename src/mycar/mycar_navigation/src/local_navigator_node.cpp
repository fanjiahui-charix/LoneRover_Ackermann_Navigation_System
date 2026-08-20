#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "mycar_navigation/msg/goal_status.hpp"
#include "mycar_navigation/msg/nav_goal.hpp"
#include "mycar_navigation/nav_core/field_odom_transform.hpp"
#include "mycar_navigation/nav_core/map_mask.hpp"
#include "mycar_navigation/navigator/navigator_core.hpp"
#include "mycar_navigation/navigator/scan_conversion.hpp"

namespace
{
using mycar_navigation::nav_core::FieldOdomTransform;
using mycar_navigation::nav_core::Pose2D;
using mycar_navigation::navigator::NavCoreConfig;
using mycar_navigation::navigator::NavInputs;
using mycar_navigation::navigator::NavOutputs;
using mycar_navigation::navigator::NavigatorCore;
using mycar_navigation::navigator::NavState;

constexpr double kPi = 3.14159265358979323846;

Pose2D quatToPose2D(const geometry_msgs::msg::Pose & pose)
{
  const auto & q = pose.orientation;
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);

  Pose2D out;
  out.x = pose.position.x;
  out.y = pose.position.y;
  out.yaw = std::atan2(siny_cosp, cosy_cosp);
  return out;
}

uint8_t mapStatusState(NavState state)
{
  switch (state) {
    case NavState::ACCEPTED:
      return mycar_navigation::msg::GoalStatus::STATE_ACCEPTED;
    case NavState::ACTIVE:
      return mycar_navigation::msg::GoalStatus::STATE_ACTIVE;
    case NavState::RECOVERING:
      return mycar_navigation::msg::GoalStatus::STATE_RECOVERING;
    case NavState::REACHED:
      return mycar_navigation::msg::GoalStatus::STATE_REACHED;
    case NavState::BLOCKED_NO_TRAJECTORY:
      return mycar_navigation::msg::GoalStatus::STATE_BLOCKED_NO_TRAJECTORY;
    case NavState::BLOCKED_NO_PROGRESS:
      return mycar_navigation::msg::GoalStatus::STATE_BLOCKED_NO_PROGRESS;
    case NavState::SENSOR_TIMEOUT:
      return mycar_navigation::msg::GoalStatus::STATE_SENSOR_TIMEOUT;
    case NavState::INVALID_GOAL:
      return mycar_navigation::msg::GoalStatus::STATE_INVALID_GOAL;
    case NavState::MAP_CONFLICT:
      return mycar_navigation::msg::GoalStatus::STATE_MAP_CONFLICT;
    case NavState::ODOM_FAULT:
      return mycar_navigation::msg::GoalStatus::STATE_ODOM_FAULT;
    case NavState::IDLE:
      break;
  }
  throw std::runtime_error("NavState::IDLE should not be published");
}

std::string markerColorName(NavState state)
{
  switch (state) {
    case NavState::ACTIVE:
      return "active";
    case NavState::RECOVERING:
      return "recovering";
    case NavState::REACHED:
      return "reached";
    case NavState::BLOCKED_NO_TRAJECTORY:
    case NavState::BLOCKED_NO_PROGRESS:
    case NavState::MAP_CONFLICT:
    case NavState::ODOM_FAULT:
    case NavState::SENSOR_TIMEOUT:
    case NavState::INVALID_GOAL:
      return "fault";
    case NavState::ACCEPTED:
      return "accepted";
    case NavState::IDLE:
      return "idle";
  }
  return "idle";
}

std::array<float, 4> markerColor(const std::string & name)
{
  if (name == "active") {
    return {0.1F, 0.8F, 0.1F, 1.0F};
  }
  if (name == "recovering") {
    return {1.0F, 0.7F, 0.1F, 1.0F};
  }
  if (name == "reached") {
    return {0.1F, 0.5F, 1.0F, 1.0F};
  }
  if (name == "accepted") {
    return {0.8F, 0.8F, 0.2F, 1.0F};
  }
  return {1.0F, 0.2F, 0.2F, 1.0F};
}

class LocalNavigatorNode : public rclcpp::Node
{
public:
  LocalNavigatorNode()
  : Node("local_navigator")
  {
    const std::string map_yaml = declare_parameter<std::string>("map_yaml", "");
    if (map_yaml.empty()) {
      throw std::runtime_error("map_yaml parameter must not be empty");
    }

    NavCoreConfig cfg;
    cfg.planner.grid_resolution = declare_parameter<double>("planner.grid_resolution", cfg.planner.grid_resolution);
    cfg.planner.grid_half_extent = declare_parameter<double>("planner.grid_half_extent", cfg.planner.grid_half_extent);
    cfg.planner.footprint_half_length = declare_parameter<double>("planner.footprint_half_length_m", cfg.planner.footprint_half_length);
    cfg.planner.footprint_half_width = declare_parameter<double>("planner.footprint_half_width_m", cfg.planner.footprint_half_width);
    cfg.planner.min_turning_radius = declare_parameter<double>("planner.min_turning_radius_m", cfg.planner.min_turning_radius);
    cfg.planner.obstacle_radius = declare_parameter<double>("planner.obstacle_radius_m", cfg.planner.obstacle_radius);
    cfg.planner.localization_margin = declare_parameter<double>("planner.localization_margin_m", cfg.planner.localization_margin);
    cfg.planner.speed_margin_per_mps = declare_parameter<double>("planner.speed_margin_per_mps", cfg.planner.speed_margin_per_mps);
    cfg.planner.max_speed = declare_parameter<double>("planner.max_speed_mps", cfg.planner.max_speed);
    cfg.planner.min_speed = declare_parameter<double>("planner.min_speed_mps", cfg.planner.min_speed);
    cfg.planner.a_lat_max = declare_parameter<double>("planner.a_lat_max", cfg.planner.a_lat_max);
    cfg.planner.a_brake = declare_parameter<double>("planner.a_brake", cfg.planner.a_brake);
    cfg.planner.a_accel = declare_parameter<double>("planner.a_accel", cfg.planner.a_accel);
    cfg.planner.t_latency = declare_parameter<double>("planner.t_latency_sec", cfg.planner.t_latency);
    cfg.planner.brake_margin = declare_parameter<double>("planner.brake_margin_m", cfg.planner.brake_margin);
    cfg.planner.n_curvatures = declare_parameter<int>("planner.n_curvatures", cfg.planner.n_curvatures);
    cfg.planner.dt = declare_parameter<double>("planner.dt_sec", cfg.planner.dt);
    cfg.planner.min_horizon = declare_parameter<double>("planner.min_horizon_m", cfg.planner.min_horizon);
    cfg.planner.goal_approach_distance = declare_parameter<double>("planner.goal_approach_distance_m", cfg.planner.goal_approach_distance);
    cfg.planner.treat_unknown_as_forbidden = declare_parameter<bool>("planner.treat_unknown_as_forbidden", cfg.planner.treat_unknown_as_forbidden);
    cfg.planner.w_goal = declare_parameter<double>("planner.w_goal", cfg.planner.w_goal);
    cfg.planner.w_clear = declare_parameter<double>("planner.w_clear", cfg.planner.w_clear);
    cfg.planner.w_smooth = declare_parameter<double>("planner.w_smooth", cfg.planner.w_smooth);
    cfg.planner.w_soft_cost = declare_parameter<double>("planner.w_soft_cost", cfg.planner.w_soft_cost);
    cfg.planner.min_feasible_length = declare_parameter<double>("planner.min_feasible_length_m", cfg.planner.min_feasible_length);

    cfg.scan_soft_timeout_sec = declare_parameter<double>("scan_soft_timeout_sec", cfg.scan_soft_timeout_sec);
    cfg.scan_hard_timeout_sec = declare_parameter<double>("scan_hard_timeout_sec", cfg.scan_hard_timeout_sec);
    cfg.odom_timeout_sec = declare_parameter<double>("odom_timeout_sec", cfg.odom_timeout_sec);
    cfg.odom_jump_dist = declare_parameter<double>("odom_jump_dist_m", cfg.odom_jump_dist);
    cfg.odom_jump_yaw = declare_parameter<double>("odom_jump_yaw_rad", cfg.odom_jump_yaw);
    cfg.scan_odom_max_dt = declare_parameter<double>("scan_odom_max_dt_sec", cfg.scan_odom_max_dt);
    cfg.soft_timeout_speed_factor = declare_parameter<double>("soft_timeout_speed_factor", cfg.soft_timeout_speed_factor);
    cfg.transit_tolerance = declare_parameter<double>("transit_tolerance_m", cfg.transit_tolerance);
    cfg.stop_tolerance = declare_parameter<double>("stop_tolerance_m", cfg.stop_tolerance);
    cfg.max_recovery_attempts = declare_parameter<int>("max_recovery_attempts", cfg.max_recovery_attempts);
    cfg.progress_min_dist = declare_parameter<double>("progress_min_dist_m", cfg.progress_min_dist);
    cfg.progress_window_sec = declare_parameter<double>("progress_window_sec", cfg.progress_window_sec);
    cfg.map_conflict_cycles = declare_parameter<int>("map_conflict_cycles", cfg.map_conflict_cycles);
    cfg.goal_frame = declare_parameter<std::string>("goal_frame", cfg.goal_frame);

    Pose2D field_anchor;
    field_anchor.x = declare_parameter<double>("field_anchor.x", 0.0);
    field_anchor.y = declare_parameter<double>("field_anchor.y", 0.0);
    field_anchor.yaw = declare_parameter<double>("field_anchor.yaw", 0.0);

    laser_pose_in_base_.x = declare_parameter<double>("laser_pose_in_base.x", 0.0);
    laser_pose_in_base_.y = declare_parameter<double>("laser_pose_in_base.y", 0.0);
    laser_pose_in_base_.yaw = declare_parameter<double>("laser_pose_in_base.yaw", 0.0);

    timer_period_ms_ = declare_parameter<int>("control_period_ms", 50);
    const auto map = mycar_navigation::nav_core::MapMask::loadFromYaml(map_yaml);
    const FieldOdomTransform tf(field_anchor);
    field_tf_ = std::make_unique<FieldOdomTransform>(field_anchor);
    core_ = std::make_unique<NavigatorCore>(cfg, map, tf);

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    goal_status_pub_ = create_publisher<mycar_navigation::msg::GoalStatus>("/nav/goal_status", 10);
    field_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/nav/field_pose", 10);
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/nav/markers", 10);

    goal_sub_ = create_subscription<mycar_navigation::msg::NavGoal>(
      "/nav/goal", 10,
      [this](const mycar_navigation::msg::NavGoal::SharedPtr msg) {
        latest_goal_ = msg;
        pending_new_goal_ = true;
      });

    cancel_sub_ = create_subscription<std_msgs::msg::String>(
      "/nav/cancel", 10,
      [this](const std_msgs::msg::String::SharedPtr msg) {
        latest_goal_.reset();
        pending_new_goal_ = false;
        if (core_) {
          publishCommand(core_->cancelCurrentGoal(msg->data));
        } else {
          publishZeroCommand();
        }
      });

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom/data_raw", 10,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        latest_odom_ = msg;
      });

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        latest_scan_ = msg;
      });

    timer_ = create_wall_timer(
      std::chrono::milliseconds(timer_period_ms_),
      std::bind(&LocalNavigatorNode::onTimer, this));
  }

private:
  void onTimer()
  {
    NavInputs in;
    in.now_sec = now().seconds();

    if (latest_odom_) {
      in.have_odom = true;
      in.odom_pose = quatToPose2D(latest_odom_->pose.pose);
      in.odom_stamp_sec = rclcpp::Time(latest_odom_->header.stamp).seconds();
      publishFieldPose(in.odom_pose, latest_odom_->header.stamp);
    }

    if (latest_scan_) {
      in.have_scan = true;
      in.scan_stamp_sec = rclcpp::Time(latest_scan_->header.stamp).seconds();
      in.scan_points_base = mycar_navigation::navigator::laserScanToBasePoints(
        latest_scan_->ranges,
        latest_scan_->angle_min,
        latest_scan_->angle_increment,
        latest_scan_->range_min,
        latest_scan_->range_max,
        laser_pose_in_base_);
    }

    if (latest_goal_) {
      in.have_goal = true;
      in.goal_is_new = pending_new_goal_;
      in.goal_id = latest_goal_->goal_id;
      in.goal_frame = latest_goal_->pose.header.frame_id;
      in.goal_field_pose = quatToPose2D(latest_goal_->pose.pose);
      in.goal_type = latest_goal_->type;
      in.goal_max_speed = latest_goal_->max_speed;
      in.goal_tolerance = latest_goal_->tolerance;
    }

    const NavOutputs out = core_->update(in);
    publishCommand(out);
    if (out.publish_status) {
      publishStatus(out);
    }
    publishMarkers(out);
    pending_new_goal_ = false;
  }

  void publishCommand(const NavOutputs & out)
  {
    if (!out.publish_cmd) {
      return;
    }

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = out.cmd_v;
    cmd.angular.z = out.cmd_v * out.cmd_kappa;
    cmd_pub_->publish(cmd);
  }

  void publishZeroCommand()
  {
    geometry_msgs::msg::Twist cmd;
    cmd_pub_->publish(cmd);
  }

  void publishFieldPose(const Pose2D & odom_pose, const builtin_interfaces::msg::Time & stamp)
  {
    if (!field_tf_) {
      return;
    }
    const Pose2D field_pose = field_tf_->odomToField(odom_pose);
    geometry_msgs::msg::PoseStamped msg;
    msg.header.frame_id = "field";
    msg.header.stamp = stamp;
    msg.pose.position.x = field_pose.x;
    msg.pose.position.y = field_pose.y;
    msg.pose.orientation.z = std::sin(field_pose.yaw * 0.5);
    msg.pose.orientation.w = std::cos(field_pose.yaw * 0.5);
    field_pose_pub_->publish(msg);
  }

  void publishStatus(const NavOutputs & out)
  {
    mycar_navigation::msg::GoalStatus msg;
    msg.goal_id = out.status_goal_id;
    msg.stamp = now();
    msg.state = mapStatusState(out.state);
    msg.reason = out.reason;
    goal_status_pub_->publish(msg);
  }

  void publishMarkers(const NavOutputs & out)
  {
    visualization_msgs::msg::MarkerArray array;

    visualization_msgs::msg::Marker clear_marker;
    clear_marker.header.frame_id = "base_link";
    clear_marker.header.stamp = now();
    clear_marker.ns = "nav_selected_trajectory";
    clear_marker.id = 0;
    clear_marker.action = visualization_msgs::msg::Marker::DELETE;
    array.markers.push_back(clear_marker);

    if (!out.selected_trajectory.empty()) {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "base_link";
      marker.header.stamp = now();
      marker.ns = "nav_selected_trajectory";
      marker.id = 0;
      marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.scale.x = 0.03;
      const auto color = markerColor(markerColorName(out.state));
      marker.color.r = color[0];
      marker.color.g = color[1];
      marker.color.b = color[2];
      marker.color.a = color[3];
      for (const auto & pose : out.selected_trajectory) {
        geometry_msgs::msg::Point point;
        point.x = pose.x;
        point.y = pose.y;
        point.z = 0.0;
        marker.points.push_back(point);
      }
      array.markers.push_back(marker);
    }

    marker_pub_->publish(array);
  }

  int timer_period_ms_{50};
  bool pending_new_goal_{false};
  Pose2D laser_pose_in_base_;

  std::unique_ptr<NavigatorCore> core_;
  std::unique_ptr<FieldOdomTransform> field_tf_;
  mycar_navigation::msg::NavGoal::SharedPtr latest_goal_;
  nav_msgs::msg::Odometry::SharedPtr latest_odom_;
  sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<mycar_navigation::msg::GoalStatus>::SharedPtr goal_status_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr field_pose_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Subscription<mycar_navigation::msg::NavGoal>::SharedPtr goal_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cancel_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LocalNavigatorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
