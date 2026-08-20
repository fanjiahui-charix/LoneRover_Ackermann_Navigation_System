#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace
{

double normalize_angle(double angle)
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

double wrap_angle_near_scan(double angle, double angle_min, double angle_increment, int count)
{
  if (count <= 0) {
    return angle;
  }
  const double angle_max = angle_min + angle_increment * static_cast<double>(count - 1);
  const double lower = std::min(angle_min, angle_max);
  const double upper = std::max(angle_min, angle_max);
  while (angle < lower) {
    angle += 2.0 * M_PI;
  }
  while (angle > upper) {
    angle -= 2.0 * M_PI;
  }
  return angle;
}

bool valid_range(float range)
{
  return std::isfinite(range) && range > 0.0F;
}

}  // namespace

class ScanDeskewNode : public rclcpp::Node
{
public:
  ScanDeskewNode()
  : Node("scan_deskew_node")
  {
    scan_topic_ = declare_parameter<std::string>("scan_topic", "/scan_raw");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/odom");
    output_topic_ = declare_parameter<std::string>("output_topic", "/scan_deskewed");
    output_frame_ = declare_parameter<std::string>("output_frame", "laser_link");
    scan_period_ = declare_parameter<double>("scan_period", 0.10);
    reference_time_ratio_ = declare_parameter<double>("reference_time_ratio", 0.5);
    enable_deskew_ = declare_parameter<bool>("enable_deskew", true);
    max_odom_age_ = declare_parameter<double>("max_odom_age", 0.4);

    reference_time_ratio_ = std::clamp(reference_time_ratio_, 0.0, 1.0);

    rclcpp::SensorDataQoS sensor_qos;
    scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>(output_topic_, sensor_qos);
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, sensor_qos,
      std::bind(&ScanDeskewNode::scanCallback, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, 20,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) { last_odom_ = msg; });

    RCLCPP_INFO(
      get_logger(),
      "scan_deskew_node: %s + %s -> %s, scan_period=%.3f, reference_ratio=%.2f",
      scan_topic_.c_str(), odom_topic_.c_str(), output_topic_.c_str(), scan_period_,
      reference_time_ratio_);
  }

private:
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    sensor_msgs::msg::LaserScan out = *msg;
    if (!output_frame_.empty()) {
      out.header.frame_id = output_frame_;
    }

    const double scan_duration = getScanDuration(*msg);
    if (out.scan_time <= 0.0F) {
      out.scan_time = static_cast<float>(scan_duration);
    }
    if (out.time_increment <= 0.0F && out.ranges.size() > 1) {
      out.time_increment = static_cast<float>(scan_duration / static_cast<double>(out.ranges.size() - 1));
    }

    if (!enable_deskew_ || !last_odom_ || !odomFreshEnough(*msg)) {
      scan_pub_->publish(out);
      return;
    }

    out.ranges.assign(msg->ranges.size(), std::numeric_limits<float>::infinity());
    out.intensities.assign(msg->ranges.size(), 0.0F);

    const double vx = last_odom_->twist.twist.linear.x;
    const double vy = last_odom_->twist.twist.linear.y;
    const double wz = last_odom_->twist.twist.angular.z;
    const double reference_t = reference_time_ratio_ * scan_duration;
    const double angle_min = msg->angle_min;
    const double angle_increment = msg->angle_increment;
    const int n = static_cast<int>(msg->ranges.size());

    for (int i = 0; i < n; ++i) {
      const float range = msg->ranges[static_cast<size_t>(i)];
      if (!valid_range(range)) {
        continue;
      }

      const double ray_t = rayTime(*msg, i, scan_duration);
      const double dt = ray_t - reference_t;
      const double dtheta = wz * dt;
      const double dx = vx * dt;
      const double dy = vy * dt;

      const double angle = angle_min + static_cast<double>(i) * angle_increment;
      const double x = range * std::cos(angle);
      const double y = range * std::sin(angle);

      const double cos_yaw = std::cos(dtheta);
      const double sin_yaw = std::sin(dtheta);
      const double corrected_x = cos_yaw * x - sin_yaw * y + dx;
      const double corrected_y = sin_yaw * x + cos_yaw * y + dy;
      const double corrected_range = std::hypot(corrected_x, corrected_y);
      const double corrected_angle = wrap_angle_near_scan(
        normalize_angle(std::atan2(corrected_y, corrected_x)), angle_min, angle_increment, n);
      const int target_index = static_cast<int>(std::llround((corrected_angle - angle_min) / angle_increment));
      if (target_index < 0 || target_index >= n) {
        continue;
      }

      auto & target = out.ranges[static_cast<size_t>(target_index)];
      if (!valid_range(target) || corrected_range < target) {
        target = static_cast<float>(corrected_range);
        if (static_cast<size_t>(i) < msg->intensities.size()) {
          out.intensities[static_cast<size_t>(target_index)] = msg->intensities[static_cast<size_t>(i)];
        }
      }
    }

    scan_pub_->publish(out);
  }

  double getScanDuration(const sensor_msgs::msg::LaserScan & scan) const
  {
    if (scan.scan_time > 0.0F) {
      return scan.scan_time;
    }
    if (scan.time_increment > 0.0F && scan.ranges.size() > 1) {
      return scan.time_increment * static_cast<double>(scan.ranges.size() - 1);
    }
    return scan_period_;
  }

  double rayTime(const sensor_msgs::msg::LaserScan & scan, int index, double scan_duration) const
  {
    if (scan.time_increment > 0.0F) {
      return static_cast<double>(index) * scan.time_increment;
    }
    if (scan.ranges.size() <= 1) {
      return 0.0;
    }
    return scan_duration * static_cast<double>(index) / static_cast<double>(scan.ranges.size() - 1);
  }

  bool odomFreshEnough(const sensor_msgs::msg::LaserScan & scan) const
  {
    if (max_odom_age_ <= 0.0) {
      return true;
    }
    const rclcpp::Time scan_stamp(scan.header.stamp);
    const rclcpp::Time odom_stamp(last_odom_->header.stamp);
    if (scan_stamp.nanoseconds() == 0 || odom_stamp.nanoseconds() == 0) {
      return true;
    }
    return std::fabs((scan_stamp - odom_stamp).seconds()) <= max_odom_age_;
  }

  std::string scan_topic_;
  std::string odom_topic_;
  std::string output_topic_;
  std::string output_frame_;
  double scan_period_{0.10};
  double reference_time_ratio_{0.5};
  bool enable_deskew_{true};
  double max_odom_age_{0.4};

  nav_msgs::msg::Odometry::SharedPtr last_odom_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ScanDeskewNode>());
  rclcpp::shutdown();
  return 0;
}
