#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

namespace hobot_nav
{

class AckermannCommandLimiter : public rclcpp::Node
{
public:
  AckermannCommandLimiter()
  : Node("ackermann_command_limiter")
  {
    // This node is the final vehicle-constraint layer. The defaults describe
    // the standalone navigation command chain.
    input_topic_ = declare_parameter<std::string>("input_topic", "/cmd_vel_nav");
    output_topic_ = declare_parameter<std::string>("output_topic", "/cmd_vel_safe");
    reverse_topic_ = declare_parameter<std::string>(
      "reverse_only_topic", "/navigation/reverse_only");
    direct_gate_topic_ = declare_parameter<std::string>(
      "direct_gate_topic", "/navigation/direct_gate_active");
    max_speed_ = declare_parameter<double>("profile_max_speed", 0.50);
    max_reverse_speed_ = declare_parameter<double>("profile_max_reverse_speed", 0.25);
    min_radius_ = declare_parameter<double>("min_turning_radius", 0.35);
    lateral_accel_ = declare_parameter<double>("lateral_accel_limit", 0.33);
    direct_gate_speed_limit_mps_ = declare_parameter<double>(
      "direct_gate_speed_limit_mps", 0.25);
    diagnostics_enabled_ = declare_parameter<bool>("capture_diagnostics_enabled", false);
    if (max_speed_ <= 0.0 || max_reverse_speed_ < 0.0 ||
      std::abs(min_radius_ - 0.35) > 1.0e-9 || lateral_accel_ <= 0.0 ||
      direct_gate_speed_limit_mps_ <= 0.0)
    {
      throw std::runtime_error("invalid Ackermann command limiter parameters");
    }

    output_pub_ = create_publisher<geometry_msgs::msg::Twist>(output_topic_, 1);
    if (diagnostics_enabled_) {
      diagnostics_pub_ = create_publisher<std_msgs::msg::String>(
        "/navigation/command_limiter/diagnostics", 1);
    }
    const auto gate_qos = rclcpp::QoS(1).reliable().transient_local();
    reverse_sub_ = create_subscription<std_msgs::msg::Bool>(
      reverse_topic_, gate_qos,
      [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
        reverse_only_.store(msg->data, std::memory_order_release);
      });
    direct_gate_sub_ = create_subscription<std_msgs::msg::Bool>(
      direct_gate_topic_, gate_qos,
      [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
        direct_gate_active_.store(msg->data, std::memory_order_release);
      });
    command_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      input_topic_, rclcpp::SensorDataQoS().keep_last(1),
      std::bind(&AckermannCommandLimiter::onCommand, this, std::placeholders::_1));
    RCLCPP_INFO(
      get_logger(),
      "Ackermann command limiter: input=%s output=%s forward=%.2f reverse=%.2f "
      "radius=%.2f lateral_accel=%.2f direct_gate=%.2f",
      input_topic_.c_str(), output_topic_.c_str(), max_speed_, max_reverse_speed_,
      min_radius_, lateral_accel_, direct_gate_speed_limit_mps_);
  }

private:
  void onCommand(geometry_msgs::msg::Twist::ConstSharedPtr msg)
  {
    if (!msg) {return;}
    geometry_msgs::msg::Twist output;
    double v = std::clamp(msg->linear.x, -max_reverse_speed_, max_speed_);
    std::string reason;
    if (reverse_only_.load(std::memory_order_acquire)) {
      v = std::min(0.0, v);
      reason = "reverse_only";
    } else {
      v = std::max(0.0, v);
      reason = "forward_only";
    }
    if (direct_gate_active_.load(std::memory_order_acquire) &&
      v > direct_gate_speed_limit_mps_)
    {
      v = direct_gate_speed_limit_mps_;
      reason = "direct_gate_limit";
    }

    double w = msg->angular.z;
    if (std::abs(v) < 1.0e-6) {
      w = 0.0;
    } else {
      w = std::clamp(w, -std::abs(v) / min_radius_, std::abs(v) / min_radius_);
      const double curvature = std::abs(w / v);
      if (curvature > 1.0e-6) {
        const double lateral_speed_limit = std::sqrt(lateral_accel_ / curvature);
        if (std::abs(v) > lateral_speed_limit) {
          const double scale = lateral_speed_limit / std::abs(v);
          v *= scale;
          w *= scale;
          reason = "lateral_accel_limit";
        }
      }
    }
    output.linear.x = v;
    output.angular.z = w;
    output_pub_->publish(output);

    if (diagnostics_pub_) {
      const auto stamp = std::chrono::steady_clock::now();
      if (last_diagnostic_ == std::chrono::steady_clock::time_point{} ||
        stamp - last_diagnostic_ >= std::chrono::seconds(1))
      {
        last_diagnostic_ = stamp;
        std_msgs::msg::String diagnostic;
        std::ostringstream text;
        text << "reason=" << reason << ",v=" << v << ",w=" << w
             << ",reverse_only=" << reverse_only_.load()
             << ",direct_gate=" << direct_gate_active_.load();
        diagnostic.data = text.str();
        diagnostics_pub_->publish(diagnostic);
      }
    }
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string reverse_topic_;
  std::string direct_gate_topic_;
  double max_speed_{0.50};
  double max_reverse_speed_{0.25};
  double min_radius_{0.35};
  double lateral_accel_{0.33};
  double direct_gate_speed_limit_mps_{0.25};
  bool diagnostics_enabled_{false};
  std::atomic<bool> reverse_only_{false};
  std::atomic<bool> direct_gate_active_{false};
  std::chrono::steady_clock::time_point last_diagnostic_{};
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr reverse_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr direct_gate_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr output_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr diagnostics_pub_;
};

}  // namespace hobot_nav

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<hobot_nav::AckermannCommandLimiter>());
  rclcpp::shutdown();
  return 0;
}
