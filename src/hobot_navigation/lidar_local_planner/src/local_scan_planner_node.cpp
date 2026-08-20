#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <string>
#include <utility>

namespace {

double Clamp(double value, double lower, double upper) {
  return std::max(lower, std::min(upper, value));
}

double DegreesToRadians(double degrees) {
  return degrees * M_PI / 180.0;
}

class LocalScanPlanner : public rclcpp::Node {
 public:
  LocalScanPlanner() : Node("local_scan_planner") {
    DeclareParameters();
    LoadParameters();

    auto cmd_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, cmd_qos);
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        scan_topic_,
        rclcpp::SensorDataQoS(),
        std::bind(&LocalScanPlanner::ScanCallback, this, std::placeholders::_1));
    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(1.0 / std::max(control_rate_, 1.0))),
        std::bind(&LocalScanPlanner::OnTimer, this));

    RCLCPP_INFO(
        get_logger(),
        "local_scan_planner started, scan_topic=%s, cmd_vel_topic=%s",
        scan_topic_.c_str(),
        cmd_vel_topic_.c_str());
  }

  struct HeadingResult {
    double angle;
    double range;
  };

  void PublishStopCommand() {
    PublishStop();
  }

 private:

  void DeclareParameters() {
    declare_parameter<std::string>("scan_topic", "/scan");
    declare_parameter<std::string>("cmd_vel_topic", "cmd_vel");
    declare_parameter<double>("control_rate", 10.0);
    declare_parameter<double>("scan_timeout", 0.5);
    declare_parameter<double>("max_planning_angle_deg", 100.0);
    declare_parameter<double>("front_sector_deg", 20.0);
    declare_parameter<double>("side_sector_deg", 60.0);
    declare_parameter<double>("stop_distance", 0.45);
    declare_parameter<double>("slow_distance", 1.20);
    declare_parameter<double>("turn_clearance_distance", 0.80);
    declare_parameter<double>("forward_speed", 0.35);
    declare_parameter<double>("min_forward_speed", 0.08);
    declare_parameter<double>("max_angular_speed", 1.20);
    declare_parameter<double>("turn_in_place_speed", 0.90);
    declare_parameter<double>("heading_gain", 1.80);
    declare_parameter<double>("heading_weight", 0.35);
    declare_parameter<double>("min_valid_range", 0.05);
    declare_parameter<double>("safety_bubble_radius", 0.10);
    declare_parameter<bool>("scan_filter_enabled", true);
    declare_parameter<int>("scan_filter_window", 5);
  }

  void LoadParameters() {
    scan_topic_ = get_parameter("scan_topic").as_string();
    cmd_vel_topic_ = get_parameter("cmd_vel_topic").as_string();
    control_rate_ = get_parameter("control_rate").as_double();
    scan_timeout_ = get_parameter("scan_timeout").as_double();
    max_planning_angle_ = DegreesToRadians(get_parameter("max_planning_angle_deg").as_double());
    front_sector_ = DegreesToRadians(get_parameter("front_sector_deg").as_double());
    side_sector_ = DegreesToRadians(get_parameter("side_sector_deg").as_double());
    stop_distance_ = get_parameter("stop_distance").as_double();
    slow_distance_ = get_parameter("slow_distance").as_double();
    turn_clearance_distance_ = get_parameter("turn_clearance_distance").as_double();
    forward_speed_ = get_parameter("forward_speed").as_double();
    min_forward_speed_ = get_parameter("min_forward_speed").as_double();
    max_angular_speed_ = get_parameter("max_angular_speed").as_double();
    turn_in_place_speed_ = get_parameter("turn_in_place_speed").as_double();
    heading_gain_ = get_parameter("heading_gain").as_double();
    heading_weight_ = get_parameter("heading_weight").as_double();
    min_valid_range_ = get_parameter("min_valid_range").as_double();
    safety_bubble_radius_ = get_parameter("safety_bubble_radius").as_double();
    scan_filter_enabled_ = get_parameter("scan_filter_enabled").as_bool();
    scan_filter_window_ = std::max(1, static_cast<int>(get_parameter("scan_filter_window").as_int()));
    if (scan_filter_window_ % 2 == 0) {
      scan_filter_window_ += 1;
    }
  }

  void ScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr message) {
    if (scan_filter_enabled_) {
      last_scan_ = std::make_shared<sensor_msgs::msg::LaserScan>(FilterScan(*message));
    } else {
      last_scan_ = message;
    }
    last_scan_stamp_ = now();
    has_last_scan_stamp_ = true;
  }

  void OnTimer() {
    if (!last_scan_ || !has_last_scan_stamp_) {
      PublishStop();
      return;
    }

    if ((now() - last_scan_stamp_).seconds() > scan_timeout_) {
      PublishStop();
      return;
    }

    cmd_pub_->publish(ComputeCommand(*last_scan_));
  }

  geometry_msgs::msg::Twist ComputeCommand(const sensor_msgs::msg::LaserScan &scan) {
    HeadingResult best_heading{};
    const bool has_best_heading = FindBestHeading(scan, best_heading);
    const double front_clearance = SectorClearance(scan, -front_sector_, front_sector_);
    const double left_clearance = SectorClearance(scan, 0.0, side_sector_);
    const double right_clearance = SectorClearance(scan, -side_sector_, 0.0);

    geometry_msgs::msg::Twist command;
    if (!has_best_heading) {
      command.angular.z = turn_in_place_speed_ * PreferredTurnSign(left_clearance, right_clearance);
      last_turn_sign_ = std::copysign(1.0, command.angular.z);
      return command;
    }

    const double best_angle = best_heading.angle;
    const double angular = Clamp(
        heading_gain_ * best_angle,
        -max_angular_speed_,
        max_angular_speed_);
    const double preferred_turn_sign =
        std::abs(angular) > 1e-3 ? std::copysign(1.0, angular)
                                : PreferredTurnSign(left_clearance, right_clearance);

    if (front_clearance <= stop_distance_) {
      command.angular.z = turn_in_place_speed_ * preferred_turn_sign;
      last_turn_sign_ = preferred_turn_sign;
      return command;
    }

    if (std::abs(best_angle) > DegreesToRadians(35.0) &&
        front_clearance < turn_clearance_distance_) {
      command.angular.z = Clamp(
          angular,
          -turn_in_place_speed_,
          turn_in_place_speed_);
      last_turn_sign_ = preferred_turn_sign;
      return command;
    }

    const double speed_scale = Clamp(
        (front_clearance - stop_distance_) /
            std::max(slow_distance_ - stop_distance_, 1e-3),
        0.0,
        1.0);
    const double alignment_scale = Clamp(
        1.0 - std::abs(best_angle) / std::max(max_planning_angle_, 1e-3),
        0.2,
        1.0);

    double linear = forward_speed_ * std::min(speed_scale, alignment_scale);
    if (linear > 0.0) {
      linear = std::max(linear, min_forward_speed_);
    }

    command.linear.x = linear;
    command.angular.z = angular;
    last_turn_sign_ = preferred_turn_sign;
    return command;
  }

  double SectorClearance(
      const sensor_msgs::msg::LaserScan &scan,
      double min_angle,
      double max_angle) const {
    double best = std::numeric_limits<double>::infinity();
    bool found = false;

    ForEachValidPoint(
        scan,
        [&](double angle, double distance) {
          if (angle >= min_angle && angle <= max_angle) {
            best = std::min(best, distance);
            found = true;
          }
        });

    return found ? best : 0.0;
  }

  bool FindBestHeading(
      const sensor_msgs::msg::LaserScan &scan,
      HeadingResult &best_heading) const {
    double best_score = -std::numeric_limits<double>::infinity();
    bool found = false;
    best_heading.angle = 0.0;
    best_heading.range = 0.0;

    ForEachValidPoint(
        scan,
        [&](double angle, double distance) {
          if (std::abs(angle) > max_planning_angle_) {
            return;
          }
          const double effective_distance = std::max(0.0, distance - safety_bubble_radius_);
          const double score = effective_distance - heading_weight_ * std::abs(angle);
          if (!found || score > best_score) {
            best_score = score;
            best_heading = HeadingResult{angle, effective_distance};
            found = true;
          }
        });

    return found;
  }

  template <typename Callback>
  void ForEachValidPoint(
      const sensor_msgs::msg::LaserScan &scan,
      Callback &&callback) const {
    const double min_range = std::max(
        static_cast<double>(scan.range_min),
        min_valid_range_);

    for (size_t index = 0; index < scan.ranges.size(); ++index) {
      const float raw_range = scan.ranges[index];
      double distance = 0.0;

      if (std::isnan(raw_range)) {
        continue;
      }

      if (std::isinf(raw_range)) {
        if (scan.range_max <= 0.0f) {
          continue;
        }
        distance = scan.range_max;
      } else {
        distance = raw_range;
      }

      if (distance < min_range) {
        continue;
      }
      if (scan.range_max > 0.0f && distance > scan.range_max) {
        continue;
      }

      const double angle =
          static_cast<double>(scan.angle_min) +
          static_cast<double>(index) * static_cast<double>(scan.angle_increment);
      callback(angle, distance);
    }
  }

  sensor_msgs::msg::LaserScan FilterScan(const sensor_msgs::msg::LaserScan &scan) const {
    sensor_msgs::msg::LaserScan filtered = scan;
    if (scan_filter_window_ <= 1 || scan.ranges.empty()) {
      return filtered;
    }

    const int radius = scan_filter_window_ / 2;
    std::vector<float> window;
    window.reserve(static_cast<size_t>(scan_filter_window_));

    for (size_t index = 0; index < scan.ranges.size(); ++index) {
      window.clear();
      const int start = std::max<int>(0, static_cast<int>(index) - radius);
      const int end = std::min<int>(static_cast<int>(scan.ranges.size()) - 1, static_cast<int>(index) + radius);

      for (int sample = start; sample <= end; ++sample) {
        const float value = scan.ranges[static_cast<size_t>(sample)];
        if (std::isfinite(value) && value >= scan.range_min &&
            (scan.range_max <= 0.0f || value <= scan.range_max)) {
          window.push_back(value);
        }
      }

      if (window.empty()) {
        continue;
      }

      const auto middle = window.begin() + static_cast<long>(window.size() / 2);
      std::nth_element(window.begin(), middle, window.end());
      filtered.ranges[index] = *middle;
    }

    return filtered;
  }

  double PreferredTurnSign(double left_clearance, double right_clearance) const {
    if (left_clearance == right_clearance) {
      return last_turn_sign_;
    }
    return left_clearance >= right_clearance ? 1.0 : -1.0;
  }

  void PublishStop() {
    if (cmd_pub_) {
      cmd_pub_->publish(geometry_msgs::msg::Twist());
    }
  }

  std::string scan_topic_;
  std::string cmd_vel_topic_;
  double control_rate_ = 10.0;
  double scan_timeout_ = 0.5;
  double max_planning_angle_ = DegreesToRadians(100.0);
  double front_sector_ = DegreesToRadians(20.0);
  double side_sector_ = DegreesToRadians(60.0);
  double stop_distance_ = 0.45;
  double slow_distance_ = 1.20;
  double turn_clearance_distance_ = 0.80;
  double forward_speed_ = 0.35;
  double min_forward_speed_ = 0.08;
  double max_angular_speed_ = 1.20;
  double turn_in_place_speed_ = 0.90;
  double heading_gain_ = 1.80;
  double heading_weight_ = 0.35;
  double min_valid_range_ = 0.05;
  double safety_bubble_radius_ = 0.10;
  bool scan_filter_enabled_ = true;
  int scan_filter_window_ = 5;
  double last_turn_sign_ = 1.0;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  sensor_msgs::msg::LaserScan::SharedPtr last_scan_;
  rclcpp::Time last_scan_stamp_{0, 0, RCL_ROS_TIME};
  bool has_last_scan_stamp_ = false;
};

}  // namespace

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LocalScanPlanner>();
  rclcpp::spin(node);
  node->PublishStopCommand();
  rclcpp::shutdown();
  return 0;
}
