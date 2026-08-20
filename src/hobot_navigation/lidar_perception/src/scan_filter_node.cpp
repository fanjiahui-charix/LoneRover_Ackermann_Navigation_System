#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace
{

double deg2rad(double degrees)
{
  return degrees * M_PI / 180.0;
}

float invalid_value(bool use_nan_for_invalid)
{
  if (use_nan_for_invalid) {
    return std::numeric_limits<float>::quiet_NaN();
  }
  return std::numeric_limits<float>::infinity();
}

bool is_finite_valid(float value)
{
  return std::isfinite(value) && value > 0.0F;
}

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

bool angle_in_range(double angle, double min_angle, double max_angle)
{
  angle = normalize_angle(angle);
  min_angle = normalize_angle(min_angle);
  max_angle = normalize_angle(max_angle);
  if (min_angle <= max_angle) {
    return angle >= min_angle && angle <= max_angle;
  }
  return angle >= min_angle || angle <= max_angle;
}

struct Point2D
{
  double x{0.0};
  double y{0.0};
};

bool point_in_polygon(const Point2D & point, const std::vector<Point2D> & polygon)
{
  bool inside = false;
  for (size_t i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++) {
    const auto & a = polygon[i];
    const auto & b = polygon[j];
    const bool crosses = (a.y > point.y) != (b.y > point.y);
    if (!crosses) {
      continue;
    }
    const double intersection_x =
      (b.x - a.x) * (point.y - a.y) / (b.y - a.y) + a.x;
    if (point.x < intersection_x) {
      inside = !inside;
    }
  }
  return inside;
}

struct FilterStats
{
  size_t total{0};
  size_t raw_invalid{0};
  size_t too_near{0};
  size_t too_far{0};
  size_t low_intensity{0};
  size_t self_masked{0};
  size_t polygon_self_masked{0};
  size_t raw_valid{0};
  size_t finite_output{0};
};

}  // namespace

class ScanFilterNode : public rclcpp::Node
{
public:
  ScanFilterNode()
  : Node("scan_filter_node")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/scan_raw");
    output_topic_ = declare_parameter<std::string>("output_topic", "/scan");
    target_frame_ = declare_parameter<std::string>("target_frame", "laser_link");

    min_range_ = declare_parameter<double>("min_range", 0.15);
    max_range_ = declare_parameter<double>("max_range", 6.0);
    range_scale_ = declare_parameter<double>("range_scale", 1.0);
    range_bias_ = declare_parameter<double>("range_bias", 0.0);
    angle_offset_ = deg2rad(declare_parameter<double>("angle_offset_deg", 0.0));
    reverse_scan_ = declare_parameter<bool>("reverse_scan", false);

    enable_median_filter_ = declare_parameter<bool>("enable_median_filter", true);
    median_window_ = declare_parameter<int>("median_window", 3);
    fill_invalid_with_median_ = declare_parameter<bool>("fill_invalid_with_median", false);
    enable_jump_filter_ = declare_parameter<bool>("enable_jump_filter", true);
    jump_threshold_base_ = declare_parameter<double>("jump_threshold_base", 0.08);
    jump_threshold_scale_ = declare_parameter<double>("jump_threshold_scale", 0.05);
    min_preserve_cluster_points_ = declare_parameter<int>("min_preserve_cluster_points", 3);
    enable_speckle_filter_ = declare_parameter<bool>("enable_speckle_filter", true);
    speckle_neighbor_count_ = declare_parameter<int>("speckle_neighbor_count", 1);
    speckle_search_radius_ = declare_parameter<int>("speckle_search_radius", 2);
    speckle_range_threshold_ = declare_parameter<double>("speckle_range_threshold", 0.12);
    min_intensity_ = declare_parameter<double>("min_intensity", 0.0);

    // Legacy angle/range mask remains opt-in for compatibility. The polygon
    // mask below is the primary self-filter and does not create a frontal blind sector.
    enable_self_mask_ = declare_parameter<bool>("enable_self_mask", false);
    self_mask_angle_min_ = deg2rad(declare_parameter<double>("self_mask_angle_min_deg", -20.0));
    self_mask_angle_max_ = deg2rad(declare_parameter<double>("self_mask_angle_max_deg", 20.0));
    self_mask_range_max_ = declare_parameter<double>("self_mask_range_max", 0.12);
    enable_self_polygon_mask_ =
      declare_parameter<bool>("enable_self_polygon_mask", true);
    self_mask_frame_ = declare_parameter<std::string>("self_mask_frame", "base_link");
    self_mask_transform_timeout_ =
      declare_parameter<double>("self_mask_transform_timeout", 0.05);
    const auto self_mask_polygon_flat = declare_parameter<std::vector<double>>(
      "self_mask_polygon",
      std::vector<double>{0.24, 0.16, 0.24, -0.16, -0.24, -0.16, -0.24, 0.16});
    if (self_mask_polygon_flat.size() < 6 || self_mask_polygon_flat.size() % 2 != 0) {
      throw std::invalid_argument(
              "self_mask_polygon must contain at least three flattened x/y pairs");
    }
    for (size_t i = 0; i < self_mask_polygon_flat.size(); i += 2) {
      self_mask_polygon_.push_back({self_mask_polygon_flat[i], self_mask_polygon_flat[i + 1]});
    }

    use_inf_for_far_ = declare_parameter<bool>("use_inf_for_far", true);
    use_nan_for_invalid_ = declare_parameter<bool>("use_nan_for_invalid", true);
    enable_diagnostics_ = declare_parameter<bool>("enable_diagnostics", true);
    diagnostics_every_n_ = declare_parameter<int>("diagnostics_every_n", 50);

    if (median_window_ < 1) {
      median_window_ = 1;
    }
    if (median_window_ % 2 == 0) {
      ++median_window_;
    }
    min_preserve_cluster_points_ = std::max(1, min_preserve_cluster_points_);
    speckle_search_radius_ = std::max(1, speckle_search_radius_);
    diagnostics_every_n_ = std::max(1, diagnostics_every_n_);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    rclcpp::SensorDataQoS sensor_qos;
    scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>(output_topic_, sensor_qos);
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      input_topic_, sensor_qos,
      std::bind(&ScanFilterNode::scanCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(), "scan_filter_node: %s -> %s, frame=%s, range=[%.2f, %.2f]",
      input_topic_.c_str(), output_topic_.c_str(), target_frame_.c_str(), min_range_, max_range_);
  }

private:
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    FilterStats stats;
    sensor_msgs::msg::LaserScan out = *msg;
    if (!target_frame_.empty()) {
      out.header.frame_id = target_frame_;
    }
    if (reverse_scan_ && !msg->ranges.empty()) {
      out.angle_min = static_cast<float>(
        msg->angle_min + static_cast<double>(msg->ranges.size() - 1) * msg->angle_increment +
        angle_offset_);
      out.angle_max = static_cast<float>(msg->angle_min + angle_offset_);
      out.angle_increment = -msg->angle_increment;
    } else {
      out.angle_min = static_cast<float>(msg->angle_min + angle_offset_);
      out.angle_max = static_cast<float>(msg->angle_max + angle_offset_);
    }
    out.range_min = static_cast<float>(min_range_);
    out.range_max = static_cast<float>(max_range_);
    out.ranges = preprocess(*msg, stats);

    if (enable_median_filter_) {
      out.ranges = medianFilter(out.ranges);
    }
    int jump_removed = 0;
    if (enable_jump_filter_) {
      jump_removed = applyJumpFilter(out.ranges);
    }
    int speckle_removed = 0;
    if (enable_speckle_filter_) {
      speckle_removed = applySpeckleFilter(out.ranges);
    }
    applySelfPolygonMask(out, stats);
    stats.finite_output = countFinite(out.ranges);
    publishDiagnostics(stats, jump_removed, speckle_removed);

    scan_pub_->publish(out);
  }

  std::vector<float> preprocess(const sensor_msgs::msg::LaserScan & scan, FilterStats & stats) const
  {
    std::vector<float> ranges(scan.ranges.size(), invalid_value(use_nan_for_invalid_));
    const float far_value = use_inf_for_far_ ?
      std::numeric_limits<float>::infinity() :
      invalid_value(use_nan_for_invalid_);

    stats.total = scan.ranges.size();
    for (size_t i = 0; i < scan.ranges.size(); ++i) {
      const size_t source_index = reverse_scan_ ? (scan.ranges.size() - 1 - i) : i;
      const float raw = scan.ranges[source_index];
      if (!std::isfinite(raw) || raw <= 0.0F) {
        if (std::isinf(raw) && use_inf_for_far_) {
          ranges[i] = std::numeric_limits<float>::infinity();
        }
        ++stats.raw_invalid;
        continue;
      }
      if (min_intensity_ > 0.0 && source_index < scan.intensities.size() &&
          scan.intensities[source_index] < min_intensity_) {
        ++stats.low_intensity;
        continue;
      }

      const double corrected = range_scale_ * static_cast<double>(raw) + range_bias_;
      if (corrected < min_range_) {
        ranges[i] = invalid_value(use_nan_for_invalid_);
        ++stats.too_near;
        continue;
      }
      if (corrected > max_range_) {
        ranges[i] = far_value;
        ++stats.too_far;
        continue;
      }
      ++stats.raw_valid;

      const double angle =
        scan.angle_min + static_cast<double>(source_index) * scan.angle_increment + angle_offset_;
      if (isSelfMasked(angle, corrected)) {
        ranges[i] = invalid_value(use_nan_for_invalid_);
        ++stats.self_masked;
        continue;
      }

      ranges[i] = static_cast<float>(corrected);
    }
    return ranges;
  }

  bool isSelfMasked(double angle, double range) const
  {
    if (!enable_self_mask_) {
      return false;
    }
    return angle_in_range(angle, self_mask_angle_min_, self_mask_angle_max_) &&
           range <= self_mask_range_max_;
  }

  void applySelfPolygonMask(sensor_msgs::msg::LaserScan & scan, FilterStats & stats)
  {
    if (!enable_self_polygon_mask_ || self_mask_polygon_.size() < 3) {
      return;
    }

    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_->lookupTransform(
        self_mask_frame_, scan.header.frame_id, rclcpp::Time(scan.header.stamp),
        rclcpp::Duration::from_seconds(self_mask_transform_timeout_));
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "self polygon mask waiting for %s <- %s TF: %s",
        self_mask_frame_.c_str(), scan.header.frame_id.c_str(), error.what());
      return;
    }

    const auto & q = transform.transform.rotation;
    const double r00 = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    const double r01 = 2.0 * (q.x * q.y - q.z * q.w);
    const double r10 = 2.0 * (q.x * q.y + q.z * q.w);
    const double r11 = 1.0 - 2.0 * (q.x * q.x + q.z * q.z);
    const double tx = transform.transform.translation.x;
    const double ty = transform.transform.translation.y;

    for (size_t i = 0; i < scan.ranges.size(); ++i) {
      const float range = scan.ranges[i];
      if (!is_finite_valid(range)) {
        continue;
      }
      const double angle = scan.angle_min + static_cast<double>(i) * scan.angle_increment;
      const double laser_x = static_cast<double>(range) * std::cos(angle);
      const double laser_y = static_cast<double>(range) * std::sin(angle);
      const Point2D base_point{
        r00 * laser_x + r01 * laser_y + tx,
        r10 * laser_x + r11 * laser_y + ty};
      if (point_in_polygon(base_point, self_mask_polygon_)) {
        scan.ranges[i] = invalid_value(use_nan_for_invalid_);
        ++stats.polygon_self_masked;
      }
    }
  }

  std::vector<float> medianFilter(const std::vector<float> & ranges) const
  {
    if (median_window_ <= 1 || ranges.empty()) {
      return ranges;
    }

    std::vector<float> filtered = ranges;
    const int radius = median_window_ / 2;
    for (int i = 0; i < static_cast<int>(ranges.size()); ++i) {
      if (!fill_invalid_with_median_ && !is_finite_valid(ranges[static_cast<size_t>(i)])) {
        continue;
      }
      std::vector<float> window;
      const int begin = std::max(0, i - radius);
      const int end = std::min(static_cast<int>(ranges.size()) - 1, i + radius);
      for (int j = begin; j <= end; ++j) {
        if (is_finite_valid(ranges[j])) {
          window.push_back(ranges[j]);
        }
      }
      if (window.empty()) {
        continue;
      }
      std::sort(window.begin(), window.end());
      filtered[i] = window[window.size() / 2];
    }
    return filtered;
  }

  int applyJumpFilter(std::vector<float> & ranges) const
  {
    int removed = 0;
    if (ranges.size() < 3) {
      return removed;
    }

    const std::vector<float> original = ranges;
    for (size_t i = 1; i + 1 < original.size(); ++i) {
      const float current = original[i];
      if (!is_finite_valid(current)) {
        continue;
      }

      const float left = nearestFinite(original, static_cast<int>(i) - 1, -1);
      const float right = nearestFinite(original, static_cast<int>(i) + 1, 1);
      if (!is_finite_valid(left) || !is_finite_valid(right)) {
        continue;
      }

      const double threshold = jump_threshold_base_ + jump_threshold_scale_ * current;
      const bool current_far_from_neighbors =
        std::fabs(current - left) > threshold && std::fabs(current - right) > threshold;
      const bool neighbors_agree = std::fabs(left - right) <= threshold;
      const bool part_of_small_cluster =
        localClusterSize(original, static_cast<int>(i)) >= min_preserve_cluster_points_;
      if (current_far_from_neighbors && neighbors_agree && !part_of_small_cluster) {
        ranges[i] = invalid_value(use_nan_for_invalid_);
        ++removed;
      }
    }
    return removed;
  }

  int applySpeckleFilter(std::vector<float> & ranges) const
  {
    int removed = 0;
    if (speckle_neighbor_count_ <= 0 || ranges.size() < 3) {
      return removed;
    }

    const std::vector<float> original = ranges;
    for (int i = 0; i < static_cast<int>(original.size()); ++i) {
      const float current = original[static_cast<size_t>(i)];
      if (!is_finite_valid(current)) {
        continue;
      }

      int neighbors = 0;
      const int begin = std::max(0, i - speckle_search_radius_);
      const int end =
        std::min(static_cast<int>(original.size()) - 1, i + speckle_search_radius_);
      for (int j = begin; j <= end; ++j) {
        if (j == i) {
          continue;
        }
        const float other = original[static_cast<size_t>(j)];
        if (is_finite_valid(other) && std::fabs(other - current) <= speckle_range_threshold_) {
          ++neighbors;
        }
      }
      const bool part_of_small_cluster =
        localClusterSize(original, i) >= min_preserve_cluster_points_;
      if (neighbors < speckle_neighbor_count_ && !part_of_small_cluster) {
        ranges[static_cast<size_t>(i)] = invalid_value(use_nan_for_invalid_);
        ++removed;
      }
    }
    return removed;
  }

  size_t countFinite(const std::vector<float> & ranges) const
  {
    return static_cast<size_t>(
      std::count_if(
        ranges.begin(), ranges.end(), [](float value) {return is_finite_valid(value);}));
  }

  void publishDiagnostics(const FilterStats & stats, int jump_removed, int speckle_removed)
  {
    if (!enable_diagnostics_) {
      return;
    }
    ++scan_count_;
    if (scan_count_ % static_cast<size_t>(diagnostics_every_n_) != 0) {
      return;
    }
    RCLCPP_INFO(
      get_logger(),
      "scan_filter stats: total=%zu raw_valid=%zu finite=%zu raw_invalid=%zu near=%zu "
      "far=%zu intensity=%zu legacy_self=%zu polygon_self=%zu jump=%d speckle=%d",
      stats.total, stats.raw_valid, stats.finite_output, stats.raw_invalid, stats.too_near,
      stats.too_far, stats.low_intensity, stats.self_masked, stats.polygon_self_masked,
      jump_removed, speckle_removed);
  }

  int localClusterSize(const std::vector<float> & ranges, int index) const
  {
    const float center = ranges[static_cast<size_t>(index)];
    if (!is_finite_valid(center)) {
      return 0;
    }

    int count = 1;
    float previous = center;
    for (int i = index - 1; i >= 0; --i) {
      if (!sameLocalSurface(previous, ranges[static_cast<size_t>(i)])) {
        break;
      }
      previous = ranges[static_cast<size_t>(i)];
      ++count;
    }

    previous = center;
    for (int i = index + 1; i < static_cast<int>(ranges.size()); ++i) {
      if (!sameLocalSurface(previous, ranges[static_cast<size_t>(i)])) {
        break;
      }
      previous = ranges[static_cast<size_t>(i)];
      ++count;
    }
    return count;
  }

  bool sameLocalSurface(float a, float b) const
  {
    if (!is_finite_valid(a) || !is_finite_valid(b)) {
      return false;
    }
    const double range = std::min(static_cast<double>(a), static_cast<double>(b));
    const double threshold = jump_threshold_base_ + jump_threshold_scale_ * range;
    return std::fabs(static_cast<double>(a) - static_cast<double>(b)) <= threshold;
  }

  float nearestFinite(const std::vector<float> & ranges, int start, int step) const
  {
    int checked = 0;
    for (int i = start; i >= 0 && i < static_cast<int>(ranges.size()) && checked < 3; i += step) {
      if (is_finite_valid(ranges[i])) {
        return ranges[i];
      }
      ++checked;
    }
    return std::numeric_limits<float>::quiet_NaN();
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string target_frame_;

  double min_range_{0.15};
  double max_range_{6.0};
  double range_scale_{1.0};
  double range_bias_{0.0};
  double angle_offset_{0.0};
  bool reverse_scan_{false};
  bool enable_median_filter_{true};
  int median_window_{3};
  bool fill_invalid_with_median_{false};
  bool enable_jump_filter_{true};
  double jump_threshold_base_{0.08};
  double jump_threshold_scale_{0.05};
  int min_preserve_cluster_points_{3};
  bool enable_speckle_filter_{true};
  int speckle_neighbor_count_{1};
  int speckle_search_radius_{2};
  double speckle_range_threshold_{0.12};
  double min_intensity_{0.0};
  bool enable_self_mask_{true};
  double self_mask_angle_min_{0.0};
  double self_mask_angle_max_{0.0};
  double self_mask_range_max_{0.12};
  bool enable_self_polygon_mask_{true};
  std::string self_mask_frame_{"base_link"};
  double self_mask_transform_timeout_{0.05};
  std::vector<Point2D> self_mask_polygon_;
  bool use_inf_for_far_{true};
  bool use_nan_for_invalid_{true};
  bool enable_diagnostics_{true};
  int diagnostics_every_n_{50};
  size_t scan_count_{0};

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ScanFilterNode>());
  rclcpp::shutdown();
  return 0;
}
