#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose_array.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "std_msgs/msg/header.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace
{

struct ScanPoint
{
  int index{0};
  double x{0.0};
  double y{0.0};
  double range{0.0};
};

struct ConeCandidate
{
  double x{0.0};
  double y{0.0};
  double range{0.0};
  double width{0.0};
  double radial_thickness{0.0};
  double pca_ratio{1.0};
  int count{0};
};

enum class ClusterClass
{
  noise,
  cone_candidate,
  non_cone
};

struct ClusterDebug
{
  double x{0.0};
  double y{0.0};
  double range{0.0};
  double width{0.0};
  double radial_thickness{0.0};
  double pca_ratio{1.0};
  int count{0};
  ClusterClass type{ClusterClass::noise};
};

struct Track
{
  int id{0};
  double x{0.0};
  double y{0.0};
  double range{0.0};
  double width{0.0};
  double radial_thickness{0.0};
  double pca_ratio{1.0};
  int point_count{0};
  int hits{0};
  rclcpp::Time last_seen;
};

double distance2d(double ax, double ay, double bx, double by)
{
  return std::hypot(ax - bx, ay - by);
}

double clusterThreshold(double range, double base, double scale)
{
  return base + scale * range;
}

void addFloatField(sensor_msgs::msg::PointCloud2 & cloud, const std::string & name, uint32_t offset)
{
  sensor_msgs::msg::PointField field;
  field.name = name;
  field.offset = offset;
  field.datatype = sensor_msgs::msg::PointField::FLOAT32;
  field.count = 1;
  cloud.fields.push_back(field);
}

void writeFloat(std::vector<uint8_t> & data, size_t offset, float value)
{
  const auto * ptr = reinterpret_cast<const uint8_t *>(&value);
  std::copy(ptr, ptr + sizeof(float), data.begin() + static_cast<std::ptrdiff_t>(offset));
}

}  // namespace

class ConeDetectorNode : public rclcpp::Node
{
public:
  ConeDetectorNode()
  : Node("cone_detector_node")
  {
    scan_topic_ = declare_parameter<std::string>("scan_topic", "/scan");
    cone_points_topic_ = declare_parameter<std::string>("cone_points_topic", "/cones/points");
    cone_poses_topic_ = declare_parameter<std::string>("cone_poses_topic", "/cones/poses");
    marker_topic_ = declare_parameter<std::string>("marker_topic", "/cones/markers");
    output_frame_ = declare_parameter<std::string>("output_frame", "laser_link");

    min_range_ = declare_parameter<double>("min_range", 0.15);
    max_range_ = declare_parameter<double>("max_range", 2.8);
    cluster_base_threshold_ = declare_parameter<double>("cluster_base_threshold", 0.035);
    cluster_range_scale_ = declare_parameter<double>("cluster_range_scale", 0.03);
    min_cluster_points_ = declare_parameter<int>("min_cluster_points", 3);
    max_cluster_points_ = declare_parameter<int>("max_cluster_points", 40);
    min_cluster_width_ = declare_parameter<double>("min_cluster_width", 0.04);
    max_cluster_width_ = declare_parameter<double>("max_cluster_width", 0.35);
    min_radial_thickness_ = declare_parameter<double>("min_radial_thickness", 0.0);
    max_radial_thickness_ = declare_parameter<double>("max_radial_thickness", 0.18);
    max_cluster_index_gap_ = declare_parameter<int>("max_cluster_index_gap", 1);
    sparse_cluster_max_points_ = declare_parameter<int>("sparse_cluster_max_points", 2);
    sparse_min_cluster_width_ = declare_parameter<double>("sparse_min_cluster_width", 0.01);
    sparse_min_track_hits_ = declare_parameter<int>("sparse_min_track_hits", 3);
    sparse_isolation_index_gap_ = declare_parameter<int>("sparse_isolation_index_gap", 5);
    sparse_isolation_distance_ = declare_parameter<double>("sparse_isolation_distance", 0.12);
    sparse_line_guard_min_points_ = declare_parameter<int>("sparse_line_guard_min_points", 4);
    sparse_line_guard_min_length_ = declare_parameter<double>("sparse_line_guard_min_length", 0.12);
    sparse_line_guard_distance_ = declare_parameter<double>("sparse_line_guard_distance", 0.08);
    sparse_line_guard_extension_ = declare_parameter<double>("sparse_line_guard_extension", 0.25);
    sparse_line_support_radius_ = declare_parameter<double>("sparse_line_support_radius", 0.80);
    sparse_line_support_distance_ = declare_parameter<double>("sparse_line_support_distance", 0.035);
    sparse_line_support_min_offset_ =
      declare_parameter<double>("sparse_line_support_min_offset", 0.10);
    sparse_line_support_min_points_ =
      declare_parameter<int>("sparse_line_support_min_points", 4);
    enable_map_free_space_filter_ =
      declare_parameter<bool>("enable_map_free_space_filter", false);
    map_topic_ = declare_parameter<std::string>("map_topic", "/map");
    map_occupied_threshold_ = declare_parameter<int>("map_occupied_threshold", 65);
    map_filter_tf_timeout_ = declare_parameter<double>("map_filter_tf_timeout", 0.03);
    map_filter_fail_open_ = declare_parameter<bool>("map_filter_fail_open", true);
    map_reject_unknown_ = declare_parameter<bool>("map_reject_unknown", false);
    map_reject_occupied_ = declare_parameter<bool>("map_reject_occupied", false);
    map_bounds_margin_ = declare_parameter<double>("map_bounds_margin", 0.05);
    enable_pca_filter_ = declare_parameter<bool>("enable_pca_filter", true);
    max_pca_ratio_ = declare_parameter<double>("max_pca_ratio", 8.0);
    // The lidar sees the cone's scan-height cross-section, not its maximum
    // ground-contact radius.  Keep this compensation separate from the
    // 0.13 m lethal disk written by ConeLayer.
    scan_surface_radius_ = declare_parameter<double>("scan_surface_radius", 0.045);
    range_center_compensation_ = declare_parameter<bool>("range_center_compensation", false);
    marker_diameter_ = declare_parameter<double>("marker_diameter", 0.08);
    enable_temporal_filter_ = declare_parameter<bool>("enable_temporal_filter", true);
    association_distance_ = declare_parameter<double>("association_distance", 0.25);
    track_timeout_ = declare_parameter<double>("track_timeout", 0.6);
    min_track_hits_ = declare_parameter<int>("min_track_hits", 2);
    publish_debug_markers_ = declare_parameter<bool>("publish_debug_markers", true);
    publish_rejected_markers_ = declare_parameter<bool>("publish_rejected_markers", true);
    max_debug_markers_ = declare_parameter<int>("max_debug_markers", 80);
    publish_empty_when_no_tracks_ = declare_parameter<bool>("publish_empty_when_no_tracks", true);

    max_cluster_index_gap_ = std::max(1, max_cluster_index_gap_);
    sparse_cluster_max_points_ = std::max(1, sparse_cluster_max_points_);
    sparse_min_cluster_width_ = std::max(0.0, sparse_min_cluster_width_);
    sparse_min_track_hits_ = std::max(min_track_hits_, sparse_min_track_hits_);
    sparse_isolation_index_gap_ = std::max(max_cluster_index_gap_, sparse_isolation_index_gap_);
    sparse_isolation_distance_ = std::max(0.0, sparse_isolation_distance_);
    sparse_line_guard_min_points_ = std::max(3, sparse_line_guard_min_points_);
    sparse_line_guard_min_length_ = std::max(0.0, sparse_line_guard_min_length_);
    sparse_line_guard_distance_ = std::max(0.0, sparse_line_guard_distance_);
    sparse_line_guard_extension_ = std::max(0.0, sparse_line_guard_extension_);
    sparse_line_support_radius_ = std::max(0.0, sparse_line_support_radius_);
    sparse_line_support_distance_ = std::max(0.0, sparse_line_support_distance_);
    sparse_line_support_min_offset_ = std::max(0.0, sparse_line_support_min_offset_);
    sparse_line_support_min_points_ = std::max(2, sparse_line_support_min_points_);
    map_occupied_threshold_ = std::clamp(map_occupied_threshold_, 1, 100);
    map_filter_tf_timeout_ = std::max(0.0, map_filter_tf_timeout_);
    map_bounds_margin_ = std::max(0.0, map_bounds_margin_);
    scan_surface_radius_ = std::max(0.0, scan_surface_radius_);
    marker_diameter_ = marker_diameter_ > 0.0 ? marker_diameter_ : 2.0 * scan_surface_radius_;
    max_debug_markers_ = std::max(0, max_debug_markers_);

    if (enable_map_free_space_filter_) {
      tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
      const auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
      map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        map_topic_, map_qos,
        [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) {latest_map_ = msg;});
    }

    rclcpp::SensorDataQoS sensor_qos;
    cone_points_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(cone_points_topic_, sensor_qos);
    cone_poses_pub_ = create_publisher<geometry_msgs::msg::PoseArray>(cone_poses_topic_, 10);
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic_, 10);
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, sensor_qos,
      std::bind(&ConeDetectorNode::scanCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(), "cone_detector_node: scan=%s, points=%s",
      scan_topic_.c_str(), cone_points_topic_.c_str());
  }

private:
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    const auto started = std::chrono::steady_clock::now();
    const auto points = scanToPoints(*msg);
    std::vector<ClusterDebug> debug_clusters;
    const auto clusters = clusterPoints(points, msg->ranges.size());
    const auto candidates = buildCandidates(clusters, msg->ranges.size(), debug_clusters);
    const auto tracked =
      enable_temporal_filter_ ? updateTracks(candidates, msg->header.stamp) : candidates;
    const auto stable = filterByMapFreeSpace(tracked, *msg);

    std_msgs::msg::Header header = msg->header;
    if (!output_frame_.empty()) {
      header.frame_id = output_frame_;
    }

    if (publish_empty_when_no_tracks_ || !stable.empty()) {
      publishPointCloud(header, stable);
      publishPoses(header, stable);
    }
    publishMarkers(header, stable, debug_clusters);
    const double processing_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - started).count();
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "cone_stats scan_stamp=%d.%09u cluster_count=%zu accepted_cone_count=%zu "
      "published_center_count=%zu processing_ms=%.2f",
      msg->header.stamp.sec, msg->header.stamp.nanosec, debug_clusters.size(),
      candidates.size(), stable.size(), processing_ms);
  }

  std::vector<ScanPoint> scanToPoints(const sensor_msgs::msg::LaserScan & scan) const
  {
    std::vector<ScanPoint> points;
    points.reserve(scan.ranges.size());
    for (size_t i = 0; i < scan.ranges.size(); ++i) {
      const float range = scan.ranges[i];
      if (!std::isfinite(range) || range < min_range_ || range > max_range_) {
        continue;
      }
      const double angle = scan.angle_min + static_cast<double>(i) * scan.angle_increment;
      points.push_back(
        {static_cast<int>(i), range * std::cos(angle), range * std::sin(angle), range});
    }
    return points;
  }

  std::vector<std::vector<ScanPoint>> clusterPoints(
    const std::vector<ScanPoint> & points, size_t scan_size) const
  {
    std::vector<std::vector<ScanPoint>> clusters;
    if (points.empty()) {
      return clusters;
    }

    std::vector<ScanPoint> current;
    current.push_back(points.front());
    for (size_t i = 1; i < points.size(); ++i) {
      const auto & prev = points[i - 1];
      const auto & point = points[i];
      const double reference_range = std::min(prev.range, point.range);
      const double gap = distance2d(prev.x, prev.y, point.x, point.y);
      const bool contiguous = (point.index - prev.index) <= max_cluster_index_gap_;
      if (contiguous &&
          gap <= clusterThreshold(reference_range, cluster_base_threshold_, cluster_range_scale_)) {
        current.push_back(point);
      } else {
        clusters.push_back(current);
        current.clear();
        current.push_back(point);
      }
    }
    clusters.push_back(current);
    mergeWraparoundCluster(clusters, scan_size);
    return clusters;
  }

  void mergeWraparoundCluster(
    std::vector<std::vector<ScanPoint>> & clusters, size_t scan_size) const
  {
    if (clusters.size() < 2 || scan_size == 0 || clusters.front().empty() || clusters.back().empty()) {
      return;
    }

    const auto & first = clusters.front().front();
    const auto & last = clusters.back().back();
    const int wrapped_index_gap =
      first.index + static_cast<int>(scan_size) - last.index;
    const double reference_range = std::min(first.range, last.range);
    const double gap = distance2d(first.x, first.y, last.x, last.y);
    if (wrapped_index_gap > max_cluster_index_gap_ + 1 ||
        gap > clusterThreshold(reference_range, cluster_base_threshold_, cluster_range_scale_)) {
      return;
    }

    std::vector<ScanPoint> merged = clusters.back();
    merged.insert(merged.end(), clusters.front().begin(), clusters.front().end());
    clusters.front() = std::move(merged);
    clusters.pop_back();
  }

  bool sparseClusterIsIsolated(
    size_t cluster_index,
    const std::vector<std::vector<ScanPoint>> & clusters,
    size_t scan_size) const
  {
    if (sparse_isolation_distance_ <= 0.0 || scan_size == 0U) {
      return true;
    }
    const auto & cluster = clusters[cluster_index];
    for (size_t other_index = 0; other_index < clusters.size(); ++other_index) {
      if (other_index == cluster_index) {
        continue;
      }
      for (const auto & point : cluster) {
        for (const auto & other : clusters[other_index]) {
          const int direct_gap = std::abs(point.index - other.index);
          const int index_gap = std::min(
            direct_gap, static_cast<int>(scan_size) - direct_gap);
          if (index_gap <= sparse_isolation_index_gap_ &&
            distance2d(point.x, point.y, other.x, other.y) <= sparse_isolation_distance_)
          {
            return false;
          }
        }
      }
    }
    return true;
  }

  bool sparseClusterNearLinearSurface(
    size_t cluster_index,
    const std::vector<std::vector<ScanPoint>> & clusters) const
  {
    if (sparse_line_guard_distance_ <= 0.0) {
      return false;
    }
    const auto & sparse = clusters[cluster_index];
    double center_x = 0.0;
    double center_y = 0.0;
    for (const auto & point : sparse) {
      center_x += point.x;
      center_y += point.y;
    }
    center_x /= static_cast<double>(sparse.size());
    center_y /= static_cast<double>(sparse.size());

    for (size_t other_index = 0; other_index < clusters.size(); ++other_index) {
      if (other_index == cluster_index) {
        continue;
      }
      const auto & line = clusters[other_index];
      if (static_cast<int>(line.size()) < sparse_line_guard_min_points_) {
        continue;
      }
      const auto & start = line.front();
      const auto & end = line.back();
      const double vx = end.x - start.x;
      const double vy = end.y - start.y;
      const double length_squared = vx * vx + vy * vy;
      if (length_squared < sparse_line_guard_min_length_ * sparse_line_guard_min_length_) {
        continue;
      }
      const double length = std::sqrt(length_squared);
      const double projection =
        ((center_x - start.x) * vx + (center_y - start.y) * vy) / length_squared;
      const double extension_fraction = sparse_line_guard_extension_ / length;
      if (projection < -extension_fraction || projection > 1.0 + extension_fraction) {
        continue;
      }
      const double projected_x = start.x + projection * vx;
      const double projected_y = start.y + projection * vy;
      if (distance2d(center_x, center_y, projected_x, projected_y) <=
        sparse_line_guard_distance_)
      {
        return true;
      }
    }
    return false;
  }

  bool sparseClusterHasCollinearSupport(
    size_t cluster_index,
    const std::vector<std::vector<ScanPoint>> & clusters) const
  {
    if (sparse_line_support_radius_ <= 0.0 || sparse_line_support_distance_ <= 0.0) {
      return false;
    }
    const auto & sparse = clusters[cluster_index];
    double center_x = 0.0;
    double center_y = 0.0;
    for (const auto & point : sparse) {
      center_x += point.x;
      center_y += point.y;
    }
    center_x /= static_cast<double>(sparse.size());
    center_y /= static_cast<double>(sparse.size());

    const double min_offset_squared =
      sparse_line_support_min_offset_ * sparse_line_support_min_offset_;
    const double radius_squared = sparse_line_support_radius_ * sparse_line_support_radius_;
    for (size_t anchor_cluster = 0; anchor_cluster < clusters.size(); ++anchor_cluster) {
      if (anchor_cluster == cluster_index) {
        continue;
      }
      for (const auto & anchor : clusters[anchor_cluster]) {
        const double axis_x = anchor.x - center_x;
        const double axis_y = anchor.y - center_y;
        const double axis_length_squared = axis_x * axis_x + axis_y * axis_y;
        if (axis_length_squared < min_offset_squared || axis_length_squared > radius_squared) {
          continue;
        }
        const double inverse_length = 1.0 / std::sqrt(axis_length_squared);
        const double unit_x = axis_x * inverse_length;
        const double unit_y = axis_y * inverse_length;
        int support = 0;
        for (size_t support_cluster = 0; support_cluster < clusters.size(); ++support_cluster) {
          if (support_cluster == cluster_index) {
            continue;
          }
          for (const auto & point : clusters[support_cluster]) {
            const double dx = point.x - center_x;
            const double dy = point.y - center_y;
            const double projection = dx * unit_x + dy * unit_y;
            if (std::fabs(projection) < sparse_line_support_min_offset_ ||
              std::fabs(projection) > sparse_line_support_radius_)
            {
              continue;
            }
            const double perpendicular = std::fabs(dx * unit_y - dy * unit_x);
            if (perpendicular <= sparse_line_support_distance_ &&
              ++support >= sparse_line_support_min_points_)
            {
              return true;
            }
          }
        }
      }
    }
    return false;
  }

  std::vector<ConeCandidate> buildCandidates(
    const std::vector<std::vector<ScanPoint>> & clusters,
    size_t scan_size,
    std::vector<ClusterDebug> & debug_clusters) const
  {
    std::vector<ConeCandidate> candidates;
    for (size_t cluster_index = 0; cluster_index < clusters.size(); ++cluster_index) {
      const auto & cluster = clusters[cluster_index];
      const int count = static_cast<int>(cluster.size());
      const bool sparse_cluster = count <= sparse_cluster_max_points_;
      ConeCandidate candidate;
      candidate.count = count;
      double min_x = std::numeric_limits<double>::max();
      double max_x = std::numeric_limits<double>::lowest();
      double min_y = std::numeric_limits<double>::max();
      double max_y = std::numeric_limits<double>::lowest();
      double min_range = std::numeric_limits<double>::max();
      double max_range = std::numeric_limits<double>::lowest();

      for (const auto & point : cluster) {
        candidate.x += point.x;
        candidate.y += point.y;
        min_x = std::min(min_x, point.x);
        max_x = std::max(max_x, point.x);
        min_y = std::min(min_y, point.y);
        max_y = std::max(max_y, point.y);
        min_range = std::min(min_range, point.range);
        max_range = std::max(max_range, point.range);
      }
      candidate.x /= count;
      candidate.y /= count;
      candidate.range = std::hypot(candidate.x, candidate.y);
      candidate.width = std::hypot(max_x - min_x, max_y - min_y);
      candidate.radial_thickness = max_range - min_range;
      candidate.pca_ratio = pcaRatio(cluster, candidate.x, candidate.y);

      ClusterDebug debug;
      debug.x = candidate.x;
      debug.y = candidate.y;
      debug.range = candidate.range;
      debug.width = candidate.width;
      debug.radial_thickness = candidate.radial_thickness;
      debug.pca_ratio = candidate.pca_ratio;
      debug.count = candidate.count;

      if (count < min_cluster_points_) {
        debug.type = ClusterClass::noise;
        debug_clusters.push_back(debug);
        continue;
      }
      if (sparse_cluster &&
        (!sparseClusterIsIsolated(cluster_index, clusters, scan_size) ||
        sparseClusterNearLinearSurface(cluster_index, clusters) ||
        sparseClusterHasCollinearSupport(cluster_index, clusters)))
      {
        debug.type = ClusterClass::non_cone;
        debug_clusters.push_back(debug);
        continue;
      }
      if (count > max_cluster_points_ || candidate.width > max_cluster_width_) {
        debug.type = ClusterClass::non_cone;
        debug_clusters.push_back(debug);
        continue;
      }
      const double required_min_width = sparse_cluster ?
        sparse_min_cluster_width_ : min_cluster_width_;
      if (candidate.width < required_min_width) {
        debug.type = ClusterClass::noise;
        debug_clusters.push_back(debug);
        continue;
      }
      if (candidate.radial_thickness < min_radial_thickness_ ||
          candidate.radial_thickness > max_radial_thickness_) {
        debug.type = candidate.radial_thickness > max_radial_thickness_ ?
          ClusterClass::non_cone : ClusterClass::noise;
        debug_clusters.push_back(debug);
        continue;
      }
      // PCA has no minor eigenvalue for exactly two points and therefore
      // reports infinity regardless of whether the returns are a real cone.
      // Sparse candidates rely on bounded size/radial geometry plus the
      // stronger temporal-hit gate in updateTracks(); 3+ points keep PCA.
      if (enable_pca_filter_ && !sparse_cluster &&
        candidate.pca_ratio > max_pca_ratio_)
      {
        debug.type = ClusterClass::non_cone;
        debug_clusters.push_back(debug);
        continue;
      }
      debug.type = ClusterClass::cone_candidate;
      debug_clusters.push_back(debug);
      applyConeRadiusCompensation(candidate);
      candidates.push_back(candidate);
    }
    return candidates;
  }

  void applyConeRadiusCompensation(ConeCandidate & candidate) const
  {
    if (!range_center_compensation_ || scan_surface_radius_ <= 0.0) {
      return;
    }
    const double range = std::hypot(candidate.x, candidate.y);
    if (range <= 1e-6) {
      return;
    }
    candidate.x += candidate.x / range * scan_surface_radius_;
    candidate.y += candidate.y / range * scan_surface_radius_;
    candidate.range = std::hypot(candidate.x, candidate.y);
  }

  double pcaRatio(const std::vector<ScanPoint> & cluster, double mean_x, double mean_y) const
  {
    if (cluster.size() < 2) {
      return 1.0;
    }

    double xx = 0.0;
    double xy = 0.0;
    double yy = 0.0;
    for (const auto & point : cluster) {
      const double dx = point.x - mean_x;
      const double dy = point.y - mean_y;
      xx += dx * dx;
      xy += dx * dy;
      yy += dy * dy;
    }
    xx /= static_cast<double>(cluster.size());
    xy /= static_cast<double>(cluster.size());
    yy /= static_cast<double>(cluster.size());

    const double trace = xx + yy;
    const double determinant = xx * yy - xy * xy;
    const double disc = std::max(0.0, trace * trace * 0.25 - determinant);
    const double lambda1 = trace * 0.5 + std::sqrt(disc);
    const double lambda2 = trace * 0.5 - std::sqrt(disc);
    if (lambda2 <= 1e-9) {
      return lambda1 > 1e-9 ? std::numeric_limits<double>::infinity() : 1.0;
    }
    return lambda1 / lambda2;
  }

  std::vector<ConeCandidate> filterByMapFreeSpace(
    const std::vector<ConeCandidate> & candidates,
    const sensor_msgs::msg::LaserScan & scan)
  {
    if (!enable_map_free_space_filter_ || !latest_map_ || candidates.empty()) {
      return candidates;
    }
    const auto & map = *latest_map_;
    if (map.info.resolution <= 0.0F || map.info.width == 0U || map.info.height == 0U ||
      map.data.size() != static_cast<size_t>(map.info.width) * map.info.height ||
      map.header.frame_id.empty() || scan.header.frame_id.empty())
    {
      return map_filter_fail_open_ ? candidates : std::vector<ConeCandidate>{};
    }

    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_->lookupTransform(
        map.header.frame_id, scan.header.frame_id, rclcpp::Time(scan.header.stamp),
        rclcpp::Duration::from_seconds(map_filter_tf_timeout_));
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "cone map filter waiting for %s <- %s TF: %s",
        map.header.frame_id.c_str(), scan.header.frame_id.c_str(), error.what());
      return map_filter_fail_open_ ? candidates : std::vector<ConeCandidate>{};
    }

    const auto & tq = transform.transform.rotation;
    const double tr00 = 1.0 - 2.0 * (tq.y * tq.y + tq.z * tq.z);
    const double tr01 = 2.0 * (tq.x * tq.y - tq.z * tq.w);
    const double tr10 = 2.0 * (tq.x * tq.y + tq.z * tq.w);
    const double tr11 = 1.0 - 2.0 * (tq.x * tq.x + tq.z * tq.z);
    const double tx = transform.transform.translation.x;
    const double ty = transform.transform.translation.y;

    const auto & oq = map.info.origin.orientation;
    const double or00 = 1.0 - 2.0 * (oq.y * oq.y + oq.z * oq.z);
    const double or01 = 2.0 * (oq.x * oq.y - oq.z * oq.w);
    const double or10 = 2.0 * (oq.x * oq.y + oq.z * oq.w);
    const double or11 = 1.0 - 2.0 * (oq.x * oq.x + oq.z * oq.z);
    const double resolution = static_cast<double>(map.info.resolution);

    std::vector<ConeCandidate> accepted;
    accepted.reserve(candidates.size());
    const int margin_cells = static_cast<int>(std::ceil(map_bounds_margin_ / resolution));
    for (const auto & candidate : candidates) {
      const double world_x = tr00 * candidate.x + tr01 * candidate.y + tx;
      const double world_y = tr10 * candidate.x + tr11 * candidate.y + ty;
      const double dx = world_x - map.info.origin.position.x;
      const double dy = world_y - map.info.origin.position.y;
      // Inverse origin rotation (R^T) maps world coordinates into grid axes.
      const double grid_x = (or00 * dx + or10 * dy) / resolution;
      const double grid_y = (or01 * dx + or11 * dy) / resolution;
      const int col = static_cast<int>(std::floor(grid_x));
      const int row = static_cast<int>(std::floor(grid_y));
      if (col < margin_cells || row < margin_cells ||
        col >= static_cast<int>(map.info.width) - margin_cells ||
        row >= static_cast<int>(map.info.height) - margin_cells)
      {
        continue;
      }
      const int8_t occupancy = map.data[
        static_cast<size_t>(row) * map.info.width + static_cast<size_t>(col)];
      if ((map_reject_unknown_ && occupancy < 0) ||
        (map_reject_occupied_ && occupancy >= map_occupied_threshold_))
      {
        continue;
      }
      accepted.push_back(candidate);
    }
    return accepted;
  }

  std::vector<ConeCandidate> updateTracks(
    const std::vector<ConeCandidate> & candidates,
    const rclcpp::Time & stamp)
  {
    std::vector<bool> matched_tracks(tracks_.size(), false);
    for (const auto & candidate : candidates) {
      int best_index = -1;
      double best_distance = association_distance_;
      for (size_t i = 0; i < tracks_.size(); ++i) {
        if (matched_tracks[i]) {
          continue;
        }
        const double distance = distance2d(candidate.x, candidate.y, tracks_[i].x, tracks_[i].y);
        if (distance < best_distance) {
          best_distance = distance;
          best_index = static_cast<int>(i);
        }
      }

      if (best_index >= 0) {
        auto & track = tracks_[static_cast<size_t>(best_index)];
        // Tracks are maintained in laser_link.  The vehicle moves between
        // scans, so smoothing x/y in this moving frame creates a position
        // lag and publishes an old position with the current scan stamp.
        // Keep temporal filtering for hit confirmation and association, but
        // publish the current frame's geometry immediately.
        track.x = candidate.x;
        track.y = candidate.y;
        track.range = candidate.range;
        track.width = candidate.width;
        track.radial_thickness = candidate.radial_thickness;
        track.pca_ratio = candidate.pca_ratio;
        track.point_count = candidate.count;
        track.hits += 1;
        track.last_seen = stamp;
        matched_tracks[static_cast<size_t>(best_index)] = true;
      } else {
        tracks_.push_back(
          {next_track_id_++, candidate.x, candidate.y, candidate.range, candidate.width,
            candidate.radial_thickness, candidate.pca_ratio, candidate.count, 1, stamp});
        matched_tracks.push_back(true);
      }
    }

    tracks_.erase(
      std::remove_if(
        tracks_.begin(), tracks_.end(),
        [&](const Track & track) {
          return (stamp - track.last_seen).seconds() > track_timeout_;
        }),
      tracks_.end());

    std::vector<ConeCandidate> stable;
    for (const auto & track : tracks_) {
      const int required_hits = track.point_count <= sparse_cluster_max_points_ ?
        sparse_min_track_hits_ : min_track_hits_;
      if (track.hits < required_hits) {
        continue;
      }
      stable.push_back(
        {track.x, track.y, track.range, track.width, track.radial_thickness, track.pca_ratio,
          track.point_count});
    }
    return stable;
  }

  void publishPointCloud(const std_msgs::msg::Header & header, const std::vector<ConeCandidate> & cones)
  {
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header = header;
    cloud.height = 1;
    cloud.width = static_cast<uint32_t>(cones.size());
    cloud.is_bigendian = false;
    cloud.is_dense = true;
    cloud.point_step = 16;
    cloud.row_step = cloud.point_step * cloud.width;
    addFloatField(cloud, "x", 0);
    addFloatField(cloud, "y", 4);
    addFloatField(cloud, "z", 8);
    addFloatField(cloud, "intensity", 12);
    cloud.data.assign(cloud.row_step, 0);

    for (size_t i = 0; i < cones.size(); ++i) {
      const size_t offset = i * cloud.point_step;
      writeFloat(cloud.data, offset + 0, static_cast<float>(cones[i].x));
      writeFloat(cloud.data, offset + 4, static_cast<float>(cones[i].y));
      writeFloat(cloud.data, offset + 8, 0.0F);
      writeFloat(cloud.data, offset + 12, static_cast<float>(cones[i].range));
    }
    cone_points_pub_->publish(cloud);
  }

  void publishPoses(const std_msgs::msg::Header & header, const std::vector<ConeCandidate> & cones)
  {
    geometry_msgs::msg::PoseArray poses;
    poses.header = header;
    for (const auto & cone : cones) {
      geometry_msgs::msg::Pose pose;
      pose.position.x = cone.x;
      pose.position.y = cone.y;
      pose.position.z = 0.0;
      pose.orientation.w = 1.0;
      poses.poses.push_back(pose);
    }
    cone_poses_pub_->publish(poses);
  }

  void publishMarkers(
    const std_msgs::msg::Header & header,
    const std::vector<ConeCandidate> & cones,
    const std::vector<ClusterDebug> & debug_clusters)
  {
    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker delete_old;
    delete_old.header = header;
    delete_old.ns = "cones";
    delete_old.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(delete_old);

    if (!publish_debug_markers_) {
      marker_pub_->publish(markers);
      return;
    }

    int marker_id = 0;
    if (publish_rejected_markers_) {
      for (const auto & cluster : debug_clusters) {
        if (marker_id >= max_debug_markers_) {
          break;
        }
        visualization_msgs::msg::Marker marker;
        marker.header = header;
        marker.ns = "cone_cluster_debug";
        marker.id = marker_id++;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = cluster.x;
        marker.pose.position.y = cluster.y;
        marker.pose.position.z = 0.04;
        marker.pose.orientation.w = 1.0;
        const double marker_size = std::clamp(cluster.width, 0.04, 0.18);
        marker.scale.x = marker_size;
        marker.scale.y = marker_size;
        marker.scale.z = 0.06;
        if (cluster.type == ClusterClass::cone_candidate) {
          marker.color.r = 0.10F;
          marker.color.g = 0.45F;
          marker.color.b = 1.00F;
          marker.color.a = 0.70F;
        } else if (cluster.type == ClusterClass::non_cone) {
          marker.color.r = 0.55F;
          marker.color.g = 0.55F;
          marker.color.b = 0.55F;
          marker.color.a = 0.45F;
        } else {
          marker.color.r = 1.00F;
          marker.color.g = 0.10F;
          marker.color.b = 0.10F;
          marker.color.a = 0.45F;
        }
        marker.lifetime.sec = 0;
        marker.lifetime.nanosec = 300000000;
        markers.markers.push_back(marker);
      }
    }

    for (const auto & cone : cones) {
      visualization_msgs::msg::Marker marker;
      marker.header = header;
      marker.ns = "cones";
      marker.id = marker_id++;
      marker.type = visualization_msgs::msg::Marker::CYLINDER;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.position.x = cone.x;
      marker.pose.position.y = cone.y;
      marker.pose.position.z = 0.10;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = marker_diameter_;
      marker.scale.y = marker_diameter_;
      marker.scale.z = 0.20;
      marker.color.r = 0.05F;
      marker.color.g = 0.90F;
      marker.color.b = 0.20F;
      marker.color.a = 0.9F;
      marker.lifetime.sec = 0;
      marker.lifetime.nanosec = 400000000;
      markers.markers.push_back(marker);
    }
    marker_pub_->publish(markers);
  }

  std::string scan_topic_;
  std::string cone_points_topic_;
  std::string cone_poses_topic_;
  std::string marker_topic_;
  std::string output_frame_;

  double min_range_{0.15};
  double max_range_{2.8};
  double cluster_base_threshold_{0.035};
  double cluster_range_scale_{0.03};
  int min_cluster_points_{3};
  int max_cluster_points_{40};
  double min_cluster_width_{0.04};
  double max_cluster_width_{0.35};
  double min_radial_thickness_{0.0};
  double max_radial_thickness_{0.18};
  int max_cluster_index_gap_{1};
  int sparse_cluster_max_points_{2};
  double sparse_min_cluster_width_{0.01};
  int sparse_min_track_hits_{3};
  int sparse_isolation_index_gap_{5};
  double sparse_isolation_distance_{0.12};
  int sparse_line_guard_min_points_{4};
  double sparse_line_guard_min_length_{0.12};
  double sparse_line_guard_distance_{0.08};
  double sparse_line_guard_extension_{0.25};
  double sparse_line_support_radius_{0.80};
  double sparse_line_support_distance_{0.035};
  double sparse_line_support_min_offset_{0.10};
  int sparse_line_support_min_points_{4};
  bool enable_map_free_space_filter_{false};
  std::string map_topic_{"/map"};
  int map_occupied_threshold_{65};
  double map_filter_tf_timeout_{0.03};
  bool map_filter_fail_open_{true};
  bool map_reject_unknown_{false};
  bool map_reject_occupied_{false};
  double map_bounds_margin_{0.05};
  bool enable_pca_filter_{true};
  double max_pca_ratio_{8.0};
  double scan_surface_radius_{0.045};
  bool range_center_compensation_{false};
  double marker_diameter_{0.08};
  bool enable_temporal_filter_{true};
  double association_distance_{0.25};
  double track_timeout_{0.30};
  int min_track_hits_{2};
  bool publish_debug_markers_{true};
  bool publish_rejected_markers_{true};
  int max_debug_markers_{80};
  bool publish_empty_when_no_tracks_{true};

  int next_track_id_{1};
  std::vector<Track> tracks_;
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr latest_map_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cone_points_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr cone_poses_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ConeDetectorNode>());
  rclcpp::shutdown();
  return 0;
}
