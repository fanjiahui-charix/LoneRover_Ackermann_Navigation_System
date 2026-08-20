#include "lidar_perception/cone_layer.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

PLUGINLIB_EXPORT_CLASS(lidar_perception::ConeLayer, nav2_costmap_2d::Layer)

namespace lidar_perception
{

void ConeLayer::onInitialize()
{
  global_frame_ = layered_costmap_->getGlobalFrameID();
  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("topic", rclcpp::ParameterValue(std::string("/cones/points")));
  declareParameter("enable_soft_cost", rclcpp::ParameterValue(true));
  declareParameter("effective_radius", rclcpp::ParameterValue(0.13));
  declareParameter("soft_cost_radius", rclcpp::ParameterValue(0.23));
  declareParameter("ttl_sec", rclcpp::ParameterValue(0.30));
  declareParameter("tf_tolerance_sec", rclcpp::ParameterValue(0.05));
  declareParameter("pending_tf_max_age_sec", rclcpp::ParameterValue(0.30));
  declareParameter("pending_queue_depth", rclcpp::ParameterValue(4));

  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("failed to lock lifecycle node for ConeLayer");
  }
  node->get_parameter(name_ + ".enabled", enabled_);
  node->get_parameter(name_ + ".topic", topic_);
  node->get_parameter(name_ + ".enable_soft_cost", soft_cost_enabled_);
  node->get_parameter(name_ + ".effective_radius", effective_radius_);
  node->get_parameter(name_ + ".soft_cost_radius", soft_cost_radius_);
  node->get_parameter(name_ + ".ttl_sec", ttl_sec_);
  node->get_parameter(name_ + ".tf_tolerance_sec", tf_tolerance_sec_);
  node->get_parameter(name_ + ".pending_tf_max_age_sec", pending_tf_max_age_sec_);
  node->get_parameter(name_ + ".pending_queue_depth", pending_queue_depth_);
  effective_radius_ = std::max(0.01, effective_radius_);
  soft_cost_radius_ = soft_cost_enabled_ ?
    std::max(effective_radius_, soft_cost_radius_) : effective_radius_;
  ttl_sec_ = std::max(0.05, ttl_sec_);
  tf_tolerance_sec_ = std::max(0.0, tf_tolerance_sec_);
  pending_tf_max_age_sec_ = std::max(tf_tolerance_sec_, pending_tf_max_age_sec_);
  pending_queue_depth_ = std::max(1, pending_queue_depth_);

  cloud_sub_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    topic_, rclcpp::SensorDataQoS().keep_last(1),
    std::bind(&ConeLayer::cloudCallback, this, std::placeholders::_1));
  current_ = true;
  RCLCPP_INFO(
    logger_, "ConeLayer topic=%s target_frame=%s hard_radius=%.3f "
    "soft=%s soft_radius=%.3f ttl=%.2f pending_tf_age=%.2f queue=%d",
    topic_.c_str(), global_frame_.c_str(), effective_radius_,
    soft_cost_enabled_ ? "true" : "false", soft_cost_radius_, ttl_sec_,
    pending_tf_max_age_sec_, pending_queue_depth_);
}

void ConeLayer::reset()
{
  std::lock_guard<std::mutex> lock(mutex_);
  pending_clouds_.clear();
  centers_.clear();
  have_cloud_ = false;
  last_received_ns_ = 0;
  have_previous_bounds_ = false;
  current_ = true;
}

bool ConeLayer::cloudFreshLocked(const rclcpp::Time & now) const
{
  return have_cloud_ &&
    last_received_ns_ != 0 &&
    (now.nanoseconds() - last_received_ns_) <=
    static_cast<int64_t>(ttl_sec_ * 1e9);
}

bool ConeLayer::transformCloud(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg,
  double timeout_sec, std::vector<Center> & transformed,
  std::string & error) const
{
  const std::size_t count = static_cast<std::size_t>(msg->width) * msg->height;
  transformed.clear();
  transformed.reserve(count);
  try {
    geometry_msgs::msg::TransformStamped transform;
    const bool needs_transform =
      !msg->header.frame_id.empty() && msg->header.frame_id != global_frame_;
    if (needs_transform) {
      transform = tf_->lookupTransform(
        global_frame_, msg->header.frame_id, rclcpp::Time(msg->header.stamp),
        rclcpp::Duration::from_seconds(timeout_sec));
    }
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");
    for (std::size_t i = 0; i < count; ++i, ++iter_x, ++iter_y, ++iter_z) {
      if (!std::isfinite(*iter_x) || !std::isfinite(*iter_y) || !std::isfinite(*iter_z)) {
        continue;
      }
      geometry_msgs::msg::PointStamped input;
      input.header = msg->header;
      input.point.x = *iter_x;
      input.point.y = *iter_y;
      input.point.z = *iter_z;
      geometry_msgs::msg::PointStamped output;
      if (!needs_transform) {
        output = input;
      } else {
        tf2::doTransform(input, output, transform);
      }
      transformed.push_back({output.point.x, output.point.y});
    }
  } catch (const std::exception & ex) {
    error = ex.what();
    return false;
  }
  error.clear();
  return true;
}

void ConeLayer::promotePendingCloud()
{
  std::deque<PendingCloud> candidates;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    candidates = pending_clouds_;
  }
  if (candidates.empty()) {
    return;
  }

  const auto now = clock_->now();
  std::string last_error;
  for (auto it = candidates.rbegin(); it != candidates.rend(); ++it) {
    const auto & pending = *it;
    const auto & msg = pending.msg;
    const rclcpp::Time stamp(msg->header.stamp);
    if ((now.nanoseconds() - pending.received_ns) >
      static_cast<int64_t>(pending_tf_max_age_sec_ * 1e9))
    {
      continue;
    }
    std::vector<Center> transformed;
    if (!transformCloud(msg, 0.0, transformed, last_error)) {
      continue;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    centers_ = std::move(transformed);
    last_received_ns_ = now.nanoseconds();
    have_cloud_ = true;
    current_ = true;
    // A newer exact-time cloud supersedes every older pending complete set.
    // Keep only messages newer than the promoted sample.
    while (!pending_clouds_.empty() &&
      rclcpp::Time(pending_clouds_.front().msg->header.stamp) <= stamp)
    {
      pending_clouds_.pop_front();
    }
    return;
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    while (!pending_clouds_.empty()) {
      const auto waited_ns = now.nanoseconds() - pending_clouds_.front().received_ns;
      if (waited_ns <= static_cast<int64_t>(pending_tf_max_age_sec_ * 1e9))
      {
        break;
      }
      pending_clouds_.pop_front();
    }
  }
  if (!last_error.empty()) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 2000,
      "ConeLayer waiting for exact-time TF; cloud retained in bounded queue: %s",
      last_error.c_str());
  }
}

void ConeLayer::cloudCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  if (!msg) {
    return;
  }
  const std::size_t count = static_cast<std::size_t>(msg->width) * msg->height;
  if (count == 0U) {
    std::lock_guard<std::mutex> lock(mutex_);
    pending_clouds_.clear();
    // A single missed detection must not blink a stationary cone out of the
    // costmap. Keep only already transformed map/odom centers; their normal
    // TTL clears them if no later exact-time observation arrives.
    current_ = true;
    return;
  }

  std::vector<Center> transformed;
  std::string error;
  if (!transformCloud(msg, tf_tolerance_sec_, transformed, error)) {
    std::lock_guard<std::mutex> lock(mutex_);
    pending_clouds_.push_back({msg, clock_->now().nanoseconds()});
    while (static_cast<int>(pending_clouds_.size()) > pending_queue_depth_) {
      pending_clouds_.pop_front();
    }
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 2000,
      "ConeLayer waiting for exact-time TF; cloud retained in bounded queue: %s",
      error.c_str());
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  // A successfully transformed non-empty message replaces the complete
  // current set. Empty/missed frames expire through the bounded TTL above.
  centers_ = std::move(transformed);
  pending_clouds_.clear();
  // TTL is measured from reception, not from a potentially stale/zero sensor
  // stamp. The stamp remains the TF lookup time above, but it must not keep a
  // zero-stamped or delayed cloud alive indefinitely.
  last_received_ns_ = clock_->now().nanoseconds();
  have_cloud_ = true;
  current_ = true;
}

void ConeLayer::updateBounds(
  double, double, double, double * min_x, double * min_y, double * max_x, double * max_y)
{
  // A full-stack CPU spike can make the cone scan arrive before its matching
  // odom/map TF. Re-check the newest bounded sample here instead of dropping
  // the complete obstacle set or applying an inaccurate latest transform.
  promotePendingCloud();
  const auto now = clock_->now();
  std::lock_guard<std::mutex> lock(mutex_);
  // Always include the previous footprint first. This clears moved, empty,
  // and expired cones when LayeredCostmap resets the master window.
  if (have_previous_bounds_) {
    *min_x = std::min(*min_x, previous_min_x_ - soft_cost_radius_);
    *min_y = std::min(*min_y, previous_min_y_ - soft_cost_radius_);
    *max_x = std::max(*max_x, previous_max_x_ + soft_cost_radius_);
    *max_y = std::max(*max_y, previous_max_y_ + soft_cost_radius_);
  }
  if (!cloudFreshLocked(now) || centers_.empty()) {
    centers_.clear();
    have_previous_bounds_ = false;
    return;
  }
  double current_min_x = std::numeric_limits<double>::max();
  double current_min_y = std::numeric_limits<double>::max();
  double current_max_x = std::numeric_limits<double>::lowest();
  double current_max_y = std::numeric_limits<double>::lowest();
  for (const auto & center : centers_) {
    current_min_x = std::min(current_min_x, center.x);
    current_min_y = std::min(current_min_y, center.y);
    current_max_x = std::max(current_max_x, center.x);
    current_max_y = std::max(current_max_y, center.y);
  }
  *min_x = std::min(*min_x, current_min_x - soft_cost_radius_);
  *min_y = std::min(*min_y, current_min_y - soft_cost_radius_);
  *max_x = std::max(*max_x, current_max_x + soft_cost_radius_);
  *max_y = std::max(*max_y, current_max_y + soft_cost_radius_);
  previous_min_x_ = current_min_x;
  previous_min_y_ = current_min_y;
  previous_max_x_ = current_max_x;
  previous_max_y_ = current_max_y;
  have_previous_bounds_ = true;
}

void ConeLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  std::vector<Center> centers;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!cloudFreshLocked(clock_->now())) {
      return;
    }
    centers = centers_;
  }
  const int radius_cells = static_cast<int>(
    std::ceil(soft_cost_radius_ / master_grid.getResolution()));
  const double soft_band_width = soft_cost_radius_ - effective_radius_;
  for (const auto & center : centers) {
    unsigned int center_x = 0;
    unsigned int center_y = 0;
    if (!master_grid.worldToMap(center.x, center.y, center_x, center_y)) {
      continue;
    }
    // Static fences and the occupied map boundary already own their cells.
    // A sparse lidar fragment on those surfaces must not grow a cone-only
    // soft-cost disk back into the driveable course.
    const unsigned char center_cost = master_grid.getCost(center_x, center_y);
    if (center_cost == nav2_costmap_2d::LETHAL_OBSTACLE ||
      center_cost == nav2_costmap_2d::NO_INFORMATION)
    {
      continue;
    }
    const int cx = static_cast<int>(center_x);
    const int cy = static_cast<int>(center_y);
    for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
      for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
        const int x = cx + dx;
        const int y = cy + dy;
        if (x < min_i || x >= max_i || y < min_j || y >= max_j ||
          x < 0 || y < 0 ||
          x >= static_cast<int>(master_grid.getSizeInCellsX()) ||
          y >= static_cast<int>(master_grid.getSizeInCellsY())) {
          continue;
        }
        double world_x = 0.0;
        double world_y = 0.0;
        master_grid.mapToWorld(
          static_cast<unsigned int>(x), static_cast<unsigned int>(y), world_x, world_y);
        const double distance = std::hypot(world_x - center.x, world_y - center.y);
        unsigned char cone_cost = 0U;
        if (distance <= effective_radius_) {
          cone_cost = nav2_costmap_2d::LETHAL_OBSTACLE;
        } else if (soft_cost_enabled_ && distance <= soft_cost_radius_ &&
          soft_band_width > 1.0e-9) {
          // A cone-only gradient: near the physical disk it is almost as
          // expensive as an inscribed obstacle, and it reaches free-space
          // cost at the configured outer radius. Never lower a cost written
          // by StaticLayer, keepout, or another obstacle layer.
          const double fraction =
            (soft_cost_radius_ - distance) / soft_band_width;
          // Keep the entire soft band strictly below INSCRIBED and LETHAL;
          // the band is a preference signal, never a second hard obstacle.
          constexpr long kMaxSoftCost =
            static_cast<long>(nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) - 1L;
          cone_cost = static_cast<unsigned char>(std::clamp(
              std::lround(fraction *
              static_cast<double>(kMaxSoftCost)), 1L, kMaxSoftCost));
        }
        if (cone_cost > 0U) {
          auto & cell = master_grid.getCharMap()[
            master_grid.getIndex(static_cast<unsigned int>(x), static_cast<unsigned int>(y))];
          cell = std::max(cell, cone_cost);
        }
      }
    }
  }
}

}  // namespace lidar_perception
