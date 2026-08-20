#ifndef LIDAR_PERCEPTION__CONE_LAYER_HPP_
#define LIDAR_PERCEPTION__CONE_LAYER_HPP_

#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <vector>

#include "nav2_costmap_2d/layer.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace lidar_perception
{

/** Dynamic semantic cone layer. The input message is a complete current set. */
class ConeLayer : public nav2_costmap_2d::Layer
{
public:
  ConeLayer() = default;
  ~ConeLayer() override = default;

  void onInitialize() override;
  void reset() override;
  bool isClearable() override {return true;}
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y) override;
  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;

private:
  struct Center
  {
    double x{0.0};
    double y{0.0};
  };

  struct PendingCloud
  {
    sensor_msgs::msg::PointCloud2::ConstSharedPtr msg;
    int64_t received_ns{0};
  };

  void cloudCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
  bool transformCloud(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg,
    double timeout_sec, std::vector<Center> & transformed,
    std::string & error) const;
  void promotePendingCloud();
  bool cloudFreshLocked(const rclcpp::Time & now) const;

  std::string global_frame_;
  std::string topic_;
  bool soft_cost_enabled_{true};
  double effective_radius_{0.13};
  // Outer radius of the cone-only soft cost band.  The hard physical disk is
  // effective_radius_; this must remain larger than it.
  double soft_cost_radius_{0.23};
  double ttl_sec_{0.30};
  double tf_tolerance_sec_{0.05};
  double pending_tf_max_age_sec_{0.30};
  int pending_queue_depth_{4};

  std::mutex mutex_;
  std::deque<PendingCloud> pending_clouds_;
  std::vector<Center> centers_;
  int64_t last_received_ns_{0};
  bool have_cloud_{false};
  double previous_min_x_{0.0};
  double previous_min_y_{0.0};
  double previous_max_x_{0.0};
  double previous_max_y_{0.0};
  bool have_previous_bounds_{false};

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
};

}  // namespace lidar_perception

#endif  // LIDAR_PERCEPTION__CONE_LAYER_HPP_
