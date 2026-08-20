#include "lidar_perception/gate_side_soft_layer.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "nav2_costmap_2d/cost_values.hpp"
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(lidar_perception::GateSideSoftLayer, nav2_costmap_2d::Layer)

namespace lidar_perception
{

void GateSideSoftLayer::onInitialize()
{
  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("left_mesh_inner_x", rclcpp::ParameterValue(2.00));
  declareParameter("right_mesh_inner_x", rclcpp::ParameterValue(3.00));
  declareParameter("y_min", rclcpp::ParameterValue(2.00));
  declareParameter("y_max", rclcpp::ParameterValue(2.90));
  declareParameter("band_width", rclcpp::ParameterValue(0.10));
  declareParameter("maximum_cost", rclcpp::ParameterValue(220));

  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("failed to lock lifecycle node for GateSideSoftLayer");
  }
  node->get_parameter(name_ + ".enabled", enabled_);
  node->get_parameter(name_ + ".left_mesh_inner_x", left_mesh_inner_x_);
  node->get_parameter(name_ + ".right_mesh_inner_x", right_mesh_inner_x_);
  node->get_parameter(name_ + ".y_min", y_min_);
  node->get_parameter(name_ + ".y_max", y_max_);
  node->get_parameter(name_ + ".band_width", band_width_);
  node->get_parameter(name_ + ".maximum_cost", maximum_cost_);
  if (!std::isfinite(left_mesh_inner_x_) || !std::isfinite(right_mesh_inner_x_) ||
    !std::isfinite(y_min_) || !std::isfinite(y_max_) ||
    !std::isfinite(band_width_) || right_mesh_inner_x_ <= left_mesh_inner_x_ ||
    y_max_ <= y_min_ || band_width_ <= 0.0 ||
    maximum_cost_ < 1 ||
    maximum_cost_ >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
  {
    throw std::runtime_error("invalid GateSideSoftLayer geometry/cost parameters");
  }
  current_ = true;
  RCLCPP_INFO(
    logger_, "GateSideSoftLayer x=[%.2f,%.2f]/[%.2f,%.2f] y=[%.2f,%.2f] max=%d",
    left_mesh_inner_x_, left_mesh_inner_x_ + band_width_,
    right_mesh_inner_x_ - band_width_, right_mesh_inner_x_,
    y_min_, y_max_, maximum_cost_);
}

void GateSideSoftLayer::reset()
{
  current_ = true;
}

void GateSideSoftLayer::updateBounds(
  double, double, double, double * min_x, double * min_y, double * max_x, double * max_y)
{
  if (!enabled_) {
    return;
  }
  *min_x = std::min(*min_x, left_mesh_inner_x_);
  *max_x = std::max(*max_x, right_mesh_inner_x_);
  *min_y = std::min(*min_y, y_min_);
  *max_y = std::max(*max_y, y_max_);
}

void GateSideSoftLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_) {
    return;
  }
  const int bounded_min_i = std::max(0, min_i);
  const int bounded_min_j = std::max(0, min_j);
  const int bounded_max_i = std::min(
    max_i, static_cast<int>(master_grid.getSizeInCellsX()));
  const int bounded_max_j = std::min(
    max_j, static_cast<int>(master_grid.getSizeInCellsY()));
  for (int my = bounded_min_j; my < bounded_max_j; ++my) {
    for (int mx = bounded_min_i; mx < bounded_max_i; ++mx) {
      double world_x = 0.0;
      double world_y = 0.0;
      master_grid.mapToWorld(
        static_cast<unsigned int>(mx), static_cast<unsigned int>(my),
        world_x, world_y);
      if (world_y < y_min_ || world_y > y_max_) {
        continue;
      }
      double inward_distance = band_width_;
      if (world_x >= left_mesh_inner_x_ &&
        world_x <= left_mesh_inner_x_ + band_width_)
      {
        inward_distance = world_x - left_mesh_inner_x_;
      } else if (world_x <= right_mesh_inner_x_ &&
        world_x >= right_mesh_inner_x_ - band_width_)
      {
        inward_distance = right_mesh_inner_x_ - world_x;
      } else {
        continue;
      }
      const double fraction = std::clamp(
        (band_width_ - inward_distance) / band_width_, 0.0, 1.0);
      const auto soft_cost = static_cast<unsigned char>(std::clamp(
          std::lround(fraction * static_cast<double>(maximum_cost_)),
          1L, static_cast<long>(maximum_cost_)));
      auto & cell = master_grid.getCharMap()[master_grid.getIndex(
          static_cast<unsigned int>(mx), static_cast<unsigned int>(my))];
      cell = std::max(cell, soft_cost);
    }
  }
}

}  // namespace lidar_perception
