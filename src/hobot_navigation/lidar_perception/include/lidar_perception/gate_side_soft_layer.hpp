#ifndef LIDAR_PERCEPTION__GATE_SIDE_SOFT_LAYER_HPP_
#define LIDAR_PERCEPTION__GATE_SIDE_SOFT_LAYER_HPP_

#include "nav2_costmap_2d/layer.hpp"
#include "rclcpp/rclcpp.hpp"

namespace lidar_perception
{

/** Global-only preference gradient immediately inside the two B-Gate meshes. */
class GateSideSoftLayer : public nav2_costmap_2d::Layer
{
public:
  GateSideSoftLayer() = default;
  ~GateSideSoftLayer() override = default;

  void onInitialize() override;
  void reset() override;
  bool isClearable() override {return false;}
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y) override;
  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;

private:
  double left_mesh_inner_x_{2.00};
  double right_mesh_inner_x_{3.00};
  double y_min_{2.00};
  double y_max_{2.90};
  double band_width_{0.10};
  int maximum_cost_{220};
};

}  // namespace lidar_perception

#endif  // LIDAR_PERCEPTION__GATE_SIDE_SOFT_LAYER_HPP_
