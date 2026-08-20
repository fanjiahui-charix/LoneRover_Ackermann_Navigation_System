#include <rclcpp/rclcpp.hpp>

#include "origincarpro_base/origincarpro_base.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<origincarpro_base::OriginCarProBase>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
