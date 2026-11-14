#include <memory>

#include "hobot_nav/competition_supervisor.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<hobot_nav::CompetitionSupervisor>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
