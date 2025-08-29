
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "mvp_c2_traffic_manager.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MvpC2TrafficManager>();

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}