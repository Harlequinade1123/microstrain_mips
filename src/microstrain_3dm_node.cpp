#include "microstrain_3dm.h"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<microstrain::Microstrain>();
  node->run();

  rclcpp::shutdown();
  return 0;
}