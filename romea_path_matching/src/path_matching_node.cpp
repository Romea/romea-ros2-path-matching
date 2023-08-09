#include <rclcpp/rclcpp.hpp>

#include "romea_path_matching/path_matching_base.hpp"

int main(int argc, char ** argv)
{
  auto args = rclcpp::init_and_remove_ros_arguments(argc, argv);

  rclcpp::NodeOptions options;
  options.arguments(args);

  rclcpp::shutdown();
  return 0;
}
