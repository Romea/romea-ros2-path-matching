#include <rclcpp/rclcpp.hpp>

#include "romea_path_matching/path_matching.hpp"

int main(int argc, char ** argv)
{
  auto args = rclcpp::init_and_remove_ros_arguments(argc, argv);
  rclcpp::NodeOptions options;
  options.arguments(args);

  auto path_matching = std::make_shared<romea::PathMatching>(std::move(options));
  rclcpp::spin(path_matching->get_node_base_interface());

  rclcpp::shutdown();
  return 0;
}
