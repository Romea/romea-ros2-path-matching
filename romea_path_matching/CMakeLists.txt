cmake_minimum_required(VERSION 3.8)
project(romea_path_matching)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(romea_common REQUIRED)
find_package(romea_common_utils REQUIRED)
find_package(romea_path_msgs REQUIRED)
find_package(romea_path_utils REQUIRED)
find_package(romea_core_path REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(diagnostic_updater REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/path_matching_base.cpp
  src/path_matching.cpp
  src/path_matching_display.cpp
  src/path_matching_display_base.cpp
  src/path_matching_diagnostic.cpp
  src/path_matching_diagnostic_base.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_components
  rclcpp_lifecycle
  nav_msgs
  romea_common
  romea_common_utils
  romea_path_msgs
  romea_path_utils
  romea_core_path
  tf2_ros
  visualization_msgs
  std_srvs
  diagnostic_updater
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "romea::ros2::PathMatching"
  EXECUTABLE path_matching_node
)

install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
