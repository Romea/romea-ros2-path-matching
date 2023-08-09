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
find_package(nav_msgs REQUIRED)
find_package(romea_common REQUIRED)
find_package(romea_common_utils REQUIRED)
find_package(romea_path_msgs REQUIRED)
find_package(romea_path_utils REQUIRED)
find_package(romea_core_path REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(path_matching_node
  src/path_matching_node.cpp
  src/path_matching_base.cpp
  src/path_matching.cpp
  src/path_matching_tf.cpp
)

ament_target_dependencies(path_matching_node
  rclcpp
  nav_msgs
  romea_common
  romea_common_utils
  romea_path_msgs
  romea_path_utils
  romea_core_path
  tf2_ros
)

target_include_directories(path_matching_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

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

install(TARGETS
 path_matching_node
 DESTINATION lib/${PROJECT_NAME})

ament_package()
