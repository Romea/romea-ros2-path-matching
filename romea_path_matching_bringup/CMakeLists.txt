cmake_minimum_required(VERSION 3.8)
project(romea_path_matching_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
