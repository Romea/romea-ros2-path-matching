// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ROMEA_PATH_MATCHING__PATH_MATCHING_DISPLAY_BASE_HPP_
#define ROMEA_PATH_MATCHING__PATH_MATCHING_DISPLAY_BASE_HPP_

// std
#include <string>

// ros
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

// romea
#include "romea_core_path/PathSection2D.hpp"
#include "romea_core_path/PathWayPoint2D.hpp"
#include "romea_core_common/geometry/Pose2D.hpp"


namespace romea
{

class PathMatchingDisplayBase
{
public:
  using Marker = visualization_msgs::msg::Marker;
  using MarkerArray = visualization_msgs::msg::MarkerArray;

public:
  PathMatchingDisplayBase();

  virtual ~PathMatchingDisplayBase() = default;

  void init(rclcpp_lifecycle::LifecycleNode::SharedPtr node, const std::string & path_frame_id);

  void load_curve(const PathCurve2D & path_curve);

  void clear();

  void publish();

protected:
  void initMarkers(const std::string & path_frame_id);

protected:
  bool is_display_activated_ = false;

  rclcpp_lifecycle::LifecyclePublisher<MarkerArray>::SharedPtr marker_pub_;
  Marker path_marker_;
  Marker curve_marker_;
  Marker clear_marker_;
};

}  // namespace romea

#endif  // ROMEA_PATH_MATCHING__PATH_MATCHING_DISPLAY_BASE_HPP_
