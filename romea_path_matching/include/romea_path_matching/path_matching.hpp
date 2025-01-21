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

#ifndef ROMEA_PATH_MATCHING__PATH_MATCHING_HPP_
#define ROMEA_PATH_MATCHING__PATH_MATCHING_HPP_

// std
#include <memory>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <string>

// ros
#include <rclcpp/time.hpp>

// romea
#include <romea_core_path_matching/PathMatching.hpp>
#include <romea_path_msgs/msg/path_annotations.hpp>

// local
#include "path_matching_base.hpp"
#include "path_matching_display.hpp"

namespace romea::ros2
{

class PathMatching : public PathMatchingBase
{
public:
  using PathAnnotations = romea_path_msgs::msg::PathAnnotations;

public:
  explicit PathMatching(const rclcpp::NodeOptions & options);

  CallbackReturn on_configure(const rclcpp_lifecycle::State &);

  CallbackReturn on_activate(const rclcpp_lifecycle::State &);

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &);

  void reset() override;

private:
  void process_odom_(const Odometry & msg) override;

  void timer_callback_() override;

  void publishNearAnnotations(const core::PathMatchedPoint2D & point, const rclcpp::Time & stamp);

private:
  PathMatchingDisplay display_;
  std::string path_frame_id_;
  bool display_activated_;

  std::unique_ptr<core::PathMatching> path_matching_;

  rclcpp_lifecycle::LifecyclePublisher<PathAnnotations>::SharedPtr annotations_pub_;
  double annotation_dist_max_;
  double annotation_dist_min_;
};

}  // namespace romea::ros2

#endif  // ROMEA_PATH_MATCHING__PATH_MATCHING_HPP_
