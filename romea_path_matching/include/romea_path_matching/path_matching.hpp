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
#include <functional>
#include <memory>
#include <string>
#include <vector>

// ros
#include "rclcpp/rclcpp.hpp"

// romea
#include "romea_core_path_matching/PathMatching.hpp"

// local
#include "path_matching_base.hpp"
#include "path_matching_display.hpp"

namespace romea
{
namespace ros2
{

class PathMatching : public PathMatchingBase
{
public:
  explicit PathMatching(const rclcpp::NodeOptions & options);

  CallbackReturn on_configure(const rclcpp_lifecycle::State &);

  CallbackReturn on_activate(const rclcpp_lifecycle::State &);

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &);

  void reset() override;

protected:
  void process_odom_(const Odometry & msg) override;

  void timer_callback_() override;

protected:
  PathMatchingDisplay display_;
  std::string path_frame_id_;
  bool autostart_;
  bool display_activated_;

  std::unique_ptr<core::PathMatching> path_matching_;
};

}  // namespace ros2
}  // namespace romea

#endif  // ROMEA_PATH_MATCHING__PATH_MATCHING_HPP_
