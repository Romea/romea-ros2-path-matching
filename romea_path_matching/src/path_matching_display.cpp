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

// romea
#include "romea_path_matching/path_matching_display.hpp"


namespace romea
{
namespace ros2
{

void PathMatchingDisplay::load_waypoints(const WayPoints & path_way_points)
{
  path_marker_.points.clear();

  for (const auto & section_way_points : path_way_points) {
    for (const auto & way_point : section_way_points) {
      add_waypoint_(way_point);
    }
  }
}

void PathMatchingDisplay::load_path(const core::Path2D & path)
{
  path_marker_.points.clear();

  for (auto const & section : path.getSections()) {
    auto const & x_list = section.getX();
    auto const & y_list = section.getY();

    for (std::size_t i = 0; i < section.size(); ++i) {
      auto & pt = path_marker_.points.emplace_back();
      pt.x = x_list[i];
      pt.y = y_list[i];
      pt.z = 0.1;
    }
  }
}

void PathMatchingDisplay::add_waypoint_(const core::PathWayPoint2D & way_point)
{
  auto & pt = path_marker_.points.emplace_back();
  pt.x = way_point.position.x();
  pt.y = way_point.position.y();
  pt.z = 0.1;
}

}  // namespace ros2
}  // namespace romea
