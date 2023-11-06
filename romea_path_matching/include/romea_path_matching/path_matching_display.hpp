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

#ifndef ROMEA_PATH_MATCHING__PATH_MATCHING_DISPLAY_HPP_
#define ROMEA_PATH_MATCHING__PATH_MATCHING_DISPLAY_HPP_

// std
#include <vector>

// romea
#include "romea_core_path/Path2D.hpp"
#include "path_matching_display_base.hpp"

namespace romea
{

class PathMatchingDisplay : public PathMatchingDisplayBase
{
public:
  using WayPoints = std::vector<std::vector<PathWayPoint2D>>;

public:
  void load_waypoints(const WayPoints & path_way_points);
  void load_path(const Path2D & path);

protected:
  void add_waypoint_(const PathWayPoint2D & point);
};

}  // namespace romea

#endif  // ROMEA_PATH_MATCHING__PATH_MATCHING_DISPLAY_HPP_
