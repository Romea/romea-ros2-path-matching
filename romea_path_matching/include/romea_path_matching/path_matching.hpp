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
#include "romea_common_utils/deferred_call.hpp"
#include "romea_core_common/geometry/Pose2D.hpp"
#include "romea_core_common/geometry/Twist2D.hpp"
#include "romea_core_path/Path2D.hpp"
#include "romea_core_path/PathMatchedPoint2D.hpp"
#include "romea_core_path/PathPosture2D.hpp"

#include "path_matching_base.hpp"
#include "path_matching_diagnostic.hpp"
#include "path_matching_display.hpp"
// #include "std_msgs/Bool.h"

namespace romea
{
namespace ros2
{

class PathMatching : public PathMatchingBase
{
private:
  using UpdateCb = std::function<void ()>;

public:
  explicit PathMatching(const rclcpp::NodeOptions & options);

  CallbackReturn on_configure(const rclcpp_lifecycle::State &);

  CallbackReturn on_activate(const rclcpp_lifecycle::State &);

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &);

  void loadPath(const std::string & filename);

  void updateDisplay();

  void reset() override;

  bool isMatching() const {return matched_points_.size();}

  const core::PathSection2D * getCurrentSection() const;
  size_t getCurrentSectionIndex() const;

  const core::Path2D & getPath() const {return *path_;}
  core::Path2D & getPath() {return *path_;}

  const std::string & getFrame() const {return path_frame_id_;}
  const core::Pose2D & getVehiclePose() const {return vehicle_pose_;}

  // void publishNearAnnotations(const PathMatchedPoint2D & point, const ros::Time & stamp);

protected:
  void processOdom_(const Odometry & msg) override;

  bool tryToMatchOnPath_(const core::Pose2D & vehicle_pose, const core::Twist2D & vehicle_twist);

  void displayResults_(const core::Pose2D & vehicle_pose);

protected:
  PathMatchingDisplay display_;
  PathMatchingDiagnostic diagnostics_;
  std::string path_frame_id_;
  // UturnGenerator uturn_generator_;
  bool autostart_;
  bool display_activated_;

  UpdateCb update_cb_;
  std::unique_ptr<core::Path2D> path_;
  std::vector<core::PathMatchedPoint2D> matched_points_;
  size_t tracked_matched_point_index_;
  core::Pose2D vehicle_pose_;
  // ros::Publisher annotations_pub_;
  // double annotation_dist_max_;
  // double annotation_dist_min_;
  bool previous_matching_status_ = false;

  rclcpp::Logger logger_;
  std::unique_ptr<DeferredCall> transition_call_;
};

}  // namespace ros2
}  // namespace romea

#endif  // ROMEA_PATH_MATCHING__PATH_MATCHING_HPP_
