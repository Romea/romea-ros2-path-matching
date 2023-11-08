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

// std
#include <optional>
#include <memory>
#include <utility>
#include <string>

// romea
#include "romea_common_utils/conversions/pose_and_twist3d_conversions.hpp"
#include "romea_common_utils/conversions/twist2d_conversions.hpp"
#include "romea_common_utils/params/node_parameters.hpp"
#include "romea_core_common/geometry/PoseAndTwist3D.hpp"
#include "romea_core_path/PathFile.hpp"
#include "romea_core_path/PathMatching2D.hpp"
#include "romea_path_utils/path_matching_info_conversions.hpp"
#include "romea_path_matching/path_matching.hpp"

// #include "uturn_generator.hpp"
// #include <romea_path_msgs/PathAnnotations.h>


namespace romea
{

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
PathMatching::PathMatching(const rclcpp::NodeOptions & options)
: PathMatchingBase(options),
  // uturn_generator_(*this),
  path_(nullptr),
  matched_points_(),
  tracked_matched_point_index_(0),
  logger_(rclcpp::get_logger("path_matching"))
{
  rcl_interfaces::msg::ParameterDescriptor path_frame_descr;
  path_frame_descr.description = "The frame is used to publish a transform from the world frame";
  declare_parameter("path_frame_id", "path", std::move(path_frame_descr));

  rcl_interfaces::msg::ParameterDescriptor path_descr;
  path_descr.description = "Filename of the path to follow";
  declare_parameter("path", rclcpp::PARAMETER_STRING, std::move(path_descr));

  rcl_interfaces::msg::ParameterDescriptor autostart_descr;
  autostart_descr.description = "Automatic configuration and activation when the node is started";
  declare_parameter("autostart", false, std::move(autostart_descr));
  get_parameter("autostart", autostart_);

  rcl_interfaces::msg::ParameterDescriptor display_descr;
  display_descr.description = "Enable the publication of rviz markers";
  declare_parameter("display", true, std::move(display_descr));

  if (autostart_) {
    transition_call_ = deferred_call(*this, [this] {configure();});
  }
}

//-----------------------------------------------------------------------------
PathMatching::CallbackReturn PathMatching::on_configure(const rclcpp_lifecycle::State & state)
try
{
  PathMatchingBase::on_configure(state);

  path_frame_id_ = romea::get_parameter<std::string>(shared_from_this(), "path_frame_id");
  std::string path = romea::get_parameter<std::string>(shared_from_this(), "path");
  display_activated_ = romea::get_parameter<bool>(shared_from_this(), "display");

  // annotation_dist_max_ = get_parameter_or(node_, "annotation_dist_max", 5.);
  // annotation_dist_min_ = get_parameter_or(node_, "annotation_dist_min", -0.5);

  display_.init(shared_from_this(), path_frame_id_);
  diagnostics_.init(shared_from_this());
  // comparator_.init();
  // uturn_generator_.init();
  // reset_sub_ = private_nh.subscribe<std_msgs::Bool>(
  //    "reset", 1, &PathMatching::resetCallback, this);
  // annotations_pub_ = private_nh.advertise<romea_path_msgs::PathAnnotations>("annotations", 1);

  if (autostart_) {
    transition_call_ = deferred_call(this, [this] {activate();});
  }

  loadPath(path);

  RCLCPP_INFO(logger_, "configured");
  return CallbackReturn::SUCCESS;
} catch (const std::runtime_error & e) {
  RCLCPP_ERROR_STREAM(logger_, "configuration failed: " << e.what());
  return CallbackReturn::FAILURE;
}

//-----------------------------------------------------------------------------
PathMatching::CallbackReturn PathMatching::on_activate(const rclcpp_lifecycle::State & state)
{
  CallbackReturn result = PathMatchingBase::on_activate(state);
  RCLCPP_INFO(logger_, "activated");
  return result;
}

//-----------------------------------------------------------------------------
PathMatching::CallbackReturn PathMatching::on_deactivate(const rclcpp_lifecycle::State & state)
{
  CallbackReturn result = PathMatchingBase::on_deactivate(state);
  RCLCPP_INFO(logger_, "deactivated");
  return result;
}

//-----------------------------------------------------------------------------
void PathMatching::loadPath(const std::string & filename)
{
  PathFile path_file(filename);
  path_ = std::make_unique<Path2D>(
    path_file.getWayPoints(), interpolation_window_length_, path_file.getAnnotations());
  display_.load_path(*path_);
  diagnostics_.update_path_status(filename, true);
}

void PathMatching::updateDisplay()
{
  display_.load_path(*path_);
}

//-----------------------------------------------------------------------------
void PathMatching::reset()
{
  display_.clear();
  matched_points_.clear();

  if (path_) {
    display_.load_path(*path_);
  }
}

//-----------------------------------------------------------------------------
void PathMatching::processOdom_(const Odometry & msg)
{
  if (!is_active_) {return;}

  PoseAndTwist3D enuPoseAndBodyTwist3D;
  to_romea(msg.pose, enuPoseAndBodyTwist3D.pose);
  to_romea(msg.twist, enuPoseAndBodyTwist3D.twist);
  diagnostics_.update_odom_rate(to_romea_duration(msg.header.stamp));

  vehicle_pose_ = toPose2D(enuPoseAndBodyTwist3D.pose);
  auto vehicle_twist = toTwist2D(enuPoseAndBodyTwist3D.twist);
  bool matching_status = tryToMatchOnPath_(vehicle_pose_, vehicle_twist);

  if (matching_status) {
    match_pub_->publish(
      to_ros_msg(
        msg.header.stamp, matched_points_, tracked_matched_point_index_, path_->getLength(),
        vehicle_twist));

    // const auto & matched_point = matched_points_[tracked_matched_point_index_];
    // publishNearAnnotations(matched_point, msg.header.stamp);
  }

  // force a diagnostic update when matching status changes
  if (matching_status != previous_matching_status_) {
    diagnostics_.publish();
  }

  if (display_activated_) {
    displayResults_(vehicle_pose_);
  }

  previous_matching_status_ = matching_status;
}

//-----------------------------------------------------------------------------
bool PathMatching::tryToMatchOnPath_(const Pose2D & vehicle_pose, const Twist2D & vehicle_twist)
{
  double vehicle_speed = vehicle_twist.linearSpeeds.x();

  if (matched_points_.empty()) {
    matched_points_ = match(
      *path_, vehicle_pose, vehicle_speed, prediction_time_horizon_, maximal_research_radius_);

  } else {
    matched_points_ = match(
      *path_, vehicle_pose, vehicle_speed, matched_points_[tracked_matched_point_index_], 2,
      prediction_time_horizon_, maximal_research_radius_);
  }

  if (!matched_points_.empty()) {
    tracked_matched_point_index_ = bestMatchedPointIndex(matched_points_, vehicle_speed);
  }

  diagnostics_.update_matching_status(!matched_points_.empty());
  return !matched_points_.empty();
}

//-----------------------------------------------------------------------------
void PathMatching::displayResults_(const Pose2D & /*vehicle_pose*/)
{
  if (!matched_points_.empty()) {
    const auto & section =
      path_->getSection(matched_points_[tracked_matched_point_index_].sectionIndex);
    const auto & curve = section.getCurve(matched_points_[tracked_matched_point_index_].curveIndex);
    display_.load_curve(curve);
  }

  display_.publish();
}

//-----------------------------------------------------------------------------
const PathSection2D * PathMatching::getCurrentSection() const
{
  if (matched_points_.size()) {
    size_t section_index = matched_points_[tracked_matched_point_index_].sectionIndex;
    return &path_->getSection(section_index);
  }
  return nullptr;
}

//-----------------------------------------------------------------------------
size_t PathMatching::getCurrentSectionIndex() const
{
  if (matched_points_.size()) {
    return matched_points_[tracked_matched_point_index_].sectionIndex;
  }
  return path_->size();
}

//-----------------------------------------------------------------------------
// void PathMatching::publishNearAnnotations(
//  const PathMatchedPoint2D & point,
//  const ros::Time & stamp)
// {
//   romea_path_msgs::PathAnnotations msg;
//   msg.header.stamp = stamp;
//   msg.header.frame_id = path_frame_id_;
//
//   const auto & annotations = path_->getAnnotations();
//   // auto annotation_it = annotations.lower_bound(point.globalIndex);
//   auto annotation_it = begin(annotations);
//   double abscissa_max = point.frenetPose.curvilinearAbscissa + annotation_dist_max_;
//   double abscissa_min = point.frenetPose.curvilinearAbscissa + annotation_dist_min_;
//
//   while (annotation_it != end(annotations)) {
//     const auto & annotation = annotation_it->second;
//     if (annotation.abscissa < abscissa_max && annotation.abscissa > abscissa_min) {
//       auto & new_a = msg.annotations.emplace_back();
//       new_a.type = annotation.type;
//       new_a.value = annotation.value;
//       new_a.curvilinear_abcissa = annotation.abscissa;
//       new_a.curvilinear_distance = annotation.abscissa - point.frenetPose.curvilinearAbscissa;
//     }
//
//     ++annotation_it;
//   }
//
//   if (!annotations.empty()) {
//     annotations_pub_.publish(msg);
//   }
// }

}  // namespace romea

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(romea::PathMatching)
