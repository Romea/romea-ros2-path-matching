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
namespace ros2
{

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
PathMatching::PathMatching(const rclcpp::NodeOptions & options)
: PathMatchingBase(options),
  path_matching_(nullptr)
{
  node_->register_on_configure(
    std::bind(&PathMatching::on_configure, this, std::placeholders::_1));
  node_->register_on_activate(
    std::bind(&PathMatching::on_activate, this, std::placeholders::_1));
  node_->register_on_deactivate(
    std::bind(&PathMatching::on_deactivate, this, std::placeholders::_1));

  rcl_interfaces::msg::ParameterDescriptor path_frame_descr;
  path_frame_descr.description = "Frame used to publish path messages";
  node_->declare_parameter("path_frame_id", "map", std::move(path_frame_descr));

  rcl_interfaces::msg::ParameterDescriptor path_descr;
  path_descr.description = "Filename of the path to follow";
  node_->declare_parameter("path", rclcpp::PARAMETER_STRING, std::move(path_descr));

  rcl_interfaces::msg::ParameterDescriptor autoconf_descr;
  autoconf_descr.description = "Automatic configuration when the node is created";
  node_->declare_parameter("autoconfigure", false, std::move(autoconf_descr));

  rcl_interfaces::msg::ParameterDescriptor autostart_descr;
  autostart_descr.description = "Automatically start the robot when the node is configured";
  node_->declare_parameter("autostart", false, std::move(autostart_descr));

  rcl_interfaces::msg::ParameterDescriptor display_descr;
  display_descr.description = "Enable the publication of rviz markers";
  node_->declare_parameter("display", true, std::move(display_descr));

  if (get_parameter<bool>(node_, "autoconfigure")) {
    auto state = node_->configure();
    if (get_parameter<bool>(node_, "autostart") && state.label() == "inactive") {
      node_->activate();
    }
  }

}

//-----------------------------------------------------------------------------
PathMatching::CallbackReturn PathMatching::on_configure(const rclcpp_lifecycle::State & state)
try
{
  PathMatchingBase::on_configure(state);

  path_frame_id_ = get_parameter<std::string>(node_, "path_frame_id");
  std::string path = get_parameter<std::string>(node_, "path");
  display_activated_ = get_parameter<bool>(node_, "display");

  // annotation_dist_max_ = get_parameter_or(node_, "annotation_dist_max", 5.);
  // annotation_dist_min_ = get_parameter_or(node_, "annotation_dist_min", -0.5);
  path_matching_ = std::make_unique<core::PathMatching>(
    path, maximal_research_radius_, interpolation_window_length_);

  display_.init(node_, path_frame_id_);
  display_.load_path(path_matching_->getPath());


  // comparator_.init();
  // uturn_generator_.init();
  // reset_sub_ = private_nh.subscribe<std_msgs::Bool>(
  //    "reset", 1, &PathMatching::resetCallback, this);
  // annotations_pub_ = private_nh.advertise<romea_path_msgs::PathAnnotations>("annotations", 1);

  // loadPath(path);

  auto callback = std::bind(&PathMatching::timer_callback_, this);
  timer_ = node_->create_wall_timer(std::chrono::milliseconds(100), callback);

  RCLCPP_INFO(node_->get_logger(), "configured");
  return CallbackReturn::SUCCESS;
} catch (const std::runtime_error & e) {
  RCLCPP_ERROR_STREAM(node_->get_logger(), "configuration failed: " << e.what());
  return CallbackReturn::FAILURE;
}

//-----------------------------------------------------------------------------
PathMatching::CallbackReturn PathMatching::on_activate(const rclcpp_lifecycle::State & state)
{
  CallbackReturn result = PathMatchingBase::on_activate(state);
  RCLCPP_INFO(node_->get_logger(), "activated");
  return result;
}

//-----------------------------------------------------------------------------
PathMatching::CallbackReturn PathMatching::on_deactivate(const rclcpp_lifecycle::State & state)
{
  CallbackReturn result = PathMatchingBase::on_deactivate(state);
  RCLCPP_INFO(node_->get_logger(), "deactivated");
  return result;
}

//-----------------------------------------------------------------------------
void PathMatching::reset()
{
  display_.clear();
  if (path_matching_) {
    path_matching_->reset();
    display_.load_path(path_matching_->getPath());
  }
}

//-----------------------------------------------------------------------------
void PathMatching::process_odom_(const Odometry & msg)
{

  if (!is_active_) {return;}

  auto stamp = to_romea_duration(msg.header.stamp);
  core::PoseAndTwist3D enuPoseAndBodyTwist3D;
  to_romea(msg.pose, enuPoseAndBodyTwist3D.pose);
  to_romea(msg.twist, enuPoseAndBodyTwist3D.twist);

  const auto & path = path_matching_->getPath();
  auto vehicle_pose = core::toPose2D(enuPoseAndBodyTwist3D.pose);
  auto vehicle_twist = core::toTwist2D(enuPoseAndBodyTwist3D.twist);

  auto matched_point = path_matching_->match(
    stamp, vehicle_pose, vehicle_twist, prediction_time_horizon_);


  if (matched_point.has_value()) {

    match_pub_->publish(
      to_ros_msg(msg.header.stamp, {*matched_point}, 0, path.getLength(), vehicle_twist));

    // publishNearAnnotations(matched_point, msg.header.stamp);

    if (display_activated_) {
      const auto & section = path.getSection(matched_point->sectionIndex);
      const auto & curve = section.getCurve(matched_point->curveIndex);
      display_.load_curve(curve);
    }
  }
  display_.publish();
}

//-----------------------------------------------------------------------------
void PathMatching::timer_callback_()
{
  auto stamp = node_->get_clock()->now();
  auto report = path_matching_->getReport(to_romea_duration(stamp));
  diagnostics_pub_->publish(stamp, report);
}


}  // namespace ros2
}  // namespace romea

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(romea::ros2::PathMatching)
