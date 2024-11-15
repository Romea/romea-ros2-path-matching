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
#include "romea_common_utils/params/geodesy_parameters.hpp"
#include "romea_core_common/geometry/PoseAndTwist3D.hpp"
#include "romea_core_path/PathMatching2D.hpp"
#include "romea_path_utils/path_matching_info_conversions.hpp"
#include "romea_path_matching/on_the_fly_path_matching.hpp"

namespace
{
const double MINIMAL_DISTANCE_BETWEEN_TWO_POINTS = 0.2;
const double MINIMAL_VEHICLE_SPEED_TO_INSERT_POINT = 0.1;
}

namespace romea
{
namespace ros2
{

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
OnTheFlyPathMatching::OnTheFlyPathMatching(const rclcpp::NodeOptions & options)
: PathMatchingBase(options),
  path_matching_(nullptr)
{
  node_->register_on_configure(
    std::bind(&OnTheFlyPathMatching::on_configure, this, std::placeholders::_1));
  node_->register_on_activate(
    std::bind(&OnTheFlyPathMatching::on_activate, this, std::placeholders::_1));
  node_->register_on_deactivate(
    std::bind(&OnTheFlyPathMatching::on_deactivate, this, std::placeholders::_1));

  rcl_interfaces::msg::ParameterDescriptor mdbtp_descr;
  mdbtp_descr.description =
    "Minimal distance (in meters) covered by vehicle before adding a point in path";
  node_->declare_parameter(
    "minimal_distance_between_two_points",
    MINIMAL_DISTANCE_BETWEEN_TWO_POINTS, mdbtp_descr);

  rcl_interfaces::msg::ParameterDescriptor mvstip_descr;
  mvstip_descr.description =
    "Minimal vehicle speed (in meters/second)required to insert a point into the path";
  node_->declare_parameter(
    "minimal_vehicle_speed_to_insert_point",
    MINIMAL_VEHICLE_SPEED_TO_INSERT_POINT, mvstip_descr);

  if (get_parameter<bool>(node_, "autoconfigure")) {
    auto state = node_->configure();
    if (get_parameter<bool>(node_, "autostart") && state.label() == "inactive") {
      node_->activate();
    }
  }
}

//-----------------------------------------------------------------------------
OnTheFlyPathMatching::CallbackReturn OnTheFlyPathMatching::on_configure(
  const rclcpp_lifecycle::State & state)
try
{
  PathMatchingBase::on_configure(state);

  auto minimal_distance_between_two_points = get_parameter<double>(
    node_, "minimal_distance_between_two_points");

  auto minimal_vehicle_speed_to_insert_point = get_parameter<double>(
    node_, "minimal_vehicle_speed_to_insert_point");

  path_matching_ = std::make_unique<core::OnTheFlyPathMatching>(
    maximal_research_radius_,
    interpolation_window_length_,
    minimal_distance_between_two_points,
    minimal_vehicle_speed_to_insert_point);

  auto callback = std::bind(&OnTheFlyPathMatching::timer_callback_, this);
  timer_ = node_->create_wall_timer(std::chrono::milliseconds(100), callback);

  RCLCPP_INFO(node_->get_logger(), "configured");
  return CallbackReturn::SUCCESS;
} catch (const std::runtime_error & e) {
  RCLCPP_ERROR_STREAM(node_->get_logger(), "configuration failed: " << e.what());
  return CallbackReturn::FAILURE;
}

//-----------------------------------------------------------------------------
OnTheFlyPathMatching::CallbackReturn OnTheFlyPathMatching::on_activate(
  const rclcpp_lifecycle::State & state)
{
  CallbackReturn result = PathMatchingBase::on_activate(state);
  RCLCPP_INFO(node_->get_logger(), "activated");
  return result;
}

//-----------------------------------------------------------------------------
OnTheFlyPathMatching::CallbackReturn OnTheFlyPathMatching::on_deactivate(
  const rclcpp_lifecycle::State & state)
{
  CallbackReturn result = PathMatchingBase::on_deactivate(state);
  RCLCPP_INFO(node_->get_logger(), "deactivated");
  return result;
}

//-----------------------------------------------------------------------------
void OnTheFlyPathMatching::reset()
{
  display_.clear();
  if (path_matching_) {
    path_matching_->reset();
  }
}

//-----------------------------------------------------------------------------
void OnTheFlyPathMatching::process_leader_odom_(const Odometry & msg)
{
  auto leader_odom_stamp = to_romea_duration(msg.header.stamp);
  auto leader_odom_pose = core::toPose2D(to_romea(msg.pose));
  std::lock_guard<std::mutex> guard(mutex_);
  if (path_matching_->updatePath(leader_odom_stamp, leader_odom_pose, odom_twist_)) {
    display_.add_waypoint(core::PathWayPoint2D(leader_odom_pose.position));
  }
}

//-----------------------------------------------------------------------------
void OnTheFlyPathMatching::process_odom_(const Odometry & msg)
{
  if (!is_active_) {return;}

  const auto & section = path_matching_->getSection();
  odom_stamp_ = to_romea_duration(msg.header.stamp);
  odom_pose_ = core::toPose2D(to_romea(msg.pose));
  odom_twist_ = core::toTwist2D(to_romea(msg.twist));

  std::lock_guard<std::mutex> guard(mutex_);
  auto matched_point = path_matching_->match(
    odom_stamp_, odom_pose_, odom_twist_, prediction_time_horizon_);

  if (matched_point.has_value()) {
    match_pub_->publish(
      to_ros_msg(msg.header.stamp, *matched_point, section.getLength(), odom_twist_));
    if (display_activated_) {
      display_.load_curve(section.getCurve(matched_point->curveIndex));
    }
  }
  display_.publish();
}

//-----------------------------------------------------------------------------
void OnTheFlyPathMatching::timer_callback_()
{
  auto stamp = node_->get_clock()->now();
  auto report = path_matching_->getReport(to_romea_duration(stamp));
  diagnostics_pub_->publish(stamp, report);
}


}  // namespace ros2
}  // namespace romea

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(romea::ros2::OnTheFlyPathMatching)
