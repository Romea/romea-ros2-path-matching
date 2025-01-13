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
#include <memory>
#include <string>
#include <utility>

// romea
#include <romea_common_utils/conversions/pose3d_conversions.hpp>
#include <romea_common_utils/conversions/twist3d_conversions.hpp>
#include <romea_common_utils/params/geodesy_parameters.hpp>
#include <romea_common_utils/params/node_parameters.hpp>
#include <romea_common_utils/qos.hpp>
#include <romea_core_common/geometry/PoseAndTwist3D.hpp>
#include <romea_path_msgs/msg/path_annotations.hpp>
#include <romea_path_utils/path_matching_info_conversions.hpp>

// local
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
: PathMatchingBase(options), path_matching_(nullptr)
{
  node_->register_on_configure(std::bind(&PathMatching::on_configure, this, std::placeholders::_1));
  node_->register_on_activate(std::bind(&PathMatching::on_activate, this, std::placeholders::_1));
  node_->register_on_deactivate(
    std::bind(&PathMatching::on_deactivate, this, std::placeholders::_1));

  rcl_interfaces::msg::ParameterDescriptor path_descr;
  path_descr.description = "Filename of the path to follow";
  node_->declare_parameter("path", rclcpp::PARAMETER_STRING, path_descr);

  rcl_interfaces::msg::ParameterDescriptor anno_dist_min_descr;
  anno_dist_min_descr.description = "Lower bound of the abscissa interval for annotations";
  node_->declare_parameter("annotation_dist_min", -5.0, anno_dist_min_descr);

  rcl_interfaces::msg::ParameterDescriptor anno_dist_max_descr;
  anno_dist_max_descr.description = "Upper bound of the abscissa interval for annotations";
  node_->declare_parameter("annotation_dist_max", 5.0, anno_dist_max_descr);

  declare_geodetic_coordinates_parameter(node_, "wgs84_anchor");

  if (get_parameter<bool>(node_, "autoconfigure")) {
    auto state = node_->configure();
    if (get_parameter<bool>(node_, "autostart") && state.label() == "inactive") {
      node_->activate();
    }
  }
}

//-----------------------------------------------------------------------------
PathMatching::CallbackReturn PathMatching::on_configure(const rclcpp_lifecycle::State & state)
try {
  PathMatchingBase::on_configure(state);

  auto path = get_parameter<std::string>(node_, "path");
  auto wgs84_anchor = get_geodetic_coordinates_parameter(node_, "wgs84_anchor");
  annotation_dist_min_ = get_parameter<double>(node_, "annotation_dist_min");
  annotation_dist_max_ = get_parameter<double>(node_, "annotation_dist_max");

  path_matching_ = std::make_unique<core::PathMatching>(
    path,
    wgs84_anchor,
    maximal_research_radius_,
    interpolation_window_length_);

  display_.load_path(path_matching_->getPath());

  // comparator_.init();
  // uturn_generator_.init();
  // reset_sub_ = private_nh.subscribe<std_msgs::Bool>(
  //    "reset", 1, &PathMatching::resetCallback, this);

  annotations_pub_ =
    node_->create_publisher<romea_path_msgs::msg::PathAnnotations>("~/annotations", reliable(1));

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
  annotations_pub_->on_activate();

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
  if (!is_active_) {
    return;
  }

  const auto & path = path_matching_->getPath();
  odom_stamp_ = to_romea_duration(msg.header.stamp);
  odom_pose_ = core::toPose2D(to_romea(msg.pose));
  odom_twist_ = core::toTwist2D(to_romea(msg.twist));

  auto matched_points = path_matching_->match(
    odom_stamp_, odom_pose_, odom_twist_, prediction_time_horizon_);

  if (!matched_points.empty()) {
    match_pub_->publish(
      to_ros_msg(msg.header.stamp, matched_points, 0, path.getLength(), odom_twist_));

    // use the last matched point for publishing annotation (higher curvilinear abscissa)
    const auto & last_matched_point = matched_points.back();
    publishNearAnnotations(last_matched_point, msg.header.stamp);

    if (display_activated_) {
      display_.load_curve(
        path_matching_->getCurve(
          matched_points[0].sectionIndex,
          matched_points[0].curveIndex));
    }
  } else {
    match_pub_->publish(
      to_ros_msg(msg.header.stamp, {}, 0, path.getLength(), odom_twist_));
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

void PathMatching::publishNearAnnotations(
  const core::PathMatchedPoint2D & point, const rclcpp::Time & stamp)
{
  romea_path_msgs::msg::PathAnnotations msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = path_frame_id_;

  const auto & annotations = path_matching_->getPath().getAnnotations();
  // auto annotation_it = annotations.lower_bound(point.globalIndex);
  auto annotation_it = begin(annotations);
  double abscissa_max = point.frenetPose.curvilinearAbscissa + annotation_dist_max_;
  double abscissa_min = point.frenetPose.curvilinearAbscissa + annotation_dist_min_;

  while (annotation_it != end(annotations)) {
    const auto & annotation = annotation_it->second;
    if (annotation.abscissa < abscissa_max && annotation.abscissa > abscissa_min) {
      auto & new_a = msg.annotations.emplace_back();
      new_a.type = annotation.type;
      new_a.value = annotation.value;
      new_a.curvilinear_abcissa = annotation.abscissa;
      new_a.curvilinear_distance = annotation.abscissa - point.frenetPose.curvilinearAbscissa;
    }

    ++annotation_it;
  }

  annotations_pub_->publish(msg);
}

}  // namespace ros2
}  // namespace romea

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(romea::ros2::PathMatching)
