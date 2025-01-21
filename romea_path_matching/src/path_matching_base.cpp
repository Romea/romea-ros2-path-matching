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

// romea
#include <romea_common_utils/qos.hpp>

// local
#include "romea_path_matching/path_matching_base.hpp"


namespace
{
const double MAXIMAL_REASEARCH_RADIUS = 10;
const double INTERPOLATION_WINDOW_LENGTH = 3;
const double PREDICTION_TIME_HORIZON = 0.5;
}  // namespace

namespace romea
{
namespace ros2
{

//-----------------------------------------------------------------------------
PathMatchingBase::PathMatchingBase(const rclcpp::NodeOptions & options)
: node_(std::make_shared<rclcpp_lifecycle::LifecycleNode>("path_matching", options))
{
  using rcl_interfaces::msg::ParameterDescriptor;
  ParameterDescriptor radius_descr;
  radius_descr.description =
    "Maximal distance (in meters) of the robot to the path to accept a matching";
  node_->declare_parameter("maximal_research_radius", MAXIMAL_REASEARCH_RADIUS, radius_descr);

  ParameterDescriptor iwl_descr;
  iwl_descr.description = "Length (in meters) of the piece of path used to compute interpolation";
  node_->declare_parameter("interpolation_window_length", INTERPOLATION_WINDOW_LENGTH, iwl_descr);

  ParameterDescriptor pth_descr;
  pth_descr.description = "Time (in seconds) to look ahead on the path depending of robot speed";
  node_->declare_parameter("prediction_time_horizon", PREDICTION_TIME_HORIZON, pth_descr);
}

//-----------------------------------------------------------------------------
rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
PathMatchingBase::get_node_base_interface() const
{
  return node_->get_node_base_interface();
}


//-----------------------------------------------------------------------------
PathMatchingBase::CallbackReturn PathMatchingBase::on_configure(const rclcpp_lifecycle::State &)
{
  node_->get_parameter<double>("maximal_research_radius", maximal_research_radius_);
  node_->get_parameter<double>("prediction_time_horizon", prediction_time_horizon_);
  node_->get_parameter<double>("interpolation_window_length", interpolation_window_length_);

  match_pub_ = node_->create_publisher<PathMatchingInfo2D>("~/info", reliable(1));

  diagnostics_pub_ = romea::ros2::make_diagnostic_publisher<romea::core::DiagnosticReport>(
    node_, std::string(node_->get_namespace()) + "/" + std::string(node_->get_name()), 1.0);

  using namespace std::placeholders;
  auto callback = std::bind(&PathMatchingBase::process_odom_, this, _1);
  odom_sub_ = node_->create_subscription<Odometry>("odom", best_effort(1), callback);

  auto reset_callback = std::bind(&PathMatchingBase::reset_srv_callback_, this, _1, _2);
  reset_srv_ = node_->create_service<std_srvs::srv::Empty>("~/reset", reset_callback);

  return CallbackReturn::SUCCESS;
}

//-----------------------------------------------------------------------------
PathMatchingBase::CallbackReturn PathMatchingBase::on_activate(const rclcpp_lifecycle::State &)
{
  match_pub_->on_activate();
  diagnostics_pub_->activate();
  is_active_ = true;
  return CallbackReturn::SUCCESS;
}

//-----------------------------------------------------------------------------
PathMatchingBase::CallbackReturn PathMatchingBase::on_deactivate(const rclcpp_lifecycle::State &)
{
  match_pub_->on_deactivate();
  diagnostics_pub_->deactivate();
  is_active_ = false;
  return CallbackReturn::SUCCESS;
}

//-----------------------------------------------------------------------------
void PathMatchingBase::reset_srv_callback_(
  ResetSrv::Request::SharedPtr, ResetSrv::Response::SharedPtr)
{
  reset();
}

}  // namespace ros2
}  // namespace romea
