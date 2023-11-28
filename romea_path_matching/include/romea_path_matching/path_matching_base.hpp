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

#ifndef ROMEA_PATH_MATCHING__PATH_MATCHING_BASE_HPP_
#define ROMEA_PATH_MATCHING__PATH_MATCHING_BASE_HPP_

// ros
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_srvs/srv/empty.hpp"

// romea
#include "romea_path_msgs/msg/path_matching_info2_d.hpp"

namespace romea
{
namespace ros2
{

class PathMatchingBase
{
public:
  using Odometry = nav_msgs::msg::Odometry;
  using PathMatchingInfo2D = romea_path_msgs::msg::PathMatchingInfo2D;
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  using ResetSrv = std_srvs::srv::Empty;

public:
  explicit PathMatchingBase(const rclcpp::NodeOptions & options);

  virtual ~PathMatchingBase() = default;

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
  get_node_base_interface() const;

  virtual void reset() = 0;

  CallbackReturn on_configure(const rclcpp_lifecycle::State &);

  CallbackReturn on_activate(const rclcpp_lifecycle::State &);

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &);

protected:
  virtual void processOdom_(const Odometry & msg) = 0;

  void reset_srv_callback_(ResetSrv::Request::SharedPtr, ResetSrv::Response::SharedPtr);

protected:
  double prediction_time_horizon_;
  double maximal_research_radius_;
  double interpolation_window_length_;

  bool is_active_ = false;

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  rclcpp::Subscription<Odometry>::SharedPtr odom_sub_;
  rclcpp_lifecycle::LifecyclePublisher<PathMatchingInfo2D>::SharedPtr match_pub_;
  rclcpp::Service<ResetSrv>::SharedPtr reset_srv_;
};

}  // namespace ros2
}  // namespace romea

#endif  // ROMEA_PATH_MATCHING__PATH_MATCHING_BASE_HPP_
