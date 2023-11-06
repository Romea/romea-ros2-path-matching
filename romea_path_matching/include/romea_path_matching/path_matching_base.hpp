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

#ifndef romea_PathMatchingBase_HPP
#define romea_PathMatchingBase_HPP

//ros
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <romea_path_msgs/msg/path_matching_info2_d.hpp>
#include <std_srvs/srv/empty.hpp>

namespace romea
{

class PathMatchingBase : public rclcpp_lifecycle::LifecycleNode
{
public:
  using Odometry = nav_msgs::msg::Odometry;
  using PathMatchingInfo2D = romea_path_msgs::msg::PathMatchingInfo2D;
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  using ResetSrv = std_srvs::srv::Empty;

public:
  PathMatchingBase(const rclcpp::NodeOptions & options);

  virtual ~PathMatchingBase() = default;

  virtual void reset() = 0;

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;

  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;

protected:
  virtual void processOdom_(const Odometry & msg) = 0;

  void reset_srv_callback_(ResetSrv::Request::SharedPtr, ResetSrv::Response::SharedPtr);

protected:
  double prediction_time_horizon_;
  double maximal_research_radius_;
  double interpolation_window_length_;

  bool is_active_ = false;

  rclcpp::Subscription<Odometry>::SharedPtr odom_sub_;
  rclcpp_lifecycle::LifecyclePublisher<PathMatchingInfo2D>::SharedPtr match_pub_;
  rclcpp::Service<ResetSrv>::SharedPtr reset_srv_;
};

}  // namespace romea
#endif
