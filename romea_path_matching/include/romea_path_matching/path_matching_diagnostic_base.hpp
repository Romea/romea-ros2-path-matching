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

#ifndef ROMEA_PATH_MATCHING__PATH_MATCHING_DIAGNOSTIC_BASE_HPP_
#define ROMEA_PATH_MATCHING__PATH_MATCHING_DIAGNOSTIC_BASE_HPP_

// std
#include <memory>

// ros
#include "diagnostic_updater/diagnostic_updater.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

// romea
#include "romea_core_common/diagnostic/CheckupRate.hpp"

namespace romea
{

class PathMatchingDiagnosticBase
{
public:
  using StatusWrapper = diagnostic_updater::DiagnosticStatusWrapper;

public:
  PathMatchingDiagnosticBase();
  virtual ~PathMatchingDiagnosticBase() = default;

  void init(rclcpp_lifecycle::LifecycleNode::SharedPtr node);
  void update_odom_rate(const romea::Duration & duration);
  void update_matching_status(bool status);
  void publish();

private:
  void odom_rate_callback(StatusWrapper & stat);
  void matching_callback(StatusWrapper & stat);

protected:
  bool matching_status_ = false;
  CheckupGreaterThanRate odom_rate_diagnostic_;

  diagnostic_updater::FunctionDiagnosticTask odom_rate_task_;
  diagnostic_updater::FunctionDiagnosticTask matching_task_;
  diagnostic_updater::CompositeDiagnosticTask composite_diagnostic_;

  std::shared_ptr<diagnostic_updater::Updater> updater_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace romea

#endif  // ROMEA_PATH_MATCHING__PATH_MATCHING_DIAGNOSTIC_BASE_HPP_
