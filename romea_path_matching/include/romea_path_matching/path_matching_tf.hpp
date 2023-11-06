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

#ifndef ROMEA_PATH_MATCHING__PATH_MATCHING_TF_HPP_
#define ROMEA_PATH_MATCHING__PATH_MATCHING_TF_HPP_

// std
#include <memory>
#include <string>

// ros
#include "tf2_eigen/tf2_eigen.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace romea
{

class PathMatchingTf
{
public:
  PathMatchingTf();

  void init(rclcpp_lifecycle::LifecycleNode::SharedPtr node, const std::string & path_frame_id);

  void setWorldToPathTransformation(const Eigen::Affine3d & tf);

  const Eigen::Affine3d & getMapToPathTransformation() const;

  bool evaluateMapToPathTransformation(
    const rclcpp::Time & stamp, const std::string & map_frame_id);

  void publish();

  void reset();

  tf2_ros::Buffer & getTfBuffer();

private:
  Eigen::Affine3d map_to_path_;
  Eigen::Affine3d world_to_path_;
  Eigen::Affine3d world_to_map_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
  geometry_msgs::msg::TransformStamped tf_world_to_path_msg_;

  bool is_tf_loaded;

  rclcpp::Logger logger_;
};

}  // namespace romea

#endif  // ROMEA_PATH_MATCHING__PATH_MATCHING_TF_HPP_
