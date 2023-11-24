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

// ros
#include "tf2_ros/buffer.h"

// romea
#include "romea_path_matching/path_matching_tf.hpp"
#include "romea_common_utils/conversions/transform_conversions.hpp"

namespace romea
{

namespace ros2
{

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
PathMatchingTf::PathMatchingTf()
: is_tf_loaded(false), logger_(rclcpp::get_logger("path_matching_tf"))
{
}

//-----------------------------------------------------------------------------
void PathMatchingTf::publish()
{
  // to_ros_transform_msg(world_to_path_,tf_world_to_path_msg_.transform);
  // tf_broadcaster_->sendTransform(tf_world_to_path_msg_);
}

//-----------------------------------------------------------------------------
void PathMatchingTf::init(
  rclcpp_lifecycle::LifecycleNode::SharedPtr node, const std::string & path_frame_id)
{
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node);
  tf_static_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(node);

  tf_world_to_path_msg_.header.frame_id = "world";
  tf_world_to_path_msg_.child_frame_id = path_frame_id;
}

//-----------------------------------------------------------------------------
void PathMatchingTf::setWorldToPathTransformation(const Eigen::Affine3d & tf)
{
  world_to_path_ = tf;
  is_tf_loaded = true;

  to_ros_transform_msg(world_to_path_, tf_world_to_path_msg_.transform);
  tf_static_broadcaster_->sendTransform(tf_world_to_path_msg_);
}

//-----------------------------------------------------------------------------
const Eigen::Affine3d & PathMatchingTf::getMapToPathTransformation() const
{
  return map_to_path_;
}

//-----------------------------------------------------------------------------
bool PathMatchingTf::evaluateMapToPathTransformation(
  const rclcpp::Time & stamp, const std::string & map_frame_id)
{
  if (!is_tf_loaded) {
    return false;
  }

  tf_world_to_path_msg_.header.stamp = stamp;

  try {
    geometry_msgs::msg::TransformStamped transformStamped;
    transformStamped = tf_buffer_->lookupTransform("world", map_frame_id, rclcpp::Time{});
    world_to_map_ = tf2::transformToEigen(transformStamped);
    map_to_path_ = tf2::transformToEigen(transformStamped).inverse() * world_to_map_;
    return true;
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN_STREAM(logger_, "transform failed: " << ex.what());
    return false;
  }
}

//-----------------------------------------------------------------------------
void PathMatchingTf::reset()
{
  is_tf_loaded = false;
}

tf2_ros::Buffer & PathMatchingTf::getTfBuffer()
{
  return *tf_buffer_;
}

}  // namespace ros2
}  // namespace romea
