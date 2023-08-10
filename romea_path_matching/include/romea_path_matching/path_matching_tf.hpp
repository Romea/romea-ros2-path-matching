#ifndef PathMatchingTf_HPP
#define PathMatchingTf_HPP

//ros
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

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
#endif
