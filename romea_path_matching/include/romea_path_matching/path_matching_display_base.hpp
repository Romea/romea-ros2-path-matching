#ifndef PathMatchingDisplayBase_HPP
#define PathMatchingDisplayBase_HPP

#include <romea_core_path/PathSection2D.hpp>
#include <romea_core_path/PathWayPoint2D.hpp>
#include <romea_core_common/geometry/Pose2D.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace romea
{

class PathMatchingDisplayBase
{
public:
  using Marker = visualization_msgs::msg::Marker;
  using MarkerArray = visualization_msgs::msg::MarkerArray;

public:
  PathMatchingDisplayBase();

  virtual ~PathMatchingDisplayBase() = default;

  void init(rclcpp_lifecycle::LifecycleNode::SharedPtr node, const std::string & path_frame_id);

  void load_curve(const PathCurve2D & path_curve);

  void clear();

  void publish();

protected:
  void initMarkers(const std::string & path_frame_id);

protected:
  bool is_display_activated_ = false;

  rclcpp_lifecycle::LifecyclePublisher<MarkerArray>::SharedPtr marker_pub_;
  Marker path_marker_;
  Marker curve_marker_;
};

}  // namespace romea
#endif
