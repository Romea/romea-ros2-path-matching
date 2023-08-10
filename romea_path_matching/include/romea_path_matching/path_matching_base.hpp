#ifndef romea_PathMatchingBase_HPP
#define romea_PathMatchingBase_HPP

//ros
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <romea_path_msgs/msg/path_matching_info2_d.hpp>

namespace romea
{

class PathMatchingBase
{
public:
  using Odometry = nav_msgs::msg::Odometry;
  using PathMatchingInfo2D = romea_path_msgs::msg::PathMatchingInfo2D;
  using NodeBasePtr = rclcpp::node_interfaces::NodeBaseInterface::SharedPtr;

public:
  PathMatchingBase(const rclcpp::NodeOptions & options);

  virtual ~PathMatchingBase() = default;

  virtual void init() = 0;

  // virtual void publishDiagnostics(const ros::TimerEvent & event)=0;

  virtual void reset() = 0;

  NodeBasePtr get_node_base_interface() const;

protected:
  virtual void processOdom_(const Odometry & msg) = 0;

  void configureMaximalResearshRadius_();

  void configureInterpolationWindowLength_();

  void configurePredictionTimeHorizon_();

  void configureMatchingInfoPublisher_();

  void configureOdomSubscriber_();

protected:
  rclcpp::Node::SharedPtr node_;

  double prediction_time_horizon_;
  double maximal_research_radius_;
  double interpolation_window_length_;

  rclcpp::Subscription<Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<PathMatchingInfo2D>::SharedPtr match_pub_;
};

}  // namespace romea
#endif
