#include "romea_path_matching/path_matching_base.hpp"

#include <romea_common_utils/qos.hpp>

namespace
{
const double MAXIMAL_REASEARCH_RADIUS = 10;
const double INTERPOLATION_WINDOW_LENGTH = 3;
const double PREDICTION_TIME_HORIZON = 0.5;
}  // namespace

namespace romea
{

PathMatchingBase::PathMatchingBase(const rclcpp::NodeOptions & options)
: node_(std::make_shared<rclcpp::Node>("path_matching", options)),
  prediction_time_horizon_(0),
  maximal_research_radius_(0),
  interpolation_window_length_(0)
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

PathMatchingBase::NodeBasePtr PathMatchingBase::get_node_base_interface() const
{
  return node_->get_node_base_interface();
}

void PathMatchingBase::configureMatchingInfoPublisher_()
{
  match_pub_ = node_->create_publisher<PathMatchingInfo2D>("path_matching_info", best_effort(1));
}

void PathMatchingBase::configureOdomSubscriber_()
{
  auto callback = std::bind(&PathMatchingBase::processOdom_, this, std::placeholders::_1);
  odom_sub_ = node_->create_subscription<Odometry>("odom", best_effort(1), callback);
}

void PathMatchingBase::configureMaximalResearshRadius_()
{
  node_->get_parameter<double>("maximal_research_radius", maximal_research_radius_);
}

void PathMatchingBase::configureInterpolationWindowLength_()
{
  node_->get_parameter<double>("interpolation_window_length", interpolation_window_length_);
  //  path_.setInterpolationWindowLength(interpolation_window_length);
}

void PathMatchingBase::configurePredictionTimeHorizon_()
{
  node_->get_parameter<double>("prediction_time_horizon", prediction_time_horizon_);
}

}  // namespace romea
