#include "romea_path_matching/path_matching_base.hpp"

#include <romea_common_utils/qos.hpp>

namespace
{
const double DEFAULT_MAXIMAL_REASEARCH_RADIUS = 10;
const double DEFAULT_INTERPOLATION_WINDOW_LENGTH = 3;
const double DEFAULT_PREDICTION_TIME_HORIZON = 0.5;
}  // namespace

namespace romea
{

PathMatchingBase::PathMatchingBase(const rclcpp::NodeOptions & options)
: node_(std::make_shared<rclcpp::Node>("path_matching", options)),
  prediction_time_horizon_(0),
  maximal_research_radius_(0),
  interpolation_window_length_(0)
{
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
  node_->get_parameter_or(
    "maximal_research_radius", maximal_research_radius_, DEFAULT_MAXIMAL_REASEARCH_RADIUS);
}

void PathMatchingBase::configureInterpolationWindowLength_()
{
  node_->get_parameter_or(
    "interpolation_window_length", interpolation_window_length_,
    DEFAULT_INTERPOLATION_WINDOW_LENGTH);
  //  path_.setInterpolationWindowLength(interpolation_window_length);
}

void PathMatchingBase::configurePredictionTimeHorizon_()
{
  node_->get_parameter_or(
    "prediction_time_horizon", prediction_time_horizon_, DEFAULT_PREDICTION_TIME_HORIZON);
}

}  // namespace romea
