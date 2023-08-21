#include "romea_path_matching/path_matching_diagnostic_base.hpp"

namespace romea
{

//-----------------------------------------------------------------------------
PathMatchingDiagnosticBase::PathMatchingDiagnosticBase()
: odom_rate_diagnostic_("odom_rate", 0, 0.5),
  odom_rate_task_(
    "odom_rate",
    std::bind(&PathMatchingDiagnosticBase::odom_rate_callback, this, std::placeholders::_1)),
  matching_task_(
    "matching_status",
    std::bind(&PathMatchingDiagnosticBase::matching_callback, this, std::placeholders::_1)),
  composite_diagnostic_("path_matching")
{
  composite_diagnostic_.addTask(&odom_rate_task_);
  composite_diagnostic_.addTask(&matching_task_);
}

void PathMatchingDiagnosticBase::init(rclcpp_lifecycle::LifecycleNode::SharedPtr node)
{
  updater_ = std::make_shared<diagnostic_updater::Updater>(node, 0.5);
  updater_->setHardwareID("none");
  updater_->add(composite_diagnostic_);
}

//-----------------------------------------------------------------------------
void PathMatchingDiagnosticBase::update_odom_rate(const romea::Duration & stamp)
{
  odom_rate_diagnostic_.evaluate(stamp);
}

//-----------------------------------------------------------------------------
void PathMatchingDiagnosticBase::update_matching_status(bool status)
{
  matching_status_ = status;
}

//-----------------------------------------------------------------------------
void PathMatchingDiagnosticBase::publish()
{
  if (updater_) {
    updater_->force_update();
  } else {
    RCLCPP_WARN(rclcpp::get_logger("diagnostic"), "publish() is called before initialization");
  }
}

void PathMatchingDiagnosticBase::odom_rate_callback(StatusWrapper & stat)
{
  DiagnosticReport report = odom_rate_diagnostic_.getReport();
  const auto & [level, msg] = report.diagnostics.front();
  stat.summary(static_cast<unsigned char>(level), msg);
  for (const auto & [name, value] : report.info) {
    stat.add(name, value);
  }
}

void PathMatchingDiagnosticBase::matching_callback(StatusWrapper & stat)
{
  if (matching_status_) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "matching succeeded");
  } else {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "matching failed");
  }
  stat.add("matching", matching_status_);
}

}  // namespace romea
