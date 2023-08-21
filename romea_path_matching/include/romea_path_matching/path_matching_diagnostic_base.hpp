#ifndef __DiagnosticPathMatchingBase_HPP__
#define __DiagnosticPathMatchingBase_HPP__

//ros
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

//romea
#include <romea_core_common/diagnostic/CheckupRate.hpp>

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

#endif
