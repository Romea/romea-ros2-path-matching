#ifndef PathMatching_HPP
#define PathMatching_HPP

#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <romea_common_utils/deferred_call.hpp>
#include <romea_core_common/geometry/Pose2D.hpp>
#include <romea_core_common/geometry/Twist2D.hpp>
#include <romea_core_path/Path2D.hpp>
#include <romea_core_path/PathMatchedPoint2D.hpp>
#include <romea_core_path/PathPosture2D.hpp>

#include "path_matching_base.hpp"
#include "path_matching_diagnostic.hpp"
#include "path_matching_display.hpp"
#include "path_matching_tf.hpp"
// #include "std_msgs/Bool.h"

namespace romea
{

class PathMatching : public PathMatchingBase
{
private:
  using UpdateCb = std::function<void ()>;

public:
  PathMatching(const rclcpp::NodeOptions & options);

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;

  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;

  void loadPath(const std::string & filename);

  void updateDisplay();

  void resetMatching();

  virtual void reset() override;

  bool isMatching() const {return matched_points_.size();}

  const PathSection2D * getCurrentSection() const;
  size_t getCurrentSectionIndex() const;

  const Path2D & getPath() const {return *path_;}
  Path2D & getPath() {return *path_;}

  const std::string & getFrame() const {return path_frame_id_;}
  const Pose2D & getVehiclePose() const {return vehicle_pose_;}

  // void resetCallback(const std_msgs::Bool::ConstPtr & msg);

  // void publishNearAnnotations(const PathMatchedPoint2D & point, const ros::Time & stamp);

protected:
  void processOdom_(const Odometry & msg) override;

  bool tryToEvaluateMapToPathTransformation_(
    const rclcpp::Time & stamp, const std::string & map_frame_id);

  bool tryToMatchOnPath_(const Pose2D & vehicle_pose, const Twist2D & vehicle_twist);

  void displayResults_(const Pose2D & vehicle_pose);

protected:
  PathMatchingTf tf_;
  PathMatchingDisplay display_;
  PathMatchingDiagnostic diagnostics_;
  std::string path_frame_id_;
  // UturnGenerator uturn_generator_;
  bool autostart_;
  bool display_activated_;

  UpdateCb update_cb_;
  std::unique_ptr<Path2D> path_;
  std::vector<PathMatchedPoint2D> matched_points_;
  size_t tracked_matched_point_index_;
  Pose2D vehicle_pose_;
  // ros::Subscriber reset_sub_;
  // ros::Publisher annotations_pub_;
  // double annotation_dist_max_;
  // double annotation_dist_min_;

  rclcpp::Logger logger_;
  std::unique_ptr<DeferredCall> transition_call_;
};

}  // namespace romea
#endif
