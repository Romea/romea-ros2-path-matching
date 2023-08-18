#ifndef PathMatchingDisplay_HPP
#define PathMatchingDisplay_HPP

#include <romea_core_path/Path2D.hpp>

#include "path_matching_display_base.hpp"

namespace romea
{

class PathMatchingDisplay : public PathMatchingDisplayBase
{
public:
  using WayPoints = std::vector<std::vector<PathWayPoint2D>>;

public:
  void load_waypoints(const WayPoints & path_way_points);
  void load_path(const Path2D & path);

protected:
  void add_waypoint_(const PathWayPoint2D & point);
};

}  // namespace romea
#endif
