//romea
#include "romea_path_matching/path_matching_display.hpp"


namespace romea
{

void PathMatchingDisplay::load_waypoints(const WayPoints & path_way_points)
{
  path_marker_.points.clear();

  for (const auto & section_way_points : path_way_points) {
    for (const auto & way_point : section_way_points) {
      add_waypoint_(way_point);
    }
  }
}

void PathMatchingDisplay::load_path(const Path2D & path)
{
  path_marker_.points.clear();

  for (auto const & section : path.getSections()) {
    auto const & x_list = section.getX();
    auto const & y_list = section.getY();

    for (std::size_t i = 0; i < section.size(); ++i) {
      auto & pt = path_marker_.points.emplace_back();
      pt.x = x_list[i];
      pt.y = y_list[i];
      pt.z = 0.1;
    }
  }
}

void PathMatchingDisplay::add_waypoint_(const PathWayPoint2D & way_point)
{
  auto & pt = path_marker_.points.emplace_back();
  pt.x = way_point.position.x();
  pt.y = way_point.position.y();
  pt.z = 0.1;
}


}
