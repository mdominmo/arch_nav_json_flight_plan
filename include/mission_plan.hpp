#ifndef ARCH_NAV_MISSION_FILE__MISSION_PLAN_HPP_
#define ARCH_NAV_MISSION_FILE__MISSION_PLAN_HPP_

#include <vector>

#include "core/model/vehicle/geo_waypoint.hpp"

namespace arch_nav_mission_file {

struct MissionPlan {
  double takeoff_height;
  std::vector<arch_nav::vehicle::GeoWaypoint> waypoints;
  bool land{true};
};

}  // namespace arch_nav_mission_file

#endif  // ARCH_NAV_MISSION_FILE__MISSION_PLAN_HPP_
