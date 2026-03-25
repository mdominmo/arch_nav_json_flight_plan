#ifndef ARCH_NAV_MISSION_FILE__MISSION_PLAN_HPP_
#define ARCH_NAV_MISSION_FILE__MISSION_PLAN_HPP_

#include <vector>

#include "arch_nav/arch_nav_api.hpp"

namespace arch_nav_mission_file {

struct MissionPlan {
  double takeoff_height;
  std::vector<arch_nav::vehicle::Waypoint> waypoints;
  bool land{true};
};

}  // namespace arch_nav_mission_file

#endif  // ARCH_NAV_MISSION_FILE__MISSION_PLAN_HPP_
