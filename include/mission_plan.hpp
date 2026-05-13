#ifndef ARCH_NAV_MISSION_FILE__MISSION_PLAN_HPP_
#define ARCH_NAV_MISSION_FILE__MISSION_PLAN_HPP_

#include <vector>

#include "arch_nav/constants/reference_frame.hpp"
#include "arch_nav/model/vehicle/global_position.hpp"
#include "arch_nav/model/vehicle/waypoint.hpp"

namespace arch_nav_mission_file {

struct MissionOperation {
  enum class Type {
    TAKEOFF,
    CHANGE_YAW,
    WAYPOINT_FOLLOWING,
    LAND,
    SET_ROI,
  };

  Type type;

  double takeoff_height{0.0};
  double yaw_rad{0.0};
  arch_nav::vehicle::GlobalPosition roi_target;

  arch_nav::constants::ReferenceFrame frame{
      arch_nav::constants::ReferenceFrame::LOCAL_NED};

  std::vector<arch_nav::vehicle::Waypoint> waypoints;
};

struct MissionPlan {
  std::vector<MissionOperation> operations;
};

}  // namespace arch_nav_mission_file

#endif  // ARCH_NAV_MISSION_FILE__MISSION_PLAN_HPP_
