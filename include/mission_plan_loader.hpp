#ifndef ARCH_NAV_MISSION_FILE__MISSION_PLAN_LOADER_HPP_
#define ARCH_NAV_MISSION_FILE__MISSION_PLAN_LOADER_HPP_

#include <string>

#include "mission_plan.hpp"

namespace arch_nav_mission_file {

class MissionPlanLoader {
 public:
  static MissionPlan load_from(const std::string& path);
};

}  // namespace arch_nav_mission_file

#endif  // ARCH_NAV_MISSION_FILE__MISSION_PLAN_LOADER_HPP_
