#ifndef ARCH_NAV_MISSION_FILE__MISSION_EXECUTOR_HPP_
#define ARCH_NAV_MISSION_FILE__MISSION_EXECUTOR_HPP_

#include "rclcpp/rclcpp.hpp"

#include "arch_nav_api.hpp"
#include "mission_plan.hpp"

namespace arch_nav_mission_file {

class MissionExecutor {
 public:
  MissionExecutor(arch_nav::ArchNavApi& api, rclcpp::Node& node);

  void start(const MissionPlan& plan);

 private:
  enum class Step { WAITING_ARM, ARMING, TAKEOFF, WAITING_TAKEOFF,
                    WAYPOINTS, WAITING_WAYPOINTS,
                    LAND, WAITING_LAND, DONE };

  void on_tick();

  arch_nav::ArchNavApi&   api_;
  rclcpp::Node&              node_;
  MissionPlan                plan_;
  Step                       step_{Step::WAITING_ARM};
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace arch_nav_mission_file

#endif  // ARCH_NAV_MISSION_FILE__MISSION_EXECUTOR_HPP_
