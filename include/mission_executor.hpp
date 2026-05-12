#ifndef ARCH_NAV_MISSION_FILE__MISSION_EXECUTOR_HPP_
#define ARCH_NAV_MISSION_FILE__MISSION_EXECUTOR_HPP_

#include <cstddef>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "arch_nav/arch_nav_api.hpp"
#include "mission_plan.hpp"

namespace arch_nav_mission_file {

class MissionExecutor {
 public:
  MissionExecutor(arch_nav::ArchNavApi& api, rclcpp::Node& node);

  void start(const MissionPlan& plan);

 private:
  enum class Step {
    WAITING_ARM,
    ARMING,
    WAITING_IDLE_TO_START_OP,
    WAITING_OPERATION_COMPLETE,
    DONE,
  };

  void on_tick();
  void on_operation_complete(const arch_nav::report::OperationReport& report);
  void on_operation_progress(const arch_nav::report::OperationReport& report);
  void abort_mission(const std::string& reason);

  bool start_next_operation();

  arch_nav::ArchNavApi& api_;
  rclcpp::Node& node_;
  MissionPlan plan_;
  Step step_{Step::WAITING_ARM};
  std::size_t next_op_index_{0};
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace arch_nav_mission_file

#endif  // ARCH_NAV_MISSION_FILE__MISSION_EXECUTOR_HPP_
