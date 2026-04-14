#include "mission_executor.hpp"

namespace arch_nav_mission_file {

using arch_nav::constants::CommandResponse;
using arch_nav::constants::OperationStatus;
using arch_nav::constants::ReferenceFrame;
using arch_nav::report::ReportStatus;

MissionExecutor::MissionExecutor(arch_nav::ArchNavApi& api, rclcpp::Node& node)
: api_(api), node_(node) {}

void MissionExecutor::start(const MissionPlan& plan)
{
  plan_ = plan;
  step_ = Step::WAITING_ARM;
  RCLCPP_INFO(node_.get_logger(), "Mission loaded - waiting for vehicle ready");
  timer_ = node_.create_wall_timer(
      std::chrono::milliseconds(100),
      [this]() { on_tick(); });
}

void MissionExecutor::abort_mission(const char* reason)
{
  RCLCPP_ERROR(node_.get_logger(), "Mission aborted: %s", reason);
  timer_->cancel();
  step_ = Step::DONE;
}

void MissionExecutor::on_tick()
{
  const auto status = api_.operation_status();

  switch (step_) {
    case Step::WAITING_ARM:
      if (status == OperationStatus::DISARMED) {
        RCLCPP_INFO(node_.get_logger(), "Vehicle connected - arming");
        api_.arm();
        step_ = Step::ARMING;
      }
      break;

    case Step::ARMING:
      if (status == OperationStatus::IDLE) {
        RCLCPP_INFO(node_.get_logger(), "Armed - taking off to %.1fm", plan_.takeoff_height);
        if (api_.takeoff(plan_.takeoff_height) != CommandResponse::ACCEPTED) {
          abort_mission("takeoff command rejected");
        } else {
          step_ = Step::WAITING_TAKEOFF;
        }
      }
      break;

    case Step::WAITING_TAKEOFF:
      if (status == OperationStatus::HANDOVER) {
        abort_mission("vehicle lost control during takeoff");
      } else if (status == OperationStatus::IDLE) {
        const auto* report = api_.last_operation_report();
        if (report && report->status() == ReportStatus::ABORTED) {
          abort_mission("takeoff aborted");
        } else {
          RCLCPP_INFO(node_.get_logger(), "Takeoff complete - starting waypoint following");
          if (api_.waypoint_following(plan_.waypoints) != CommandResponse::ACCEPTED) {
            abort_mission("waypoint following command rejected");
          } else {
            step_ = Step::WAITING_WAYPOINTS;
          }
        }
      }
      break;

    case Step::WAITING_WAYPOINTS:
      if (status == OperationStatus::HANDOVER) {
        abort_mission("vehicle lost control during waypoint following");
      } else if (status == OperationStatus::IDLE) {
        const auto* report = api_.last_operation_report();
        if (report && report->status() == ReportStatus::ABORTED) {
          abort_mission("waypoint following aborted");
        } else if (plan_.land) {
          RCLCPP_INFO(node_.get_logger(), "Waypoints complete - landing");
          if (api_.land() != CommandResponse::ACCEPTED) {
            abort_mission("land command rejected");
          } else {
            step_ = Step::WAITING_LAND;
          }
        } else {
          RCLCPP_INFO(node_.get_logger(), "Mission complete");
          timer_->cancel();
          step_ = Step::DONE;
        }
      }
      break;

    case Step::WAITING_LAND:
      if (status == OperationStatus::HANDOVER) {
        abort_mission("vehicle lost control during landing");
      } else if (status == OperationStatus::IDLE || status == OperationStatus::DISARMED) {
        const auto* report = api_.last_operation_report();
        if (report && report->status() == ReportStatus::ABORTED) {
          abort_mission("landing aborted");
        } else {
          RCLCPP_INFO(node_.get_logger(), "Mission complete");
          timer_->cancel();
          step_ = Step::DONE;
        }
      }
      break;

    case Step::DONE:
      break;
  }
}

}  // namespace arch_nav_mission_file
