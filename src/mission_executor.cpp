#include "mission_executor.hpp"

#include "core/constants/operation_status.hpp"

namespace arch_nav_mission_file {

using arch_nav::constants::OperationStatus;

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
        api_.takeoff(plan_.takeoff_height);
        step_ = Step::TAKEOFF;
      }
      break;

    case Step::TAKEOFF:
      if (status == OperationStatus::RUNNING) {
        step_ = Step::WAITING_TAKEOFF;
      }
      break;

    case Step::WAITING_TAKEOFF:
      if (status == OperationStatus::FAILED) {
        RCLCPP_ERROR(node_.get_logger(), "Takeoff failed");
        timer_->cancel();
        step_ = Step::DONE;
      } else if (status == OperationStatus::IDLE) {
        RCLCPP_INFO(node_.get_logger(), "Takeoff complete - starting waypoint following");
        api_.waypoint_following(plan_.waypoints);
        step_ = Step::WAYPOINTS;
      }
      break;

    case Step::WAYPOINTS:
      if (status == OperationStatus::RUNNING) {
        step_ = Step::WAITING_WAYPOINTS;
      }
      break;

    case Step::WAITING_WAYPOINTS:
      if (status == OperationStatus::FAILED) {
        RCLCPP_ERROR(node_.get_logger(), "Waypoint following failed");
        timer_->cancel();
        step_ = Step::DONE;
      } else if (status == OperationStatus::IDLE) {
        if (plan_.land) {
          RCLCPP_INFO(node_.get_logger(), "Waypoints complete - landing");
          api_.land();
          step_ = Step::LAND;
        } else {
          RCLCPP_INFO(node_.get_logger(), "Mission complete");
          timer_->cancel();
          step_ = Step::DONE;
        }
      }
      break;

    case Step::LAND:
      if (status == OperationStatus::RUNNING) {
        step_ = Step::WAITING_LAND;
      }
      break;

    case Step::WAITING_LAND:
      if (status == OperationStatus::FAILED) {
        RCLCPP_ERROR(node_.get_logger(), "Landing failed");
      } else if (status == OperationStatus::IDLE) {
        RCLCPP_INFO(node_.get_logger(), "Mission complete");
      }
      timer_->cancel();
      step_ = Step::DONE;
      break;

    case Step::DONE:
      break;
  }
}

}  // namespace arch_nav_mission_file
