#include "mission_executor.hpp"

#include "arch_nav/model/report/takeoff_report.hpp"
#include "arch_nav/model/report/waypoint_report.hpp"

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

  api_.on_operation_complete([this](const arch_nav::report::OperationReport& report) {
    on_operation_complete(report);
  });

  api_.on_operation_progress([this](const arch_nav::report::OperationReport& report) {
    on_operation_progress(report);
  });

  RCLCPP_INFO(node_.get_logger(), "Mission loaded - waiting for vehicle ready");
  timer_ = node_.create_wall_timer(
      std::chrono::milliseconds(100),
      [this]() { on_tick(); });
}

void MissionExecutor::abort_mission(const char* reason)
{
  RCLCPP_ERROR(node_.get_logger(), "Mission aborted: %s", reason);
  if (timer_) timer_->cancel();
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
        timer_->cancel();
        RCLCPP_INFO(node_.get_logger(), "Armed - taking off to %.1fm", plan_.takeoff_height);
        if (api_.takeoff(plan_.takeoff_height) != CommandResponse::ACCEPTED) {
          abort_mission("takeoff command rejected");
        } else {
          step_ = Step::WAITING_TAKEOFF;
        }
      }
      break;

    default:
      break;
  }
}

void MissionExecutor::on_operation_progress(const arch_nav::report::OperationReport& report)
{
  if (const auto* r = dynamic_cast<const arch_nav::report::TakeoffReport*>(&report)) {
    RCLCPP_INFO(node_.get_logger(), "[takeoff] altitude: %.1f / %.1f m",
        r->driver_data().current_altitude.load(),
        r->driver_data().target_altitude.load());
  } else if (const auto* r = dynamic_cast<const arch_nav::report::WaypointReport*>(&report)) {
    RCLCPP_INFO(node_.get_logger(), "[waypoints] waypoint: %d / %d",
        r->driver_data().current_waypoint.load(),
        r->driver_data().total_waypoints.load());
  }
}

void MissionExecutor::on_operation_complete(const arch_nav::report::OperationReport& report)
{
  if (report.status() == ReportStatus::ABORTED) {
    abort_mission("operation aborted or vehicle lost control");
    return;
  }

  switch (step_) {
    case Step::WAITING_TAKEOFF:
      RCLCPP_INFO(node_.get_logger(), "Takeoff complete - starting waypoint following");
      if (api_.waypoint_following(plan_.waypoints) != CommandResponse::ACCEPTED) {
        abort_mission("waypoint following command rejected");
      } else {
        step_ = Step::WAITING_WAYPOINTS;
      }
      break;

    case Step::WAITING_WAYPOINTS:
      if (plan_.land) {
        RCLCPP_INFO(node_.get_logger(), "Waypoints complete - landing");
        if (api_.land() != CommandResponse::ACCEPTED) {
          abort_mission("land command rejected");
        } else {
          step_ = Step::WAITING_LAND;
        }
      } else {
        RCLCPP_INFO(node_.get_logger(), "Mission complete");
        step_ = Step::DONE;
      }
      break;

    case Step::WAITING_LAND:
      RCLCPP_INFO(node_.get_logger(), "Mission complete");
      step_ = Step::DONE;
      break;

    default:
      break;
  }
}

}  // namespace arch_nav_mission_file
