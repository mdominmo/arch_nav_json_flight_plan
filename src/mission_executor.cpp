#include "mission_executor.hpp"

#include <sstream>

#include "arch_nav/model/report/takeoff_report.hpp"
#include "arch_nav/model/report/waypoint_report.hpp"

namespace arch_nav_mission_file {

using arch_nav::constants::CommandResponse;
using arch_nav::constants::OperationStatus;
using arch_nav::constants::ReferenceFrame;
using arch_nav::report::ReportStatus;

namespace {

const char* frame_to_cstr(ReferenceFrame frame) {
  switch (frame) {
    case ReferenceFrame::GLOBAL_WGS84: return "GLOBAL_WGS84";
    case ReferenceFrame::LOCAL_NED: return "LOCAL_NED";
    case ReferenceFrame::LOCAL_ENU: return "LOCAL_ENU";
    case ReferenceFrame::BODY_FCS: return "BODY_FCS";
    default: return "UNKNOWN";
  }
}

}  // namespace

MissionExecutor::MissionExecutor(arch_nav::ArchNavApi& api, rclcpp::Node& node)
: api_(api), node_(node) {}

void MissionExecutor::start(const MissionPlan& plan)
{
  plan_ = plan;
  step_ = Step::WAITING_ARM;
  next_op_index_ = 0;

  api_.on_operation_complete([this](const arch_nav::report::OperationReport& report) {
    on_operation_complete(report);
  });

  api_.on_operation_progress([this](const arch_nav::report::OperationReport& report) {
    on_operation_progress(report);
  });

  RCLCPP_INFO(node_.get_logger(), "Mission loaded (%zu operations) - waiting for vehicle ready",
      plan_.operations.size());
  timer_ = node_.create_wall_timer(
      std::chrono::milliseconds(100),
      [this]() { on_tick(); });
}

void MissionExecutor::abort_mission(const std::string& reason)
{
  RCLCPP_ERROR(node_.get_logger(), "Mission aborted: %s", reason.c_str());
  if (timer_) timer_->cancel();
  step_ = Step::DONE;
}

bool MissionExecutor::start_next_operation()
{
  if (next_op_index_ >= plan_.operations.size()) {
    RCLCPP_INFO(node_.get_logger(), "Mission complete");
    if (timer_) timer_->cancel();
    step_ = Step::DONE;
    return true;
  }

  const auto& op = plan_.operations[next_op_index_];
  CommandResponse response = CommandResponse::DENIED;

  switch (op.type) {
    case MissionOperation::Type::TAKEOFF:
      RCLCPP_INFO(node_.get_logger(), "[op %zu/%zu] takeoff: height=%.2f m frame=%s",
          next_op_index_ + 1, plan_.operations.size(), op.takeoff_height, frame_to_cstr(op.frame));
      response = api_.takeoff(op.takeoff_height, op.frame);
      break;

    case MissionOperation::Type::CHANGE_YAW:
      RCLCPP_INFO(node_.get_logger(), "[op %zu/%zu] change_yaw: yaw=%.3f rad frame=%s",
          next_op_index_ + 1, plan_.operations.size(), op.yaw_rad, frame_to_cstr(op.frame));
      response = api_.change_yaw(op.yaw_rad, op.frame);
      break;

    case MissionOperation::Type::WAYPOINT_FOLLOWING:
      RCLCPP_INFO(node_.get_logger(), "[op %zu/%zu] waypoint_following: %zu waypoints frame=%s",
          next_op_index_ + 1, plan_.operations.size(), op.waypoints.size(), frame_to_cstr(op.frame));
      response = api_.waypoint_following(op.waypoints, op.frame);
      break;

    case MissionOperation::Type::LAND:
      RCLCPP_INFO(node_.get_logger(), "[op %zu/%zu] land",
          next_op_index_ + 1, plan_.operations.size());
      response = api_.land();
      break;
  }

  if (response != CommandResponse::ACCEPTED) {
    std::ostringstream oss;
    oss << "operation " << (next_op_index_ + 1) << " rejected";
    abort_mission(oss.str());
    return false;
  }

  ++next_op_index_;
  step_ = Step::WAITING_OPERATION_COMPLETE;
  return true;
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
        RCLCPP_INFO(node_.get_logger(), "Armed - starting mission operations");
        step_ = Step::WAITING_IDLE_TO_START_OP;
      }
      break;

    case Step::WAITING_IDLE_TO_START_OP:
      if (status == OperationStatus::IDLE) {
        start_next_operation();
      }
      break;

    case Step::WAITING_OPERATION_COMPLETE:
    case Step::DONE:
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
  if (step_ != Step::WAITING_OPERATION_COMPLETE) {
    return;
  }

  if (report.status() == ReportStatus::ABORTED) {
    abort_mission("operation aborted or vehicle lost control");
    return;
  }

  if (report.status() == ReportStatus::FAILED) {
    abort_mission("operation failed");
    return;
  }

  step_ = Step::WAITING_IDLE_TO_START_OP;
}

}  // namespace arch_nav_mission_file
