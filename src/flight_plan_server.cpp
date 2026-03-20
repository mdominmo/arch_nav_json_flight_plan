#include "flight_plan_server.hpp"

#include "core/constants/operation_status.hpp"
#include "geographic_msgs/msg/geo_pose.hpp"

namespace arch_nav_flight_plan {

using arch_nav::constants::OperationStatus;
using FlightPlan = origami_interfaces::srv::FlightPlan;
using Trigger    = std_srvs::srv::Trigger;

// PoseStamped encodes geographic coordinates as:
//   position.x = latitude  (degrees)
//   position.y = longitude (degrees)
//   position.z = altitude  (metres)
//   orientation = heading quaternion
static geographic_msgs::msg::GeoPose to_geo_pose(
    const geometry_msgs::msg::PoseStamped& stamped)
{
  geographic_msgs::msg::GeoPose geo;
  geo.position.latitude  = stamped.pose.position.x;
  geo.position.longitude = stamped.pose.position.y;
  geo.position.altitude  = stamped.pose.position.z;
  geo.orientation        = stamped.pose.orientation;
  return geo;
}

FlightPlanServer::FlightPlanServer(arch_nav::ArchNavApi& api, rclcpp::Node& node)
: api_(api), node_(node)
{
  flight_plan_srv_ = node_.create_service<FlightPlan>(
      "flight_plan",
      [this](const FlightPlan::Request::SharedPtr req, FlightPlan::Response::SharedPtr res) {
        on_flight_plan(req, res);
      });

  cancel_srv_ = node_.create_service<Trigger>(
      "cancel_operation",
      [this](const Trigger::Request::SharedPtr req, Trigger::Response::SharedPtr res) {
        on_cancel_operation(req, res);
      });

  timer_ = node_.create_wall_timer(
      std::chrono::milliseconds(100),
      [this]() { on_tick(); });
}

void FlightPlanServer::on_flight_plan(
    const FlightPlan::Request::SharedPtr request,
    FlightPlan::Response::SharedPtr response)
{
  const auto status = api_.operation_status();
  if (status != OperationStatus::IDDLE && status != OperationStatus::DISARMED) {
    RCLCPP_WARN(node_.get_logger(),
        "FlightPlan rejected — UAV not available (status=%d)",
        static_cast<int>(status));
    response->success      = false;
    response->failure_code = FlightPlan::Response::UAV_NOT_AVAILABLE;
    return;
  }

  takeoff_height_ = static_cast<double>(request->takeoff_height);

  waypoints_.clear();
  for (const auto& poi : request->poi_vector) {
    waypoints_.push_back(to_geo_pose(poi));
  }
  waypoints_.push_back(to_geo_pose(request->depot));

  RCLCPP_INFO(node_.get_logger(),
      "FlightPlan accepted — mission_id=%u, %zu POIs + depot, takeoff=%.1fm",
      request->mission_id,
      request->poi_vector.size(),
      takeoff_height_);

  if (status == OperationStatus::DISARMED) {
    api_.arm();
  }

  step_ = Step::WAITING_ARM;

  response->success      = true;
  response->failure_code = 0;
}

void FlightPlanServer::on_cancel_operation(
    const Trigger::Request::SharedPtr /*request*/,
    Trigger::Response::SharedPtr response)
{
  api_.cancel_operation();
  step_ = Step::IDLE;
  RCLCPP_INFO(node_.get_logger(), "Operation cancelled via service");
  response->success = true;
  response->message = "Operation cancelled";
}

void FlightPlanServer::on_tick()
{
  if (step_ == Step::IDLE || step_ == Step::DONE) {
    return;
  }

  const auto status = api_.operation_status();

  switch (step_) {
    case Step::WAITING_ARM:
      if (status == OperationStatus::IDDLE) {
        RCLCPP_INFO(node_.get_logger(),
            "Armed and in offboard — taking off to %.1fm", takeoff_height_);
        api_.takeoff(takeoff_height_);
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
        step_ = Step::DONE;
      } else if (status == OperationStatus::IDDLE) {
        RCLCPP_INFO(node_.get_logger(), "Takeoff complete — starting waypoint following");
        api_.waypoint_following(waypoints_);
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
        step_ = Step::DONE;
      } else if (status == OperationStatus::IDDLE) {
        RCLCPP_INFO(node_.get_logger(), "Waypoints complete — landing");
        api_.land();
        step_ = Step::LAND;
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
      } else if (status == OperationStatus::IDDLE) {
        RCLCPP_INFO(node_.get_logger(), "Mission complete");
      }
      step_ = Step::DONE;
      break;

    default:
      break;
  }
}

}  // namespace arch_nav_flight_plan
