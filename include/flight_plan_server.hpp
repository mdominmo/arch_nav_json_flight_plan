#ifndef ARCH_NAV_FLIGHT_PLAN__FLIGHT_PLAN_SERVER_HPP_
#define ARCH_NAV_FLIGHT_PLAN__FLIGHT_PLAN_SERVER_HPP_

#include "rclcpp/rclcpp.hpp"

#include "arch_nav_api.hpp"
#include "origami_interfaces/srv/flight_plan.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace arch_nav_flight_plan {

class FlightPlanServer {
 public:
  FlightPlanServer(arch_nav::ArchNavApi& api, rclcpp::Node& node);

 private:
  enum class Step {
    IDLE,
    WAITING_ARM,
    TAKEOFF, WAITING_TAKEOFF,
    WAYPOINTS, WAITING_WAYPOINTS,
    LAND, WAITING_LAND,
    DONE
  };

  void on_flight_plan(
      const origami_interfaces::srv::FlightPlan::Request::SharedPtr request,
      origami_interfaces::srv::FlightPlan::Response::SharedPtr response);

  void on_cancel_operation(
      const std_srvs::srv::Trigger::Request::SharedPtr request,
      std_srvs::srv::Trigger::Response::SharedPtr response);

  void on_tick();

  arch_nav::ArchNavApi&  api_;
  rclcpp::Node&             node_;

  rclcpp::Service<origami_interfaces::srv::FlightPlan>::SharedPtr flight_plan_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr              cancel_srv_;
  rclcpp::TimerBase::SharedPtr                                     timer_;

  Step    step_{Step::IDLE};
  double  takeoff_height_{0.0};
  std::vector<geographic_msgs::msg::GeoPose> waypoints_;
};

}  // namespace arch_nav_flight_plan

#endif  // ARCH_NAV_FLIGHT_PLAN__FLIGHT_PLAN_SERVER_HPP_
