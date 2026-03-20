#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "arch_nav.hpp"
#include "flight_plan_server.hpp"

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("arch_nav_flight_plan");

  auto arch_nav = arch_nav::ArchNav::create();

  arch_nav_flight_plan::FlightPlanServer server(arch_nav->api(), *node);

  rclcpp::spin(node);

  return 0;
}
