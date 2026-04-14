#include <cstdlib>
#include <exception>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "arch_nav/arch_nav.hpp"
#include "mission_executor.hpp"
#include "mission_plan_loader.hpp"

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("arch_nav_json_flight_plan");

  node->declare_parameter("mission_file", "");
  node->declare_parameter("config", "");
  node->declare_parameter("driver", "");

  const auto mission_file_path = node->get_parameter("mission_file").as_string();
  const auto config_path       = node->get_parameter("config").as_string();
  const auto driver_name       = node->get_parameter("driver").as_string();

  if (mission_file_path.empty()) {
    RCLCPP_FATAL(node->get_logger(), "Parameter 'mission_file' is required");
    return 1;
  }

  if (!config_path.empty()) {
    setenv("ARCH_NAV_DRIVER_CONFIG", config_path.c_str(), 1);
  }
  if (!driver_name.empty()) {
    setenv("ARCH_NAV_DRIVER", driver_name.c_str(), 1);
  }

  try {
    RCLCPP_INFO(node->get_logger(), "Loading mission file: %s", mission_file_path.c_str());
    const auto plan = arch_nav_mission_file::MissionPlanLoader::load_from(mission_file_path);

    RCLCPP_INFO(node->get_logger(), "Initializing arch-nav");
    auto arch_nav = arch_nav::ArchNav::create();

    arch_nav_mission_file::MissionExecutor executor(arch_nav->api(), *node);
    executor.start(plan);

    rclcpp::spin(node);
  } catch (const std::exception& e) {
    RCLCPP_FATAL(node->get_logger(), "Startup failed: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }

  return 0;
}
