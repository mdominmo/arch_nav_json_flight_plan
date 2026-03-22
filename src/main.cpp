#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "arch_nav.hpp"
#include "mission_executor.hpp"
#include "mission_plan_loader.hpp"

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("arch_nav_json_flight_plan");

  node->declare_parameter("config", "");
  node->declare_parameter("mission_file", "");

  const auto config_path = node->get_parameter("config").as_string();
  const auto mission_file_path = node->get_parameter("mission_file").as_string();

  if (mission_file_path.empty()) {
    RCLCPP_FATAL(node->get_logger(), "Parameter 'mission_file' is required");
    return 1;
  }

  auto arch_nav = config_path.empty()
      ? arch_nav::ArchNav::create()
      : arch_nav::ArchNav::create(config_path);

  const auto plan = arch_nav_mission_file::MissionPlanLoader::load_from(mission_file_path);
  arch_nav_mission_file::MissionExecutor executor(arch_nav->api(), *node);
  executor.start(plan);

  rclcpp::spin(node);

  return 0;
}
