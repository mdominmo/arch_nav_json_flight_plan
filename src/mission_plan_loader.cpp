#include "mission_plan_loader.hpp"

#include <fstream>
#include <stdexcept>

#include <nlohmann/json.hpp>

namespace arch_nav_mission_file {

MissionPlan MissionPlanLoader::load_from(const std::string& path)
{
  std::ifstream file(path);
  if (!file.is_open()) {
    throw std::runtime_error("Cannot open mission file: " + path);
  }

  const auto json = nlohmann::json::parse(file);

  MissionPlan plan;
  plan.takeoff_height = json.at("takeoff").at("height").get<double>();
  plan.land = json.value("land", true);

  for (const auto& wp : json.at("waypoints")) {
    plan.waypoints.emplace_back(
        wp.at("latitude").get<double>(),
        wp.at("longitude").get<double>(),
        wp.at("altitude").get<double>());
  }

  return plan;
}

}  // namespace arch_nav_mission_file
