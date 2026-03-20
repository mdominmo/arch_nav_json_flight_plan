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
  plan.land           = json.value("land", true);

  for (const auto& wp : json.at("waypoints")) {
    geographic_msgs::msg::GeoPose pose;
    pose.position.latitude  = wp.at("latitude").get<double>();
    pose.position.longitude = wp.at("longitude").get<double>();
    pose.position.altitude  = wp.at("altitude").get<double>();
    // Orientation defaults to identity (heading north) unless specified
    pose.orientation.w = wp.value("qw", 1.0);
    pose.orientation.x = wp.value("qx", 0.0);
    pose.orientation.y = wp.value("qy", 0.0);
    pose.orientation.z = wp.value("qz", 0.0);
    plan.waypoints.push_back(pose);
  }

  return plan;
}

}  // namespace arch_nav_mission_file
