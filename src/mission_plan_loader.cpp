#include "mission_plan_loader.hpp"

#include <cstddef>
#include <fstream>
#include <stdexcept>
#include <string>

#include <nlohmann/json.hpp>

namespace arch_nav_mission_file {
namespace {

arch_nav::constants::ReferenceFrame parse_reference_frame(
    const std::string& frame) {
  using arch_nav::constants::ReferenceFrame;
  if (frame == "GLOBAL_WGS84") return ReferenceFrame::GLOBAL_WGS84;
  if (frame == "LOCAL_NED") return ReferenceFrame::LOCAL_NED;
  if (frame == "LOCAL_ENU") return ReferenceFrame::LOCAL_ENU;
  if (frame == "BODY_FCS") return ReferenceFrame::BODY_FCS;
  throw std::runtime_error("Unsupported reference frame: " + frame);
}

MissionOperation parse_takeoff_op(const nlohmann::json& payload) {
  MissionOperation op;
  op.type = MissionOperation::Type::TAKEOFF;
  op.takeoff_height = payload.at("height").get<double>();
  op.frame = parse_reference_frame(
      payload.value("frame", std::string("LOCAL_NED")));
  return op;
}

MissionOperation parse_change_yaw_op(const nlohmann::json& payload) {
  MissionOperation op;
  op.type = MissionOperation::Type::CHANGE_YAW;
  op.yaw_rad = payload.at("yaw_rad").get<double>();
  op.frame = parse_reference_frame(
      payload.value("frame", std::string("BODY_FCS")));
  return op;
}

MissionOperation parse_waypoint_following_op(const nlohmann::json& payload) {
  MissionOperation op;
  op.type = MissionOperation::Type::WAYPOINT_FOLLOWING;
  const nlohmann::json* wp_array = nullptr;
  if (payload.is_array()) {
    op.frame = arch_nav::constants::ReferenceFrame::GLOBAL_WGS84;
    wp_array = &payload;
  } else if (payload.is_object()) {
    op.frame = parse_reference_frame(
        payload.value("frame", std::string("GLOBAL_WGS84")));
    wp_array = &payload.at("items");
  } else {
    throw std::runtime_error("waypoints payload must be array or object");
  }

  for (const auto& wp : *wp_array) {
    op.waypoints.emplace_back(
        wp.at("latitude").get<double>(),
        wp.at("longitude").get<double>(),
        wp.at("altitude").get<double>());
  }
  return op;
}

MissionOperation parse_land_op(const nlohmann::json&) {
  MissionOperation op;
  op.type = MissionOperation::Type::LAND;
  return op;
}

}  // namespace

MissionPlan MissionPlanLoader::load_from(const std::string& path)
{
  std::ifstream file(path);
  if (!file.is_open()) {
    throw std::runtime_error("Cannot open mission file: " + path);
  }

  const auto json = nlohmann::json::parse(file);

  if (!json.is_object() || !json.contains("operations") ||
      !json.at("operations").is_array()) {
    throw std::runtime_error(
        "Mission file must contain an 'operations' array");
  }

  MissionPlan plan;
  const auto& operations = json.at("operations");

  for (std::size_t i = 0; i < operations.size(); ++i) {
    const auto& entry = operations.at(i);
    if (!entry.is_object() || entry.size() != 1) {
      throw std::runtime_error(
          "Operation at index " + std::to_string(i) +
          " must be an object with a single operation key");
    }

    const auto it = entry.begin();
    const std::string op_name = it.key();
    const auto& payload = it.value();

    if (op_name == "takeoff") {
      plan.operations.push_back(parse_takeoff_op(payload));
    } else if (op_name == "change_yaw") {
      plan.operations.push_back(parse_change_yaw_op(payload));
    } else if (op_name == "waypoints") {
      plan.operations.push_back(parse_waypoint_following_op(payload));
    } else if (op_name == "land") {
      plan.operations.push_back(parse_land_op(payload));
    } else {
      throw std::runtime_error(
          "Unsupported operation '" + op_name +
          "' at index " + std::to_string(i));
    }
  }

  return plan;
}

}  // namespace arch_nav_mission_file
