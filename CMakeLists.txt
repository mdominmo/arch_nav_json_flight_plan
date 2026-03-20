cmake_minimum_required(VERSION 3.8)
project(arch_nav_mission_file)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(arch-nav REQUIRED)
find_package(arch_nav_px4 REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nlohmann_json QUIET)
if(NOT nlohmann_json_FOUND)
  include(FetchContent)
  FetchContent_Declare(nlohmann_json
    URL https://github.com/nlohmann/json/releases/download/v3.11.3/json.tar.xz
  )
  FetchContent_MakeAvailable(nlohmann_json)
endif()

add_executable(arch_nav_mission_file
  src/mission_plan_loader.cpp
  src/mission_executor.cpp
  src/main.cpp
)
target_include_directories(arch_nav_mission_file PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
ament_target_dependencies(arch_nav_mission_file
  arch-nav
  ament_index_cpp
  geographic_msgs
  rclcpp
)
target_link_libraries(arch_nav_mission_file
  nlohmann_json::nlohmann_json
  -Wl,--no-as-needed ${arch_nav_px4_LIBRARIES} -Wl,--as-needed
)

install(TARGETS arch_nav_mission_file
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY missions/
  DESTINATION share/${PROJECT_NAME}/missions
)

ament_package()
