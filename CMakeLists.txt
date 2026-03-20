cmake_minimum_required(VERSION 3.8)
project(arch_nav_flight_plan)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(arch-nav REQUIRED)
find_package(arch_nav_px4 REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(origami_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(arch_nav_flight_plan
  src/flight_plan_server.cpp
  src/main.cpp
)
target_include_directories(arch_nav_flight_plan PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
ament_target_dependencies(arch_nav_flight_plan
  arch-nav
  ament_index_cpp
  geographic_msgs
  geometry_msgs
  origami_interfaces
  rclcpp
  std_srvs
)
target_link_libraries(arch_nav_flight_plan
  -Wl,--no-as-needed ${arch_nav_px4_LIBRARIES} -Wl,--as-needed
)

install(TARGETS arch_nav_flight_plan
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
