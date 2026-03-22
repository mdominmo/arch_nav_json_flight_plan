# arch_nav_json_flight_plan

![License](https://img.shields.io/github/license/mdominmo/arch_nav_json_flight_plan) ![Version](https://img.shields.io/github/v/release/mdominmo/arch_nav_json_flight_plan)

ROS 2 node that executes flight missions defined in JSON files using the [arch-nav](https://github.com/mdominmo/arch-nav) navigation kernel.

This is an external module — the arch-nav kernel and a compatible driver must be installed on the system before building. See the [arch-nav build instructions](https://github.com/mdominmo/arch-nav#build-and-install).

## Mission format

```json
{
  "takeoff": {
    "height": 10.0
  },
  "waypoints": [
    { "latitude": 52.11495, "longitude": -6.61300, "altitude": 12.0 },
    { "latitude": 52.11495, "longitude": -6.61285, "altitude": 12.0 }
  ],
  "land": true
}
```

## Prerequisites

- ROS 2 Humble
- [arch-nav](https://github.com/mdominmo/arch-nav) kernel + driver installed system-wide

## Build

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select arch_nav_json_flight_plan
```

## Run

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run arch_nav_json_flight_plan arch_nav_json_flight_plan \
  --ros-args -p mission_file:=/path/to/mission.json
```
