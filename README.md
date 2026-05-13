# arch_nav_json_flight_plan

![License](https://img.shields.io/github/license/mdominmo/arch_nav_json_flight_plan) ![Version](https://img.shields.io/github/v/release/mdominmo/arch_nav_json_flight_plan)

ROS 2 node that loads and executes autonomous flight missions defined in JSON files using the [arch-nav](https://github.com/mdominmo/arch-nav) navigation kernel.

This is an external module — the arch-nav kernel and a compatible driver must be installed on the system before building. See the [arch-nav build instructions](https://github.com/mdominmo/arch-nav#build-and-install).

## Mission format

A mission is an ordered list of operations executed sequentially. The vehicle arms automatically before the first operation and the mission aborts if any command is rejected.

```json
{
  "operations": [
    {
      "takeoff": {
        "height": 10.0,
        "frame": "LOCAL_NED"
      }
    },
    {
      "change_yaw": {
        "yaw_rad": 3.141592653589793,
        "frame": "BODY_FCS"
      }
    },
    {
      "waypoints": [
        { "latitude": 52.11495, "longitude": -6.61300, "altitude": 12.0 },
        { "latitude": 52.11495, "longitude": -6.61285, "altitude": 12.0 }
      ]
    },
    {
      "land": {}
    }
  ]
}
```

### Operations

| Operation | Required fields | Optional fields | Default frame |
|-----------|----------------|-----------------|---------------|
| `takeoff` | `height` (m) | `frame` | `LOCAL_NED` |
| `change_yaw` | `yaw_rad` | `frame` | `BODY_FCS` |
| `waypoints` | array of `{latitude, longitude, altitude}` | `frame` (object form) | `GLOBAL_WGS84` |
| `land` | — | — | — |

### Waypoints: array vs object form

Waypoints can be specified as a plain array (frame defaults to `GLOBAL_WGS84`):

```json
{
  "waypoints": [
    { "latitude": 52.11495, "longitude": -6.61300, "altitude": 12.0 }
  ]
}
```

Or as an object to set the frame explicitly:

```json
{
  "waypoints": {
    "frame": "LOCAL_ENU",
    "items": [
      { "latitude": 0.0, "longitude": 0.0, "altitude": 10.0 }
    ]
  }
}
```

### Supported reference frames

`GLOBAL_WGS84`, `LOCAL_NED`, `LOCAL_ENU`, `BODY_FCS`

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

### Node parameters

| Parameter | Required | Description |
|-----------|----------|-------------|
| `mission_file` | yes | Path to the JSON mission file |
| `config` | no | Path to the arch-nav driver config file |
| `driver` | no | Driver name override (e.g. `"mavsdk"`) |

## Example mission

A complete example is provided in [`missions/example.json`](missions/example.json). It takes off to 10 m, performs a full 360° rotation, follows a 4-waypoint square path, rotates again, and lands.
