# waterlinked_sonar_3d15

ROS 2 driver for the [Water Linked Sonar 3D-15](https://www.waterlinked.com/3dsonar), built on the official [`wlsonar`](https://github.com/waterlinked/wlsonar) Python library.

## Features

- **High / low frequency mode** switching (firmware >= 1.7.0)
- **PointCloud2**, **range image**, and **intensity image** publishing
- Full sonar configuration via ROS parameters (speed of sound, range, salinity, UDP mode)
- Runtime parameter reconfiguration
- Diagnostic publishing (temperature, system status)
- Non-blocking UDP receiver (dedicated thread)
- Clean shutdown with acoustics disable

## Requirements

- ROS 2 (Humble / Jazzy / Rolling)
- Python >= 3.10
- Water Linked Sonar 3D-15 with firmware >= 1.5.1 (>= 1.7.0 for mode/salinity features)

## Installation

### Install the `wlsonar` dependency

```bash
pip install wlsonar
```

### Build the package

```bash
cd ~/your_ws
colcon build --packages-select waterlinked_sonar_3d15
source install/setup.bash
```

## Usage

### Launch with default parameters

```bash
ros2 launch waterlinked_sonar_3d15 sonar_3d15.launch.py
```

### Launch with custom parameters

```bash
ros2 launch waterlinked_sonar_3d15 sonar_3d15.launch.py \
    params_file:=/path/to/your_params.yaml
```

### Run the node directly

```bash
ros2 run waterlinked_sonar_3d15 sonar_node --ros-args \
    -p sonar_ip:=192.168.194.96 \
    -p mode:=high-frequency \
    -p acoustics_enabled:=true
```

## Topics

| Topic | Type | Description |
|---|---|---|
| `~/point_cloud` | `sensor_msgs/PointCloud2` | XYZ point cloud from range images |
| `~/range_image` | `sensor_msgs/Image` (32FC1) | Range image as float32 distances in meters |
| `~/intensity_image` | `sensor_msgs/Image` (8UC1) | Logarithmic signal strength image |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | Temperature, firmware, system status |

## Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `sonar_ip` | string | `192.168.194.96` | IP address of the Sonar 3D-15 |
| `frame_id` | string | `sonar_link` | TF frame ID for published messages |
| `acoustics_enabled` | bool | `true` | Enable acoustic imaging |
| `speed_of_sound` | double | `1480.0` | Speed of sound in m/s |
| `mode` | string | `low-frequency` | `low-frequency` or `high-frequency` (fw >= 1.7.0) |
| `salinity` | string | `salt` | `salt` or `fresh` (fw >= 1.7.0) |
| `range_min` | double | `0.3` | Minimum imaging range in meters |
| `range_max` | double | `15.0` | Maximum imaging range in meters |
| `udp_mode` | string | `multicast` | `multicast` or `unicast` |
| `unicast_destination_ip` | string | `""` | Unicast destination IP |
| `unicast_destination_port` | int | `0` | Unicast destination port |
| `diagnostics_period` | double | `5.0` | Seconds between diagnostic queries |

Parameters `acoustics_enabled`, `speed_of_sound`, `mode`, `salinity`, `range_min`, and `range_max` can be changed at runtime via `ros2 param set`.

## Architecture

```
┌──────────────────────────────────────────────────┐
│ sonar_node                                       │
│                                                  │
│  ┌────────────┐    HTTP     ┌──────────────┐     │
│  │ wlsonar    │────────────▶│ Sonar 3D-15  │     │
│  │ .Sonar3D   │◀────────────│              │     │
│  └────────────┘             │              │     │
│                    UDP/RIP2 │              │     │
│  ┌────────────┐◀────────────│              │     │
│  │ recv thread│             └──────────────┘     │
│  └─────┬──────┘                                  │
│        │ wlsonar.range_image_protocol.unpackb()  │
│        ▼                                         │
│  ┌─────────────┐  ┌──────────────┐  ┌─────────┐ │
│  │PointCloud2  │  │ range_image  │  │intensity│ │
│  │ publisher   │  │ publisher    │  │publisher│ │
│  └─────────────┘  └──────────────┘  └─────────┘ │
└──────────────────────────────────────────────────┘
```

## Sonar Modes

The Sonar 3D-15 (firmware >= 1.7.0) supports two imaging modes:

| Mode | Horizontal Res | Vertical Res | Image Size |
|---|---|---|---|
| `low-frequency` | 0.6° (150 px) | 2.4° (16 px) | 2400 pixels |
| `high-frequency` | 0.4° (225 px) | 1.1° (36 px) | 8100 pixels |

Both modes have 90° horizontal and 40° vertical FOV.

## License

MIT
