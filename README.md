# Mark 4 — Autonomous Mars Rover (ROS 2 Jazzy + Gazebo Harmonic)

[![License](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![ROS 2](https://img.shields.io/badge/ROS_2-Jazzy_Jalisco-22314E?logo=ros&logoColor=white)](https://docs.ros.org/en/jazzy/)
[![Gazebo](https://img.shields.io/badge/Simulator-Gazebo_Harmonic-orange)](https://gazebosim.org/)
[![Platform](https://img.shields.io/badge/OS-Ubuntu_24.04_LTS-E95420?logo=ubuntu&logoColor=white)](https://ubuntu.com/)

Official physics simulation, hazard mapping, and autonomous navigation repository for the **Mark 4** planetary exploration rover, built on **ROS 2 Jazzy Jalisco** and **Gazebo Harmonic** with a modular Sim-to-Real architecture designed for physical deployment (**Raspberry Pi 5 + ESP32-S3 running micro-ROS**).

---

## 1. Key Features

* **Kinematic & Physical Model (`mark4.urdf.xacro`):** **Rocker-Bogie** suspension system derived from SolidWorks CAD assemblies with `.stl` collision and visual meshes, 4-wheel drive, front-wheel servo steering, wide-angle RGB camera, 6-axis IMU, and LiDAR/ToF sensors.
* **Cliff & Edge Avoidance:** Downward-facing optical ToF sensor (`/scan_precipicio`) with debounce filtering to detect terrain drop-offs and map edges before the rover loses traction.
* **Impassable Slope Detection:** Continuous comparison between commanded velocity (`/cmd_vel`) and actual odometry displacement (`/sim/odom`) to detect wheel slip or stalling on steep inclines.
* **Persistent Hazard Map (`/map`):** Real-time 2D `nav_msgs/msg/OccupancyGrid` generation paired with persistent JSON storage (`mapa_peligros.json`) for preventive path planning.
* **Decoupled Brain–Cerebellum Architecture:** Clean separation between high-level ROS 2 autonomy (PC / Raspberry Pi 5) and real-time low-level motor/sensor control (ESP32-S3 via `micro-ROS`).

---

## 2. System Architecture

### Data Flow Diagram

```text
[Gazebo Harmonic / Mark 4 Rover]
        │
        ├──► /scan ──────────────► [nodo_sensores] ──► /sensor/distancia_frontal ──┐
        ├──► /camera/image_raw ──► [nodo_camara]   ──► /camera/detections ─────────┼──► [nodo_mapa_peligros]
        ├──► /scan_precipicio ─────────────────────────────────────────────────────┤           │
        └──► /sim/odom ────────────────────────────────────────────────────────────┘           │
                                                                                               ├──► /map (RViz2 / Nav2)
                                                                                               └──► /alerta_peligro
                                                                                                          │
                                   [Motors & Servos] ◄── /cmd_vel ◄── [nodo_control_autonomo] ◄───────────┘
```

### ROS 2 Nodes (`rover_autonomo_pkg`)

| Node | Source File | Responsibility |
| :--- | :--- | :--- |
| `nodo_control_autonomo` | `nodo_control.py` | Executes autonomous navigation decisions, reverse escape maneuvers after hazard alerts, safety geofencing, and obstacle avoidance around logged hazards. |
| `nodo_mapa_peligros` | `nodo_peligros.py` | Builds the 2D `/map` occupancy grid, detects cliffs and critical slopes, and saves hazardous coordinates to `mapa_peligros.json`. |
| `nodo_sensores` | `nodo_sensores.py` | Filters raw front scanner beams (`/scan`) and publishes the clean minimum distance to the nearest obstacle. |
| `nodo_camara` | `nodo_camara.py` | Processes RGB frames from `/camera/image_raw` to detect visual obstacles and publish 2D bounding detections. |
| `nodo_odometria` | `nodo_odometria.py` | Computes kinematic rover odometry and broadcasts `odom -> base_footprint` TF transforms. |
| `nodo_mapeo` | `nodo_mapeo.py` | Integrates spatial readings to construct a traversable terrain representation. |
| `nodo_esp32_bridge` | `nodo_esp32_bridge.py` | Communication bridge between ROS 2 and the physical ESP32-S3 microcontroller. |

### Main ROS 2 Topics

| Topic | Message Type | Description |
| :--- | :--- | :--- |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Linear (`x`) and angular (`z`) velocity commands sent to the rover. |
| `/sim/odom` | `nav_msgs/msg/Odometry` | Real-time pose `(x, y, z)`, orientation, and velocity of the rover. |
| `/scan` | `sensor_msgs/msg/LaserScan` | Front LiDAR/ToF scan used for rock and obstacle detection. |
| `/scan_precipicio` | `sensor_msgs/msg/LaserScan` | Downward-pointing ToF ray used to detect cliffs and terrain edges. |
| `/alerta_peligro` | `std_msgs/msg/String` | Immediate JSON alert triggered when a cliff or impassable slope is detected. |
| `/map` | `nav_msgs/msg/OccupancyGrid` | 2D occupancy grid highlighting free space and marked hazard zones. |

---

## 3. Repository Structure

```text
Mark4-rover-simulation-and-autonomous-navigation/
├── LICENSE                                  # Apache License 2.0
├── README.md                                # Project documentation
└── src/
    └── rover_autonomo_pkg/
        ├── config/
        │   └── nav2_params.yaml             # Nav2 planner and costmap parameters
        ├── launch/
        │   ├── simulacion_marte.launch.py   # Launches Gazebo Harmonic, rover URDF, and ros_gz_bridge
        │   ├── automatizacion.launch.py     # Launches control, sensor, and hazard mapping nodes
        │   └── Nav2.launch.py               # Launches the Nav2 autonomous navigation stack
        ├── meshes/                          # 3D STL meshes (Chassis, Rocker, Wheel, Steer)
        ├── rover_autonomo_pkg/              # Python (rclpy) node source files
        ├── urdf/
        │   └── mark4.urdf.xacro             # Physical robot description, joints, and Gazebo plugins
        ├── package.xml                      # ROS 2 package dependencies and metadata
        └── setup.py                         # Package installation and console entry points
```

---

## 4. Step-by-Step Installation Guide (From Scratch)

Follow these steps on a terminal running **Ubuntu 24.04 LTS** (native installation or Windows WSL2):

### Step 1: Install ROS 2 Jazzy Jalisco
If you do not have ROS 2 Jazzy installed yet, run the following commands to configure the official repositories and install the desktop distribution:

```bash
sudo apt update && sudo apt install -y locales curl
