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
sudo apt update && sudo apt install -y locales curl software-properties-common
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

sudo add-apt-repository universe -y
sudo curl -sSL [https://raw.githubusercontent.com/ros/rosdistro/master/ros.key](https://raw.githubusercontent.com/ros/rosdistro/master/ros.key) -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] [http://packages.ros.org/ros2/ubuntu](http://packages.ros.org/ros2/ubuntu) $(. /etc/os-release && echo$UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install -y ros-jazzy-desktop python3-colcon-common-extensions python3-rosdep git
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Step 2: Install Gazebo Harmonic & Navigation Dependencies
Install the simulation bridge, Xacro processor, vision messages, and Nav2 stack:

```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-ros-gz \
  ros-jazzy-xacro \
  ros-jazzy-joint-state-publisher \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-vision-msgs \
  ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup \
  python3-numpy \
  python3-opencv
```

### Step 3: Clone and Build This Repository
Download the source code into your local workspace and build it using `colcon`:

```bash
git clone [https://github.com/astraeusteam1/Mark4-rover-simulation-and-autonomous-navigation.git](https://github.com/astraeusteam1/Mark4-rover-simulation-and-autonomous-navigation.git) ~/Robot_low_cost
cd ~/Robot_low_cost

sudo rosdep init 2>/dev/null || true
rosdep update
rosdep install --from-paths src --ignore-src -r -y

colcon build --symlink-install
echo "source ~/Robot_low_cost/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## 5. How to Run the Simulation

Open **two separate terminal windows** in Ubuntu:

### Terminal 1: Launch the 3D Gazebo Harmonic Simulation
```bash
ros2 launch rover_autonomo_pkg simulacion_marte.launch.py
```
*Wait a few seconds until the Gazebo window opens and spawns the Mark 4 rover onto the terrain.*

### Terminal 2: Start Autonomous Navigation & Hazard Detection
```bash
ros2 launch rover_autonomo_pkg automatizacion.launch.py
```
*The rover will begin exploring autonomously—avoiding rocks, monitoring slopes, and logging terrain hazards.*

### Terminal 3 (Optional): Visualize the Live Map in RViz2 or Run Nav2
To inspect the `/map` occupancy grid and sensor rays in real time:
```bash
rviz2
```
1. In the left panel (**Global Options -> Fixed Frame**), set the value to `map`.
2. Click **Add** (bottom-left) -> **By topic** tab -> add `/map` (**Map**) and `/scan` (**LaserScan**).

To launch goal-based navigation with **Nav2**:
```bash
ros2 launch rover_autonomo_pkg Nav2.launch.py
```

---

## 6. Troubleshooting

* **`Package 'rover_autonomo_pkg' not found` error:** Run `source ~/Robot_low_cost/install/setup.bash` in your active terminal and retry the launch command.
* **Reset persistent hazard memory:** To clear previously recorded hazards before starting a fresh run:
  ```bash
  rm -f ~/Robot_low_cost/mapa_peligros.json
  ```
* **Stop lingering Gazebo processes:** Press `Ctrl + C` in all active terminals and run:
  ```bash
  pkill -f "gz sim"
  pkill -f parameter_bridge
  ```

---

## 7. License

This project is licensed under the **Apache License 2.0**. See the [LICENSE](LICENSE) file in this repository for full terms and conditions.
