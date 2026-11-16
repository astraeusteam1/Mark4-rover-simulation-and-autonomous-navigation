# Mark 4 — Autonomous Mars Rover (ROS 2 Jazzy + Gazebo Harmonic)

[![License](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![ROS 2](https://img.shields.io/badge/ROS_2-Jazzy_Jalisco-22314E?logo=ros&logoColor=white)](https://docs.ros.org/en/jazzy/)
[![Gazebo](https://img.shields.io/badge/Simulator-Gazebo_Harmonic-orange)](https://gazebosim.org/)
[![Platform](https://img.shields.io/badge/OS-Ubuntu_24.04_LTS-E95420?logo=ubuntu&logoColor=white)](https://ubuntu.com/)

Simulación física, mapeo de peligros y navegación autónoma para el rover de exploración planetaria **Mark 4**, desarrollado en **ROS 2 Jazzy Jalisco** y **Gazebo Harmonic**, con arquitectura modular preparada para despliegue en hardware real (**Raspberry Pi 5 + ESP32-S3 con micro-ROS**).

---

## 1. Características Principales

* **Modelo Cinemático y Físico (`mark4.urdf.xacro`):** Suspensión tipo **Rocker-Bogie** derivada de ensamblaje CAD (SolidWorks) con mallas `.stl`, tracción en las 4 ruedas, dirección delantera con servomotores, cámara RGB, IMU y sensores LiDAR/ToF.
* **Evasión de Acantilados y Bordes:** Sensor óptico inferior (`/scan_precipicio`) con filtrado antirrebote que detecta caídas o bordes del terreno antes de que el rover pierda apoyo.
* **Detección de Pendientes Infranqueables:** Comparación continua entre la velocidad comandada (`/cmd_vel`) y el avance real por odometría (`/sim/odom`) para detectar atascamientos o patinaje en rampas empinadas.
* **Mapa Persistente de Peligros (`/map`):** Generación en tiempo real de un mapa de ocupación 2D (`OccupancyGrid`) y guardado automático de coordenadas peligrosas en `mapa_peligros.json`.

---

## 2. Arquitectura del Sistema

### Flujo de Comunicación

```text
[Gazebo Harmonic / Rover Mark 4]
        │
        ├──► /scan ──────────────► [nodo_sensores] ──► /sensor/distancia_frontal ──┐
        ├──► /camera/image_raw ──► [nodo_camara]   ──► /camera/detections ─────────┼──► [nodo_mapa_peligros]
        ├──► /scan_precipicio ─────────────────────────────────────────────────────┤           │
        └──► /sim/odom ────────────────────────────────────────────────────────────┘           │
                                                                                               ├──► /map (RViz2 / Nav2)
                                                                                               └──► /alerta_peligro
                                                                                                          │
                                   [Motores y Servos] ◄── /cmd_vel ◄── [nodo_control_autonomo] ◄──────────┘
```

### Descripción de los Nodos (`rover_autonomo_pkg`)

| Nodo | Archivo | Función |
| :--- | :--- | :--- |
| `nodo_control_autonomo` | `nodo_control.py` | Toma decisiones de navegación, ejecuta maniobras de escape en reversa tras alertas, respeta geocercas y rodea peligros registrados. |
| `nodo_mapa_peligros` | `nodo_peligros.py` | Construye el mapa 2D (`/map`), detecta acantilados y pendientes críticas, y guarda las zonas peligrosas en disco. |
| `nodo_sensores` | `nodo_sensores.py` | Filtra las lecturas del sensor frontal (`/scan`) y publica la distancia mínima limpia al obstáculo más cercano. |
| `nodo_camara` | `nodo_camara.py` | Procesa las imágenes RGB de `/camera/image_raw` para detectar obstáculos visuales y publicar sus coordenadas 2D. |
| `nodo_odometria` | `nodo_odometria.py` | Calcula la odometría cinemática del rover y publica las transformaciones `odom -> base_footprint`. |
| `nodo_mapeo` | `nodo_mapeo.py` | Integra las lecturas espaciales del entorno para la representación del terreno navegable. |
| `nodo_esp32_bridge` | `nodo_esp32_bridge.py` | Puente de comunicación entre ROS 2 y el microcontrolador físico ESP32-S3. |

### Tópicos Principales

| Tópico | Tipo de Mensaje | Descripción |
| :--- | :--- | :--- |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Comandos de velocidad lineal (`x`) y angular (`z`) enviados al rover. |
| `/sim/odom` | `nav_msgs/msg/Odometry` | Posición y velocidad real del rover en el entorno. |
| `/scan` | `sensor_msgs/msg/LaserScan` | Lecturas del escáner frontal para detección de rocas y obstáculos. |
| `/scan_precipicio` | `sensor_msgs/msg/LaserScan` | Sensor apuntando hacia el suelo para detectar bordes o caídas. |
| `/alerta_peligro` | `std_msgs/msg/String` | Aviso inmediato cuando se detecta un precipicio o pendiente peligrosa. |
| `/map` | `nav_msgs/msg/OccupancyGrid` | Mapa 2D de zonas libres y zonas marcadas como peligro. |

---

## 3. Estructura del Proyecto

```text
Mark4-rover-simulation-and-autonomous-navigation/
├── LICENSE                                  # Licencia Apache 2.0
├── README.md                                # Documentación principal
└── src/
    └── rover_autonomo_pkg/
        ├── config/
        │   └── nav2_params.yaml             # Configuración de parámetros para Nav2
        ├── launch/
        │   ├── simulacion_marte.launch.py   # Inicia Gazebo Harmonic, el robot y el puente ROS-Gazebo
        │   ├── automatizacion.launch.py     # Inicia los nodos de control, sensores y mapa de peligros
        │   └── Nav2.launch.py               # Inicia la navegación autónoma con Nav2
        ├── meshes/                          # Modelos 3D STL del rover (Chassis, Rocker, Wheel, Steer)
        ├── rover_autonomo_pkg/              # Código fuente en Python de los nodos
        ├── urdf/
        │   └── mark4.urdf.xacro             # Descripción física, articulaciones y sensores del rover
        ├── package.xml                      # Dependencias del paquete ROS 2
        └── setup.py                         # Configuración de ejecutables del paquete
```

---

## 4. Guía de Instalación (Desde Cero)

Sigue estos pasos en una terminal de **Ubuntu 24.04 LTS** (nativo o en Windows WSL2):

### Paso 1: Instalar ROS 2 Jazzy Jalisco
Si aún no tienes instalado ROS 2 Jazzy en tu sistema:

```bash
sudo apt update && sudo apt install -y locales curl software-properties-common
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

sudo add-apt-repository universe -y
sudo curl -sSL [https://raw.githubusercontent.com/ros/rosdistro/master/ros.key](https://raw.githubusercontent.com/ros/rosdistro/master/ros.key) -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] [http://packages.ros.org/ros2/ubuntu](http://packages.ros.org/ros2/ubuntu) $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install -y ros-jazzy-desktop python3-colcon-common-extensions python3-rosdep git
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Paso 2: Instalar Gazebo Harmonic y Paquetes de Navegación
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

### Paso 3: Descargar y Compilar este Repositorio
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

## 5. Cómo Ejecutar la Simulación

Abre **dos terminales** separadas en Ubuntu:

### Terminal 1: Abrir el simulador 3D en Gazebo
```bash
ros2 launch rover_autonomo_pkg simulacion_marte.launch.py
```
*Espera unos segundos hasta que se abra la ventana de Gazebo y aparezca el rover Mark 4 sobre el terreno.*

### Terminal 2: Activar la navegación autónoma y detección de peligros
```bash
ros2 launch rover_autonomo_pkg automatizacion.launch.py
```
*El rover comenzará a desplazarse de forma autónoma, esquivando rocas, midiendo pendientes y registrando peligros.*

### Terminal 3 (Opcional): Ver el mapa en tiempo real con RViz2
```bash
rviz2
```
1. En el panel izquierdo (**Global Options -> Fixed Frame**), cambia el valor a `map`.
2. Haz clic en **Add** (abajo a la izquierda) → pestaña **By topic** → agrega `/map` y `/scan`.

---

## 6. Solución de Problemas Rápidos

* **Error `Package 'rover_autonomo_pkg' not found`:** Ejecuta `source ~/Robot_low_cost/install/setup.bash` en esa terminal y vuelve a lanzar el comando.
* **Reiniciar la memoria de peligros:** Si deseas borrar los peligros guardados de pruebas anteriores, ejecuta:
  ```bash
  rm -f ~/Robot_low_cost/mapa_peligros.json
  ```
* **Cerrar procesos de Gazebo si quedaron abiertos:** Presiona `Ctrl + C` en las terminales y ejecuta `pkill -f "gz sim"`.

---

## 7. Licencia

Este proyecto se distribuye bajo la licencia **Apache License 2.0**. Consulta el archivo [LICENSE](LICENSE)
