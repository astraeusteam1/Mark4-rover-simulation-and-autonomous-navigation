#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range, LaserScan
from geometry_msgs.msg import Vector3


class NodoSensores(Node):
    def __init__(self):
        super().__init__('nodo_sensores')

        # Tiempo máximo sin recibir datos del ESP32 antes de pasar a simulación (segundos)
        self.timeout_esp32 = 1.0
        self.ultimo_tiempo_esp32 = None
        self.modo_actual = None

        # Almacenamiento de últimas lecturas
        self.ultimo_range_hw = None
        self.ultimo_vel_hw = Vector3()
        self.ultimo_range_sim = None

        # 1. Suscriptores a datos reales provenientes del ESP32-S3
        self.create_subscription(Range, '/hw/distancia_frontal', self.cb_hw_range, 10)
        self.create_subscription(Vector3, '/hw/wheel_velocities', self.cb_hw_vel, 10)

        # 2. Suscriptores a datos de Simulación (Gazebo Harmonic vía ros_gz_bridge)
        # Soporta tanto un sensor Range simulado (/sim/distancia_frontal) como un LiDAR (/scan)
        self.create_subscription(Range, '/sim/distancia_frontal', self.cb_sim_range, 10)
        self.create_subscription(LaserScan, '/scan', self.cb_sim_scan, 10)

        # 3. Publicadores unificados hacia el resto del robot (Control, Odometría, Mapa, Nav2)
        self.pub_distancia = self.create_publisher(Range, '/sensor/distancia_frontal', 10)
        self.pub_wheel_vel = self.create_publisher(Vector3, '/wheel_velocities', 10)

        # Bucle de decisión y publicación a 20 Hz (0.05 s)
        self.timer = self.create_timer(0.05, self.ciclo_sensores)
        self.get_logger().info('Nodo Unificador de Sensores iniciado (Conmutación automática ESP32 / Simulación).')

    # --- Callbacks de Hardware Real (ESP32-S3) ---
    def cb_hw_range(self, msg: Range):
        self.ultimo_range_hw = msg
        self.ultimo_tiempo_esp32 = self.get_clock().now()

    def cb_hw_vel(self, msg: Vector3):
        self.ultimo_vel_hw = msg
        self.ultimo_tiempo_esp32 = self.get_clock().now()

    # --- Callbacks de Simulación (Gazebo Harmonic) ---
    def cb_sim_range(self, msg: Range):
        self.ultimo_range_sim = msg

    def cb_sim_scan(self, msg: LaserScan):
        """Extrae el rayo frontal (0 rad) del LaserScan de Gazebo y lo convierte a Range."""
        if not msg.ranges:
            return
        indice_central = len(msg.ranges) // 2
        dist_frontal = msg.ranges[indice_central]

        # Filtrar valores infinitos o NaN propios de Gazebo cuando no hay obstáculo cerca
        if math.isinf(dist_frontal) or math.isnan(dist_frontal):
            dist_frontal = msg.range_max

        r = Range()
        r.header.stamp = self.get_clock().now().to_msg()
        r.header.frame_id = 'sensor_frontal_link'
        r.radiation_type = Range.INFRARED
        r.field_of_view = 0.26
        r.min_range = msg.range_min
        r.max_range = msg.range_max
        r.range = float(max(msg.range_min, min(dist_frontal, msg.range_max)))
        self.ultimo_range_sim = r

    # --- Lógica Principal de Conmutación (if ESP32 activo -> Hardware, else -> Simulación) ---
    def esp32_esta_conectado(self) -> bool:
        if self.ultimo_tiempo_esp32 is None:
            return False
        tiempo_sin_datos = (self.get_clock().now() - self.ultimo_tiempo_esp32).nanoseconds / 1e9
        return tiempo_sin_datos < self.timeout_esp32

    def ciclo_sensores(self):
        if self.esp32_esta_conectado():
            # === MODO HARDWARE REAL (ESP32-S3) ===
            if self.modo_actual != 'HARDWARE':
                self.modo_actual = 'HARDWARE'
                self.get_logger().info('>>> ESP32-S3 DETECTADO: Usando sensores y encoders REALES.')

            if self.ultimo_range_hw is not None:
                self.ultimo_range_hw.header.stamp = self.get_clock().now().to_msg()
                self.pub_distancia.publish(self.ultimo_range_hw)
            self.pub_wheel_vel.publish(self.ultimo_vel_hw)

        else:
            # === MODO SIMULACIÓN (GAZEBO HARMONIC) ===
            if self.modo_actual != 'SIMULACION':
                self.modo_actual = 'SIMULACION'
                self.get_logger().warn('>>> ESP32-S3 NO DETECTADO: Pasando automáticamente a MODO SIMULACIÓN.')

            if self.ultimo_range_sim is not None:
                self.ultimo_range_sim.header.stamp = self.get_clock().now().to_msg()
                self.pub_distancia.publish(self.ultimo_range_sim)
            else:
                # Si Gazebo aún no publica, enviar lectura despejada segura (2.0 m)
                msg_vacio = Range()
                msg_vacio.header.stamp = self.get_clock().now().to_msg()
                msg_vacio.header.frame_id = 'sensor_frontal_link'
                msg_vacio.min_range = 0.05
                msg_vacio.max_range = 4.0
                msg_vacio.range = 2.0
                self.pub_distancia.publish(msg_vacio)


def main(args=None):
    rclpy.init(args=args)
    nodo = NodoSensores()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()