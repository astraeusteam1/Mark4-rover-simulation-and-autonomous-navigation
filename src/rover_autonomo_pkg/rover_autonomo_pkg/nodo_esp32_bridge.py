#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Range


class NodoESP32Bridge(Node):
    def __init__(self):
        super().__init__('nodo_esp32_bridge')

        # QoS Best Effort: estándar usado por micro-ROS en ESP32 para mínima latencia
        qos_microros = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )

        # Suscriptores a los tópicos crudos que envía el ESP32-S3 por micro-ROS
        self.sub_tof = self.create_subscription(
            Float32,
            '/esp32/tof_raw',
            self.cb_tof_esp32,
            qos_microros
        )
        self.sub_encoders = self.create_subscription(
            Vector3,
            '/esp32/wheel_vel_raw',
            self.cb_encoders_esp32,
            qos_microros
        )

        # Publicadores limpios y estampados con reloj del sistema
        self.pub_range_esp32 = self.create_publisher(Range, '/hw/distancia_frontal', 10)
        self.pub_vel_esp32 = self.create_publisher(Vector3, '/hw/wheel_velocities', 10)

        self.get_logger().info('Nodo Bridge ESP32-S3 iniciado. Esperando datos de micro-ROS (/esp32/tof_raw)...')

    def cb_tof_esp32(self, msg: Float32):
        # Convertir el dato crudo en metros a un mensaje estándar sensor_msgs/Range
        range_msg = Range()
        range_msg.header.stamp = self.get_clock().now().to_msg()
        range_msg.header.frame_id = 'sensor_frontal_link'
        range_msg.radiation_type = Range.INFRARED
        range_msg.field_of_view = 0.26
        range_msg.min_range = 0.03
        range_msg.max_range = 4.00
        range_msg.range = float(msg.data)

        self.pub_range_esp32.publish(range_msg)

    def cb_encoders_esp32(self, msg: Vector3):
        # Reenviar velocidades de rueda izquierda (x) y derecha (y) en m/s
        self.pub_vel_esp32.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    nodo = NodoESP32Bridge()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()