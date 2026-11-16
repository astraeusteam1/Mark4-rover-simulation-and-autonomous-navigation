#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range
from vision_msgs.msg import Detection2DArray
from visualization_msgs.msg import Marker, MarkerArray


class NodoMapeoSemantico(Node):
    def __init__(self):
        super().__init__('nodo_mapeo')

        # Parámetros intrínsecos aproximados de cámara 640x480 (~60° FOV horizontal)
        self.fx = 554.0
        self.cx = 320.0

        # Estado actual del robot
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.distancia_frontal = 1.5  # metros (actualizado por sensor ToF/LiDAR)

        # Lista de objetos registrados en el mapa [(clase, x, y), ...]
        self.mapa_objetos = []

        # Suscriptores
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.cb_odom, 10)
        self.sub_range = self.create_subscription(Range, '/sensor/distancia_frontal', self.cb_range, 10)
        self.sub_det = self.create_subscription(Detection2DArray, '/camera/detections', self.cb_detecciones, 10)

        # Publicador de marcadores 3D para visualizar el mapa semántico en RViz2
        self.pub_markers = self.create_publisher(MarkerArray, '/mapa/objetos_detectados', 10)

        self.get_logger().info('Nodo de Mapeo Semántico activo: fusionando /camera/detections + /odom.')

    def cb_odom(self, msg: Odometry):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        # Extraer ángulo Yaw desde cuaternión
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)

    def cb_range(self, msg: Range):
        self.distancia_frontal = msg.range

    def cb_detecciones(self, msg: Detection2DArray):
        if not msg.detections:
            return

        for det in msg.detections:
            if not det.results:
                continue

            clase = det.results[0].hypothesis.class_id
            u_centro = det.bbox.center.position.x

            # Ángulo horizontal del objeto respecto al eje óptico de la cámara
            angulo_camara = math.atan2(self.cx - u_centro, self.fx)
            angulo_global = self.robot_yaw + angulo_camara

            # Proyección al marco global 'odom'
            obj_x = self.robot_x + self.distancia_frontal * math.cos(angulo_global)
            obj_y = self.robot_y + self.distancia_frontal * math.sin(angulo_global)

            # Evitar duplicados si ya hay un objeto de la misma clase a menos de 0.40 m
            if not self.es_duplicado(clase, obj_x, obj_y, radio=0.40):
                self.mapa_objetos.append((clase, obj_x, obj_y))
                self.get_logger().info(
                    f'[MAPA] Nuevo objeto "{clase}" registrado en X={obj_x:.2f} m, Y={obj_y:.2f} m '
                    f'(Total mapeados: {len(self.mapa_objetos)})'
                )

        self.publicar_marcadores_rviz()

    def es_duplicado(self, clase, x, y, radio=0.40):
        for c, ox, oy in self.mapa_objetos:
            if c == clase and math.hypot(x - ox, y - oy) < radio:
                return True
        return False

    def publicar_marcadores_rviz(self):
        marker_array = MarkerArray()
        ahora = self.get_clock().now().to_msg()

        for idx, (clase, ox, oy) in enumerate(self.mapa_objetos):
            m = Marker()
            m.header.frame_id = 'odom'
            m.header.stamp = ahora
            m.ns = 'objetos_semanticos'
            m.id = idx
            m.type = Marker.CYLINDER
            m.action = Marker.ADD
            m.pose.position.x = ox
            m.pose.position.y = oy
            m.pose.position.z = 0.15
            m.scale.x = 0.20
            m.scale.y = 0.20
            m.scale.z = 0.30
            m.color.a = 0.9
            m.color.r = 1.0
            m.color.g = 0.4
            m.color.b = 0.0
            marker_array.markers.append(m)

        self.pub_markers.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    nodo = NodoMapeoSemantico()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()