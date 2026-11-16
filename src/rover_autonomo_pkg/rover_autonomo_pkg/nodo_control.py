#!/usr/bin/env python3
import os
import json
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String


class NodoControlAutonomo(Node):
    def __init__(self):
        super().__init__('nodo_control_autonomo')

        # Suscriptor al sensor frontal
        self.subscription = self.create_subscription(
            Range,
            '/sensor/distancia_frontal',
            self.callback_sensor,
            10
        )

        # Suscriptor a alertas de peligro (pendientes infranqueables y precipicios)
        self.sub_alerta = self.create_subscription(
            String,
            '/alerta_peligro',
            self.callback_alerta_peligro,
            10
        )

        # Suscriptores de odometría (simulación y real) para geocerca y zonas prohibidas
        self.sub_sim_odom = self.create_subscription(Odometry, '/sim/odom', self.callback_odom, 10)
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.callback_odom, 10)

        # Publicador de velocidad para los motores
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Parámetros de navegación
        self.umbral_seguridad = 0.60   # metros para obstáculos frontales
        self.limite_mapa = 8.5         # metros desde (0,0) para no acercarse al borde

        # Estado de posición y maniobra de escape
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.pasos_escape = 0
        self.sentido_giro = 1.0
        self.cooldown_zona_peligro = 0

        # Cargar coordenadas peligrosas conocidas desde disco
        self.archivo_peligros = os.path.expanduser('~/Robot_low_cost/mapa_peligros.json')
        self.zonas_peligrosas = self.cargar_zonas_peligrosas()

        self.get_logger().info('Nodo de Control Autónomo listo (con evasión de pendientes, precipicios y geocerca).')

    def cargar_zonas_peligrosas(self):
        if os.path.exists(self.archivo_peligros):
            try:
                with open(self.archivo_peligros, 'r') as f:
                    return json.load(f)
            except Exception:
                return []
        return []

    def callback_odom(self, msg: Odometry):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny, cosy)

    def callback_alerta_peligro(self, msg: String):
        """Se ejecuta cuando nodo_mapa_peligros detecta pendiente infranqueable o precipicio."""
        try:
            datos = json.loads(msg.data)
            self.zonas_peligrosas.append(datos)
            self.get_logger().warn(
                f"¡Alerta recibida ({datos.get('tipo')}) en X={datos.get('x')}, Y={datos.get('y')}! "
                "Retrocediendo y cambiando de ruta..."
            )
        except Exception:
            self.get_logger().warn('¡Alerta de peligro recibida! Retrocediendo y cambiando de ruta...')

        # Activar 35 ciclos de maniobra evasiva (~3.5 s a 10 Hz)
        self.pasos_escape = 35
        self.sentido_giro *= -1.0  # Alternar izquierda/derecha para evitar bucles

    def cerca_de_zona_peligrosa(self) -> bool:
        """Verifica si el punto 0.45 m frente al rover cae dentro de una pendiente o precipicio registrado."""
        frente_x = self.robot_x + 0.45 * math.cos(self.robot_yaw)
        frente_y = self.robot_y + 0.45 * math.sin(self.robot_yaw)
        for p in self.zonas_peligrosas:
            if p.get('tipo') in ('pendiente_infranqueable', 'precipicio_borde'):
                if math.hypot(frente_x - p['x'], frente_y - p['y']) < 0.65:
                    return True
        return False

    def callback_sensor(self, msg: Range):
        cmd = Twist()
        distancia = msg.range

        if self.cooldown_zona_peligro > 0:
            self.cooldown_zona_peligro -= 1

        # 1. PRIORIDAD MÁXIMA: Maniobra de escape tras atasco en pendiente o precipicio
        if self.pasos_escape > 0:
            if self.pasos_escape > 20:
                # Fase 1 (~1.5 s): Retroceder para alejarse del borde o bajar de la pendiente
                cmd.linear.x = -0.22
                cmd.angular.z = 0.0
            else:
                # Fase 2 (~2.0 s): Girar sobre su eje buscando nueva ruta
                cmd.linear.x = 0.0
                cmd.angular.z = 0.85 * self.sentido_giro
            self.pasos_escape -= 1
            self.cmd_pub.publish(cmd)
            return

        # 2. GEOCERCA: Evitar acercarse a los bordes exteriores del mapa
        if abs(self.robot_x) > self.limite_mapa or abs(self.robot_y) > self.limite_mapa:
            angulo_centro = math.atan2(-self.robot_y, -self.robot_x)
            error_yaw = math.atan2(
                math.sin(angulo_centro - self.robot_yaw),
                math.cos(angulo_centro - self.robot_yaw)
            )
            if abs(error_yaw) > 0.35:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.7 if error_yaw > 0 else -0.7
            else:
                cmd.linear.x = 0.30
                cmd.angular.z = 0.0
            self.get_logger().warn('Límite del mapa alcanzado -> Regresando al centro')
            self.cmd_pub.publish(cmd)
            return

        # 3. EVASIÓN PREVENTIVA: Si tiene enfrente una pendiente o precipicio guardado en el mapa
        if self.cooldown_zona_peligro == 0 and self.cerca_de_zona_peligrosa():
            self.pasos_escape = 18  # Girar ~1.8 s para rodear la zona peligrosa
            self.cooldown_zona_peligro = 30
            cmd.linear.x = 0.0
            cmd.angular.z = 0.85 * self.sentido_giro
            self.get_logger().info('Zona peligrosa registrada delante -> Rodeando área')
            self.cmd_pub.publish(cmd)
            return

        # 4. OPERACIÓN NORMAL: Evasión de obstáculos frontales por sensor
        if distancia < self.umbral_seguridad:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.8
            self.get_logger().warn(f'Obstáculo a {distancia:.2f} m -> GIRANDO')
        else:
            cmd.linear.x = 0.35
            cmd.angular.z = 0.0

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    nodo = NodoControlAutonomo()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        stop_cmd = Twist()
        nodo.cmd_pub.publish(stop_cmd)
        nodo.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()