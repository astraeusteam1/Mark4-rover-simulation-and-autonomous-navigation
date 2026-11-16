#!/usr/bin/env python3
import os
import json
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import Range, LaserScan
from geometry_msgs.msg import TransformStamped, Twist
from vision_msgs.msg import Detection2DArray
from std_msgs.msg import String
from tf2_ros import StaticTransformBroadcaster


class NodoMapaPeligros(Node):
    def __init__(self):
        super().__init__('nodo_mapa_peligros')

        # Parámetros del mapa 2D (20m x 20m con resolución de 5 cm por celda)
        self.resolucion = 0.05       # metros/celda
        self.ancho_celdas = 400      # 400 * 0.05 = 20 metros
        self.alto_celdas = 400       # 20 metros
        self.origen_x = -10.0        # El robot empieza en el centro (0,0)
        self.origen_y = -10.0

        # Matriz del mapa: -1 = desconocido, 0 = libre, 100 = peligro/obstáculo
        self.grid = np.full((self.alto_celdas, self.ancho_celdas), -1, dtype=np.int8)

        # Archivo persistente para guardar y recargar peligros detectados
        self.archivo_guardado = os.path.expanduser('~/Robot_low_cost/mapa_peligros.json')
        self.peligros_guardados = []

        # Clases de YOLO/OpenCV que consideramos peligrosas
        self.clases_peligro = {'baliza_naranja', 'person', 'chair', 'bottle', 'rock', 'hole', 'stop sign'}

        # Parámetros intrínsecos de cámara (640x480)
        self.fx = 554.0
        self.cx = 320.0

        # Estado del rover
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.distancia_frontal = 2.0

        # Variables para detección de atascamiento en pendientes
        self.cmd_lineal = 0.0
        self.vel_real = 0.0
        self.prev_x = None
        self.prev_y = None
        self.prev_time = None
        self.tiempo_atascado = 0.0
        self.cooldown_atasco = 0.0

        # Variables para detección de acantilados / bordes del mapa
        self.conteo_vacio = 0
        self.cooldown_precipicio = 0.0

        # QoS Transient Local obligatorio para que RViz2 y Nav2 reciban /map aunque se abran después
        qos_mapa = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )
        self.pub_mapa = self.create_publisher(OccupancyGrid, '/map', qos_mapa)
        self.pub_alerta_ruta = self.create_publisher(String, '/alerta_peligro', 10)

        # Publicar TF estática map -> odom (necesaria para Nav2 y RViz2)
        self.tf_static = StaticTransformBroadcaster(self)
        self.publicar_tf_mapa()

        # Suscriptores
        self.create_subscription(Odometry, '/odom', self.cb_odom, 10)
        self.create_subscription(Odometry, '/sim/odom', self.cb_odom, 10)
        self.create_subscription(Twist, '/cmd_vel', self.cb_cmd_vel, 10)
        self.create_subscription(Range, '/sensor/distancia_frontal', self.cb_sensor, 10)
        self.create_subscription(LaserScan, '/scan_precipicio', self.cb_precipicio, 10)
        self.create_subscription(Detection2DArray, '/camera/detections', self.cb_camara, 10)

        # Cargar peligros previos si el archivo existe
        self.cargar_peligros_previos()

        # Timers: 10 Hz para vigilar atascos en pendientes, 1 Hz para publicar /map
        self.timer_atasco = self.create_timer(0.1, self.verificar_pendiente_o_atasco)
        self.timer = self.create_timer(1.0, self.publicar_occupancy_grid)
        self.get_logger().info('Nodo de Mapa de Peligros iniciado (pendientes + sensor de precipicio).')

    def publicar_tf_mapa(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'
        t.transform.rotation.w = 1.0
        self.tf_static.sendTransform(t)

    def mundo_a_celda(self, x: float, y: float):
        """Convierte coordenadas en metros (X, Y) a índices (fila, columna) de la matriz."""
        col = int((x - self.origen_x) / self.resolucion)
        fila = int((y - self.origen_y) / self.resolucion)
        if 0 <= col < self.ancho_celdas and 0 <= fila < self.alto_celdas:
            return fila, col
        return None, None

    def pintar_circulo(self, cx_m: float, cy_m: float, radio_m: float, valor: int):
        """Marca una zona circular en la matriz (0 para camino libre, 100 para peligro)."""
        celdas_radio = int(radio_m / self.resolucion)
        f_centro, c_centro = self.mundo_a_celda(cx_m, cy_m)
        if f_centro is None:
            return

        for df in range(-celdas_radio, celdas_radio + 1):
            for dc in range(-celdas_radio, celdas_radio + 1):
                if df * df + dc * dc <= celdas_radio * celdas_radio:
                    f, c = f_centro + df, c_centro + dc
                    if 0 <= f < self.alto_celdas and 0 <= c < self.ancho_celdas:
                        if valor == 0 and self.grid[f, c] == 100:
                            continue
                        self.grid[f, c] = valor

    def registrar_peligro(self, tipo: str, x: float, y: float, radio_seguridad: float = 0.25):
        """Registra un peligro nuevo si no existe ya uno cercano, lo pinta en /map y lo guarda en disco."""
        for p in self.peligros_guardados:
            if math.hypot(x - p['x'], y - p['y']) < 0.35:
                return  # Ya estaba registrado

        nuevo = {'tipo': tipo, 'x': round(x, 3), 'y': round(y, 3), 'radio': radio_seguridad}
        self.peligros_guardados.append(nuevo)
        self.pintar_circulo(x, y, radio_seguridad, 100)
        self.guardar_en_disco()
        self.get_logger().warn(f'[PELIGRO GUARDADO] "{tipo}" en X={x:.2f} m, Y={y:.2f} m')

    def cb_cmd_vel(self, msg: Twist):
        self.cmd_lineal = msg.linear.x

    def cb_odom(self, msg: Odometry):
        ahora = self.get_clock().now().nanoseconds / 1e9
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z

        # Calcular velocidad real por desplazamiento
        if self.prev_time is not None and (ahora - self.prev_time) > 0.05:
            dt = ahora - self.prev_time
            dist = math.hypot(x - self.prev_x, y - self.prev_y)
            self.vel_real = dist / dt
        else:
            self.vel_real = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)

        self.prev_x = x
        self.prev_y = y
        self.prev_time = ahora

        self.robot_x = x
        self.robot_y = y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny, cosy)

        # Respaldo por caída de altura Z o cabeceo extremo (> 28 grados)
        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        pitch = math.asin(max(-1.0, min(1.0, sinp)))
        if (z < 0.80 or abs(pitch) > 0.50) and self.cooldown_precipicio <= 0.0:
            self.cooldown_precipicio = 4.0
            px = self.robot_x + 0.25 * math.cos(self.robot_yaw)
            py = self.robot_y + 0.25 * math.sin(self.robot_yaw)
            self.registrar_peligro('precipicio_borde', px, py, radio_seguridad=0.35)
            alerta = String()
            alerta.data = json.dumps({'tipo': 'precipicio_borde', 'x': round(px, 2), 'y': round(py, 2)})
            self.pub_alerta_ruta.publish(alerta)

        # Marcar como espacio libre (0) la huella por donde el rover ya pasó sin atascarse
        if self.tiempo_atascado < 0.5:
            self.pintar_circulo(self.robot_x, self.robot_y, radio_m=0.20, valor=0)

    def cb_precipicio(self, msg: LaserScan):
        """Detecta ausencia real de suelo delante de las ruedas (acantilado o borde del mapa)."""
        if self.cooldown_precipicio > 0.0:
            self.cooldown_precipicio -= 0.05
            return

        # Ignorar NaN; en suelo plano mide ~0.17 m, sobre una piedra mide < 0.17 m
        validas = [r for r in msg.ranges if not math.isnan(r)]
        dist_suelo = min(validas) if validas else 2.0

        # Exigir 3 lecturas consecutivas > 0.38 m para confirmar acantilado real y no un rebote en piedra
        if dist_suelo > 0.38 or math.isinf(dist_suelo):
            self.conteo_vacio += 1
        else:
            self.conteo_vacio = 0

        if self.conteo_vacio >= 3:
            self.conteo_vacio = 0
            self.cooldown_precipicio = 4.0

            px = self.robot_x + 0.25 * math.cos(self.robot_yaw)
            py = self.robot_y + 0.25 * math.sin(self.robot_yaw)
            self.registrar_peligro('precipicio_borde', px, py, radio_seguridad=0.35)

            alerta = String()
            alerta.data = json.dumps({'tipo': 'precipicio_borde', 'x': round(px, 2), 'y': round(py, 2)})
            self.pub_alerta_ruta.publish(alerta)
            self.get_logger().error(f'¡ACANTILADO REAL DETECTADO! Suelo a {dist_suelo:.2f} m -> Retrocediendo.')

    def verificar_pendiente_o_atasco(self):
        """Detecta si se pide avanzar pero el rover no logra subir la pendiente."""
        dt = 0.1
        if self.cooldown_atasco > 0.0:
            self.cooldown_atasco -= dt
            return

        # Se ordena avanzar (> 0.08 m/s) pero el desplazamiento real es casi nulo (< 0.025 m/s)
        if self.cmd_lineal > 0.08 and self.vel_real < 0.025:
            self.tiempo_atascado += dt
        else:
            self.tiempo_atascado = 0.0

        # Si lleva 2.0 segundos intentando subir sin avanzar:
        if self.tiempo_atascado >= 2.0:
            self.tiempo_atascado = 0.0
            self.cooldown_atasco = 4.5

            px = self.robot_x + 0.45 * math.cos(self.robot_yaw)
            py = self.robot_y + 0.45 * math.sin(self.robot_yaw)

            self.registrar_peligro('pendiente_infranqueable', px, py, radio_seguridad=0.45)

            alerta = String()
            alerta.data = json.dumps({'tipo': 'pendiente_infranqueable', 'x': round(px, 2), 'y': round(py, 2)})
            self.pub_alerta_ruta.publish(alerta)
            self.get_logger().warn('¡PENDIENTE INFRANQUEABLE! Enviando orden de retroceso y cambio de ruta.')

    def cb_sensor(self, msg: Range):
        self.distancia_frontal = msg.range
        if msg.min_range < msg.range < 0.65:
            px = self.robot_x + msg.range * math.cos(self.robot_yaw)
            py = self.robot_y + msg.range * math.sin(self.robot_yaw)
            self.registrar_peligro('obstaculo_sensor_tof', px, py, radio_seguridad=0.25)

    def cb_camara(self, msg: Detection2DArray):
        for det in msg.detections:
            if not det.results:
                continue
            clase = det.results[0].hypothesis.class_id
            if clase not in self.clases_peligro:
                continue

            u_centro = det.bbox.center.position.x
            angulo_cam = math.atan2(self.cx - u_centro, self.fx)
            angulo_global = self.robot_yaw + angulo_cam

            dist = min(self.distancia_frontal, 2.5)
            px = self.robot_x + dist * math.cos(angulo_global)
            py = self.robot_y + dist * math.sin(angulo_global)

            self.registrar_peligro(f'camara_{clase}', px, py, radio_seguridad=0.30)

    def guardar_en_disco(self):
        with open(self.archivo_guardado, 'w') as f:
            json.dump(self.peligros_guardados, f, indent=2)

    def cargar_peligros_previos(self):
        if os.path.exists(self.archivo_guardado):
            with open(self.archivo_guardado, 'r') as f:
                self.peligros_guardados = json.load(f)
            for p in self.peligros_guardados:
                self.pintar_circulo(p['x'], p['y'], p.get('radio', 0.25), 100)
            self.get_logger().info(f'Cargados {len(self.peligros_guardados)} peligros previos desde {self.archivo_guardado}')

    def publicar_occupancy_grid(self):
        grid_msg = OccupancyGrid()
        grid_msg.header.stamp = self.get_clock().now().to_msg()
        grid_msg.header.frame_id = 'map'

        grid_msg.info.resolution = self.resolucion
        grid_msg.info.width = self.ancho_celdas
        grid_msg.info.height = self.alto_celdas
        grid_msg.info.origin.position.x = self.origen_x
        grid_msg.info.origin.position.y = self.origen_y
        grid_msg.info.origin.orientation.w = 1.0

        grid_msg.data = self.grid.flatten().tolist()
        self.pub_mapa.publish(grid_msg)


def main(args=None):
    rclpy.init(args=args)
    nodo = NodoMapaPeligros()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()