#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster


def euler_a_cuaternion(yaw: float):
    qx = 0.0
    qy = 0.0
    qz = math.sin(yaw / 2.0)
    qw = math.cos(yaw / 2.0)
    return qx, qy, qz, qw


class NodoOdometriaDiferencial(Node):
    def __init__(self):
        super().__init__('nodo_odometria')

        # Parámetros físicos del rover (útiles cuando conectes encoders reales)
        self.declare_parameter('wheel_separation', 0.30)  # Distancia entre ruedas (m)
        self.declare_parameter('wheel_radius', 0.065)     # Radio de rueda (m)
        self.L = self.get_parameter('wheel_separation').value
        self.R = self.get_parameter('wheel_radius').value

        # Estado inicial de pose en el plano
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Velocidades actuales
        self.vx = 0.0
        self.vth = 0.0

        # Suscriptor a /cmd_vel (puedes cambiarlo luego a /wheel_velocities del ESP32)
        self.sub_vel = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.callback_velocidad,
            10
        )

        # Publicador de Odometría y Broadcaster de TF
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Bucle de integración cinemática a 50 Hz (dt = 0.02 s)
        self.ultima_actualizacion = self.get_clock().now()
        self.timer = self.create_timer(0.02, self.actualizar_odometria)

        self.get_logger().info('Nodo de Odometría Diferencial iniciado (publicando /odom y TF odom->base_link).')

    def callback_velocidad(self, msg: Twist):
        self.vx = msg.linear.x
        self.vth = msg.angular.z

    def actualizar_odometria(self):
        ahora = self.get_clock().now()
        dt = (ahora - self.ultima_actualizacion).nanoseconds / 1e9
        self.ultima_actualizacion = ahora

        if dt <= 0.0:
            return

        # Integración cinemática diferencial (Runge-Kutta de 2do orden / punto medio)
        delta_theta = self.vth * dt
        theta_medio = self.theta + (delta_theta / 2.0)

        self.x += self.vx * math.cos(theta_medio) * dt
        self.y += self.vx * math.sin(theta_medio) * dt
        self.theta += delta_theta

        # Normalizar ángulo theta entre [-pi, pi]
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        qx, qy, qz, qw = euler_a_cuaternion(self.theta)
        stamp_msg = ahora.to_msg()

        # 1. Publicar transformación TF: odom -> base_link
        t = TransformStamped()
        t.header.stamp = stamp_msg
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(t)

        # 2. Publicar mensaje Odometry en /odom
        odom = Odometry()
        odom.header.stamp = stamp_msg
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        # Posición
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        # Velocidad en el marco local del robot (base_link)
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.angular.z = self.vth

        self.odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    nodo = NodoOdometriaDiferencial()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()