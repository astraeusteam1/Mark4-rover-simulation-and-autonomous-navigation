#!/usr/bin/env python3
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge

try:
    from ultralytics import YOLO
    YOLO_DISPONIBLE = True
except ImportError:
    YOLO_DISPONIBLE = False


class NodoCamaraVision(Node):
    def __init__(self):
        super().__init__('nodo_camara_vision')

        self.declare_parameter('camera_index', 0)
        self.declare_parameter('usar_yolo', True)
        self.declare_parameter('modelo_yolo', 'yolov8n.pt')

        cam_idx = self.get_parameter('camera_index').value
        usar_yolo = self.get_parameter('usar_yolo').value
        modelo_path = self.get_parameter('modelo_yolo').value

        self.bridge = CvBridge()

        # Publicadores: video anotado + datos estructurados de detección
        self.pub_imagen = self.create_publisher(Image, '/camera/image_annotated', 10)
        self.pub_detecciones = self.create_publisher(Detection2DArray, '/camera/detections', 10)

        # Inicializar cámara
        self.cap = cv2.VideoCapture(cam_idx)
        self.modo_simulado = not self.cap.isOpened()
        if self.modo_simulado:
            self.get_logger().warn('No se detectó cámara física. Activando MODO SIMULACIÓN DE VIDEO.')
            self.sim_x = 100
            self.sim_dir = 8
        else:
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            self.get_logger().info(f'Cámara abierta en índice {cam_idx}.')

        # Inicializar detector
        self.modelo = None
        if usar_yolo and YOLO_DISPONIBLE:
            self.modelo = YOLO(modelo_path)
            self.get_logger().info(f'Detector YOLOv8 ({modelo_path}) cargado.')
        else:
            self.get_logger().info('Usando detector OpenCV HSV (objetos rojos/naranjas como balizas/obstáculos).')

        # Procesar a 15 FPS (dt = 0.066 s) para cuidar CPU en Raspberry Pi 5
        self.timer = self.create_timer(0.066, self.procesar_frame)

    def generar_frame_simulado(self):
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        frame[:] = (35, 35, 40)  # Fondo gris oscuro
        # Dibujar un objeto naranja en movimiento (ej. una roca o baliza)
        self.sim_x += self.sim_dir
        if self.sim_x > 540 or self.sim_x < 100:
            self.sim_dir *= -1
        cv2.rectangle(frame, (self.sim_x - 40, 200), (self.sim_x + 40, 320), (0, 110, 255), -1)
        cv2.putText(frame, 'SIMULACION CAMARA ROVER', (15, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 200, 200), 2)
        return frame

    def procesar_frame(self):
        if self.modo_simulado:
            frame = self.generar_frame_simulado()
        else:
            ret, frame = self.cap.read()
            if not ret:
                return

        ahora = self.get_clock().now().to_msg()
        detecciones_msg = Detection2DArray()
        detecciones_msg.header.stamp = ahora
        detecciones_msg.header.frame_id = 'camera_link'

        if self.modelo is not None:
            # --- INFERENCIA CON YOLOv8 ---
            resultados = self.modelo(frame, verbose=False, conf=0.45)
            for r in resultados:
                for box in r.boxes:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    conf = float(box.conf[0])
                    cls_id = int(box.cls[0])
                    etiqueta = self.modelo.names[cls_id]

                    det = self.crear_deteccion(ahora, x1, y1, x2, y2, etiqueta, conf)
                    detecciones_msg.detections.append(det)

                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(frame, f'{etiqueta} {conf:.2f}', (x1, y1 - 8),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        else:
            # --- DETECCIÓN OPENCV CLÁSICA (Segmentación HSV de balizas/rocas naranjas) ---
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            mascara = cv2.inRange(hsv, np.array([5, 120, 100]), np.array([22, 255, 255]))
            contornos, _ = cv2.findContours(mascara, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for cnt in contornos:
                if cv2.contourArea(cnt) > 800:
                    x, y, w, h = cv2.boundingRect(cnt)
                    det = self.crear_deteccion(ahora, x, y, x + w, y + h, 'baliza_naranja', 0.90)
                    detecciones_msg.detections.append(det)

                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 255), 2)
                    cv2.putText(frame, 'baliza_naranja (0.90)', (x, y - 8),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        # Publicar datos al nodo de mapeo y video anotado
        self.pub_detecciones.publish(detecciones_msg)
        img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        img_msg.header.stamp = ahora
        img_msg.header.frame_id = 'camera_link'
        self.pub_imagen.publish(img_msg)

    def crear_deteccion(self, stamp, x1, y1, x2, y2, clase_id, score):
        det = Detection2D()
        det.header.stamp = stamp
        det.header.frame_id = 'camera_link'
        det.bbox.center.position.x = float((x1 + x2) / 2.0)
        det.bbox.center.position.y = float((y1 + y2) / 2.0)
        det.bbox.size_x = float(x2 - x1)
        det.bbox.size_y = float(y2 - y1)

        hipotesis = ObjectHypothesisWithPose()
        hipotesis.hypothesis.class_id = str(clase_id)
        hipotesis.hypothesis.score = float(score)
        det.results.append(hipotesis)
        return det

    def destroy_node(self):
        if self.cap.isOpened():
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    nodo = NodoCamaraVision()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()