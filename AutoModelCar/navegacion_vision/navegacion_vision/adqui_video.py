import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

class VideoAcquisitionNode(Node):
    def __init__(self):
        super().__init__('adqui_video')
        
        # --- PARÁMETROS ---
        # Declaramos un parámetro para la fuente del video.
        # 0 = Cámara web predeterminada
        # 'ruta/al/video.mp4' = Archivo de video
        # --- PARÁMETROS ---
        # Ponemos tu ruta exacta aquí como valor por defecto
        self.declare_parameter('video_source', '/home/alereyes/elementos_externos/Manejo_video_corto.mp4')
        
        # Obtenemos el valor del parámetro
        self.source = self.get_parameter('video_source').get_parameter_value().string_value
        
        # Si es un número (como "0"), lo convertimos a int para que OpenCV use la cámara
        if self.source.isdigit():
            self.source = int(self.source)

        # --- CONFIGURACIÓN ---
        self.publisher_ = self.create_publisher(Image, '/camera/image_raw', 10)
        self.timer_period = 0.033  # 30 FPS (1/30)
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(self.source)

        if not self.cap.isOpened():
            self.get_logger().error(f'No se pudo abrir la fuente de video: {self.source}')
            # Opcional: Cerrar nodo si falla
            # exit()
        else:
            self.get_logger().info(f'Iniciando adquisición desde: {self.source}')

    def timer_callback(self):
        ret, frame = self.cap.read()
        
        if ret:
            # Redimensionar si es necesario (Opcional, ayuda al rendimiento en la Pi)
            # frame = cv2.resize(frame, (640, 480))

            # Convertir a mensaje ROS
            msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "camera_link"
            
            self.publisher_.publish(msg)
        else:
            # Lógica de "Loop" para el video mp4
            if isinstance(self.source, str):
                self.get_logger().info('Fin del video, reiniciando loop...')
                self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = VideoAcquisitionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()