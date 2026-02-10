import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class StopSignDetector(Node):
    def __init__(self):
        super().__init__('proc_senial')

        # --- TÓPICOS ---
        # 1. Suscripción a Color (RGB)
        self.sub_rgb = self.create_subscription(
            Image,
            '/ascamera_hp60c/camera_publisher/rgb0/image',
            self.rgb_callback,
            10)

        # 2. Suscripción a Profundidad (Depth Raw)
        # OJO: La imagen depth viene en formato 16UC1 (Enteros de 16 bits en milímetros)
        self.sub_depth = self.create_subscription(
            Image,
            '/ascamera_hp60c/camera_publisher/depth0/image_raw',
            self.depth_callback,
            10)

        # Publicador de Debug (para ver qué detecta)
        self.pub_debug = self.create_publisher(Image, '/vision/stop_debug', 10)

        self.bridge = CvBridge()

        # Variables para guardar la última imagen recibida
        self.latest_depth_img = None
        
        # --- RANGOS DE COLOR ROJO (HSV) ---
        # El rojo en HSV es tricky porque está al principio (0) y al final (180).
        # Rango 1 (0-10)
        self.lower_red1 = np.array([0, 100, 100])
        self.upper_red1 = np.array([10, 255, 255])
        # Rango 2 (170-180)
        self.lower_red2 = np.array([170, 100, 100])
        self.upper_red2 = np.array([180, 255, 255])

        self.get_logger().info("Detector de STOP iniciado 🛑")

    def depth_callback(self, msg):
        try:
            # Convertimos a imagen de OpenCV (passthrough mantiene la codificación 16bit original)
            self.latest_depth_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as e:
            self.get_logger().error(f"Error depth: {e}")

    def rgb_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Error RGB: {e}")
            return

        # Redimensionar para velocidad (640x480)
        frame = cv2.resize(frame, (640, 480))
        debug = frame.copy()

        # 1. FILTRO DE COLOR (ROJO)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Combinamos las dos máscaras de rojo
        mask1 = cv2.inRange(hsv, self.lower_red1, self.upper_red1)
        mask2 = cv2.inRange(hsv, self.lower_red2, self.upper_red2)
        mask = mask1 + mask2 # Suma lógica (OR)

        # Limpieza de ruido (Dilatar un poco para unir huecos)
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_DILATE, kernel)

        # 2. DETECCIÓN DE CONTORNOS
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            area = cv2.contourArea(cnt)
            
            # Filtro de tamaño: Si es muy chico, es ruido
            if area > 1000:
                # Aproximación de Polígono (Shape Detection)
                epsilon = 0.02 * cv2.arcLength(cnt, True)
                approx = cv2.approxPolyDP(cnt, epsilon, True)
                vertices = len(approx)

                # Buscamos Octágono (8 lados). Damos tolerancia (7 a 9)
                # También verificamos que sea convexo (sin huecos hacia adentro)
                if 7 <= vertices <= 9 and cv2.isContourConvex(approx):
                    
                    # Obtenemos el rectángulo que lo encierra y el centro
                    x, y, w, h = cv2.boundingRect(approx)
                    cx = x + w // 2
                    cy = y + h // 2

                    # Verificar proporción (debe ser casi cuadrado)
                    aspect_ratio = float(w) / h
                    if 0.8 < aspect_ratio < 1.2:
                        
                        # --- FUSIÓN CON PROFUNDIDAD ---
                        distancia_metros = 0.0
                        
                        if self.latest_depth_img is not None:
                            # Tenemos que escalar las coordenadas porque depth puede tener otra resolución
                            # Asumimos que la cámara manda depth y rgb del mismo tamaño o proporcional.
                            # Si RGB lo redimensionamos a 640x480, debemos mapearlo.
                            
                            depth_h, depth_w = self.latest_depth_img.shape
                            scale_x = depth_w / 640.0
                            scale_y = depth_h / 480.0
                            
                            d_x = int(cx * scale_x)
                            d_y = int(cy * scale_y)

                            # Protección de índices
                            d_x = min(max(d_x, 0), depth_w - 1)
                            d_y = min(max(d_y, 0), depth_h - 1)

                            # Leer profundidad (usualmente en milímetros)
                            depth_val = self.latest_depth_img[d_y, d_x]
                            
                            # Convertir a metros (dividir entre 1000)
                            distancia_metros = depth_val / 1000.0

                        # DIBUJAR RESULTADO
                        cv2.drawContours(debug, [approx], 0, (0, 255, 0), 3)
                        cv2.putText(debug, "STOP", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                        
                        msg_dist = f"Dist: {distancia_metros:.2f}m"
                        cv2.putText(debug, msg_dist, (x, y + h + 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
                        
                        self.get_logger().info(f"STOP detectado a {distancia_metros:.2f} m")

        # Publicar imagen debug
        self.pub_debug.publish(self.bridge.cv2_to_imgmsg(debug, "bgr8"))

def main(args=None):
    rclpy.init(args=args)
    node = StopSignDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()