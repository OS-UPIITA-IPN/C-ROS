import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge
import cv2
import numpy as np

# Función auxiliar vacía necesaria para los sliders de OpenCV
def nothing(x):
    pass

class LaneProcessorNode(Node):
    def __init__(self):
        super().__init__('proc_carril')

        # --- PARÁMETROS ---
        # Definimos el tópico por defecto (Cámara Real)
        # Si quieres usar el simulador o video grabado, puedes cambiar este parámetro al ejecutar
        self.declare_parameter('image_topic', '/ascamera_hp60c/camera_publisher/rgb0/image')
        topic_name = self.get_parameter('image_topic').get_parameter_value().string_value

        self.get_logger().info(f"Suscribiéndose a: {topic_name}")

        # --- SUSCRIPCIONES Y PUBLICADORES ---
        self.subscription = self.create_subscription(
            Image, 
            topic_name, 
            self.image_callback, 
            10)
        
        self.publisher_error = self.create_publisher(Float32, '/steering_error', 10)
        self.publisher_debug = self.create_publisher(Image, '/vision/debug_img', 10)
        
        self.bridge = CvBridge()

        # --- VARIABLES DE MEMORIA (ESTADO) ---
        self.last_error = 0.0       
        self.left_lane = None       
        self.right_lane = None      

        # ====================================================================
        #                     INTERFAZ DE CALIBRACIÓN (GUI)
        # ====================================================================
        cv2.namedWindow("Calibracion", cv2.WINDOW_NORMAL)

        # VALORES INICIALES (Ajustados para probar)
        cv2.createTrackbar("H Min", "Calibracion", 0, 179, nothing)
        cv2.createTrackbar("H Max", "Calibracion", 179, 179, nothing)
        cv2.createTrackbar("S Min", "Calibracion", 0, 255, nothing)
        cv2.createTrackbar("S Max", "Calibracion", 60, 255, nothing)  
        cv2.createTrackbar("V Min", "Calibracion", 150, 255, nothing) 
        cv2.createTrackbar("V Max", "Calibracion", 255, 255, nothing)

        cv2.createTrackbar("Corte Y (%)", "Calibracion", 55, 100, nothing) 
        cv2.createTrackbar("Ancho Top", "Calibracion", 120, 400, nothing)  
        cv2.createTrackbar("Ancho Bot", "Calibracion", 600, 900, nothing)  

        cv2.createTrackbar("Suavizado (%)", "Calibracion", 20, 100, nothing)
        cv2.createTrackbar("Ancho Carril (px)", "Calibracion", 350, 800, nothing)

        self.get_logger().info("Nodo de Visión Iniciado. Abre 'Calibracion' para ajustar.")

    def fit_lane(self, points):
        """ Ajusta línea recta a puntos blancos. Retorna [vx, vy, x0, y0] """
        if len(points) < 50: return None
        line = cv2.fitLine(points, cv2.DIST_L2, 0, 0.01, 0.01)
        vx, vy, x0, y0 = line.flatten()
        if abs(vy) < 0.1: return None # Rechazar horizontales
        return line.flatten()

    def draw_lane(self, img, lane, color):
        """ Dibuja línea y retorna intersección inferior X """
        vx, vy, x0, y0 = lane
        h = img.shape[0]
        y1 = int(h * 0.4); y2 = h
        x1 = int(x0 + (y1 - y0) * vx / vy)
        x2 = int(x0 + (y2 - y0) * vx / vy)
        cv2.line(img, (x1, y1), (x2, y2), color, 5)
        return x2

    def image_callback(self, msg):
        try:
            # Convertimos mensaje ROS -> OpenCV
            orig_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().warn(f"Error convirtiendo imagen: {e}")
            return

        # --- IMPORTANTE: REDIMENSIONAR ---
        # Para asegurar rendimiento en tiempo real, trabajamos siempre a 640x480
        # independientemente de lo que mande la cámara.
        frame = cv2.resize(orig_frame, (640, 480))

        h, w, _ = frame.shape
        debug = frame.copy() 

        # 1. LEER SLIDERS
        h_min = cv2.getTrackbarPos("H Min", "Calibracion")
        h_max = cv2.getTrackbarPos("H Max", "Calibracion")
        s_min = cv2.getTrackbarPos("S Min", "Calibracion")
        s_max = cv2.getTrackbarPos("S Max", "Calibracion")
        v_min = cv2.getTrackbarPos("V Min", "Calibracion")
        v_max = cv2.getTrackbarPos("V Max", "Calibracion")

        corte_pct = cv2.getTrackbarPos("Corte Y (%)", "Calibracion")
        w_top = cv2.getTrackbarPos("Ancho Top", "Calibracion")
        w_bot = cv2.getTrackbarPos("Ancho Bot", "Calibracion")
        
        alpha = cv2.getTrackbarPos("Suavizado (%)", "Calibracion") / 100.0
        lane_width_virtual = cv2.getTrackbarPos("Ancho Carril (px)", "Calibracion")

        # 2. ROI (TRAPECIO)
        corte_y = int(h * (corte_pct / 100))
        cx = w // 2 
        polygon = np.array([[
            (cx - w_bot // 2, h), (cx + w_bot // 2, h),
            (cx + w_top, corte_y), (cx - w_top, corte_y)
        ]], np.int32)

        mask_roi = np.zeros((h, w), dtype=np.uint8)
        cv2.fillPoly(mask_roi, polygon, 255)
        cv2.polylines(debug, polygon, True, (255, 0, 0), 2)

        # 3. COLOR
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask_color = cv2.inRange(hsv, np.array([h_min, s_min, v_min]), np.array([h_max, s_max, v_max]))
        mask = cv2.bitwise_and(mask_color, mask_roi)

        # 4. CARRIL
        ys, xs = np.where(mask > 0)
        
        if len(xs) == 0:
            self.publisher_debug.publish(self.bridge.cv2_to_imgmsg(debug, "bgr8"))
            cv2.waitKey(1)
            return

        points = np.column_stack((xs, ys))
        mid = w // 2
        
        left_pts = points[points[:, 0] < mid]
        right_pts = points[points[:, 0] >= mid]

        new_left = self.fit_lane(left_pts)
        new_right = self.fit_lane(right_pts)

        # 5. MEMORIA
        if new_left is not None:
            self.left_lane = new_left if self.left_lane is None else alpha * new_left + (1 - alpha) * self.left_lane
        
        if new_right is not None:
            self.right_lane = new_right if self.right_lane is None else alpha * new_right + (1 - alpha) * self.right_lane

        # 6. TARGET
        target_x = mid 
        if self.left_lane is not None and self.right_lane is not None:
            xl = self.draw_lane(debug, self.left_lane, (0, 255, 0)) 
            xr = self.draw_lane(debug, self.right_lane, (0, 255, 0)) 
            target_x = (xl + xr) // 2
            cv2.line(debug, (xl, h-50), (xr, h-50), (0, 255, 255), 2)
        elif self.left_lane is not None:
            xl = self.draw_lane(debug, self.left_lane, (0, 0, 255))
            target_x = xl + (lane_width_virtual // 2)
        elif self.right_lane is not None:
            xr = self.draw_lane(debug, self.right_lane, (0, 0, 255)) 
            target_x = xr - (lane_width_virtual // 2)

        # 7. ERROR Y PUBLICAR
        error = (target_x - mid) / float(mid)
        error = np.clip(error, -1.0, 1.0)
        smooth_error = 0.7 * self.last_error + 0.3 * error
        self.last_error = smooth_error

        # Visuales finales
        vis_target = int(mid + smooth_error * mid)
        cv2.circle(debug, (vis_target, int(h*0.8)), 15, (255, 255, 0), -1)
        cv2.line(debug, (mid, int(h*0.5)), (mid, h), (255, 0, 0), 2)

        pip = cv2.resize(mask, (160, 120))
        pip_color = cv2.cvtColor(pip, cv2.COLOR_GRAY2BGR)
        debug[0:120, 0:160] = pip_color
        cv2.rectangle(debug, (0,0), (160,120), (255,255,255), 1)

        self.publisher_debug.publish(self.bridge.cv2_to_imgmsg(debug, "bgr8"))
        msg_err = Float32()
        msg_err.data = float(smooth_error)
        self.publisher_error.publish(msg_err)

        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = LaneProcessorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()