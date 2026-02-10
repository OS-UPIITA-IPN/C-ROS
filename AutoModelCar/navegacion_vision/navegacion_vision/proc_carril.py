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

        # --- SUSCRIPCIONES Y PUBLICADORES ---
        self.subscription = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        
        self.publisher_error = self.create_publisher(Float32, '/steering_error', 10)
        self.publisher_debug = self.create_publisher(Image, '/vision/debug_img', 10)
        
        self.bridge = CvBridge()

        # --- VARIABLES DE MEMORIA (ESTADO) ---
        self.last_error = 0.0       # Guarda el error anterior para suavizar
        self.left_lane = None       # Guarda la ecuación de la línea izquierda detectada
        self.right_lane = None      # Guarda la ecuación de la línea derecha detectada

        # ====================================================================
        #                     INTERFAZ DE CALIBRACIÓN (GUI)
        # ====================================================================
        # Creamos una ventana llamada "Calibracion" que permite redimensionar
        cv2.namedWindow("Calibracion", cv2.WINDOW_NORMAL)

        # --- EXPLICACIÓN DE HSV ---
        # H (Hue - Matiz): El tipo de color (0-179). Rojo, Verde, Azul, etc.
        # S (Saturation - Saturación): Qué tan intenso es el color (0-255).
        #     0 = Gris/Blanco (sin color), 255 = Color muy vivo.
        # V (Value - Brillo): Qué tanta luz tiene (0-255).
        #     0 = Negro total, 255 = Brillo máximo.
        
        # Para detectar líneas BLANCAS en asfalto:
        # H: No importa mucho (0-179).
        # S: Debe ser BAJA (0-50), porque el blanco no tiene saturación.
        # V: Debe ser ALTA (200-255), porque el blanco refleja mucha luz.

        cv2.createTrackbar("H Min", "Calibracion", 0, 179, nothing)
        cv2.createTrackbar("H Max", "Calibracion", 179, 179, nothing)
        cv2.createTrackbar("S Min", "Calibracion", 0, 255, nothing)
        cv2.createTrackbar("S Max", "Calibracion", 60, 255, nothing)  # Bajo para blanco
        cv2.createTrackbar("V Min", "Calibracion", 150, 255, nothing) # Alto para brillo
        cv2.createTrackbar("V Max", "Calibracion", 255, 255, nothing)

        # --- PARÁMETROS DE REGIÓN DE INTERÉS (ROI) ---
        # Definen el trapecio donde buscamos la carretera.
        cv2.createTrackbar("Corte Y (%)", "Calibracion", 55, 100, nothing) # Altura horizonte
        cv2.createTrackbar("Ancho Top", "Calibracion", 120, 400, nothing)  # Ancho arriba
        cv2.createTrackbar("Ancho Bot", "Calibracion", 700, 900, nothing)  # Ancho abajo

        # --- PARÁMETROS DE CONTROL ---
        # Suavizado: Qué tanto confiamos en el nuevo valor vs el anterior.
        cv2.createTrackbar("Suavizado (%)", "Calibracion", 20, 100, nothing)
        # Ancho Carril: Píxeles estimados entre líneas (útil si solo vemos una línea).
        cv2.createTrackbar("Ancho Carril (px)", "Calibracion", 350, 800, nothing)

        self.get_logger().info("Nodo de Visión Iniciado. Abre 'Calibracion' para ajustar.")

    def fit_lane(self, points):
        """
        Intenta ajustar una línea recta a un conjunto de puntos blancos.
        Retorna la línea [vx, vy, x0, y0] o None si no es válida.
        """
        # Si hay muy pocos puntos, es ruido. Ignoramos.
        if len(points) < 50:
            return None
            
        # cv2.fitLine usa mínimos cuadrados para hallar la mejor línea
        line = cv2.fitLine(points, cv2.DIST_L2, 0, 0.01, 0.01)
        vx, vy, x0, y0 = line.flatten()
        
        # --- FILTRO DE PENDIENTE (Slope Filter) ---
        # Evita que detectemos líneas horizontales (como coches cruzando o sombras).
        # La pendiente m = vy / vx. 
        # Si vy es cercano a 0, la línea es horizontal. Queremos líneas verticales.
        if abs(vy) < 0.1: # Tolerancia para rechazar horizontales
            return None
            
        return line.flatten()

    def draw_lane(self, img, lane, color):
        """
        Dibuja una línea infinita en la imagen basándose en la ecuación de la recta.
        Retorna la coordenada X donde la línea cruza la parte inferior (para cálculos).
        """
        vx, vy, x0, y0 = lane
        h = img.shape[0]

        # Puntos para dibujar (desde 60% de altura hasta el fondo)
        y1 = int(h * 0.4) 
        y2 = h

        # Fórmula de la recta: x = x0 + (y - y0) * (vx / vy)
        x1 = int(x0 + (y1 - y0) * vx / vy)
        x2 = int(x0 + (y2 - y0) * vx / vy)

        cv2.line(img, (x1, y1), (x2, y2), color, 5)
        return x2

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().warn(f"Error convirtiendo imagen: {e}")
            return

        h, w, _ = frame.shape
        debug = frame.copy() # Copia para dibujar sin alterar el análisis

        # =================================================
        # 1. LEER VALORES DE LOS SLIDERS (CALIBRACIÓN)
        # =================================================
        h_min = cv2.getTrackbarPos("H Min", "Calibracion")
        h_max = cv2.getTrackbarPos("H Max", "Calibracion")
        s_min = cv2.getTrackbarPos("S Min", "Calibracion")
        s_max = cv2.getTrackbarPos("S Max", "Calibracion")
        v_min = cv2.getTrackbarPos("V Min", "Calibracion")
        v_max = cv2.getTrackbarPos("V Max", "Calibracion")

        corte_pct = cv2.getTrackbarPos("Corte Y (%)", "Calibracion")
        w_top = cv2.getTrackbarPos("Ancho Top", "Calibracion")
        w_bot = cv2.getTrackbarPos("Ancho Bot", "Calibracion")
        
        # Factor alpha: 0.2 significa 20% nuevo, 80% memoria (muy suave)
        alpha = cv2.getTrackbarPos("Suavizado (%)", "Calibracion") / 100.0
        # Ancho estimado del carril en píxeles (para modo de una sola línea)
        lane_width_virtual = cv2.getTrackbarPos("Ancho Carril (px)", "Calibracion")

        # =================================================
        # 2. DEFINIR REGIÓN DE INTERÉS (ROI - TRAPECIO)
        # =================================================
        corte_y = int(h * (corte_pct / 100))
        cx = w // 2 # Centro de la imagen

        # Definimos los 4 puntos del trapecio
        polygon = np.array([[
            (cx - w_bot // 2, h),       # Abajo Izquierda
            (cx + w_bot // 2, h),       # Abajo Derecha
            (cx + w_top, corte_y),      # Arriba Derecha
            (cx - w_top, corte_y)       # Arriba Izquierda
        ]], np.int32)

        # Creamos una máscara negra y "encendemos" (blanco) solo el trapecio
        mask_roi = np.zeros((h, w), dtype=np.uint8)
        cv2.fillPoly(mask_roi, polygon, 255)
        
        # Dibujamos el trapecio azul en el debug para referencia
        cv2.polylines(debug, polygon, True, (255, 0, 0), 2)

        # =================================================
        # 3. FILTRADO DE COLOR (Detección de Líneas)
        # =================================================
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Rango de colores definido por sliders
        lower_bound = np.array([h_min, s_min, v_min])
        upper_bound = np.array([h_max, s_max, v_max])
        
        mask_color = cv2.inRange(hsv, lower_bound, upper_bound)

        # Combinamos: Solo lo que tenga COLOR correcto Y esté dentro del ROI
        mask = cv2.bitwise_and(mask_color, mask_roi)

        # =================================================
        # 4. SEPARACIÓN Y AJUSTE DE CARRILES
        # =================================================
        # Obtenemos coordenadas de todos los píxeles blancos
        ys, xs = np.where(mask > 0)
        
        # Si no vemos nada, salimos temprano
        if len(xs) == 0:
            # Publicar imagen debug aunque no haya líneas
            self.publisher_debug.publish(self.bridge.cv2_to_imgmsg(debug, "bgr8"))
            cv2.waitKey(1)
            return

        points = np.column_stack((xs, ys))
        mid = w // 2
        
        # Dividimos la pantalla en Izquierda y Derecha
        left_pts = points[points[:, 0] < mid]
        right_pts = points[points[:, 0] >= mid]

        # Intentamos ajustar una línea matemática a cada lado
        new_left = self.fit_lane(left_pts)
        new_right = self.fit_lane(right_pts)

        # =================================================
        # 5. MEMORIA Y SUAVIZADO (Smoothing)
        # =================================================
        # Si detectamos una línea nueva, la promediamos con la anterior.
        # Si no, mantenemos la anterior (memoria).
        
        if new_left is not None:
            if self.left_lane is None:
                self.left_lane = new_left
            else:
                self.left_lane = alpha * new_left + (1 - alpha) * self.left_lane
        
        if new_right is not None:
            if self.right_lane is None:
                self.right_lane = new_right
            else:
                self.right_lane = alpha * new_right + (1 - alpha) * self.right_lane

        # =================================================
        # 6. LÓGICA DE NAVEGACIÓN (CÁLCULO DEL TARGET)
        # =================================================
        target_x = mid # Por defecto, queremos ir al centro de la imagen

        # CASO A: Vemos AMBOS carriles (Ideal)
        if self.left_lane is not None and self.right_lane is not None:
            xl = self.draw_lane(debug, self.left_lane, (0, 255, 0)) # Verde
            xr = self.draw_lane(debug, self.right_lane, (0, 255, 0)) # Verde
            
            # El objetivo es el punto medio exacto entre las dos líneas
            target_x = (xl + xr) // 2
            
            # Dibujamos línea de conexión amarilla
            cv2.line(debug, (xl, h-50), (xr, h-50), (0, 255, 255), 2)

        # CASO B: Solo vemos carril IZQUIERDO
        elif self.left_lane is not None:
            xl = self.draw_lane(debug, self.left_lane, (0, 0, 255)) # Rojo (advertencia)
            # Asumimos que el centro está a la derecha del carril izquierdo
            target_x = xl + (lane_width_virtual // 2)

        # CASO C: Solo vemos carril DERECHO
        elif self.right_lane is not None:
            xr = self.draw_lane(debug, self.right_lane, (0, 0, 255)) # Rojo (advertencia)
            # Asumimos que el centro está a la izquierda del carril derecho
            target_x = xr - (lane_width_virtual // 2)

        # =================================================
        # 7. CÁLCULO DE ERROR Y SALIDA
        # =================================================
        # Error normalizado: 
        # -1.0 (Izquierda extrema), 0.0 (Centrado), +1.0 (Derecha extrema)
        error = (target_x - mid) / float(mid)
        
        # Limitamos el error para que no sea excesivo (-1 a 1)
        error = np.clip(error, -1.0, 1.0)

        # Aplicamos suavizado también al error final de salida
        smooth_error = 0.7 * self.last_error + 0.3 * error
        self.last_error = smooth_error

        # --- DIBUJAR GUIAS VISUALES ---
        # Bolita CIAN: A donde queremos ir (Target)
        vis_target = int(mid + smooth_error * mid)
        cv2.circle(debug, (vis_target, int(h*0.8)), 15, (255, 255, 0), -1)
        
        # Línea AZUL: El centro del coche (Referencia)
        cv2.line(debug, (mid, int(h*0.5)), (mid, h), (255, 0, 0), 2)

        # Mostrar máscara pequeñita en la esquina (Picture in Picture)
        # Ayuda a ver qué "basura" está detectando el filtro de color
        pip = cv2.resize(mask, (160, 120))
        pip_color = cv2.cvtColor(pip, cv2.COLOR_GRAY2BGR)
        debug[0:120, 0:160] = pip_color
        cv2.rectangle(debug, (0,0), (160,120), (255,255,255), 1)

        # Publicar
        self.publisher_debug.publish(self.bridge.cv2_to_imgmsg(debug, "bgr8"))
        msg_err = Float32()
        msg_err.data = float(smooth_error)
        self.publisher_error.publish(msg_err)

        # NECESARIO para que los sliders funcionen
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