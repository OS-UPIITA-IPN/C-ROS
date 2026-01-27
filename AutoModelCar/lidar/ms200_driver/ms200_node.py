#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import serial
import struct
import math
import time

# CONFIGURACIÓN DEL PROTOCOLO MS200 / LD06
# Estructura del paquete (47 bytes en total):
# Header(2) + Speed(2) + StartAngle(2) + Data(36) + EndAngle(2) + Timestamp(2) + CRC(1)
HEADER = b'\x54\x2C'
PACKET_SIZE = 47

class MS200Node(Node):
    def __init__(self):
        super().__init__('ms200_node')
        
        # Parámetros configurables
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 230400)
        self.declare_parameter('frame_id', 'laser_frame')
        
        port = self.get_parameter('port').value
        baud = self.get_parameter('baudrate').value
        self.frame_id = self.get_parameter('frame_id').value
        
        # Publicador de ROS 2
        self.publisher_ = self.create_publisher(LaserScan, 'scan', 10)
        
        # Conexión Serial
        try:
            self.ser = serial.Serial(port, baud, timeout=1)
            self.get_logger().info(f'Conectado al LiDAR en {port} a {baud}')
        except Exception as e:
            self.get_logger().error(f'No se pudo conectar al puerto {port}: {e}')
            return

        # Variables para armar el scan de 360 grados
        self.scan_points = []  # Lista temporal de (ángulo, distancia)
        self.current_packet = bytearray()
        
        # Timer para leer el puerto constantemente
        self.create_timer(0.001, self.read_serial_data)

    def read_serial_data(self):
        if not self.ser.is_open:
            return
            
        # Leer todo lo que haya en el buffer
        try:
            if self.ser.in_waiting > 0:
                data = self.ser.read(self.ser.in_waiting)
                self.current_packet.extend(data)
        except Exception as e:
            self.get_logger().error(f'Error leyendo serial: {e}')
            
        # Procesar paquetes completos
        while len(self.current_packet) >= PACKET_SIZE:
            # Buscar el encabezado 54 2C
            if self.current_packet[0] == 0x54 and self.current_packet[1] == 0x2C:
                # Extraer un paquete completo
                packet = self.current_packet[:PACKET_SIZE]
                self.parse_packet(packet)
                # Quitar el paquete procesado del buffer
                self.current_packet = self.current_packet[PACKET_SIZE:]
            else:
                # Si no es encabezado, borrar el primer byte y seguir buscando
                self.current_packet.pop(0)

    def parse_packet(self, data):
        # Desempaquetar datos usando struct (Little Endian <)
        # H = unsigned short (2 bytes), B = unsigned char (1 byte)
        # fmt: Speed(H), StartAngle(H), 12x[Dist(H), Conf(B)], EndAngle(H), Timestamp(H), CRC(B)
        try:
            # Bytes 2-46 (ignoramos header 0-1)
            # Speed (2-3), Start (4-5), Data (6-41), End (42-43), Time (44-45), CRC (46)
            
            speed = struct.unpack('<H', data[2:4])[0]
            start_angle = struct.unpack('<H', data[4:6])[0] / 100.0
            end_angle = struct.unpack('<H', data[42:44])[0] / 100.0
            
            # Ajuste de vuelta completa (cruce por cero)
            if end_angle < start_angle:
                end_angle += 360.0
                
            # Calcular paso angular por punto (son 12 puntos)
            step = (end_angle - start_angle) / 11.0
            
            # Extraer los 12 puntos de medición
            for i in range(12):
                # Cada punto son 3 bytes: Dist(2) + Intensidad(1)
                base = 6 + (i * 3)
                distance_mm = struct.unpack('<H', data[base:base+2])[0]
                intensity = data[base+2]
                
                # Calcular ángulo real de este punto
                angle = start_angle + (step * i)
                if angle >= 360.0:
                    angle -= 360.0
                
                # Guardar punto: (ángulo en grados, distancia en metros)
                # Filtramos distancias inválidas (0) o muy grandes
                if distance_mm > 0:
                    self.scan_points.append((angle, distance_mm / 1000.0))
            
            # Detectar fin de vuelta (cuando el ángulo reinicia cerca de 0)
            # Si el start_angle es pequeño (ej. < 10) y tenemos muchos puntos, publicamos
            if start_angle < 10.0 and len(self.scan_points) > 100:
                self.publish_scan()
                
        except Exception as e:
            self.get_logger().warn(f'Error parseando paquete: {e}')

    def publish_scan(self):
        if not self.scan_points:
            return

        # Ordenar puntos por ángulo
        self.scan_points.sort(key=lambda x: x[0])
        
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        
        msg.angle_min = 0.0
        msg.angle_max = 2.0 * math.pi
        msg.range_min = 0.02  # 2cm
        msg.range_max = 12.0  # 12m
        
        # Resolución: 360 grados / número de puntos
        msg.angle_increment = (2.0 * math.pi) / len(self.scan_points)
        msg.time_increment = 0.0 # Opcional
        
        # Llenar el array de rangos
        # Creamos una lista vacía de tamaño fijo (360 grados / resolución)
        num_readings = len(self.scan_points)
        msg.ranges = [float('inf')] * num_readings
        
        # Mapear nuestros puntos a la estructura de ROS
        for i, (angle_deg, dist_m) in enumerate(self.scan_points):
            msg.ranges[i] = dist_m
            
        self.publisher_.publish(msg)
        # self.get_logger().info(f'Publicado scan con {num_readings} puntos')
        
        # Limpiar para la siguiente vuelta
        self.scan_points = []

def main(args=None):
    rclpy.init(args=args)
    node = MS200Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
