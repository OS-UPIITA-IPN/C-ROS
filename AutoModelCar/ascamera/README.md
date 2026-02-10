# Driver de Cámara RGB-D (ASCamera)

El paquete ascamera permite la integración de una cámara RGB-D en ROS 2.
Proporciona imágenes RGB, mapas de profundidad y nubes de puntos PointCloud2
para aplicaciones de visión y percepción 3D.

==================================================

CONTENIDO DEL PAQUETE

- Publicación de imagen RGB
- Publicación de mapa de profundidad
- Publicación de nube de puntos 3D
- Soporte para arquitecturas x86_64 y ARM (Raspberry Pi)

==================================================

INSTALACIÓN DE DEPENDENCIAS (CRÍTICO)

Este driver utiliza librerías propietarias que deben instalarse manualmente
antes de compilar. Si se omite este paso, el paquete no funcionará.

--------------------------------------------------

PC / Laptop (x86_64)

1) Desde tu workspace, ve a la carpeta de librerías:

cd src/ascamera/libs/lib/x86_64-linux-gnu/

2) Copia las librerías al sistema y actualiza la caché:

sudo cp *.so /usr/lib/
sudo ldconfig

--------------------------------------------------

Raspberry Pi / ARM (aarch64)

cd src/ascamera/libs/lib/aarch64-linux-gnu/
sudo cp *.so /usr/lib/
sudo ldconfig

==================================================

CONFIGURACIÓN USB (Reglas udev)

Para usar la cámara sin sudo y evitar problemas de permisos:

cd src/ascamera/scripts
sudo bash create_udev_rules.sh

IMPORTANTE:
Desconecta y vuelve a conectar la cámara después de este paso.

==================================================

COMPILACIÓN DEL PAQUETE

cd ~/tu_workspace
colcon build --packages-select ascamera
source install/setup.bash

==================================================

EJECUCIÓN

ros2 launch ascamera hp60c.launch.py

==================================================

TÓPICOS PRINCIPALES

/ascamera_hp60c/camera_publisher/rgb0/image
Imagen RGB

/ascamera_hp60c/camera_publisher/depth0/image_raw
Mapa de profundidad

/ascamera_hp60c/camera_publisher/depth0/points
Nube de puntos 3D

==================================================

VISUALIZACIÓN (RViz2)

Para ver la Nube de Puntos (PointCloud2) correctamente, es necesaria
una configuración específica de QoS debido al alto ancho de banda.

1) Ejecuta RViz2:
ros2 run rviz2 rviz2

2) Configuración Global (Panel Izquierdo):
- Fixed Frame: ascamera_hp60c_color_0

3) Añadir la Nube:
- Botón "Add" -> Pestaña "By Topic"
- Selecciona: /ascamera_hp60c/camera_publisher/depth0/points
- Tipo: PointCloud2

4) CONFIGURACIÓN PARA EVITAR COMPLICACIONES (opcional):
En el panel izquierdo, despliega las opciones de "PointCloud2" y ajusta:
- Reliability Policy: Best Effort  <-- (Vital para evitar lag)
- Durability Policy: Volatile
- Style: Points
- Color Transformer: AxisColor     <-- (Para colorear por profundidad)

==================================================

NOTAS

- Verifica que las librerías propietarias correspondan a tu arquitectura.
- Paquete diseñado para ROS 2.

==================================================