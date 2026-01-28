
# ascamera: Driver de Cámara RGBD para ROS 2

Este paquete permite la integración de la cámara de profundidad en ROS 2. 
Incluye configuración automática y soporte para arquitecturas x86 (PC) y ARM64 (Raspberry Pi).

![RViz Preview](rviz_preview.png)

## 1. Instalación y Clonado
Sitúate en la carpeta `src` de tu espacio de trabajo y clona este repositorio:

```bash
cd ~/ascam_ros2_ws/src
git clone <TU_URL_DEL_REPOSITORIO> ascamera

```

> **Nota:** Asegúrate de que la carpeta del paquete se llame `ascamera` y esté directamente dentro de `src`.

## 2. Instalación de Dependencias (CRÍTICO)

El driver utiliza librerías propietarias (.so) que deben instalarse manualmente en el sistema para que funcione.

### Opción A: Si estás en Raspberry Pi (ARM64)

Ejecuta estos comandos para instalar las librerías específicas de ARM:

```bash
# 1. Ve a la carpeta de librerías ARM
cd ~/ascam_ros2_ws/src/ascamera/libs/lib/aarch64-linux-gnu/

# 2. Copia las librerías necesarias al sistema
sudo cp libAngstrongCameraSdk.so libAngKondyorArith.so libalg_kunlun.so libFilt.so libasusb.so libasuvc.so /usr/lib/

# 3. Actualiza la caché de librerías
sudo ldconfig

```

### Opción B: Si estás en PC / Laptop (x86_64)

Ejecuta estos comandos para instalar las librerías de PC:

```bash
# 1. Ve a la carpeta de librerías x86
cd ~/ascam_ros2_ws/src/ascamera/libs/lib/x86_64-linux-gnu/

# 2. Copia todas las librerías al sistema
sudo cp *.so /usr/lib/

# 3. Actualiza la caché de librerías
sudo ldconfig

```

## 3. Compilación

Compila el paquete desde la raíz de tu espacio de trabajo:

```bash
cd ~/ascam_ros2_ws/
colcon build --symlink-install --packages-select ascamera

```

## 4. Configuración de permisos USB (Reglas udev)

Para usar la cámara sin necesidad de `sudo` cada vez:

```bash
cd ~/ascam_ros2_ws/src/ascamera/scripts
sudo bash create_udev_rules.sh

```

> **Importante:** Desconecta y vuelve a conectar la cámara USB después de este paso.

## 5. Ejecución (Launch)

Carga el entorno y lanza el driver. El sistema detectará automáticamente la ruta de configuración.

```bash
source ~/ascam_ros2_ws/install/setup.bash
ros2 launch ascamera hp60c.launch.py

```

## 6. Visualización

Para ver las imágenes y la nube de puntos:

### Opción 1: RQT Image View (Rápido)

```bash
ros2 run rqt_image_view rqt_image_view

```

### Opción 2: RViz2 (Completo)

```bash
rviz2

```

**Configuración recomendada en RViz:**

* **Fixed Frame:** `ascamera_hp60c_color_0`
* **Add -> By Topic:** Selecciona `/ascamera/images` o `/ascamera/depth/points`.