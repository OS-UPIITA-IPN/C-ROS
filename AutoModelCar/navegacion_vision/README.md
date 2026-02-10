# 👁️ Navegación y Percepción Visual - AutoModelCar

![ROS 2 Jazzy](https://img.shields.io/badge/ROS_2-Jazzy-blue?logo=ros&logoColor=white)
![Python](https://img.shields.io/badge/Python-3.10+-yellow?logo=python&logoColor=white)
![License](https://img.shields.io/badge/License-MIT-green)

Este paquete (`navegacion_vision`) implementa el sistema de percepción para la navegación autónoma del **AutoModelCar**. Integra visión por computadora clásica con **sensores de profundidad (RGB-D)** para robustecer la toma de decisiones.

## 🚀 Módulos Principales

### 1. Detección de Carriles (`proc_carril`)
* **Segmentación HSV Dinámica:** Interfaz gráfica para ajustar filtros de color en tiempo real.
* **Algoritmo de Memoria:** Suavizado de trayectoria para tolerar líneas discontinuas o reflejos.
* **Cálculo de Error:** Determina el error de dirección (-1.0 a 1.0) para el control PID.

### 2. Detección de Señales (`proc_senial`)
* **Fusión RGB-D:** Utiliza la cámara de color para detectar la forma (Octágono Rojo) y el sensor de profundidad para medir la distancia exacta.
* **Estimación de Distancia:** Lectura directa del mapa de profundidad en milímetros para frenado preciso ante señales de STOP.

---

## 🛠️ Requisitos del Sistema

* **Sistema Operativo:** Ubuntu 24.04 (Noble Numbat)
* **Middleware:** ROS 2 Jazzy Jalisco
* **Hardware:** Cámara RGB-D compatible con driver `ascamera` (ej. HP60C).
* **Dependencias:**
    * `ascamera` (Driver de cámara, debe estar en el workspace)
    * `cv_bridge`, `sensor_msgs`, `std_msgs`
    * Librerías Python: `opencv-python`, `numpy`

---

## 💻 Instalación

1.  **Compilar el paquete:**
    ```bash
    cd ~/tu_workspace
    colcon build --packages-select navegacion_vision
    source install/setup.bash
    ```

> **Nota:** Asegúrate de que el paquete `ascamera` ya esté compilado y configurado previamente.

---

## ⚙️ Uso y Ejecución

El sistema funciona mediante una arquitectura de nodos distribuidos. Se recomienda abrir **3 terminales**.

### Terminal 1: Driver de Cámara
Inicia la comunicación con el hardware y publica las imágenes RGB y mapas de profundidad.
```bash
source install/setup.bash
ros2 launch ascamera hp60c.launch.py
```

### Terminal 2: Procesamiento de Carriles
Inicia el algoritmo de seguimiento. Se abrirá una ventana "Calibración" para ajustar los filtros HSV y el ROI en tiempo real.

```bash
source install/setup.bash
ros2 run navegacion_vision proc_carril
```

- Opcional: Si deseas utilizar un video de prueba o una cámara diferente, puedes especificar el tópico de entrada:

```bash
ros2 run navegacion_vision proc_carril --ros-args -p image_topic:="/video_source/image_raw"
```

### Terminal 3: Detección de Señales (STOP)
Inicia el nodo de percepción RGB-D para detectar señales de alto y medir la distancia al obstáculo.
```bash
source install/setup.bash
ros2 run navegacion_vision detect_stop
```

## 📡 Tópicos CLave

| Tópico | Tipo de Mensaje | Dirección | Descripción |
| :--- | :--- | :---: | :--- |
| `/ascamera_hp60c/camera_publisher/rgb0/image` | `sensor_msgs/Image` | Flujo de video a color (RGB) de la cámara real. |
| `/ascamera_hp60c/camera_publisher/depth0/image_raw` | `sensor_msgs/Image`| Mapa de profundidad en milímetros (para medir distancia). |
| `/steering_error` | `std_msgs/Float32` | Valor de error calculado (-1.0 a 1.0) para el control PID. |
| `/vision/debug_img` | `sensor_msgs/Image` | Imagen procesada con líneas y objetivo dibujados (Carriles). |
| `/vision/stop_debug` | `sensor_msgs/Image` | Imagen con detección de señales STOP y distancia superpuesta. |

## 🛠️ Herramientas de Depuración
Para visualizar lo que el robot está "viendo" e interpretando en tiempo real, utiliza rqt:
```bash
ros2 run rqt_image_view rqt_image_view
```
- Selecciona `/vision/debug_img` para ver el procesamiento de carriles.
- Selecciona `/vision/stop_debug` para ver la segmentación de señales de STOP.

## 📂 Dataset de Prueba (Offline)
Si no cuentas con el robot físico, puedes validar los algoritmos utilizando grabaciones de video.

- Dataset: [Enlace al Video en Drive]
- Nota: Para usar esto, necesitarás un nodo publicador de video que inyecte la imagen en el tópico `/ascamera_hp60c/camera_publisher/rgb0/image`.
