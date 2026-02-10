# 📦 Navegación por Visión - AutoModelCar

![ROS 2 Jazzy](https://img.shields.io/badge/ROS_2-Jazzy-blue?logo=ros&logoColor=white)
![Python](https://img.shields.io/badge/Python-3.10+-yellow?logo=python&logoColor=white)
![License](https://img.shields.io/badge/License-MIT-green)

Este paquete (`navegacion_vision`) implementa un sistema de percepción visual para la navegación autónoma del **AutoModelCar**. Utiliza algoritmos de visión artificial para detectar carriles, trazar líneas guía virtuales y calcular el error de dirección en tiempo real, robusteciendo el sistema frente a líneas discontinuas o cambios de iluminación.

## 🚀 Características Principales

* **Detección de Carriles Robusta:** Algoritmo basado en segmentación de color (HSV) y Región de Interés (ROI) dinámica.
* **Manejo de Líneas Discontinuas:** Implementación de memoria temporal y suavizado para mantener la trayectoria incluso cuando las líneas del carril se pierden momentáneamente.
* **Calibración en Tiempo Real:** Interfaz gráfica (GUI) integrada que permite ajustar parámetros "en vivo" sin detener el nodo:
    * Rango de color (HSV) para diferentes condiciones de luz.
    * Dimensiones del trapecio de visión (ROI).
    * Factores de suavizado y predicción.
* **Simulación por Video:** Capacidad de inyectar archivos de video `.mp4` como si fueran un *stream* de cámara en vivo para pruebas *offline*.

## 🛠️ Requisitos del Sistema

* **Sistema Operativo:** Ubuntu 24.04 (Noble Numbat)
* **Middleware:** ROS 2 Jazzy Jalisco
* **Dependencias de Python:**
    ```bash
    pip install opencv-python numpy
    ```
* **Paquetes de ROS:**
    * `cv_bridge`
    * `sensor_msgs`
    * `std_msgs`

## 📂 Dataset de Prueba

Para validar el funcionamiento sin el vehículo físico, utilizamos grabaciones de conducción real.

* **📥 Dataset de Prueba (Video Corto):** [Descargar desde Google Drive](https://drive.google.com/file/d/1GfZ6-pa46afb4cbG6QSfBaIjuwr_Jlx5/view?usp=sharing)
    * *Nota:* Descarga este archivo y guárdalo en tu equipo para realizar las pruebas.
* **Fuente Original:** [Ver en YouTube](https://www.youtube.com/watch?v=nABR88G_2cE)

## 💻 Instalación y Compilación

1.  **Clonar el repositorio** en tu workspace (`src`):
    ```bash
    cd ~/tu_workspace/src
    git clone [https://github.com/OPEN-SOURCE-UPIITA/C-ROS.git](https://github.com/OPEN-SOURCE-UPIITA/C-ROS.git)
    ```

2.  **Instalar dependencias y compilar:**
    ```bash
    cd ~/tu_workspace
    rosdep install --from-paths src --ignore-src -r -y
    colcon build --packages-select navegacion_vision
    source install/setup.bash
    ```

## ⚙️ Uso y Ejecución

El sistema funciona con una arquitectura modular, por lo que requerirás abrir **tres terminales** distintas dentro de tu workspace.

> **Nota:** Asegúrate de ejecutar `source install/setup.bash` en cada nueva terminal antes de correr los comandos.

### 1. Nodo de Adquisición (Terminal 1)
Este nodo publica el video como si fuera una cámara en vivo. En lugar de editar el código, pasaremos la ruta del video descargado como un argumento directo.

```bash
source install/setup.bash
ros2 run navegacion_vision adq_video --ros-args -p video_source:="/ruta/absoluta/a/tu/dataset/Manejo_video_corto.mp4"
```

### 2. Nodo de Procesamiento (Terminal 2)
Inicia el algoritmo de visión artificial. Al ejecutarse, se abrirá automáticamente la ventana "Calibración" para ajustar los filtros HSV y el ROI.

```bash
source install/setup.bash
ros2 run navegacion_vision proc_carril
```

### 3. Visualización (Terminal 3)
Utiliza rqt para monitorear tanto la imagen original como la procesada con las guías de dirección y el punto objetivo (bolita cian).

```bash
source install/setup.bash
ros2 run rqt_image_view rqt_image_view
```
