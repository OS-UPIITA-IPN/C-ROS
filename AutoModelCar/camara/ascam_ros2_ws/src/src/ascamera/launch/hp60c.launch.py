import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # 1. BUSCAR LA CARPETA AUTOMÁTICAMENTE
    pkg_share = get_package_share_directory('ascamera')

    # 2. CONSTRUIR LA RUTA A LA CARPETA DE CONFIGURACIÓN
    config_dir_path = os.path.join(pkg_share, 'configurationfiles')

    print(f"--> Usando ruta de configuración dinámica: {config_dir_path}")

    ld = LaunchDescription()

    ascamera_node = Node(
        namespace= "ascamera_hp60c",
        package='ascamera',
        executable='ascamera_node',
        respawn=True,
        output='both',
        parameters=[
            {"usb_bus_no": -1},
            {"usb_path": "null"},
            {"confiPath": config_dir_path},
            {"color_pcl": False},
            {"pub_tfTree": True},
            {"depth_width": 640},
            {"depth_height": 480},
            {"rgb_width": 640},
            {"rgb_height": 480},
            {"fps": 25},
        ],
        remappings=[]
    )

    ld.add_action(ascamera_node)

    return ld