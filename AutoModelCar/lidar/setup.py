from setuptools import find_packages, setup

package_name = 'ms200_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jonathan Jason Medina Martinez',
    maintainer_email='jason240208@gmail.com',
    description='Driver de ROS 2 Jazzy para el LiDAR MS200/Oradar desarrollado desde cero.',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'ms200_node = ms200_driver.ms200_node:main'
        ],
    },
)
