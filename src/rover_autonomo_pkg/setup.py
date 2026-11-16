import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'rover_autonomo_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ivan',
    maintainer_email='ivan@todo.todo',
    description='Paquete base para robot autonomo en ROS 2 Jazzy',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ejecutable_esp32 = rover_autonomo_pkg.nodo_esp32_bridge:main',
            'ejecutable_sensores = rover_autonomo_pkg.nodo_sensores:main',
            'ejecutable_control = rover_autonomo_pkg.nodo_control:main',
            'ejecutable_odometria = rover_autonomo_pkg.nodo_odometria:main',
            'ejecutable_camara = rover_autonomo_pkg.nodo_camara:main',
            'ejecutable_mapeo = rover_autonomo_pkg.nodo_mapeo:main',
            'ejecutable_mapa_peligros = rover_autonomo_pkg.nodo_peligros:main',
        ],
    },
)