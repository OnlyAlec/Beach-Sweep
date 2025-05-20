from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'vision_ia_core'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'data'), glob('data/*.mp4')), # --> Carpeta de datos
        (os.path.join('share', package_name, 'data'), glob('data/*.pt')),  # --> Carpeta de datos
        ('share/vision_ia_core/launch', ['launch/vision_ia.launch.py']),   # --> Archivo de lanzamiento
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='eli',
    maintainer_email='eli@todo.todo',
    description='This package contains the vision IA core functionalities',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'camera_node = vision_ia_core.camera_node:main',      # --> Nodo de cámara añadido
        'detect_node = vision_ia_core.detect_node:main',      # --> Nodo de detección añadido
        ],
    },
)
