from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'vision_ia_core'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'data'), glob('data/*.pt')),  # Modelos YOLO
        (os.path.join('share', package_name, 'data'), glob('data/*.mp4')),  # Videos
        ('share/' + package_name + '/launch', [
            'launch/vision_ia.launch.py', 
            'launch/robot_system.launch.py', 
            'launch/vision.launch.py'
        ]),
        # Instalar scripts ejecutables en libexec
        (os.path.join('lib', package_name), glob('scripts/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Elizabeth Becerril, Alexis Chacon',
    maintainer_email='alexis_chacont@hotmail.com',
    author_email='elisaomg124@gmail.com',
    description='This package contains the vision IA core functionalities',
    license='MIT',
    entry_points={
        'console_scripts': [
            'camera_node = vision_ia_core.camera_node:main',
            'detect_node = vision_ia_core.detect_node:main',
        ],
    },
)
