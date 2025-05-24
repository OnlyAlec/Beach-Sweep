from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'sensor_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/sensor.launch.py']),
        # Instalar scripts ejecutables en libexec
        (os.path.join('lib', package_name), glob('scripts/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dietpi',
    maintainer_email='alexis_chacont@hotmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'sensor_node = sensor_package.sensor_node:main',
        ],
    },
)
