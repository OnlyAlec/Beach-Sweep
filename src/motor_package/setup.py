from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'motor_package'

setup(
    name=package_name,
    version='2.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/motor.launch.py']),
        # Instalar scripts ejecutables en libexec
        (os.path.join('lib', package_name), glob('scripts/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Gallo Morales, Alexis Chacon',
    maintainer_email='alexis_chacont@hotmail.com',
    author_email='elcorreodegallo@gmail.com',
    description='Package for controlling motors and servos via GPIO using lgpio.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'motor_node = motor_package.motor_node:main',
        ],
    },
)
