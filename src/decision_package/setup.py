from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'decision_package'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/decision.launch.py']),
        # Instalar scripts ejecutables en libexec
        (os.path.join('lib', package_name), glob('scripts/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sofía Becerril, Omar Díaz, Alexis Chacon',
    maintainer_email='alexis_chacont@hotmail.com',
    author_email='sofibecerril123@gmail.com',
    description='This package contains the logic behind the robot\'s actions',
    license='MIT',
    entry_points={
        'console_scripts': [
            'decision_node = decision_package.decision_node:main',
        ],
    },
)
