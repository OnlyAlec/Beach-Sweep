from setuptools import find_packages, setup

package_name = 'decision_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'vision_ia_msgs'],
    zip_safe=True,
    maintainer='dietpi',
    maintainer_email='alexis_chacont@hotmail.com',
    description='This package contains the logic behind the robots actions',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'decision_node = decision_package.decision_node:main',
        ],
    },
)
