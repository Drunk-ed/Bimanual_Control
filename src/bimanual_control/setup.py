from setuptools import setup
import os
from glob import glob

package_name = 'bimanual_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # 👇 Install launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Bimanual robot control with Interbotix RX200 arms',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'leader = bimanual_control.leader:main',
            'mimic = bimanual_control.mimic:main',
        ],
    },
)
