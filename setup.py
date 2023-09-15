from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'yolov5_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        (os.path.join('share', package_name, 'config'), glob('config/**')),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fishros',
    maintainer_email='87068644+fishros@users.noreply.github.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
       'console_scripts': [
            "yolov5_ros2=yolov5_ros2.yolov5_ros2:main"
        ],
    },
)


