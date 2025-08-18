# This is not used until you change package.xml to include the following:
#   <export>
#     <build_type>ament_python</build_type>
#   </export>

from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'face_gesture_sensor'

setup(
    name=package_name,
    version='0.1.0',
    #packages=find_packages(),
    packages=[package_name, package_name + '/utils'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # launch files
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=[
        'setuptools',
        'DFRobot_GestureFaceDetection',
        'DFRobot_RTU'],
    zip_safe=True,
    maintainer='Sergei Grichine',
    maintainer_email='slg@quakemap.com',
    description='Face and Gesture detection node',
    license='MIT',
     entry_points={
         'console_scripts': [
             f'fgs_node = {package_name}.fgs_node:main',
         ],
     },
)
