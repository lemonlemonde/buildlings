from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'buildlings_launch'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # launch file
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # aruco box model
        ('share/' + package_name + '/models/aruco_box', 
            ['models/aruco_box/model.config', 'models/aruco_box/model.sdf']),
        ('share/' + package_name + '/models/aruco_box/materials/scripts', 
            ['models/aruco_box/materials/scripts/aruco_cube.material']),
        ('share/' + package_name + '/models/aruco_box/materials/textures', 
            ['models/aruco_box/materials/textures/aruco_42.png']),
        # params
        (os.path.join('share', package_name, 'params'), glob('params/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mousey',
    maintainer_email='mirudjun@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
