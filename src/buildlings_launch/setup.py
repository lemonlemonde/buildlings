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
        # add launch file
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # add model
        (os.path.join('share', package_name, 'models', 'aruco_box'),
         glob('models/aruco_box/*.*')),  # model.sdf, model.config
        (os.path.join('share', package_name, 'models', 'aruco_box', 'media', 'materials', 'scripts'),
         glob('models/aruco_box/media/materials/scripts/*.*')),
        (os.path.join('share', package_name, 'models', 'aruco_box', 'media', 'materials', 'textures'),
         glob('models/aruco_box/media/materials/textures/*.*')),
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
