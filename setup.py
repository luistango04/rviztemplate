from setuptools import setup
import os
from glob import glob


package_name = 'rviz_transformations'


setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='ROS 2 package for visualizing OptiTrack data in RViz',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'externalnodepublish = rviz_transformations.externalnodepublish:main'
        ],
    },
    data_files=[
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*.stl')),
    ]
)