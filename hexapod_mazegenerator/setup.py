import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'hexapod_mazegenerator'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # Copy launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        
        # Copy world files
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        
        # Copy the model.config and model.sdf files
        (os.path.join('share', package_name, 'models', 'custom_maze'), glob('models/custom_maze/model.*')),
        
        # Copy the actual 3D mesh (.stl)``
        (os.path.join('share', package_name, 'models', 'custom_maze', 'meshes'), glob('models/custom_maze/meshes/*.stl')),

        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')), #for the yaml file
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@email.com',
    description='Hexapod Maze Environment',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)