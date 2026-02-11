from setuptools import find_packages, setup
import os  
from glob import glob 

package_name = 'gauntlet_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Standard ROS 2 resource indexing
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # Include all launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        
        # Include all world files
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
        
        # If you have a maps folder, uncomment this:
        # (os.path.join('share', package_name, 'maps'), glob('maps/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ayush',
    maintainer_email='ayush@todo.todo',
    description='Gauntlet Simulation Package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'planner_server = gauntlet_sim.planner_server:main',
            'path_follower = gauntlet_sim.path_follower_A_star:main',
            'path_follower_gvd = gauntlet_sim.path_follower_GVD:main',
        ],
    },
)
